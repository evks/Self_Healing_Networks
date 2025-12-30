#include "aodv.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/select.h>
#include <ctime>
#include <iomanip>

// Timer Implementation
Timer::Timer(int seconds, std::function<void()> callback)
    : seconds_(seconds), callback_(callback), active_(false) {}

Timer::~Timer() {
    cancel();
}

void Timer::start() {
    active_ = true;
    timer_thread_ = std::thread(&Timer::timer_function, this);
}

void Timer::cancel() {
    active_ = false;
    if (timer_thread_.joinable()) {
        timer_thread_.join();
    }
}

bool Timer::is_active() const {
    return active_;
}

void Timer::timer_function() {
    auto start_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::seconds(seconds_);
    
    while (active_) {
        auto current_time = std::chrono::steady_clock::now();
        if (current_time - start_time >= duration) {
            if (active_ && callback_) {
                callback_();
            }
            active_ = false;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// AODV Implementation
AODV::AODV()
    : num_nodes_(0),
      seq_no_(0),
      rreq_id_(0),
      listener_port_(0),
      listener_thread_port_(0),
      aodv_port_(0),
      tester_port_(0),
      listener_sock_(-1),
      aodv_sock_(-1),
      tester_sock_(-1),
      status_("Active"),
      running_(false) {}

AODV::~AODV() {
    stop();
    close_sockets();
}

void AODV::set_node_id(const std::string& nid) {
    node_id_ = nid;
}

void AODV::set_node_count(int count) {
    num_nodes_ = count;
}

int AODV::get_listener_thread_port(const std::string& node) const {
    static const std::map<std::string, int> port_map = {
        {"n1", 1000}, {"n2", 1100}, {"n3", 1200}, {"n4", 1300}, {"n5", 1400},
        {"n6", 1500}, {"n7", 1600}, {"n8", 1700}, {"n9", 1800}, {"n10", 1900}
    };
    auto it = port_map.find(node);
    return (it != port_map.end()) ? it->second : 0;
}

int AODV::get_listener_port(const std::string& node) const {
    static const std::map<std::string, int> port_map = {
        {"n1", 2000}, {"n2", 2100}, {"n3", 2200}, {"n4", 2300}, {"n5", 2400},
        {"n6", 2500}, {"n7", 2600}, {"n8", 2700}, {"n9", 2800}, {"n10", 2900}
    };
    auto it = port_map.find(node);
    return (it != port_map.end()) ? it->second : 0;
}

int AODV::get_aodv_port(const std::string& node) const {
    static const std::map<std::string, int> port_map = {
        {"n1", 3000}, {"n2", 3100}, {"n3", 3200}, {"n4", 3300}, {"n5", 3400},
        {"n6", 3500}, {"n7", 3600}, {"n8", 3700}, {"n9", 3800}, {"n10", 3900}
    };
    auto it = port_map.find(node);
    return (it != port_map.end()) ? it->second : 0;
}

int AODV::get_tester_port(const std::string& node) const {
    static const std::map<std::string, int> port_map = {
        {"n1", 5100}, {"n2", 5200}, {"n3", 5300}, {"n4", 5400}, {"n5", 5500},
        {"n6", 5600}, {"n7", 5700}, {"n8", 5800}, {"n9", 5900}, {"n10", 6000}
    };
    auto it = port_map.find(node);
    return (it != port_map.end()) ? it->second : 0;
}

void AODV::aodv_restart_route_timer(RouteEntry& route, bool create) {
    if (!create && route.lifetime) {
        route.lifetime->cancel();
    }
    
    auto timeout_callback = [this, dest = route.destination]() {
        this->aodv_process_route_timeout(dest);
    };
    
    route.lifetime = std::make_shared<Timer>(AODV_ACTIVE_ROUTE_TIMEOUT, timeout_callback);
    route.status = "Active";
    route.lifetime->start();
}

void AODV::aodv_send(const std::string& destination, int destination_port,
                     const std::string& message) {
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(destination_port);
    inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr);
    
    sendto(aodv_sock_, message.c_str(), message.length(), 0,
           (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void AODV::aodv_send_hello_message() {
    try {
        // Send message to each neighbor
        for (const auto& neighbor_pair : neighbors_) {
            const std::string& n = neighbor_pair.first;
            std::string message_type = "HELLO_MESSAGE";
            std::string sender = node_id_;
            std::string message_data = "Hello message from " + node_id_;
            std::string message = message_type + ":" + sender + ":" + message_data;
            
            int port = get_aodv_port(n);
            aodv_send(n, port, message);
            
            log_debug("['" + message_type + "', '" + sender + "', Sending hello message to " + n + "']");
        }
        
        // Restart the timer
        hello_timer_->cancel();
        hello_timer_ = std::make_shared<Timer>(AODV_HELLO_INTERVAL,
                                               [this]() { this->aodv_send_hello_message(); });
        hello_timer_->start();
    } catch (...) {
        // Handle exceptions silently
    }
}

void AODV::aodv_process_hello_message(const std::vector<std::string>& message) {
    if (message.size() < 2) return;
    
    std::string sender = message[1];
    
    try {
        auto neighbor_it = neighbors_.find(sender);
        if (neighbor_it != neighbors_.end()) {
            // Restart neighbor liveness timer
            neighbor_it->second.timer_callback->cancel();
            
            auto timeout_callback = [this, sender]() {
                this->aodv_process_neighbor_timeout(sender);
            };
            
            neighbor_it->second.timer_callback = std::make_shared<Timer>(
                AODV_HELLO_TIMEOUT, timeout_callback);
            neighbor_it->second.timer_callback->start();
            
            // Restart the lifetime timer for the route
            auto route_it = routing_table_.find(sender);
            if (route_it != routing_table_.end()) {
                aodv_restart_route_timer(route_it->second, false);
            }
        } else {
            // New neighbor detected
            auto timeout_callback = [this, sender]() {
                this->aodv_process_neighbor_timeout(sender);
            };
            
            Neighbor new_neighbor;
            new_neighbor.neighbor = sender;
            new_neighbor.timer_callback = std::make_shared<Timer>(
                AODV_HELLO_TIMEOUT, timeout_callback);
            new_neighbor.timer_callback->start();
            neighbors_[sender] = new_neighbor;
            
            // Update routing table
            auto route_it = routing_table_.find(sender);
            if (route_it != routing_table_.end()) {
                aodv_restart_route_timer(route_it->second, false);
            } else {
                RouteEntry new_route;
                new_route.destination = sender;
                new_route.destination_port = std::to_string(get_aodv_port(sender));
                new_route.next_hop = sender;
                new_route.next_hop_port = std::to_string(get_aodv_port(sender));
                new_route.seq_no = "1";
                new_route.hop_count = "1";
                new_route.status = "Active";
                routing_table_[sender] = new_route;
                aodv_restart_route_timer(routing_table_[sender], true);
            }
        }
    } catch (...) {
        // Neighbor not added yet, ignore
    }
}

void AODV::aodv_process_user_message(const std::vector<std::string>& message) {
    if (message.size() < 4) return;
    
    std::string sender = message[1];
    std::string receiver = message[2];
    std::string msg = message[3];
    
    if (receiver == node_id_) {
        // Message is for us
        Message new_msg;
        new_msg.sender = sender;
        new_msg.message = msg;
        message_box_[msg] = new_msg;
        
        log_debug("['" + message[0] + "', '" + sender + " to " + receiver + "', '" + msg + "']");
        std::cout << "New message arrived. Issue 'view_messages' to see the contents" << std::endl;
    } else {
        // Forward the message
        auto route_it = routing_table_.find(receiver);
        if (route_it != routing_table_.end()) {
            RouteEntry& route = route_it->second;
            std::string next_hop = route.next_hop;
            int next_hop_port = std::stoi(route.next_hop_port);
            aodv_restart_route_timer(route, false);
            
            std::string forward_msg = message[0] + ":" + message[1] + ":" + 
                                     message[2] + ":" + message[3];
            aodv_send(next_hop, next_hop_port, forward_msg);
            
            log_debug("['USER_MESSAGE', '" + sender + " to " + receiver + "', " + msg + "']");
        }
    }
}

void AODV::aodv_process_rreq_message(std::vector<std::string>& message) {
    if (message.size() < 8) return;
    
    std::string message_type = message[0];
    std::string sender = message[1];
    int hop_count = std::stoi(message[2]) + 1;
    message[2] = std::to_string(hop_count);
    int rreq_id = std::stoi(message[3]);
    std::string dest = message[4];
    int dest_seq_no = std::stoi(message[5]);
    std::string orig = message[6];
    int orig_seq_no = std::stoi(message[7]);
    
    if (status_ == "Inactive") return;
    
    log_debug("['" + message[0] + "', 'Received RREQ to " + dest + " from " + sender + "']");
    
    // Check for duplicate RREQ
    auto rreq_list_it = rreq_id_list_.find(orig);
    if (rreq_list_it != rreq_id_list_.end()) {
        auto& per_node_list = rreq_list_it->second.rreq_id_list;
        if (per_node_list.find(rreq_id) != per_node_list.end()) {
            log_debug("['RREQ_MESSAGE', 'Ignoring duplicate RREQ (" + orig + ", " + 
                     std::to_string(rreq_id) + ") from " + sender + "']");
            return;
        }
    }
    
    // Buffer the RREQ
    RreqIdList& node_list = rreq_id_list_[orig];
    node_list.node = node_id_;
    
    auto timeout_callback = [this, orig, rreq_id]() {
        this->aodv_process_path_discovery_timeout(orig, rreq_id);
    };
    
    RreqIdEntry new_entry;
    new_entry.rreq_id = rreq_id;
    new_entry.timer_callback = std::make_shared<Timer>(
        AODV_PATH_DISCOVERY_TIME, timeout_callback);
    new_entry.timer_callback->start();
    node_list.rreq_id_list[rreq_id] = new_entry;
    
    // Update/create route to originator
    int orig_port = get_aodv_port(orig);
    int sender_port = get_aodv_port(sender);
    
    auto route_it = routing_table_.find(orig);
    if (route_it != routing_table_.end()) {
        RouteEntry& route = route_it->second;
        int route_seq = std::stoi(route.seq_no);
        
        if (route_seq < orig_seq_no || 
            (route_seq == orig_seq_no && std::stoi(route.hop_count) > hop_count) ||
            route_seq == -1) {
            route.seq_no = std::to_string(orig_seq_no);
            route.hop_count = std::to_string(hop_count);
            route.next_hop = sender;
            route.next_hop_port = std::to_string(sender_port);
            aodv_restart_route_timer(route, false);
        }
    } else {
        RouteEntry new_route;
        new_route.destination = orig;
        new_route.destination_port = std::to_string(orig_port);
        new_route.next_hop = sender;
        new_route.next_hop_port = std::to_string(sender_port);
        new_route.seq_no = std::to_string(orig_seq_no);
        new_route.hop_count = std::to_string(hop_count);
        new_route.status = "Active";
        routing_table_[orig] = new_route;
        aodv_restart_route_timer(routing_table_[orig], true);
    }
    
    // Check if we are the destination
    if (node_id_ == dest) {
        aodv_send_rrep(orig, sender, dest, dest, 0, 0);
        return;
    }
    
    // Check if we have a valid route to destination
    auto dest_route_it = routing_table_.find(dest);
    if (dest_route_it != routing_table_.end()) {
        RouteEntry& route = dest_route_it->second;
        int route_dest_seq = std::stoi(route.seq_no);
        
        if (route.status == "Active" && route_dest_seq >= dest_seq_no) {
            aodv_send_rrep(orig, sender, node_id_, dest, route_dest_seq, 
                          std::stoi(route.hop_count));
            return;
        }
    }
    
    // Rebroadcast the RREQ
    aodv_forward_rreq(message);
}

void AODV::aodv_process_rrep_message(std::vector<std::string>& message) {
    if (message.size() < 6) return;
    
    std::string message_type = message[0];
    std::string sender = message[1];
    int hop_count = std::stoi(message[2]) + 1;
    message[2] = std::to_string(hop_count);
    std::string dest = message[3];
    int dest_seq_no = std::stoi(message[4]);
    std::string orig = message[5];
    
    log_debug("['" + message_type + "', 'Received RREP for " + dest + " from " + sender + "']");
    
    if (node_id_ == orig) {
        // We originated the RREQ, consume the RREP
        auto route_it = routing_table_.find(dest);
        if (route_it != routing_table_.end()) {
            RouteEntry& route = route_it->second;
            int route_hop_count = std::stoi(route.hop_count);
            if (route_hop_count > hop_count) {
                route.hop_count = std::to_string(hop_count);
                aodv_restart_route_timer(route, false);
            }
        } else {
            RouteEntry new_route;
            new_route.destination = dest;
            new_route.destination_port = std::to_string(get_aodv_port(dest));
            new_route.next_hop = sender;
            new_route.next_hop_port = std::to_string(get_aodv_port(sender));
            new_route.seq_no = std::to_string(dest_seq_no);
            new_route.hop_count = std::to_string(hop_count);
            new_route.status = "Active";
            routing_table_[dest] = new_route;
            aodv_restart_route_timer(routing_table_[dest], true);
        }
        
        // Send pending messages
        auto msg_it = pending_msg_q_.begin();
        while (msg_it != pending_msg_q_.end()) {
            std::vector<std::string> msg_parts = split_string(*msg_it, ':');
            if (msg_parts.size() >= 3 && msg_parts[2] == dest) {
                std::string next_hop = sender;
                int next_hop_port = get_aodv_port(next_hop);
                aodv_send(next_hop, next_hop_port, *msg_it);
                
                log_debug("['USER_MESSAGE', '" + msg_parts[1] + " to " + msg_parts[2] + 
                         " via " + next_hop + "', '" + msg_parts[3] + "']");
                std::cout << "Message sent" << std::endl;
                msg_it = pending_msg_q_.erase(msg_it);
            } else {
                ++msg_it;
            }
        }
    } else {
        // Forward the RREP
        auto dest_route_it = routing_table_.find(dest);
        if (dest_route_it != routing_table_.end()) {
            RouteEntry& route = dest_route_it->second;
            route.status = "Active";
            route.seq_no = std::to_string(dest_seq_no);
            aodv_restart_route_timer(route, false);
        } else {
            RouteEntry new_route;
            new_route.destination = dest;
            new_route.destination_port = std::to_string(get_aodv_port(dest));
            new_route.next_hop = sender;
            new_route.next_hop_port = std::to_string(get_aodv_port(sender));
            new_route.seq_no = std::to_string(dest_seq_no);
            new_route.hop_count = std::to_string(hop_count);
            new_route.status = "Active";
            routing_table_[dest] = new_route;
            aodv_restart_route_timer(routing_table_[dest], true);
        }
        
        // Lookup next-hop for source and forward
        auto orig_route_it = routing_table_.find(orig);
        if (orig_route_it != routing_table_.end()) {
            RouteEntry& route = orig_route_it->second;
            std::string next_hop = route.next_hop;
            int next_hop_port = std::stoi(route.next_hop_port);
            aodv_forward_rrep(message, next_hop, next_hop_port);
        }
    }
}

void AODV::aodv_process_rerr_message(const std::vector<std::string>& message) {
    if (message.size() < 5) return;
    
    std::string message_type = message[0];
    std::string sender = message[1];
    std::string dest = message[3];
    int dest_seq_no = std::stoi(message[4]);
    
    if (node_id_ == dest) return;
    
    log_debug("['" + message_type + "', 'Received RERR for " + dest + " from " + sender + "']");
    
    auto route_it = routing_table_.find(dest);
    if (route_it != routing_table_.end()) {
        RouteEntry& route = route_it->second;
        if (route.status == "Active" && route.next_hop == sender) {
            route.status = "Inactive";
            aodv_forward_rerr(message);
        } else {
            log_debug("['" + message_type + "', 'Ignoring RERR for " + dest + 
                     " from " + sender + "']");
        }
    }
}

void AODV::aodv_send_rreq(const std::string& destination, int destination_seq_no) {
    seq_no_++;
    rreq_id_++;
    
    std::string message_type = "RREQ_MESSAGE";
    std::string sender = node_id_;
    int hop_count = 0;
    
    std::string message = message_type + ":" + sender + ":" + std::to_string(hop_count) + ":" +
                         std::to_string(rreq_id_) + ":" + destination + ":" +
                         std::to_string(destination_seq_no) + ":" + node_id_ + ":" +
                         std::to_string(seq_no_);
    
    // Broadcast to all neighbors
    for (const auto& neighbor_pair : neighbors_) {
        int port = get_aodv_port(neighbor_pair.first);
        aodv_send(neighbor_pair.first, port, message);
    }
    
    log_debug("['" + message_type + "', 'Broadcasting RREQ to " + destination + "']");
    
    // Buffer the RREQ_ID
    RreqIdList& node_list = rreq_id_list_[node_id_];
    node_list.node = node_id_;
    
    auto timeout_callback = [this, node_id = node_id_, rreq_id = rreq_id_]() {
        this->aodv_process_path_discovery_timeout(node_id, rreq_id);
    };
    
    RreqIdEntry new_entry;
    new_entry.rreq_id = rreq_id_;
    new_entry.timer_callback = std::make_shared<Timer>(
        AODV_PATH_DISCOVERY_TIME, timeout_callback);
    new_entry.timer_callback->start();
    node_list.rreq_id_list[rreq_id_] = new_entry;
}

void AODV::aodv_forward_rreq(const std::vector<std::string>& message) {
    if (message.size() < 8) return;
    
    std::string msg = message[0] + ":" + node_id_ + ":" + message[2] + ":" +
                     message[3] + ":" + message[4] + ":" + message[5] + ":" +
                     message[6] + ":" + message[7];
    
    for (const auto& neighbor_pair : neighbors_) {
        int port = get_aodv_port(neighbor_pair.first);
        aodv_send(neighbor_pair.first, port, msg);
    }
    
    log_debug("['" + message[0] + "', 'Rebroadcasting RREQ to " + message[4] + "']");
}

void AODV::aodv_send_rrep(const std::string& rrep_dest, const std::string& rrep_nh,
                          const std::string& rrep_src, const std::string& rrep_int_node,
                          int dest_seq_no, int hop_count) {
    if (rrep_src == rrep_int_node) {
        seq_no_++;
        dest_seq_no = seq_no_;
        hop_count = 0;
    }
    
    std::string message_type = "RREP_MESSAGE";
    std::string message = message_type + ":" + node_id_ + ":" + std::to_string(hop_count) + ":" +
                         rrep_int_node + ":" + std::to_string(dest_seq_no) + ":" + rrep_dest;
    
    int port = get_aodv_port(rrep_nh);
    aodv_send(rrep_nh, port, message);
    
    log_debug("['" + message_type + "', 'Sending RREP for " + rrep_int_node + " to " +
             rrep_dest + " via " + rrep_nh + "']");
}

void AODV::aodv_forward_rrep(const std::vector<std::string>& message,
                             const std::string& next_hop, int next_hop_port) {
    if (message.size() < 6) return;
    
    std::string msg = message[0] + ":" + node_id_ + ":" + message[2] + ":" +
                     message[3] + ":" + message[4] + ":" + message[5];
    
    aodv_send(next_hop, next_hop_port, msg);
    
    log_debug("['" + message[0] + "', 'Forwarding RREP for " + message[5] + " to " +
             next_hop + "']");
}

void AODV::aodv_send_rerr(const std::string& dest, int dest_seq_no) {
    std::string message_type = "RERR_MESSAGE";
    std::string message = message_type + ":" + node_id_ + ":1:" + dest + ":" +
                         std::to_string(dest_seq_no + 1);
    
    for (const auto& neighbor_pair : neighbors_) {
        int port = get_aodv_port(neighbor_pair.first);
        aodv_send(neighbor_pair.first, port, message);
    }
    
    log_debug("['" + message_type + "', 'Sending RERR for " + dest + "']");
}

void AODV::aodv_forward_rerr(const std::vector<std::string>& message) {
    if (message.size() < 5) return;
    
    std::string msg = message[0] + ":" + node_id_ + ":" + message[2] + ":" +
                     message[3] + ":" + message[4];
    
    for (const auto& neighbor_pair : neighbors_) {
        int port = get_aodv_port(neighbor_pair.first);
        aodv_send(neighbor_pair.first, port, msg);
    }
    
    log_debug("['" + message[0] + "', 'Forwarding RERR for " + message[3] + "']");
}

void AODV::aodv_process_neighbor_timeout(const std::string& neighbor) {
    auto route_it = routing_table_.find(neighbor);
    if (route_it != routing_table_.end()) {
        RouteEntry& route = route_it->second;
        route.status = "Inactive";
        
        log_debug("aodv_process_neighbor_timeout: " + neighbor + " went down");
        
        aodv_send_rerr(neighbor, std::stoi(route.seq_no));
        
        int dest_seq_no = std::stoi(route.seq_no) + 1;
        aodv_send_rreq(neighbor, dest_seq_no);
    }
}

void AODV::aodv_process_path_discovery_timeout(const std::string& node, int rreq_id) {
    auto node_list_it = rreq_id_list_.find(node);
    if (node_list_it != rreq_id_list_.end()) {
        auto& per_node_list = node_list_it->second.rreq_id_list;
        per_node_list.erase(rreq_id);
    }
}

void AODV::aodv_process_route_timeout(const std::string& route_key) {
    routing_table_.erase(route_key);
    neighbors_.erase(route_key);
    
    log_debug("aodv_process_route_timeout: removing " + route_key + " from the routing table.");
}

void AODV::aodv_simulate_link_up(bool from_tester) {
    if (status_ == "Active") {
        std::cout << "Node is already active!" << std::endl;
        return;
    }
    
    hello_timer_ = std::make_shared<Timer>(AODV_HELLO_INTERVAL,
                                           [this]() { this->aodv_send_hello_message(); });
    hello_timer_->start();
    
    // Restart all lifetime timers
    for (auto& route_pair : routing_table_) {
        auto timeout_callback = [this, key = route_pair.first]() {
            this->aodv_process_route_timeout(key);
        };
        
        route_pair.second.lifetime = std::make_shared<Timer>(
            AODV_ACTIVE_ROUTE_TIMEOUT, timeout_callback);
        route_pair.second.lifetime->start();
    }
    
    status_ = "Active";
    log_debug("Activating node " + node_id_);
    std::cout << "Activated node " << node_id_ 
              << ". This node will resume sending hello messages." << std::endl;
}

void AODV::aodv_add_neighbor(bool from_tester) {
    std::vector<std::string> neighbors;
    
    if (!from_tester) {
        std::cout << "Enter the neighbors for the current node, separated by a space: ";
        std::string neighbors_raw;
        std::getline(std::cin, neighbors_raw);
        neighbors = split_string(neighbors_raw, ' ');
    } else {
        if (command_.size() > 2) {
            neighbors = split_string(command_[2], ' ');
        }
    }
    
    for (const std::string& n : neighbors) {
        auto timeout_callback = [this, n]() {
            this->aodv_process_neighbor_timeout(n);
        };
        
        Neighbor new_neighbor;
        new_neighbor.neighbor = n;
        new_neighbor.timer_callback = std::make_shared<Timer>(
            AODV_HELLO_TIMEOUT, timeout_callback);
        new_neighbor.timer_callback->start();
        neighbors_[n] = new_neighbor;
    }
    
    std::cout << "Neighbors added successfully: ";
    for (const auto& neighbor_pair : neighbors_) {
        std::cout << neighbor_pair.first << " ";
    }
    std::cout << std::endl;
    
    // Update routing table with direct routes
    for (const std::string& n : neighbors) {
        RouteEntry new_route;
        new_route.destination = n;
        new_route.destination_port = std::to_string(get_aodv_port(n));
        new_route.next_hop = n;
        new_route.next_hop_port = std::to_string(get_aodv_port(n));
        new_route.seq_no = "1";
        new_route.hop_count = "1";
        new_route.status = "Active";
        routing_table_[n] = new_route;
        aodv_restart_route_timer(routing_table_[n], true);
    }
    
    // Start hello timer
    hello_timer_ = std::make_shared<Timer>(AODV_HELLO_INTERVAL,
                                           [this]() { this->aodv_send_hello_message(); });
    hello_timer_->start();
}

void AODV::aodv_simulate_link_down(bool from_tester) {
    if (status_ == "Inactive") {
        std::cout << "Node is already down!" << std::endl;
        return;
    }
    
    hello_timer_->cancel();
    
    // Stop all lifetime timers
    for (auto& route_pair : routing_table_) {
        if (route_pair.second.lifetime) {
            route_pair.second.lifetime->cancel();
        }
    }
    
    status_ = "Inactive";
    log_debug("Deactivating node " + node_id_);
    std::cout << "Deactivated node " << node_id_ 
              << ". This node will stop sending hello messages." << std::endl;
}

void AODV::aodv_delete_messages(bool from_tester) {
    message_box_.clear();
    std::cout << "Message box has been cleared" << std::endl;
}

void AODV::aodv_send_message(bool from_tester) {
    if (command_.size() < 4) return;
    
    std::string source = command_[1];
    std::string dest = command_[2];
    std::string message_data = command_[3];
    
    std::string message_type = "USER_MESSAGE";
    std::string message = message_type + ":" + source + ":" + dest + ":" + message_data;
    
    auto dest_route_it = routing_table_.find(dest);
    if (dest_route_it != routing_table_.end()) {
        RouteEntry& destination = dest_route_it->second;
        
        if (destination.status == "Inactive") {
            aodv_send_rreq(dest, std::stoi(destination.seq_no));
        } else {
            std::string next_hop = destination.next_hop;
            int next_hop_port = std::stoi(destination.next_hop_port);
            aodv_send(next_hop, next_hop_port, message);
            aodv_restart_route_timer(destination, false);
            
            log_debug("['USER_MESSAGE', '" + source + " to " + dest + " via " + 
                     next_hop + "', '" + message_data + "']");
            std::cout << "Message sent" << std::endl;
        }
    } else {
        aodv_send_rreq(dest, -1);
        pending_msg_q_.push_back(message);
    }
}

void AODV::aodv_show_routing_table(bool from_tester) {
    std::cout << std::endl;
    std::cout << "There are " << routing_table_.size() 
              << " active route(s) in the routing-table" << std::endl;
    std::cout << std::endl;
    std::cout << "Destination Next-Hop Seq-No Hop-Count Status" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    
    for (const auto& route_pair : routing_table_) {
        const RouteEntry& route = route_pair.second;
        std::cout << route.destination << " " << route.next_hop << " " 
                  << route.seq_no << " " << route.hop_count << " " 
                  << route.status << std::endl;
    }
    std::cout << std::endl;
    status_ = "Success";
}

void AODV::aodv_show_log(bool from_tester) {
    std::ifstream log_stream(log_file_);
    std::string line;
    while (std::getline(log_stream, line)) {
        std::cout << line << std::endl;
    }
}

void AODV::aodv_show_messages(bool from_tester) {
    std::cout << std::endl;
    std::cout << "There are " << message_box_.size() 
              << " message(s) in the message-box" << std::endl;
    std::cout << std::endl;
    std::cout << "Sender Message" << std::endl;
    std::cout << "------------------" << std::endl;
    
    for (const auto& msg_pair : message_box_) {
        const Message& msg = msg_pair.second;
        std::cout << msg.sender << " " << msg.message << std::endl;
    }
    std::cout << std::endl;
    status_ = "Success";
}

void AODV::setup_logging() {
    log_file_ = "aodv_log_" + node_id_;
    // Log file is created when first write occurs
}

void AODV::log_debug(const std::string& message) {
    std::ofstream log_stream(log_file_, std::ios::app);
    
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    log_stream << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S") 
               << " - " << message << std::endl;
}

std::vector<std::string> AODV::split_string(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        if (!token.empty()) {
            tokens.push_back(token);
        }
    }
    
    return tokens;
}

int AODV::create_udp_socket(int port) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }
    
    int reuse = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    
    // Set non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(port);
    
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind socket to port " << port << std::endl;
        close(sock);
        return -1;
    }
    
    return sock;
}

void AODV::create_sockets() {
    listener_port_ = get_listener_port(node_id_);
    listener_thread_port_ = get_listener_thread_port(node_id_);
    aodv_port_ = get_aodv_port(node_id_);
    tester_port_ = get_tester_port(node_id_);
    
    listener_sock_ = create_udp_socket(listener_port_);
    tester_sock_ = create_udp_socket(tester_port_);
    aodv_sock_ = create_udp_socket(aodv_port_);
    
    log_debug("node " + node_id_ + " started on port " + std::to_string(aodv_port_) +
             " with pid " + std::to_string(getpid()));
}

void AODV::close_sockets() {
    if (listener_sock_ >= 0) close(listener_sock_);
    if (tester_sock_ >= 0) close(tester_sock_);
    if (aodv_sock_ >= 0) close(aodv_sock_);
}

void AODV::process_listener_command(const std::string& command_str) {
    command_ = split_string(command_str, ':');
    if (command_.empty()) return;
    
    std::string command_type = command_[0];
    
    if (command_type == "NODE_ACTIVATE") {
        aodv_simulate_link_up(false);
    } else if (command_type == "ADD_NEIGHBOR") {
        aodv_add_neighbor(false);
    } else if (command_type == "NODE_DEACTIVATE") {
        aodv_simulate_link_down(false);
    } else if (command_type == "DELETE_MESSAGES") {
        aodv_delete_messages(false);
    } else if (command_type == "SEND_MESSAGE") {
        aodv_send_message(false);
    } else if (command_type == "SHOW_ROUTE") {
        aodv_show_routing_table(false);
    } else if (command_type == "VIEW_LOG") {
        aodv_show_log(false);
    } else if (command_type == "VIEW_MESSAGES") {
        aodv_show_messages(false);
    }
    
    // Send status back
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(listener_thread_port_);
    inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr);
    
    sendto(listener_sock_, status_.c_str(), status_.length(), 0,
           (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void AODV::process_tester_command(const std::string& command_str) {
    command_ = split_string(command_str, ':');
    if (command_.empty()) return;
    
    std::string command_type = command_[0];
    
    if (command_type == "NODE_ACTIVATE") {
        aodv_simulate_link_up(true);
    } else if (command_type == "ADD_NEIGHBOR") {
        aodv_add_neighbor(true);
    } else if (command_type == "NODE_DEACTIVATE") {
        aodv_simulate_link_down(true);
    } else if (command_type == "DELETE_MESSAGES") {
        aodv_delete_messages(true);
    } else if (command_type == "SEND_MESSAGE") {
        aodv_send_message(true);
    } else if (command_type == "SHOW_ROUTE") {
        aodv_show_routing_table(true);
    } else if (command_type == "VIEW_LOG") {
        aodv_show_log(true);
    } else if (command_type == "VIEW_MESSAGES") {
        aodv_show_messages(true);
    }
    
    status_ = "Success";
    
    // Send status back to tester
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5000);
    inet_pton(AF_INET, "127.0.0.1", &dest_addr.sin_addr);
    
    sendto(tester_sock_, status_.c_str(), status_.length(), 0,
           (struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void AODV::process_aodv_message(const std::string& message_str) {
    std::vector<std::string> message = split_string(message_str, ':');
    if (message.empty()) return;
    
    std::string message_type = message[0];
    
    if (message_type == "HELLO_MESSAGE") {
        aodv_process_hello_message(message);
    } else if (message_type == "USER_MESSAGE") {
        aodv_process_user_message(message);
    } else if (message_type == "RREQ_MESSAGE") {
        aodv_process_rreq_message(message);
    } else if (message_type == "RREP_MESSAGE") {
        aodv_process_rrep_message(message);
    } else if (message_type == "RERR_MESSAGE") {
        aodv_process_rerr_message(message);
    }
}

void AODV::run() {
    setup_logging();
    create_sockets();
    
    status_ = "Active";
    running_ = true;
    
    fd_set read_fds;
    int max_fd = std::max({listener_sock_, tester_sock_, aodv_sock_}) + 1;
    
    char buffer[512];
    
    while (running_) {
        FD_ZERO(&read_fds);
        FD_SET(listener_sock_, &read_fds);
        FD_SET(tester_sock_, &read_fds);
        FD_SET(aodv_sock_, &read_fds);
        
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        
        int activity = select(max_fd, &read_fds, nullptr, nullptr, &timeout);
        
        if (activity < 0) {
            std::cerr << "Select error" << std::endl;
            break;
        }
        
        if (FD_ISSET(listener_sock_, &read_fds)) {
            memset(buffer, 0, sizeof(buffer));
            int n = recvfrom(listener_sock_, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
            if (n > 0) {
                buffer[n] = '\0';
                process_listener_command(std::string(buffer));
            }
        }
        
        if (FD_ISSET(tester_sock_, &read_fds)) {
            memset(buffer, 0, sizeof(buffer));
            int n = recvfrom(tester_sock_, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
            if (n > 0) {
                buffer[n] = '\0';
                process_tester_command(std::string(buffer));
            }
        }
        
        if (FD_ISSET(aodv_sock_, &read_fds)) {
            memset(buffer, 0, sizeof(buffer));
            int n = recvfrom(aodv_sock_, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
            if (n > 0) {
                buffer[n] = '\0';
                process_aodv_message(std::string(buffer));
            }
        }
    }
}

void AODV::stop() {
    running_ = false;
    if (hello_timer_) {
        hello_timer_->cancel();
    }
}
