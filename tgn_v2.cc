#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <map>
#include <vector>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TGNNDatasetGenerator");

// ====================
// GLOBAL DATA STRUCTURES
// ====================
std::ofstream phyLog, macLog, mobilityLog, networkLog, aggregateLog;

// Per-node statistics tracking
struct NodeStats {
    uint32_t txSuccess = 0;
    uint32_t txFailed = 0;
    uint32_t rxPackets = 0;
    double lastTxTime = 0.0;
    double lastRxTime = 0.0;
    std::vector<double> rssiHistory;
    std::vector<double> snrHistory;
    uint32_t retryCount = 0;
    uint32_t queueDrops = 0;
};

std::map<uint32_t, NodeStats> nodeStatistics;

// Neighbor tracking
struct NeighborInfo {
    double distance;
    double lastRssi;
    double lastSnr;
    uint32_t packetsReceived;
    double lastContactTime;
};

std::map<uint32_t, std::map<uint32_t, NeighborInfo>> neighborMap;

// ====================
// UTILITY FUNCTIONS
// ====================
uint32_t ExtractNodeId(std::string context) {
    std::size_t n1 = context.find("/NodeList/") + 10;
    std::size_t n2 = context.find("/DeviceList/");
    return std::stoul(context.substr(n1, n2 - n1));
}

double CalculateDistance(Ptr<Node> node1, Ptr<Node> node2) {
    Ptr<MobilityModel> mob1 = node1->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob2 = node2->GetObject<MobilityModel>();
    
    Vector pos1 = mob1->GetPosition();
    Vector pos2 = mob2->GetPosition();
    
    double dx = pos1.x - pos2.x;
    double dy = pos1.y - pos2.y;
    return std::sqrt(dx*dx + dy*dy);
}

Vector GetVelocity(Ptr<Node> node) {
    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    return mob->GetVelocity();
}

// ====================
// PHY LAYER TRACES
// ====================
void PhyRxTrace(std::string context,
                Ptr<const Packet> packet,
                uint16_t channelFreqMhz,
                WifiTxVector txVector,
                MpduInfo mpdu,
                SignalNoiseDbm signalNoise,
                uint16_t staId)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    double rssi = signalNoise.signal;
    double noise = signalNoise.noise;
    double snr = rssi - noise;
    
    // Update node statistics
    nodeStatistics[nodeId].rxPackets++;
    nodeStatistics[nodeId].lastRxTime = time;
    nodeStatistics[nodeId].rssiHistory.push_back(rssi);
    nodeStatistics[nodeId].snrHistory.push_back(snr);
    
    // Keep only last 100 samples
    if (nodeStatistics[nodeId].rssiHistory.size() > 100) {
        nodeStatistics[nodeId].rssiHistory.erase(nodeStatistics[nodeId].rssiHistory.begin());
        nodeStatistics[nodeId].snrHistory.erase(nodeStatistics[nodeId].snrHistory.begin());
    }
    
    // Log PHY metrics
    phyLog << time << ","
           << nodeId << ","
           << packet->GetSize() << ","
           << rssi << ","
           << noise << ","
           << snr << ","
           << static_cast<uint32_t>(txVector.GetMode().GetDataRate(txVector.GetChannelWidth())) << ","
           << static_cast<uint32_t>(txVector.GetChannelWidth()) << "\n";
}

void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPowerW)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    nodeStatistics[nodeId].lastTxTime = time;
    
    // Could log TX power if needed
    // double txPowerDbm = 10.0 * std::log10(txPowerW * 1000.0);
}

void PhyStateTrace(std::string context, Time start, Time duration, WifiPhyState state)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    std::string stateStr;
    switch(state) {
        case WifiPhyState::IDLE: stateStr = "IDLE"; break;
        case WifiPhyState::CCA_BUSY: stateStr = "CCA_BUSY"; break;
        case WifiPhyState::TX: stateStr = "TX"; break;
        case WifiPhyState::RX: stateStr = "RX"; break;
        case WifiPhyState::SWITCHING: stateStr = "SWITCHING"; break;
        case WifiPhyState::SLEEP: stateStr = "SLEEP"; break;
        default: stateStr = "UNKNOWN";
    }
    
    phyLog << time << ","
           << nodeId << ","
           << "PHY_STATE" << ","
           << stateStr << ","
           << duration.GetSeconds() << "\n";
}

// ====================
// MAC LAYER TRACES
// ====================
void MacTxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    nodeStatistics[nodeId].txSuccess++;
    
    macLog << time << ","
           << nodeId << ","
           << "TX" << ","
           << packet->GetSize() << ","
           << nodeStatistics[nodeId].retryCount << "\n";
    
    // Reset retry count after successful transmission
    nodeStatistics[nodeId].retryCount = 0;
}

void MacTxDropTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    nodeStatistics[nodeId].txFailed++;
    nodeStatistics[nodeId].queueDrops++;
    
    macLog << time << ","
           << nodeId << ","
           << "TX_DROP" << ","
           << packet->GetSize() << ","
           << "0" << "\n";
}

void MacRxTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    macLog << time << ","
           << nodeId << ","
           << "RX" << ","
           << packet->GetSize() << ","
           << "0" << "\n";
}

// ====================
// QUEUE TRACES
// ====================
void QueueDropTrace(std::string context, Ptr<const Packet> packet)
{
    uint32_t nodeId = ExtractNodeId(context);
    double time = Simulator::Now().GetSeconds();
    
    nodeStatistics[nodeId].queueDrops++;
    
    macLog << time << ","
           << nodeId << ","
           << "QUEUE_DROP" << ","
           << packet->GetSize() << ","
           << nodeStatistics[nodeId].queueDrops << "\n";
}

// ====================
// PERIODIC NETWORK SNAPSHOT
// ====================
void CaptureNetworkSnapshot(NodeContainer nodes)
{
    double time = Simulator::Now().GetSeconds();
    uint32_t numNodes = nodes.GetN();
    
    // For each node, capture comprehensive state
    for (uint32_t i = 0; i < numNodes; i++) {
        Ptr<Node> node = nodes.Get(i);
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
        Vector pos = mob->GetPosition();
        Vector vel = mob->GetVelocity();
        
        double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
        
        NodeStats& stats = nodeStatistics[i];
        
        // Calculate rolling statistics
        double avgRssi = 0.0, stdRssi = 0.0;
        double avgSnr = 0.0, stdSnr = 0.0;
        
        if (!stats.rssiHistory.empty()) {
            for (double val : stats.rssiHistory) avgRssi += val;
            avgRssi /= stats.rssiHistory.size();
            
            for (double val : stats.rssiHistory) 
                stdRssi += (val - avgRssi) * (val - avgRssi);
            stdRssi = std::sqrt(stdRssi / stats.rssiHistory.size());
        }
        
        if (!stats.snrHistory.empty()) {
            for (double val : stats.snrHistory) avgSnr += val;
            avgSnr /= stats.snrHistory.size();
            
            for (double val : stats.snrHistory) 
                stdSnr += (val - avgSnr) * (val - avgSnr);
            stdSnr = std::sqrt(stdSnr / stats.snrHistory.size());
        }
        
        // Count neighbors within communication range (assume 50m for 802.11g)
        uint32_t numNeighbors = 0;
        double avgNeighborDistance = 0.0;
        std::vector<uint32_t> neighborIds;
        
        for (uint32_t j = 0; j < numNodes; j++) {
            if (i == j) continue;
            
            double dist = CalculateDistance(node, nodes.Get(j));
            if (dist < 50.0) {  // Communication range threshold
                numNeighbors++;
                avgNeighborDistance += dist;
                neighborIds.push_back(j);
                
                // Update neighbor map
                neighborMap[i][j].distance = dist;
                neighborMap[i][j].lastContactTime = time;
            }
        }
        
        if (numNeighbors > 0) {
            avgNeighborDistance /= numNeighbors;
        }
        
        // Calculate packet delivery ratio
        double pdr = 0.0;
        if (stats.txSuccess + stats.txFailed > 0) {
            pdr = static_cast<double>(stats.txSuccess) / (stats.txSuccess + stats.txFailed);
        }
        
        // Time since last transmission/reception
        double timeSinceLastTx = time - stats.lastTxTime;
        double timeSinceLastRx = time - stats.lastRxTime;
        
        // Write aggregate snapshot
        aggregateLog << time << ","
                    << i << ","
                    << pos.x << ","
                    << pos.y << ","
                    << vel.x << ","
                    << vel.y << ","
                    << speed << ","
                    << numNeighbors << ","
                    << avgNeighborDistance << ","
                    << stats.txSuccess << ","
                    << stats.txFailed << ","
                    << stats.rxPackets << ","
                    << pdr << ","
                    << avgRssi << ","
                    << stdRssi << ","
                    << avgSnr << ","
                    << stdSnr << ","
                    << stats.queueDrops << ","
                    << timeSinceLastTx << ","
                    << timeSinceLastRx << "\n";
        
        // Write mobility log
        mobilityLog << time << ","
                   << i << ","
                   << pos.x << ","
                   << pos.y << ","
                   << vel.x << ","
                   << vel.y << ","
                   << speed << "\n";
    }
    
    // Write neighbor relationships (edges for graph)
    for (uint32_t i = 0; i < numNodes; i++) {
        for (auto& pair : neighborMap[i]) {
            uint32_t j = pair.first;
            NeighborInfo& info = pair.second;
            
            if (i < j) {  // Write each edge only once
                networkLog << time << ","
                          << i << ","
                          << j << ","
                          << info.distance << ","
                          << info.lastRssi << ","
                          << info.lastSnr << "\n";
            }
        }
    }
    
    // Schedule next snapshot
    Simulator::Schedule(MilliSeconds(500), &CaptureNetworkSnapshot, nodes);
}

// ====================
// FAILURE LABELING
// ====================
void GenerateFailureLabels(NodeContainer nodes, double currentTime, double predictionWindow)
{
    // This function checks if a node will experience failure in the next predictionWindow seconds
    // For training, you'll need to run this at post-processing
    // Here we just mark potential failure conditions
    
    static std::ofstream labelLog;
    static bool firstCall = true;
    
    if (firstCall) {
        labelLog.open("failure_labels.csv");
        labelLog << "time,node_id,will_fail,failure_type,time_to_failure\n";
        firstCall = false;
    }
    
    for (uint32_t i = 0; i < nodes.GetN(); i++) {
        NodeStats& stats = nodeStatistics[i];
        
        // Define failure conditions
        bool willFail = false;
        std::string failureType = "NONE";
        
        // Condition 1: No neighbors (isolation)
        uint32_t numNeighbors = 0;
        for (uint32_t j = 0; j < nodes.GetN(); j++) {
            if (i != j && CalculateDistance(nodes.Get(i), nodes.Get(j)) < 50.0) {
                numNeighbors++;
            }
        }
        
        if (numNeighbors == 0) {
            willFail = true;
            failureType = "ISOLATION";
        }
        
        // Condition 2: High packet loss
        double pdr = 0.0;
        if (stats.txSuccess + stats.txFailed > 10) {
            pdr = static_cast<double>(stats.txSuccess) / (stats.txSuccess + stats.txFailed);
            if (pdr < 0.5) {
                willFail = true;
                failureType = "HIGH_LOSS";
            }
        }
        
        // Condition 3: Queue drops indicating congestion
        if (stats.queueDrops > 10) {
            willFail = true;
            failureType = "CONGESTION";
        }
        
        labelLog << currentTime << ","
                << i << ","
                << (willFail ? 1 : 0) << ","
                << failureType << ","
                << "0.0" << "\n";  // Time to failure would need prediction
    }
    
    // Schedule next labeling check
    if (currentTime < 19.0) {
        Simulator::Schedule(Seconds(1.0), &GenerateFailureLabels, nodes, currentTime + 1.0, predictionWindow);
    }
}

// ====================
// MAIN SIMULATION
// ====================
int main(int argc, char *argv[])
{
    uint32_t numNodes = 10;
    double simTime = 20.0;
    
    CommandLine cmd;
    cmd.AddValue("numNodes", "Number of nodes", numNodes);
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.Parse(argc, argv);
    
    // Enable packet metadata for detailed tracking
    Packet::EnablePrinting();
    
    // ====================
    // CREATE NODES
    // ====================
    NodeContainer nodes;
    nodes.Create(numNodes);
    
    // ====================
    // WIFI SETUP
    // ====================
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::AarfWifiManager");  // Adaptive rate control
    
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(20.0));  // dBm
    phy.Set("TxPowerEnd", DoubleValue(20.0));
    
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);
    
    // ====================
    // MOBILITY
    // ====================
    MobilityHelper mobility;
    mobility.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(0, 100, 0, 100)),
        "Speed", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=5.0]"),
        "Distance", DoubleValue(10.0)
    );
    mobility.Install(nodes);
    
    // ====================
    // INTERNET STACK
    // ====================
    InternetStackHelper internet;
    internet.Install(nodes);
    
    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);
    
    // ====================
    // APPLICATIONS - Multiple traffic patterns
    // ====================
    
    // Server on node 0
    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(nodes.Get(0));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(simTime));
    
    // Multiple clients for varied traffic
    for (uint32_t i = 1; i < std::min(numNodes, static_cast<uint32_t>(5)); i++) {
        UdpEchoClientHelper client(interfaces.GetAddress(0), 9);
        client.SetAttribute("MaxPackets", UintegerValue(10000));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(50 + i * 20)));
        client.SetAttribute("PacketSize", UintegerValue(512));
        
        ApplicationContainer clientApp = client.Install(nodes.Get(i));
        clientApp.Start(Seconds(2.0 + i * 0.5));
        clientApp.Stop(Seconds(simTime));
    }
    
    // ====================
    // OPEN LOG FILES
    // ====================
    phyLog.open("phy_metrics.csv");
    phyLog << "time,node_id,packet_size,rssi_dbm,noise_dbm,snr_db,data_rate_bps,channel_width_mhz\n";
    
    macLog.open("mac_metrics.csv");
    macLog << "time,node_id,event,packet_size,retry_count\n";
    
    mobilityLog.open("mobility_metrics.csv");
    mobilityLog << "time,node_id,pos_x,pos_y,vel_x,vel_y,speed\n";
    
    networkLog.open("network_topology.csv");
    networkLog << "time,node_id,neighbor_id,distance,rssi,snr\n";
    
    aggregateLog.open("aggregate_metrics.csv");
    aggregateLog << "time,node_id,pos_x,pos_y,vel_x,vel_y,speed,num_neighbors,avg_neighbor_dist,"
                << "tx_success,tx_failed,rx_packets,pdr,avg_rssi,std_rssi,avg_snr,std_snr,"
                << "queue_drops,time_since_tx,time_since_rx\n";
    
    // ====================
    // TRACE CONNECTIONS
    // ====================
    
    // PHY Layer
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                   MakeCallback(&PhyRxTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                   MakeCallback(&PhyTxBeginTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State",
                   MakeCallback(&PhyStateTrace));
    
    // MAC Layer
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                   MakeCallback(&MacTxTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                   MakeCallback(&MacTxDropTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback(&MacRxTrace));
    
    // Queue traces (may not be available in all ns-3 versions - disabled)
    // Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/Queue/Drop",
    //                MakeCallback(&QueueDropTrace));
    
    // ====================
    // PERIODIC SNAPSHOTS
    // ====================
    Simulator::Schedule(Seconds(1.0), &CaptureNetworkSnapshot, nodes);
    Simulator::Schedule(Seconds(1.0), &GenerateFailureLabels, nodes, 1.0, 5.0);
    
    // ====================
    // RUN SIMULATION
    // ====================
    std::cout << "Starting TGNN dataset generation..." << std::endl;
    std::cout << "Nodes: " << numNodes << std::endl;
    std::cout << "Simulation time: " << simTime << " seconds" << std::endl;
    
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    
    // ====================
    // CLOSE FILES
    // ====================
    phyLog.close();
    macLog.close();
    mobilityLog.close();
    networkLog.close();
    aggregateLog.close();
    
    std::cout << "\n=== Simulation Complete ===" << std::endl;
    std::cout << "Generated files:" << std::endl;
    std::cout << "  - phy_metrics.csv (PHY layer events)" << std::endl;
    std::cout << "  - mac_metrics.csv (MAC layer events)" << std::endl;
    std::cout << "  - mobility_metrics.csv (Node positions and velocities)" << std::endl;
    std::cout << "  - network_topology.csv (Neighbor relationships)" << std::endl;
    std::cout << "  - aggregate_metrics.csv (Comprehensive per-node snapshots)" << std::endl;
    std::cout << "  - failure_labels.csv (Failure predictions)" << std::endl;
    
    return 0;
}
