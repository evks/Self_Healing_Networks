#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/aodv-module.h"

#include <fstream>
#include <map>
#include <cmath>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("TGNN_CLEAN_Dataset");

// ================= GLOBALS =================
std::ofstream nodeSnapshotFile;
std::ofstream packetFlowFile;

NodeContainer* globalNodes = nullptr;

// ================= NODE STATS =================
struct NodeStats {
    uint32_t txSuccess = 0;
    uint32_t txFailed = 0;
    uint32_t rxPackets = 0;
    double lastTxTime = 0.0;
    double lastRxTime = 0.0;
    double avgRssi = 0.0;
    double avgSnr = 0.0;
    uint32_t queueDrops = 0;
    uint32_t queueLen = 0;
};

std::map<uint32_t, NodeStats> nodeStats;

// ================= PACKET FLOW =================
// KEY: (packet_uid, receiver_node)
struct PacketFlowInfo {
    uint32_t src = 0;
    uint32_t dst = 0;
    double txTime = 0;
    double rxTime = 0;
    double rssi = 0;
    double snr = 0;
    uint32_t packetSize = 0;
    uint32_t queueSrc = 0;
    uint32_t queueDst = 0;
    Vector posSrc;
    Vector posDst;
    double speedSrc = 0;
    double speedDst = 0;
};

std::map<std::pair<uint32_t,uint32_t>, PacketFlowInfo> packetFlows;

// ================= UTILS =================
uint32_t GetNodeId(std::string ctx) {
    auto p1 = ctx.find("/NodeList/") + 10;
    auto p2 = ctx.find("/DeviceList/");
    return std::stoi(ctx.substr(p1, p2 - p1));
}

double SpeedFromMobility(Ptr<MobilityModel> m) {
    return m->GetSpeed();
}

double Distance(Ptr<Node> a, Ptr<Node> b) {
    return CalculateDistance(
        a->GetObject<MobilityModel>(),
        b->GetObject<MobilityModel>());
}

// ================= PHY RX =================
void PhyRxTrace(std::string ctx,
                Ptr<const Packet> pkt,
                uint16_t,
                WifiTxVector,
                MpduInfo,
                SignalNoiseDbm sn,
                uint16_t)
{
    uint32_t dst = GetNodeId(ctx);
    double now = Simulator::Now().GetSeconds();

    double rssi = sn.signal;
    double snr  = sn.signal - sn.noise;

    nodeStats[dst].rxPackets++;
    nodeStats[dst].lastRxTime = now;
    nodeStats[dst].avgRssi = rssi;
    nodeStats[dst].avgSnr = snr;

    uint32_t uid = pkt->GetUid();
    auto key = std::make_pair(uid, dst);

    if (packetFlows.count(key)) {
        auto &f = packetFlows[key];
        f.dst = dst;
        f.rxTime = now;
        f.rssi = rssi;
        f.snr = snr;
        f.queueDst = nodeStats[dst].queueLen;

        auto mob = globalNodes->Get(dst)->GetObject<MobilityModel>();
        f.posDst = mob->GetPosition();
        f.speedDst = SpeedFromMobility(mob);

        packetFlowFile
        << f.rxTime << ","
        << f.src << ","
        << f.dst << ","
        << f.rssi << ","
        << f.snr << ","
        << (f.rxTime - f.txTime) * 1000.0 << ","
        << f.packetSize << ","
        << f.queueSrc << ","
        << f.queueDst << ","
        << f.posSrc.x << "," << f.posSrc.y << "," << f.speedSrc << ","
        << f.posDst.x << "," << f.posDst.y << "," << f.speedDst
        << "\n";

        packetFlows.erase(key);
    }
}

// ================= PHY TX =================
void PhyTxBeginTrace(std::string ctx, Ptr<const Packet> pkt, double)
{
    uint32_t src = GetNodeId(ctx);
    double now = Simulator::Now().GetSeconds();

    nodeStats[src].lastTxTime = now;

    PacketFlowInfo f{};
    f.src = src;
    f.txTime = now;
    f.packetSize = pkt->GetSize();
    f.queueSrc = nodeStats[src].queueLen;

    auto mob = globalNodes->Get(src)->GetObject<MobilityModel>();
    f.posSrc = mob->GetPosition();
    f.speedSrc = SpeedFromMobility(mob);

    // broadcast → store placeholder for every receiver
    for (uint32_t i = 0; i < globalNodes->GetN(); i++) {
        if (i != src)
            packetFlows[{pkt->GetUid(), i}] = f;
    }
}

// ================= MAC =================
void MacTx(std::string ctx, Ptr<const Packet>) {
    nodeStats[GetNodeId(ctx)].txSuccess++;
}

void MacTxDrop(std::string ctx, Ptr<const Packet>) {
    auto &s = nodeStats[GetNodeId(ctx)];
    s.txFailed++;
    s.queueDrops++;
}

// ================= SNAPSHOT =================
void Snapshot(NodeContainer nodes, double interval) {
    double t = Simulator::Now().GetSeconds();
    uint32_t N = nodes.GetN();

    for (uint32_t i = 0; i < N; i++) {
        auto n = nodes.Get(i);
        auto m = n->GetObject<MobilityModel>();
        auto &s = nodeStats[i];

        double numNbr = 0;
        double avgDist = 0, minDist = 1e9, maxDist = 0;
        double avgNbrRssi = 0;

        for (uint32_t j = 0; j < N; j++) {
            if (i == j) continue;
            double d = Distance(n, nodes.Get(j));
            if (d < 120) {
                numNbr++;
                avgDist += d;
                minDist = std::min(minDist, d);
                maxDist = std::max(maxDist, d);
                avgNbrRssi += s.avgRssi;
            }
        }

        if (numNbr > 0) {
            avgDist /= numNbr;
            avgNbrRssi /= numNbr;
        } else {
            minDist = maxDist = avgDist = avgNbrRssi = 0;
        }

        double pdr = (s.txSuccess + s.txFailed) ?
            double(s.txSuccess)/(s.txSuccess+s.txFailed) : 1.0;

        int willFail = (numNbr <= 2 || pdr < 0.9) ? 1 : 0;
        std::string failType = willFail ? "DEGRADATION" : "NONE";

        nodeSnapshotFile
        << t << "," << i << ","
        << m->GetPosition().x << "," << m->GetPosition().y << ","
        << m->GetVelocity().x << "," << m->GetVelocity().y << ","
        << m->GetSpeed() << ","
        << numNbr << ","
        << avgDist << "," << minDist << "," << maxDist << ","
        << avgNbrRssi << ",0,"
        << s.txSuccess << "," << s.txFailed << "," << s.rxPackets << "," << pdr << ","
        << s.avgRssi << ",0," << s.avgSnr << ",0,"
        << s.queueDrops << "," << s.queueLen << ","
        << t - s.lastTxTime << "," << t - s.lastRxTime << ","
        << willFail << "," << failType
        << "\n";
    }

    Simulator::Schedule(Seconds(interval), &Snapshot, nodes, interval);
}

// ================= MAIN =================
int main(int argc, char* argv[]) {
    uint32_t N = 15;
    double simTime = 600;
    double snap = 5.0;

    CommandLine cmd;
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(N);
    globalNodes = &nodes;

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    YansWifiChannelHelper ch = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(ch.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devs = wifi.Install(phy, mac, nodes);

    MobilityHelper mob;
    mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(0,300,0,300)));
    mob.Install(nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    nodeSnapshotFile.open("node_snapshots.csv");
    nodeSnapshotFile <<
    "time,node_id,pos_x,pos_y,vel_x,vel_y,speed,"
    "num_neighbors,avg_neighbor_dist,min_neighbor_dist,max_neighbor_dist,"
    "avg_neighbor_rssi,std_neighbor_rssi,"
    "tx_success,tx_failed,rx_packets,pdr,"
    "avg_rssi,std_rssi,avg_snr,std_snr,"
    "queue_drops,queue_len,time_since_tx,time_since_rx,"
    "will_fail,failure_type\n";

    packetFlowFile.open("packet_flows.csv");
    packetFlowFile <<
    "timestamp,src,dst,rssi,snr,end_to_end_delay_ms,packet_size,"
    "queue_len_src,queue_len_dst,"
    "x_src,y_src,speed_src,x_dst,y_dst,speed_dst\n";

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                    MakeCallback(&PhyRxTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                    MakeCallback(&PhyTxBeginTrace));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",
                    MakeCallback(&MacTx));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",
                    MakeCallback(&MacTxDrop));

    Simulator::Schedule(Seconds(5.0), &Snapshot, nodes, snap);
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    nodeSnapshotFile.close();
    packetFlowFile.close();
}

