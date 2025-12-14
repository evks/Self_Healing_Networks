#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

#include <fstream>

using namespace ns3;

// --------------------
// GLOBAL CSV LOGGER
// --------------------
std::ofstream logFile;

// --------------------
// PHY RX TRACE (RSSI + NOISE) — STABLE
// --------------------
void
MonitorSnifferRxTrace(std::string context,
                      Ptr<const Packet> packet,
                      uint16_t channelFreqMhz,
                      WifiTxVector txVector,
                      MpduInfo mpdu,
                      SignalNoiseDbm signalNoise,
                      uint16_t staId)  // FIX 1: Added missing parameter (staId)
{
    // Extract Node ID from context
    std::size_t n1 = context.find("/NodeList/") + 10;
    std::size_t n2 = context.find("/DeviceList/");
    uint32_t nodeId = std::stoul(context.substr(n1, n2 - n1));

    double time = Simulator::Now().GetSeconds();

    logFile << time << ","
            << nodeId << ","
            << "PHY_RX" << ","
            << signalNoise.signal << ","
            << signalNoise.noise << "\n";
}

// --------------------
// MAC TX OK
// --------------------
void
MacTxOkTrace(std::string context, Ptr<const Packet> packet)
{
    std::size_t n1 = context.find("/NodeList/") + 10;
    std::size_t n2 = context.find("/DeviceList/");
    uint32_t nodeId = std::stoul(context.substr(n1, n2 - n1));

    double time = Simulator::Now().GetSeconds();

    logFile << time << ","
            << nodeId << ","
            << "MAC_TX_OK" << ","
            << "NA" << ","
            << "NA" << "\n";
}

// --------------------
// MAC TX FAILED
// --------------------
void
MacTxFailTrace(std::string context, Ptr<const Packet> packet)
{
    std::size_t n1 = context.find("/NodeList/") + 10;
    std::size_t n2 = context.find("/DeviceList/");
    uint32_t nodeId = std::stoul(context.substr(n1, n2 - n1));

    double time = Simulator::Now().GetSeconds();

    logFile << time << ","
            << nodeId << ","
            << "MAC_TX_FAIL" << ","
            << "NA" << ","
            << "NA" << "\n";
}

int
main(int argc, char *argv[])
{
    uint32_t numNodes = 10;
    double simTime = 20.0;

    // FIX 2: Enable logging if needed for debugging
    // LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    // --------------------
    // NODES
    // --------------------
    NodeContainer nodes;
    nodes.Create(numNodes);

    // --------------------
    // WIFI ADHOC
    // --------------------
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // --------------------
    // MOBILITY
    // --------------------
    MobilityHelper mobility;
    mobility.SetMobilityModel(
        "ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(0, 100, 0, 100))
    );
    mobility.Install(nodes);

    // --------------------
    // INTERNET
    // --------------------
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    ipv4.Assign(devices);

    // --------------------
    // UDP TRAFFIC
    // --------------------
    UdpEchoServerHelper server(9);
    ApplicationContainer serverApp = server.Install(nodes.Get(0));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(simTime));  // FIX 3: Added stop time

    // FIX 4: Get the actual IP address assigned to node 0
    Ptr<Ipv4> ipv4Node0 = nodes.Get(0)->GetObject<Ipv4>();
    Ipv4Address addr0 = ipv4Node0->GetAddress(1, 0).GetLocal();

    UdpEchoClientHelper client(addr0, 9);  // FIX 5: Use actual address
    client.SetAttribute("MaxPackets", UintegerValue(1000));  // FIX 6: Added MaxPackets
    client.SetAttribute("Interval", TimeValue(MilliSeconds(100)));
    client.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApp = client.Install(nodes.Get(1));
    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(simTime));  // FIX 7: Added stop time

    // --------------------
    // LOG FILE
    // --------------------
    logFile.open("tgn_v1_dataset.csv");
    if (!logFile.is_open())  // FIX 8: Check if file opened successfully
    {
        NS_FATAL_ERROR("Could not open log file!");
    }
    logFile << "time,node,event,signal_dbm,noise_dbm\n";

    // --------------------
    // TRACE CONNECTIONS
    // --------------------
    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
        MakeCallback(&MonitorSnifferRxTrace)
    );

    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx",  // FIX 9: Corrected path
        MakeCallback(&MacTxOkTrace)
    );

    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTxDrop",  // FIX 10: Corrected path
        MakeCallback(&MacTxFailTrace)
    );

    // --------------------
    // RUN
    // --------------------
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    logFile.close();
    
    std::cout << "Simulation complete. Data saved to tgn_v1_dataset.csv" << std::endl;
    
    return 0;
}
