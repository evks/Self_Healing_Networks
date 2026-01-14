#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ManetTGNN");

std::ofstream rssiFile;
std::ofstream topologyFile;
std::ofstream throughputFile;
std::ofstream failureFile;
std::ofstream edgeFile;
std::ofstream linkQualityFile;
std::ofstream nodeMetricsFile;

uint32_t packetsReceived = 0;
uint32_t bytesTotal = 0;
std::map<uint32_t, uint32_t> nodeTxPackets;
std::map<uint32_t, uint32_t> nodeRxPackets;
std::map<std::pair<uint32_t, uint32_t>, double> linkRssiSum;
std::map<std::pair<uint32_t, uint32_t>, uint32_t> linkRssiCount;

void ReceivePacket(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv()))
  {
    bytesTotal += packet->GetSize();
    packetsReceived++;
  }
}

Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket(node, tid);
  InetSocketAddress local = InetSocketAddress(addr, 9);
  sink->Bind(local);
  sink->SetRecvCallback(MakeCallback(&ReceivePacket));
  return sink;
}

void CheckThroughput()
{
  double kbps = (bytesTotal * 8.0) / 1000;
  throughputFile << Simulator::Now().GetSeconds()
                 << "," << kbps
                 << "," << packetsReceived << "\n";

  bytesTotal = 0;
  packetsReceived = 0;
  Simulator::Schedule(Seconds(1.0), &CheckThroughput);
}

void LogTopology(NodeContainer nodes)
{
  topologyFile << Simulator::Now().GetSeconds();

  for (uint32_t i = 0; i < nodes.GetN(); i++)
  {
    Ptr<MobilityModel> mob = nodes.Get(i)->GetObject<MobilityModel>();
    Vector pos = mob->GetPosition();
    topologyFile << "," << pos.x << "," << pos.y;
  }
  topologyFile << "\n";

  Simulator::Schedule(Seconds(1.0), &LogTopology, nodes);
}

void ComputeEdges(NodeContainer nodes, double range)
{
  double time = Simulator::Now().GetSeconds();

  for (uint32_t i = 0; i < nodes.GetN(); i++)
  {
    Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
    Vector posI = mobI->GetPosition();

    for (uint32_t j = i + 1; j < nodes.GetN(); j++)
    {
      Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
      Vector posJ = mobJ->GetPosition();

      double distance = std::sqrt(
        std::pow(posI.x - posJ.x, 2) +
        std::pow(posI.y - posJ.y, 2)
      );

      int connected = (distance <= range) ? 1 : 0;

      edgeFile << time << "," << i << "," << j
               << "," << distance << "," << connected << "\n";
    }
  }

  Simulator::Schedule(Seconds(1.0), &ComputeEdges, nodes, range);
}

void PhyRxOkTrace(std::string context, Ptr<const Packet> packet,
                  double snr, WifiMode mode, WifiPreamble preamble)
{
  size_t pos = context.find("/NodeList/");
  size_t endPos = context.find("/", pos + 10);
  uint32_t nodeId = std::stoi(context.substr(pos + 10, endPos - pos - 10));

  rssiFile << Simulator::Now().GetSeconds()
           << "," << nodeId
           << "," << snr << "\n";

  nodeRxPackets[nodeId]++;
}

void PhyTxBeginTrace(std::string context, Ptr<const Packet> packet, double txPowerW)
{
  size_t pos = context.find("/NodeList/");
  size_t endPos = context.find("/", pos + 10);
  uint32_t nodeId = std::stoi(context.substr(pos + 10, endPos - pos - 10));

  nodeTxPackets[nodeId]++;
}

void MonitorLinkQuality(NodeContainer nodes, double commRange)
{
  double time = Simulator::Now().GetSeconds();

  for (uint32_t i = 0; i < nodes.GetN(); i++)
  {
    Ptr<MobilityModel> mobI = nodes.Get(i)->GetObject<MobilityModel>();
    Vector posI = mobI->GetPosition();

    for (uint32_t j = i + 1; j < nodes.GetN(); j++)
    {
      Ptr<MobilityModel> mobJ = nodes.Get(j)->GetObject<MobilityModel>();
      Vector posJ = mobJ->GetPosition();

      double distance = std::sqrt(
        std::pow(posI.x - posJ.x, 2) +
        std::pow(posI.y - posJ.y, 2)
      );

      if (distance <= commRange)
      {
        std::pair<uint32_t, uint32_t> link = std::make_pair(i, j);

        double avgRssi = -100.0;
        if (linkRssiCount[link] > 0)
        {
          avgRssi = linkRssiSum[link] / linkRssiCount[link];
        }

        uint32_t txTotal = nodeTxPackets[i] + nodeTxPackets[j];
        uint32_t rxTotal = nodeRxPackets[i] + nodeRxPackets[j];
        double packetLoss = 0.0;
        if (txTotal > 0)
        {
          packetLoss = 1.0 - ((double)rxTotal / txTotal);
          if (packetLoss < 0) packetLoss = 0.0;
        }

        linkQualityFile << time << "," << i << "," << j
                       << "," << distance
                       << "," << avgRssi
                       << "," << packetLoss << "\n";
      }
    }
  }

  Simulator::Schedule(Seconds(1.0), &MonitorLinkQuality, nodes, commRange);
}

void MonitorNodeMetrics(NodeContainer nodes)
{
  double time = Simulator::Now().GetSeconds();

  for (uint32_t i = 0; i < nodes.GetN(); i++)
  {
    Ptr<Node> node = nodes.Get(i);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

    uint32_t activeInterfaces = 0;
    for (uint32_t j = 0; j < ipv4->GetNInterfaces(); j++)
    {
      if (ipv4->IsUp(j))
      {
        activeInterfaces++;
      }
    }

    nodeMetricsFile << time << "," << i
                    << "," << activeInterfaces
                    << "," << nodeTxPackets[i]
                    << "," << nodeRxPackets[i] << "\n";
  }

  Simulator::Schedule(Seconds(0.5), &MonitorNodeMetrics, nodes);
}

void InjectNodeFailure(Ptr<Node> node, uint32_t id)
{
  failureFile << Simulator::Now().GetSeconds()
              << "," << id << ",node_failure\n";

  // Safely disable the node by turning off its device instead
  Ptr<NetDevice> dev = node->GetDevice(0);
  Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(dev);
  if (wifiDev)
  {
    Ptr<WifiPhy> phy = wifiDev->GetPhy();
    phy->SetOffMode();
  }
}

int main(int argc, char *argv[])
{
  uint32_t nWifis = 30;
  uint32_t nSinks = 10;
  double totalTime = 120.0;
  double txp = 16.0;
  double nodeSpeed = 20.0;
  double nodePause = 0.0;
  std::string phyMode("DsssRate11Mbps");
  std::string dataRate("2048bps");

  CommandLine cmd;
  cmd.AddValue("nWifis", "Number of nodes", nWifis);
  cmd.AddValue("nSinks", "Number of sink nodes", nSinks);
  cmd.AddValue("totalTime", "Simulation time", totalTime);
  cmd.AddValue("nodeSpeed", "Node speed in m/s", nodeSpeed);
  cmd.AddValue("txp", "Tx power in dBm", txp);
  cmd.Parse(argc, argv);

  rssiFile.open("rssi_log.csv");
  topologyFile.open("topology_log.csv");
  throughputFile.open("throughput_log.csv");
  failureFile.open("failure_log.csv");
  edgeFile.open("edge_log.csv");
  linkQualityFile.open("link_quality_log.csv");
  nodeMetricsFile.open("node_metrics_log.csv");

  rssiFile << "time,node_id,snr\n";
  throughputFile << "time,throughput_kbps,packets_received\n";
  failureFile << "time,node_id,failure_type\n";
  edgeFile << "time,node_i,node_j,distance,connected\n";
  linkQualityFile << "time,node_i,node_j,distance,avg_rssi,packet_loss\n";
  nodeMetricsFile << "time,node_id,active_interfaces,tx_packets,rx_packets\n";

  topologyFile << "time";
  for (uint32_t i = 0; i < nWifis; i++)
  {
    topologyFile << ",x" << i << ",y" << i;
  }
  topologyFile << "\n";

  Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
  Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(dataRate));
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                     StringValue(phyMode));

  NodeContainer adhocNodes;
  adhocNodes.Create(nWifis);

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel(wifiChannel.Create());

  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue(phyMode),
                                "ControlMode", StringValue(phyMode));
  wifiMac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

  MobilityHelper mobilityAdhoc;
  int64_t streamIndex = 0;

  ObjectFactory pos;
  pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
  pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
  pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));

  Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
  streamIndex += taPositionAlloc->AssignStreams(streamIndex);

  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

  mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue(ssSpeed.str()),
                                  "Pause", StringValue(ssPause.str()),
                                  "PositionAllocator", PointerValue(taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator(taPositionAlloc);
  mobilityAdhoc.Install(adhocNodes);
  streamIndex += mobilityAdhoc.AssignStreams(adhocNodes, streamIndex);

  AodvHelper aodv;
  InternetStackHelper internet;
  internet.SetRoutingHelper(aodv);
  internet.Install(adhocNodes);

  Ipv4AddressHelper addressAdhoc;
  addressAdhoc.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces;
  adhocInterfaces = addressAdhoc.Assign(adhocDevices);

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/RxOk",
                  MakeCallback(&PhyRxOkTrace));

  Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                  MakeCallback(&PhyTxBeginTrace));

  OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
  onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

  for (uint32_t i = 0; i < nSinks; i++)
  {
    Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(i), adhocNodes.Get(i));

    AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(i), 9));
    onoff1.SetAttribute("Remote", remoteAddress);
    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
    ApplicationContainer temp = onoff1.Install(adhocNodes.Get(i + nSinks));
    temp.Start(Seconds(var->GetValue(0.0, 1.0)));
    temp.Stop(Seconds(totalTime));
  }

  Simulator::Schedule(Seconds(1.0), &CheckThroughput);
  Simulator::Schedule(Seconds(1.0), &LogTopology, adhocNodes);
  Simulator::Schedule(Seconds(1.0), &ComputeEdges, adhocNodes, 100.0);
  Simulator::Schedule(Seconds(2.0), &MonitorLinkQuality, adhocNodes, 100.0);
  Simulator::Schedule(Seconds(1.0), &MonitorNodeMetrics, adhocNodes);

  Simulator::Schedule(Seconds(30.0), &InjectNodeFailure, adhocNodes.Get(5), 5);
  Simulator::Schedule(Seconds(60.0), &InjectNodeFailure, adhocNodes.Get(12), 12);
  Simulator::Schedule(Seconds(90.0), &InjectNodeFailure, adhocNodes.Get(18), 18);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Stop(Seconds(totalTime));
  Simulator::Run();

  monitor->CheckForLostPackets();

  Simulator::Destroy();

  rssiFile.close();
  topologyFile.close();
  throughputFile.close();
  failureFile.close();
  edgeFile.close();
  linkQualityFile.close();
  nodeMetricsFile.close();

  return 0;
}
