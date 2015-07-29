//* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Shannon Zoch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Shannon Zoch <shannonzoch@gmail.com>
 * Department of Electrical and Computer Engineering
 * Naval Postgraduate School, Monterey, CA USA.
 *
 * Work Completed to satisfy requirements for Masters of Science in 
 * Electrical Engineering
 * USAGE:
 * CONSTRAINTS:
 * ToDo:
 *   -Random Number Variable seed and run values are not changing test results
 */

#include <sstream>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/sift-module.h"
#include "ns3/sift-helper.h"
#include "ns3/sift-main-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/propagation-module.h"
#include "ns3/aodv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/olsr-helper.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"

NS_LOG_COMPONENT_DEFINE ("DGGFCompare");

using namespace ns3;

// This code is used for simulating a network with various routing protocols  
class DGGFCompare
{
public: 
  enum Model
  {
    CONSTANTPOSITION, RANDOMDIRECTION2, RANDOMWALK2, RANDOMWAYPOINT, GAUSSMARKOV
  };
  enum Routing
  {
      DGGF, AODV, DSDV, DSR, OLSR
  };  
  DGGFCompare (); 
  bool Configure (int argc, char **argv);  // Configure script parameters
                                           // returns true on successful configuration
  void Run ();                             // Run simulation
  void Report (std::ostream & os);         // Report results
  

private:
  uint32_t nNodes;       // # of wireless nodes
  uint32_t nFlows;       // # of Sink nodes
  double totalTime;      // Total simulation time
  double dataTime;       // ns was 10000.0
  double ppers;          // Packets/Second
  uint32_t packetSize;   // Packet size
  double dataStartTime;  // time to start sending data
  double nodePauseTime;                     // Pause time between nodes movement
  double nodeMaxSpeed;      // Node movement speed
  double TxMaxRange;    // Wireless Transmission Range
  std::string rate;
  std::string dataMode;
  std::string phyMode;
  Routing RoutingProtocol;           // Routing RoutingProtocolrithm used in the simulation
  Model SelectMobilityModel;                  // Mobility model used in the simulation
  uint32_t SeedRun;
  uint32_t SeedValue;
  int initGridSpacing;              // distance between nodes in grid topology
  bool pcap;             // PCAP enable/disable
  double xmax;                 // x Length of Mobility area
  double ymax;                 // y Width of Mobility area
  double zmax;                 // z Height of Mobility Area                
  double xDelta;                    // x delta b/t two consecutive nodes
  double yDelta;                    // y delta b/t two consectutive nodes
  double zDelta;                    // z delta b/t two consectutive nodes      
                        
  NodeContainer mobileNodes;
  NetDeviceContainer allDevices;
  Ipv4InterfaceContainer allInterfaces;
  uint32_t periodicUpdateInterval;                // DSDV Parameter
  uint32_t settlingTime;                          // DSDV Parameter

  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
};

int main (int argc, char **argv)
{
  DGGFCompare test;    
  if (!test.Configure (argc, argv))
    {
      std::cout << "Configuration failed. /n";
      exit (1);
    }
  test.Run ();
  test.Report (std::cout);
  return 0;
}

DGGFCompare::DGGFCompare ()   // INITIALIZE ALL VARIABLES
{
  nNodes = 30;
  nFlows = 2;
  totalTime = 1000;
  dataTime = totalTime-totalTime*.01;
  ppers = 1;
  packetSize = 64;
  dataStartTime = 2.0;     
  nodePauseTime = 0;
  nodeMaxSpeed = 20.0;
  TxMaxRange = 250.0;
  RoutingProtocol = DSDV;	
  SelectMobilityModel = RANDOMWAYPOINT;
  xmax = 1500.0;
  ymax = 300.0;
  zmax = 1.0;
  xDelta = 200;
  yDelta = 200;
  zDelta = 200;
  initGridSpacing = xmax / xDelta;
  rate = ".512kbps";
  dataMode = "DsssRate11Mbps";
  phyMode = "DsssRate11Mbps";
  SeedValue= 12345;
  SeedRun = 54321;
  periodicUpdateInterval = 15;   // DSDV Parameter
  settlingTime = 6;              // DSDV Parameter
  pcap = false;
}

bool
DGGFCompare::Configure (int argc, char **argv)
{
  // Enable logs by default. Comment this if too noisy
  // LogComponentEnable("SiftRouting", LOG_LEVEL_ALL);
  // LogComponentEnable("SiftRouting", LOG_LEVEL_DEBUG);

  ns3::RngSeedManager::SetSeed (SeedValue);
  ns3::RngSeedManager::SetRun(SeedRun);
  ns3::SeedManager::SetSeed (SeedValue);
  ns3::SeedManager::SetRun(SeedRun);
  Ptr<UniformRandomVariable> URV = CreateObject<UniformRandomVariable> ();
  Ptr<ConstantRandomVariable> CRV = CreateObject<ConstantRandomVariable> ();
  CommandLine cmd;

  cmd.AddValue ("SeedValue", "Specify a new seed for the run.  Default:1", SeedValue);
  cmd.AddValue ("SeedRun", "Run index (for setting repeatable seeds)", SeedRun);
  cmd.AddValue ("nNodes", "Number of wifi nodes", nNodes);
  cmd.AddValue ("nFlows", "Number of SINK traffic nodes", nFlows);
  cmd.AddValue ("totalTime", "Set the total simulation run time", totalTime);
  cmd.AddValue ("packetSize", "The packet size", packetSize);
  cmd.AddValue ("nodePauseTime", "Specify node max pause time, Default:0" , nodePauseTime);
  cmd.AddValue ("nodeMaxSpeed", "Node speed in RandomWayPoint model, Default:20", nodeMaxSpeed);
  cmd.AddValue ("TxMaxRange", "Specify node's transmit range, Default:250", TxMaxRange);
  cmd.AddValue ("xmax", "Specify Simulation Area Length, Default:1500", xmax);
  cmd.AddValue ("ymax", "Specify Simulation Area Width, Default:300", ymax);
  cmd.AddValue ("zmax", "Specify Simulation Area Height, Default:1.0", zmax);
  cmd.AddValue ("xDelta", "Specify starting position x spacing, Default:200", xDelta);
  cmd.AddValue ("yDelta", "Specify starting position y spacing, Default:200", yDelta);
  cmd.AddValue ("zDelta", "Specify starting position z spacing, Default:200", zDelta);
  cmd.AddValue ("rate", "CBR traffic rate(in kbps), Default:8", rate);
  cmd.Parse (argc, argv);
  return true;
}

void
DGGFCompare::Run ()
{  
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();
  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  Simulator::Stop (Seconds (totalTime));
  AnimationInterface anim ("SiftAnim.xml");
  Simulator::Run ();
  Simulator::Destroy ();
}

void
DGGFCompare::Report (std::ostream &)
{
}

void
DGGFCompare::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)nNodes << " Nodes.\n";
  mobileNodes.Create (nNodes);
  for (uint32_t i = 0; i < nNodes; ++i)  // This will name the mobile nodes consecutive numbers
    { 
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), mobileNodes.Get (i)); 
    }

  MobilityHelper mobility;
  std::stringstream convert;
  std::string speed;
  std::string pause;
  convert << nodeMaxSpeed;
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=16|Max=" << nodeMaxSpeed << "]";
  std::stringstream ssPause;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePauseTime << "]";
  std::stringstream ssxmax;
  std::stringstream ssymax;
  std::stringstream sszmax;
  ssxmax << "ns3::UniformRandomVariable[Min=0.0|Max=" << xmax << "]";
  ssymax << "ns3::UniformRandomVariable[Min=0.0|Max=" << ymax << "]";
  sszmax << "ns3::UniformRandomVariable[Min=0.0|Max=" << zmax << "]";
  switch (SelectMobilityModel)
    {

    case GAUSSMARKOV:
          //// 3D Random Box Position Allocator Start
          mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
                "X", StringValue (ssxmax.str ()),
                "Y", StringValue (ssymax.str ()),
                "Z", StringValue (sszmax.str ()));
          //\\ 3D Random Box Position Allocator End

          ////      3D GaussMarkov Mobility Model Start        
          mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
                                       "Bounds", BoxValue (Box (0,xmax,0,ymax,0,zmax)),
                                       "Pause", StringValue (ssPause.str ()),   
                                       "MeanVelocity", StringValue (ssSpeed.str ()));
          //\\      3D GaussMarkov Mobility Model End
      break; 

    case CONSTANTPOSITION:
     mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                      "MinX", DoubleValue (0.0),
                                      "MinY", DoubleValue (0.0),
                                      "DeltaX", DoubleValue (xDelta),
                                      "DeltaY", DoubleValue (yDelta),
                                      "initGridSpacing", UintegerValue (initGridSpacing),
                                      "LayoutType", StringValue ("RowFirst"));
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      break;

    case RANDOMDIRECTION2:
      mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                      "MinX", DoubleValue (0.0),
                                      "MinY", DoubleValue (0.0),
                                      "DeltaX", DoubleValue (xDelta),
                                      "DeltaY", DoubleValue (yDelta),
                                      "initGridSpacing", UintegerValue (initGridSpacing),
                                      "LayoutType", StringValue ("RowFirst"));
      speed = "ns3::UniformRandomVariable[Min=1.0|Max=" + convert.str () + "s]";
      convert.str ("");
      convert.clear ();
      convert << nodePauseTime;
      pause = "ns3::ConstantRandomVariable[Constant=" + convert.str () + "s]";
      mobility.SetMobilityModel ("ns3::RandomDirection2MobilityModel",
                                      "Mode", StringValue ("Time"),
                                      "Pause", StringValue (ssPause.str ()),
                                      "Speed", StringValue (ssSpeed.str ()),
                                      "Bounds", RectangleValue (Rectangle (0.0, xmax, 0.0, ymax)));
      break;

    case RANDOMWALK2:
      mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                      "MinX", DoubleValue (0.0),
                                      "MinY", DoubleValue (0.0),
                                      "DeltaX", DoubleValue (xDelta),
                                      "DeltaY", DoubleValue (yDelta),
                                      "initGridSpacing", UintegerValue (initGridSpacing),
                                      "LayoutType", StringValue ("RowFirst"));
      speed = "ns3::ConstantRandomVariable[Constant=" + convert.str () + "s]";
      convert.str ("");
      convert.clear ();
      convert << nodePauseTime;
      pause = "ns3::ConstantRandomVariable[Constant=" + convert.str () + "s]";
      mobility.SetMobilityModel ("ns3::RandomDirection2MobilityModel",
                                      "Mode", StringValue ("Time"),
                                      "Time", StringValue (ssPause.str ()),
                                      "Speed", StringValue (ssSpeed.str ()),
                                      "Bounds", RectangleValue (Rectangle (0.0, xmax, 0.0, ymax)));
      break;

    case RANDOMWAYPOINT:
          //// 3D Random Position Allocation and 3D RandomWaypointMobilityModel Start
          int64_t streamIndex = 0; // used to get consistent mobility across scenarios
          ObjectFactory pos;
          pos.SetTypeId ("ns3::RandomBoxPositionAllocator");
          pos.Set ("X", StringValue (ssxmax.str ()));
          pos.Set ("Y", StringValue (ssymax.str ()));
          pos.Set ("Z", StringValue (sszmax.str ()));
          Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
          streamIndex += taPositionAlloc->AssignStreams (streamIndex);
          mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                          "Speed", StringValue (ssSpeed.str ()),
                                          "Pause", StringValue (ssPause.str ()),
                                          "PositionAllocator", PointerValue (taPositionAlloc));
          mobility.SetPositionAllocator (taPositionAlloc);
          //\\ 3D Random Position Allocation and 3D RandomWaypointMobilityModel End
      break;

    }
  mobility.Install (mobileNodes);
} //\\ DGGFCompare::CreateNodes ()


void
DGGFCompare::CreateDevices ()
{
  // NS_LOG_INFO ("setting the default phy and channel parameters");
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue (phyMode));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));

  //NS_LOG_INFO ("setting the default phy and channel parameters ");
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (TxMaxRange));
  wifiPhy.SetChannel (wifiChannel.Create ());
  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (dataMode), "ControlMode",
                                StringValue (phyMode));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  allDevices = wifi.Install (wifiPhy, wifiMac, mobileNodes);

  //NS_LOG_INFO ("Configure Tracing.");

  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream;
  switch (RoutingProtocol)
    {
    case DGGF:
      stream = ascii.CreateFileStream ("dggf.tr");
      if (pcap)
        {
          wifiPhy.EnablePcapAll (std::string ("dggfpcap"));
        }
      break;
    case AODV:
      stream = ascii.CreateFileStream ("aodv.tr");
      if (pcap)
        {
          wifiPhy.EnablePcapAll (std::string ("aodvpcap"));
        }
      break;
    case DSDV:
      stream = ascii.CreateFileStream ("dsdv.tr");
      if (pcap)
        {
          wifiPhy.EnablePcapAll (std::string ("dsdvpcap"));
        }
      break;
    case DSR:
      stream = ascii.CreateFileStream ("dsrp.tr");
      if (pcap)
        {
          wifiPhy.EnablePcapAll (std::string ("dsrpcap"));
        }
      break;
    case OLSR:
      stream = ascii.CreateFileStream ("olsr.tr");
      if (pcap)
        {
          wifiPhy.EnablePcapAll (std::string ("olsrpcap"));
        }
      break;
    }
  wifiPhy.EnableAsciiAll (stream);
}

void
DGGFCompare::InstallInternetStack ()
{
  InternetStackHelper internet;
  // DGGF --------
  SiftMainHelper siftMain;
  SiftHelper sift;
  // AODV --------
  AodvHelper aodv;
  // DSR ---------
  DsrMainHelper dsrMain;
  DsrHelper dsr;
  // DSDV --------
  DsdvHelper dsdv;
  // OLSR --------
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (list,10);

  switch (RoutingProtocol)
    {
    case DGGF:
      internet.Install (mobileNodes);
      sift.SetNodes (mobileNodes);
      siftMain.Install (sift, mobileNodes);
      break;
    case AODV:
      internet.SetRoutingHelper (aodv);
      internet.Install (mobileNodes);
      break;
    case DSDV:
      dsdv.Set ("PeriodicUpdateInterval", TimeValue (Seconds (periodicUpdateInterval)));
      dsdv.Set ("SettlingTime", TimeValue (Seconds (settlingTime)));
      internet.SetRoutingHelper (dsdv);     // has effect on the next Install ()
      internet.Install (mobileNodes);
      break;
    case DSR:
      internet.Install (mobileNodes);
      dsrMain.Install (dsr, mobileNodes);
      break;
    case OLSR:
      internet.SetRoutingHelper (list);
      internet.Install (mobileNodes);
      break;
    }
 
  //NS_LOG_INFO ("assigning ip address");
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  //Ipv4InterfaceContainer allInterfaces;
  allInterfaces = address.Assign (allDevices);
}

void
DGGFCompare::InstallApplications ()
{
  uint16_t port = 9;
  double randomStartTime = (1 / ppers) / nFlows;
  for (uint32_t i = 0; i < nFlows; ++i)
    {
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer apps_sink = sink.Install (mobileNodes.Get (i));
      apps_sink.Start (Seconds (0.0));
      apps_sink.Stop (Seconds (totalTime - 1));
      OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (allInterfaces.GetAddress (i), port)));
      onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
      onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
      onoff1.SetAttribute ("PacketSize", UintegerValue (packetSize));
      onoff1.SetAttribute ("DataRate", DataRateValue (DataRate (rate)));
      ApplicationContainer apps1 = onoff1.Install (mobileNodes.Get (i + nNodes - nFlows));
      apps1.Start (Seconds (dataStartTime + i * randomStartTime));
      apps1.Stop (Seconds (dataTime + i * randomStartTime));
    }

}
