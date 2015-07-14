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

using namespace ns3;

// This code is used for simulating a network with various routing protocols  
class DGGFtest
{
public:
/*  
  enum Model
  {
    CONSTANTPOSITION, RANDOMDIRECTION2, RANDOMWALK2, RANDOMWAYPOINT, GAUSSMARKOV
  };
  enum Routing
  {
      SIFT, AODV, DSDV, DSR, OLSR
  };  
*/
  DGGFtest (); 
  bool Configure (int argc, char **argv);  // Configure script parameters
                                           // returns true on successful configuration
  void Run ();                             // Run simulation
  void Report (std::ostream & os);         // Report results

private:
  uint32_t nNodes;       // # of wireless nodes
  uint32_t nSinks;       // # of Sink nodes
  double totalTime;      // Total simulation time
  double dataTime;       // ns was 10000.0
  double ppers;          // Packets/Second
  uint32_t packetSize;   // Packet size
  double dataStartTime;  // time to start sending data
  double nodeSpeed;      // Node movement speed
  double txpDistance;    // Wireless Transmission Range
  std::string rate;
  std::string dataMode;
  std::string phyMode;
  int run;
  int initGridSpacing;              // distance between nodes in grid topology
  bool pcap;             // PCAP enable/disable
  NodeContainer mobileNodes;
//  NodeContainer sourceNodes;
//  NodeContainer sinkNodes;
  NetDeviceContainer allDevices;
  Ipv4InterfaceContainer allInterfaces;

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
};

int main (int argc, char **argv)
{
  DGGFtest test;    
  if (!test.Configure (argc, argv))
    {
      std::cout << "Configuration failed. /n";
      exit (1);
    }
  test.Run ();
  test.Report (std::cout);
  return 0;
}

DGGFtest::DGGFtest ()   // INITIALIZE ALL VARIABLES
 :  nNodes (100),
    nSinks (1),
    totalTime (1000.0),
    dataTime (100000.0),
    ppers (1),
    packetSize (64),
    dataStartTime (2.0),
    nodeSpeed (25.0),
    txpDistance (250.0),
    rate (".512kbps"),
    dataMode ("DsssRate11Mbps"),
    phyMode ("DsssRate11Mbps"),
    run (1),
    initGridSpacing (180),
    pcap (false)
{
}

bool
DGGFtest::Configure (int argc, char **argv)
{
  // Enable Sift logs by default. Comment this if too noisy
  LogComponentEnable("SiftRouting", LOG_LEVEL_ALL);

  SeedManager::SetSeed (12345);
  CommandLine cmd;
  cmd.AddValue ("run", "Run index (for setting repeatable seeds)", run);
  cmd.AddValue ("nNodes", "Number of wifi nodes", nNodes);
  cmd.AddValue ("nSinks", "Number of SINK traffic nodes", nSinks);
  cmd.AddValue ("rate", "CBR traffic rate(in kbps), Default:8", rate);
  cmd.AddValue ("nodeSpeed", "Node speed in RandomWayPoint model, Default:20", nodeSpeed);
  cmd.AddValue ("packetSize", "The packet size", packetSize);
  cmd.AddValue ("txpDistance", "Specify node's transmit range, Default:250", txpDistance);
  cmd.Parse (argc, argv);
  return true;
}

void
DGGFtest::Run ()
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
DGGFtest::Report (std::ostream &)
{
}

void
DGGFtest::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)nNodes << " mobileNodes " << initGridSpacing << " m apart.\n";
//  sinkNodes.Create (1);
  mobileNodes.Create (nNodes);
//  sourceNodes.Create (1);
 
//  uint32_t lNode=nNodes-1;
//  std::cout << "lNode is:" << (unsigned)lNode << " \n";
  for (uint32_t i = 0; i < nNodes; ++i)  // This will name the mobile nodes consecutive numbers
    { 
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), mobileNodes.Get (i)); 
    }
  // Create starting grid
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
                               "Bounds", BoxValue (Box (0,1000,0,1000,0,200)), 
                               "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=16|Max=26]"));   //Speed range in m/s
  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
        "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
        "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=1000]"),
        "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=200]"));
  mobility.Install (mobileNodes);
}

void
DGGFtest::CreateDevices ()
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
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue (txpDistance));
  wifiPhy.SetChannel (wifiChannel.Create ());
  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (dataMode), "ControlMode",
                                StringValue (phyMode));

  wifiMac.SetType ("ns3::AdhocWifiMac");
  allDevices = wifi.Install (wifiPhy, wifiMac, mobileNodes);
//  allDevices = wifi.Install (wifiPhy, wifiMac, sourceNodes);
//  allDevices = wifi.Install (wifiPhy, wifiMac, sinkNodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("Siftpcap"));
    }

  //NS_LOG_INFO ("Configure Tracing.");

  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("siftTrace.tr");
  wifiPhy.EnableAsciiAll (stream);
}

void
DGGFtest::InstallInternetStack ()
{
  InternetStackHelper internet;
  // SIFT --------
  SiftMainHelper siftMain;
  SiftHelper sift;
/*
  internet.Install (sinkNodes);
  sift.SetNodes (sinkNodes);
  siftMain.Install (sift, sinkNodes);
*/
  internet.Install (mobileNodes);
  sift.SetNodes (mobileNodes);
  siftMain.Install (sift, mobileNodes);
/*
  internet.Install (sourceNodes);
  sift.SetNodes (sourceNodes);
  siftMain.Install (sift, sourceNodes);
*/
  //NS_LOG_INFO ("assigning ip address");
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  //Ipv4InterfaceContainer allInterfaces;
  allInterfaces = address.Assign (allDevices);
}

void
DGGFtest::InstallApplications ()
{
  uint16_t port = 9;
 
      
/*
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer apps_sink = sink.Install (sinkNodes.Get (0));
      apps_sink.Start (Seconds (0.0));
      apps_sink.Stop (Seconds (totalTime - 1));
      OnOffHelper onoff1 ("ns3::UdpSocketFactory", Address (InetSocketAddress (allInterfaces.GetAddress (0), port)));
      onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
      onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
      onoff1.SetAttribute ("PacketSize", UintegerValue (packetSize));
      onoff1.SetAttribute ("DataRate", DataRateValue (DataRate (rate)));
      ApplicationContainer apps1 = onoff1.Install (sourceNodes.Get (nNodes - nSinks));
      apps1.Start (Seconds (dataStartTime));
      apps1.Stop (Seconds (dataTime));
*/

  double randomStartTime = (1 / ppers) / nSinks;
  for (uint32_t i = 0; i < nSinks; ++i)
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
      ApplicationContainer apps1 = onoff1.Install (mobileNodes.Get (i + nNodes - nSinks));
      apps1.Start (Seconds (dataStartTime + i * randomStartTime));
      apps1.Stop (Seconds (dataTime + i * randomStartTime));
    }

}
