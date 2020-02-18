/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 *
 * Authors: Tien Pham Van <tien.phamvan1@hust.edu.vn>
 *          Nguyen Pham Van <nguyen.pv152737@sis.hust.edu.vn>
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/aodv-module.h"
#include "ns3/netanim-module.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/yans-wifi-helper.h"
#include <iostream>
#include <math.h>
#include "macro_param.h"
#include "handle.h"
using namespace ns3;

extern TypeId tid;
extern double dataLoad;
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
extern int uavState[NUM_CELL][NUM_UAV];
void SetupCommunication(int cellId);
void SetupApplication(int cellId);
void ReceivePacket (Ptr<Socket> socket);
void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval );
void SendPacket(Ptr<Node> src, Ptr<Node> dst, int numPkt, int pktSize, double interval);
void UavSend(int cellId);
void SensorSend(int cellId);
void SetupCommunication(int cellId)
{
  std::cout<<"set up communication cell "<<cellId<<std::endl;
  std::string phyMode ("OfdmRate6MbpsBW10MHz"); 
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  wifiPhy.Set ("RxGain", DoubleValue (0) );
  wifiPhy.Set ("TxPowerStart", DoubleValue (30));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (30));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generate a pcap trace
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode));
  allNodes[cellId] = NodeContainer(uav[cellId], sensor[cellId], gw[cellId]);  
  NetDeviceContainer wifiDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, allNodes[cellId]);
  AodvHelper aodv;
  aodv.Set("HelloInterval", TimeValue(Seconds(5)));
  Ipv4StaticRoutingHelper staticRouting;
  Ipv4ListRoutingHelper listRouting;
  listRouting.Add (staticRouting, 0);
  listRouting.Add (aodv, 10);
  InternetStackHelper internet;
  //internet.SetRoutingHelper(listRouting);
  internet.Install (allNodes[cellId]);
  Ipv4AddressHelper ipv4Address;
  std::string str = "10."+std::to_string(cellId+1)+".0.0";
  ipv4Address.SetBase (str.c_str(), "255.255.0.0");
  Ipv4InterfaceContainer wifiInterfaces = ipv4Address.Assign (wifiDevices);
}
void SetupApplication(int cellId)
{
  std::cout<<"set up apps cell "<<cellId<<std::endl;
  Ptr<Socket> *sinkUav, *sinkGw, *sinkSensor;
  sinkUav = new Ptr<Socket>[NUM_UAV];
  sinkSensor = new Ptr<Socket>[NUM_SENSOR];
  sinkGw = new Ptr<Socket> [NUM_GW ];
  for(int i = 0; i < NUM_UAV; i++)
  {
    sinkUav[i] = Socket::CreateSocket (uav[cellId].Get (i), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    sinkUav[i] ->Bind (local);
    sinkUav[i] ->SetRecvCallback (MakeCallback (&ReceivePacket));
  }
  for(int i = 0; i < NUM_SENSOR; i++)
  {
    sinkSensor[i] = Socket::CreateSocket (sensor[cellId].Get (i), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    sinkSensor[i] ->Bind (local);
    sinkSensor[i] ->SetRecvCallback (MakeCallback (&ReceivePacket));
  }
  for(int i = 0; i < NUM_GW ; i++)
  {
    sinkGw[i] = Socket::CreateSocket (gw[cellId].Get (i), tid);
    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
    sinkGw[i] ->Bind (local);
    sinkGw[i] ->SetRecvCallback (MakeCallback (&ReceivePacket));
  }
}

void ReceivePacket (Ptr<Socket> socket)
{
  while (socket->Recv ())
  {
    //NS_LOG_UNCOND ("Received one packet!");
  }
}
void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
  {
   // socket->Send (Create<Packet> (pktSize));
    Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval);
  }
  else
  {
    socket->Close ();
  }
}
void SendPacket(Ptr<Node> src, Ptr<Node> dst, int numPkt, int pktSize, double interval)
{
  //std::cout<<"send packet"<<std::endl;
  dataLoad += (numPkt*pktSize);
  Ipv4Address dstAddr = dst->GetObject<Ipv4L3Protocol> ()->GetInterface (1)->GetAddress (0).GetLocal ();
  Ptr<Socket> source = Socket::CreateSocket (src, tid);
  InetSocketAddress remote = InetSocketAddress (dstAddr, 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);
  GenerateTraffic(source, pktSize, numPkt, Seconds(interval));
} 
void UavSend(int cellId)
{
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(uavState[cellId][i])
    {
      Ptr<UAV> u = uav[cellId].GetUav(i);
      u -> IncreaseEnergy(ENERGY_SEND*UAV_NUM_PACKET);
      Simulator::Schedule(Seconds(i*3),&SendPacket, uav[cellId].Get(i), gw[cellId].Get(0), UAV_NUM_PACKET, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
    }
  }
  Simulator::Schedule(Seconds(60*UAV_INTERVAL), &UavSend, cellId);
}
void SensorSend(int cellId)
{
  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
  for(int i = 0; i < NUM_SENSOR; i++)
  {
    Simulator::Schedule(Seconds(rd->GetValue(0, 60)), &SendPacket, sensor[cellId].Get(i), gw[cellId].Get(0), SENSOR_NUM_PACKET, SENSOR_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
  }
  Simulator::Schedule(Seconds(60*SENSOR_INTERVAL), &SensorSend, cellId);
}