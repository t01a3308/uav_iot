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
/*Added header files*/
#include "kinematic.h"
#include "handle.h"
#include "macro_param.h"
#include "communication.h"
#include "misc.h"

using namespace ns3;

AnimationInterface *anim = 0;
TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
double X[maxNumCell], Y[maxNumCell];//cell center 's position
NodeContainer uav[numCell], sensor[numCell], gw[numCell], allNodes[numCell];
std::map<Ptr<Node>, double> sensorData[numCell]; // all data
std::map<Ptr<Node>, double> highData[numCell]; // all high data
std::map<Ptr<Node>, double> taskToDo[numCell][numUav]; // all tasks of a uav
std::queue<std::map<Ptr<Node>, double>> q[numCell][numUav];// sequence to do tasks

//variable to solve TSP
int appear[maxTaskPerUav+1];
double distance[maxTaskPerUav+1][maxTaskPerUav+1];
int x[maxTaskPerUav+1];
int path[maxTaskPerUav+1];
double dmin = 999999;
double result = 0;
double MIN = 9999999;
  //
int uavNext[numCell];
 
int main()
{ 
  RngSeedManager::SetSeed (3);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (7);   // Changes run number from default of 1 to 7  
  CalculateCellCenter();
  for(int i = 0; i < numCell; i++)
  {
    uavNext[i] = 0;
    uav[i].Create(numUav);
    sensor[i].Create(numSensor);
    gw[i].Create(numGw);
    SetupSensorPosition(i);
    SetupUavPosition(i);
    SetupGwPosition(i);     
    SetupCommunication(i);
    SetupApplication(i);
    GenerateSensorData(i);
    Execute(i);
  }
  /* For upcast prototype
  Ptr<UAV> uav_obj = CreateObject<UAV>();
  NodeContainer c;
  c.Add(uav_obj);
  MobilityHelper m;
  m.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  m.Install(uav_obj);
  uav_obj->GetLocalTime();
  */
    
  anim = new AnimationInterface("city.xml");
  SetPosition(uav[0].Get(0), Vector(0.0,0.0,0.0));
  SetPosition(uav[0].Get(2), Vector(1000.0,0.0,0.0));
  Simulator::Schedule(Seconds(35), &SendPacket, uav[0].Get(0), uav[0].Get(2), 3, 256, 2);
  Simulator::Stop(Seconds(50));
  Simulator::Run();
  Simulator::Destroy();
  
}


