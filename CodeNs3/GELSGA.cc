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

/*Added header files*/
#include "kinematic_GELSGA.h"
using namespace ns3;
AnimationInterface *anim = 0;
int useAnim = 0;
UAVContainer uav[NUM_CELL];
SensorContainer sensor[NUM_CELL];
GwContainer gw[NUM_CELL];
NodeContainer allNodes[NUM_CELL];
std::string method = "GELSGA";
int main()
{ 
 // LogComponentEnable ("MobilityHelper", LOG_LEVEL_DEBUG);
  RngSeedManager::SetSeed (3);  // Changes seed from default of 1 to 3
  RngSeedManager::SetRun (7);   // Changes run number from default of 1 to 7  
  CalculateCellCenter();
  for(int i = 0; i < NUM_CELL; i++)
  {
  	for(int j = 0; j < NUM_UAV; j++)
    {
      Ptr<UAV> u = CreateObject<UAV>(i, j);
      uav[i].Add(u);
    }
    for(int j = 0; j < NUM_SENSOR; j++)
    {
      Ptr<SENSOR> ss = CreateObject<SENSOR>();
      sensor[i].Add(ss);
    }   
    for(int j = 0; j < NUM_GW; j++)
    {
      Ptr<GW> g = CreateObject<GW>();
      gw[i].Add(g);
    }
    SetupSensorPosition(i);
    SetupUavPosition(i);
    SetupGwPosition(i);     
    SetupCommunication(i);
    SetupApplication(i);
    UavSend(i);
  }
  if(useAnim == 1)
  {
    anim = new AnimationInterface("animation/GELSGA.xml");
  }
  if(TOTAL_SITE >= 500)
  {

  }
  else
  {
    if(TOTAL_SITE > 400)
    {
      NUM_ITERATOR = 6000;
    }
    else if(TOTAL_SITE > 350)
    {
      NUM_ITERATOR = 2000;
    }
    else if(TOTAL_SITE > 300)
    {
      NUM_ITERATOR = 1000;
    }
    else if(TOTAL_SITE > 250)
    {
      NUM_ITERATOR = 1000;
    }
    else
    {
      NUM_ITERATOR = 500;
    }
  }
  CalculateNumberOfSites();
  AddUtilityPlot();
  NumUAVPlot();
  NewScenario();
  Simulator::Run();
  Simulator::Destroy();
}


