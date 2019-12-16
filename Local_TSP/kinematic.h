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
#include <queue>
#include "macro_param.h"
//#include "handle.h"
#include "communication.h"
using namespace ns3;
using namespace std;
extern double dataLoad;
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
extern std::map<Ptr<Node>, double> sensorData[NUM_CELL]; // all data
//
int siteData[MAX_SITE_PER_CELL*NUM_CELL];
int numberOfSites[NUM_CELL-1];
//variables for TSP
double dist[MAX_SITE_PER_CELL+1][MAX_SITE_PER_CELL+1];
std::vector <int> path;
int VISITED_ALL; // 
double dp[MAX][MAX_SITE_PER_CELL+1]; //
typedef std::pair < double, std::vector<int> > myPair;
std::map < std::pair<int, int>, std::vector<int> > myMap;
//
int finish[NUM_CELL];
int uavState[NUM_CELL][NUM_UAV];
SiteList cell_site_list[NUM_CELL];// all sites
queue < Ptr<SITE> > sortedSites[NUM_CELL]; // depend on urgency
SiteList sitesOfUav[NUM_CELL][NUM_UAV];// sites uav will visit
//
Vector GetPosition(Ptr<Node> node);
void SetPosition(Ptr<Node> node, Vector pos);
Vector GetVelocity(Ptr<Node> node);
void SetVelocity(Ptr<Node> node, Vector v);
double Goto(Ptr<Node> node, Vector dest);
double doixung(double t1, double t2);
void CalculateCellCenter();
void SetupSensorPosition(int cellId);
void SetupUavPosition(int cellId);
void SetupGwPosition(int cellId);
double CalculateCost(double distance);
void CreateSite();
//functions to solve TSP
myPair TSP(int mask, int current, int n);
//
void Execute(int cellId);
void SortSite(int cellId); //depend on urgency
void SiteAssignment(int cellId, int uavId);
int CalculateNumberOfSites(int cellId, int uavId);
void DoTask(int cellId, int uavId);
void NextRound(int cellId, int uavId);
int IsFinish();
int IsFinish(int cellId);
void StopSimulation();
/*Function implementation goes here*/

void SetPosition(Ptr<Node> node, Vector pos)
{

  Ptr<MobilityModel> m =  node -> GetObject<MobilityModel>();
  m -> SetPosition(pos);
}

Vector GetPosition(Ptr<Node> node)
{
  return node -> GetObject<MobilityModel>()->GetPosition();
}

Vector GetVelocity(Ptr<Node> node)
{
  return node->GetObject<MobilityModel>()->GetVelocity();
}
void SetVelocity(Ptr<Node> node, Vector v)
{
  node -> GetObject<ConstantVelocityMobilityModel>()->SetVelocity(v);
}
double Goto(Ptr<Node> node, Vector dest)
{
  Vector pos = GetPosition(node);
  double vx = dest.x - pos.x;
  double vy = dest.y - pos.y;
  double length = std::sqrt(vx*vx + vy*vy);
  if(length == 0)
  {
    return 0;
  }
  SetVelocity(node, Vector(vx/length*VUAV, vy/length*VUAV, 0));
  double flightTime = length/VUAV;
  Simulator::Schedule(Seconds(flightTime), &SetVelocity, node, Vector(0, 0, 0));
  return flightTime;
}
double doixung(double t1, double t2)
{
  return 2*t2-t1;
}
void CalculateCellCenter()
{
  int n=4;
  //int num=1;
  int start[n];
  start[0]=1;
  for(int i=1;i<n;i++)
  {
   // num+=6*i;
    start[i]=start[i-1]+(i-1)*6;
  }
  // double X[num];
  // double Y[num];
  X[0]=X0;
  Y[0]=Y0;
  for(int i=1;i<n;i++)
  {
    X[start[i]]=X0;
    Y[start[i]]=Y0+i*CELL_RADIUS*sqrt(3);
    int le=1,chan=2;
    for(int k=1;k<6*i;k++)
    {
      int m=start[i]+k;
      if(k<i+1)
      {
        X[m]=X[m-1]+1.5*CELL_RADIUS;
        Y[m]=Y[m-1]-0.5*CELL_RADIUS*sqrt(3);
      }
      else if(k<(1.5*i+1-0.5*(i%2)))
      {
        X[m]=X[m-1];
        Y[m]=Y[m-1]-CELL_RADIUS*sqrt(3);
      }
      else if(k<6*i/2)
      {
        if(i%2!=0)
        {
          X[m]=X[m-le];
          Y[m]=doixung(Y[m-le],Y0);
          le+=2;
        }
        else
        {
          X[m]=X[m-chan];
          Y[m]=doixung(Y[m-chan],Y0);
          chan+=2;
        }
      }
      else
      {
        X[m]=doixung(X[m-3*i], X0);
        Y[m]=doixung(Y[m-3*i], Y0);
      }    
    }
  }
}
void SetupSensorPosition(int cellId)
{
  std::cout<<"set up sensor position cell "<<cellId<<std::endl;
  MobilityHelper m;
    m.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (CELL_RADIUS/2.0*std::sqrt(3)),
                                 "X", DoubleValue (X[cellId]),
                                 "Y", DoubleValue (Y[cellId]) );
    m.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    m.Install (sensor[cellId]);
}

void SetupUavPosition(int cellId)
{
  std::cout<<"set up uav position cell "<<cellId<<std::endl;
  MobilityHelper m;
  m.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  m.Install(uav[cellId]);
  for(int i = 0; i < NUM_UAV; i++)
  {
    SetPosition(uav[cellId].Get(i), Vector(X[cellId], Y[cellId], 100));
  }
    //std::cout<<GetPosition(uav.Get(2));
}
void SetupGwPosition(int cellId)
{
  std::cout<<"set up gw position cell "<<cellId<<std::endl;
  MobilityHelper m;
  m.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  m.Install (gw[cellId]);
  for(int i = 0; i < NUM_GW ; i++)
  {
    SetPosition(gw[cellId].Get(i), Vector(X[cellId], Y[cellId], 0));
  }
    //std::cout<<GetPosition(gw.Get(3))<<std::endl;
}
void CreateSite()
{
  // calculate number of sites for each cell
  std::cout<<"Calculate number of sites for each cell"<<std::endl;
  int half = MAX_SITE_PER_CELL/2;
  if(TOTAL_SITE <= half*NUM_CELL)
  {
    int quotient = TOTAL_SITE / NUM_CELL;
    int remainder = TOTAL_SITE % NUM_CELL;
    for(int i = 0; i < NUM_CELL - 1; i++)
    {
      if(i == 0)
      {
        numberOfSites[i] = 2*quotient;
      }
      else
      {
        numberOfSites[i] = quotient;
      }
    }
    int id[] = {0,0,1,2,3,4,5};
    int k = 0;
    for(int i = 0; i < remainder; i++)
    {
      numberOfSites[id[k]]++;
      k++;
    }
  }
  else
  {
    if(TOTAL_SITE > MAX_SITE_PER_CELL*(NUM_CELL-1))
    {
      std::cout<<"Max total site is "<<MAX_SITE_PER_CELL*(NUM_CELL-1)<<std::endl;
      Simulator::Stop();
    }
    else
    {
      numberOfSites[0] = MAX_SITE_PER_CELL;
      int difference = TOTAL_SITE - half*NUM_CELL;
      int id[] = {1,2,3,4,5};
      int k = 0;
      for(int i = 0; i < difference; i++)
      {
        numberOfSites[id[k]]++;
        k++;
        if(k > 4)
        {
          k = 0;
        }
      }
    }
  }
  //create site
  std::cout<<"Creating sites"<<std::endl;
  finish[NUM_CELL-1] = 1;
  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
  for(int i = 0; i < NUM_CELL - 1; i++)
  {
    finish[i] = 0;
    for(int j = 0; j < numberOfSites[i]; j++)
    {
      double data = rd -> GetValue(MIN_VALUE, MAX_VALUE);
      Ptr<SITE> s = CreateObject<SITE>(GetPosition(sensor[i].Get(j)), data);
      cell_site_list[i].Add(s);
    }
  }
}
double CalculateCost(double distance)
{
  return Rd*distance;
}
void LocalTSP(int cellId, int uavId)
{
  /* Calculate distance matrix for TSP. Sites to visit are determined 
  according to algorithm "back tracking" */
  std::cout<<"TSP cell "<<cellId<<", uav "<<uavId<<std::endl;
  path.clear();
  int n = sitesOfUav[cellId][uavId].GetSize();
  std::cout<<"num site: "<<n<<std::endl;
  if(n == 0)
  {
    return ;
  }
  for(int i = 0; i < (1<<(n+1)); i++)
  {
    for(int j = 0; j < n+1; j++)
    {
      dp[i][j] = -1;
    }
   }
  VISITED_ALL = (1<<(n+1)) - 1;
  Vector pos[n];  
  for (int i = 0; i < n; i ++)
  {
    std::cout<<sitesOfUav[cellId][uavId].Get(i)->GetId()<<" ";
    pos[i] = sitesOfUav[cellId][uavId].Get(i)->GetSitePosition();
  }
  std::cout<<std::endl;
  
  dist[0][0] = 0;
  for(int k1 = 1; k1 <= n; k1++)
  {
    dist[0][k1] = dist[k1][0] = CalculateDistance(pos[k1-1], GetPosition(gw[cellId].Get(0)));
    for(int k2 = 1; k2 <= n; k2++)
    {
      dist[k1][k2] = CalculateDistance(pos[k1-1], pos[k2-1]);

    }
  }
  //Solve TSP
  myPair m = TSP(1, 0, n);
  path = m.second; 
  std::cout<<"path: "<<std::endl;
  
  for(int i = 1; i <= n; i++)
  {
    std::cout<<sitesOfUav[cellId][uavId].Get(path[i])->GetId()<<" ";
  }
  std::cout<<std::endl;
  std::cout<<"length : "<<m.first<<std::endl;
  return ;
}
myPair TSP(int mask,int current, int n) // 
{   
  std::vector <int> v;
  v.push_back(current-1);
  if(mask==VISITED_ALL)
  {
    return std::make_pair(dist[current][0], v);
  }
  if(dp[mask][current]!=-1)
  {
    return std::make_pair(dp[mask][current], myMap[std::make_pair(mask, current)]);
  }
   //Now from current node, we will try to go to every other node and take the min ans
   
  int ans = INT_MAX;
  //Visit all the unvisited cities and take the best route
  for(int k = 0; k < (n+1); k++)
  {
    if((mask & (1<<k)) == 0) // k haven't been visited
    {
      int newMask = mask | (1<<k);
      myPair m = TSP(newMask, k, n);
      double newAns = dist[current][k] + m.first;           
      if(newAns < ans)
      {
        ans = newAns;
        int first = v[0];
        v.clear();
        v.push_back(first);
        std::vector<int> v1 = m.second;
        for(int i = 0; i < (int)v1.size(); i++)
        {
          v.push_back(v1[i]);
        }
      }
    }
  }      
  dp[mask][current] = ans;
  myMap[std::make_pair(mask, current)] = v;
  return std::make_pair(ans, v);
} 
void Execute(int cellId)
{ 
  std::cout<<"execute cell "<<cellId<<std::endl;
  SortSite(cellId);
  for(int i = 0; i < NUM_UAV; i++)
  {
    uavState[cellId][i] = 0;
  }
  for(int i = 0 ; i < NUM_UAV; i++)
  {
    SiteAssignment(cellId, i);
  }  
}
void SortSite(int cellId)
{
  int n = cell_site_list[cellId].GetSize();
  int inQueue[n];
  for(int i = 0; i < n; i ++)
  {
    inQueue[i] = 0;
  }
  for(int i = 0; i < n; i++)
  {
    double maxUrgency = 0;
    int id = 0;
    for(int i = 0; i < n; i++)
    {
      if(inQueue[i] == 1)
      {
        continue;
      }
      double urgency = cell_site_list[cellId].Get(i)->GetUrgency();
      if(urgency > maxUrgency)
      {
        maxUrgency = urgency;
        id = i;
      }
    }
    sortedSites[cellId].push(cell_site_list[cellId].Get(id));
    inQueue[id] = 1;
  }
}
void SiteAssignment(int cellId, int uavId)
{
  int totalSite = sortedSites[cellId].size();
  if(totalSite == 0)
  {
    return;
  }
  double resource = 0;
  for(int i = 0; i < totalSite; i++)
  {
    Ptr<SITE> s = sortedSites[cellId].front();
    resource += s -> GetResource();
    if(resource > MAX_RESOURCE_PER_UAV)
    {
      break;
    }
    sitesOfUav[cellId][uavId].Add(s);
    sortedSites[cellId].pop();
  }
  if(sitesOfUav[cellId][uavId].GetSize() > 0)
  {
    uavState[cellId][uavId] = 1;
  }
  LocalTSP(cellId, uavId);
  int n = sitesOfUav[cellId][uavId].GetSize();
  Ptr<UAV> u = uav[cellId].GetUav(uavId);
  for(int i = 1; i <= n; i++)
  {
    u->AddSite(sitesOfUav[cellId][uavId].Get(path[i]));
  }
  DoTask(cellId, uavId);  
}

void DoTask(int cellId, int uavId)
{
  //std::cout<<GetNow()<<": goi ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
  Ptr<UAV> u = uav[cellId].GetUav(uavId);
  int n = u->GetSiteSize();
  if(n == 0)
  {
    sitesOfUav[cellId][uavId].Clear();
    std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    double flightTime = Goto(uav[cellId].Get(uavId), GetPosition(gw[cellId].Get(0)));    
    u -> UpdateFlightTime(flightTime);
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    Simulator::Schedule(Seconds(flightTime), &NextRound, cellId, uavId);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  Ptr<SITE> s = u->GetSite();
  u->RemoveSite();
  std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<s->GetId()<<std::endl;
  double flightTime = Goto(uav[cellId].Get(uavId), s -> GetSitePosition());
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  double visitedTime = s -> GetVisitedTime();
  u->UpdateFlightTime(flightTime + visitedTime);
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, cellId, uavId);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
  //std::cout<<"ket thuc ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
}
void NextRound(int cellId, int uavId)
{
  std::cout<<GetNow()<<": next round cell "<<cellId<<", uav "<<uavId<<std::endl;
  uavState[cellId][uavId] = 0;
  if(IsFinish(cellId))
  {
    finish[cellId] = 1;
    if(IsFinish())
    {
      StopSimulation();
    }
  }
  else
  {
    Simulator::Schedule(Seconds(60*INTERVAL_BETWEEN_TWO_ROUNDS), &SiteAssignment, cellId, uavId);
  }
}
int IsFinish()
{
  for(int i = 0; i < NUM_CELL; i++)
  {
    if(i == NUM_CELL - 1)
    {
      for(int j = 0; j < NUM_UAV; j++)
      {
        if(uavState[i][j] == 1)
        {
          return 0;
        }
      }
    }
    else if(finish[i] == 0)
    {
      return 0;
    }
  }
  return 1;
}
int IsFinish(int cellId)
{
  //std::cout<<"goi ham IsFinish"<<std::endl;
  if(sortedSites[cellId].size() > 0)
  {
   // std::cout<<"current <"<<std::endl;
    return 0;
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(uavState[cellId][i] == 1)
    {
     // std::cout<<"state uav "<<i<<std::endl;
      return 0;
    }
  }
  return 1;
}
void StopSimulation()
{
  std::cout<<GetNow()<<": stop sim"<<std::endl;
  double energy = 0;
  double fliedDistance = 0;
  double utility = 0;
  double flightTime = 0;
  for(int i = 0; i < NUM_CELL; i++)
  {
    energy += uav[i].CalculateEnergyConsumption();
    fliedDistance += uav[i].CalculateFliedDistance();
    utility += cell_site_list[i].GetUtility();
    flightTime += uav[i].CalculateFlightTime();
  }
  double cost = CalculateCost(fliedDistance);
  std::cout<<"Local TSP R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
  std::cout<<"Spanning time: "<<GetNow()<<" s"<<std::endl;
  std::cout<<"Flight time: "<<flightTime<<" s"<<std::endl;
  std::cout<<"Energy: "<<energy/1000000.0<<" MJ"<<std::endl;
  std::cout<<"Flied distance: "<<fliedDistance/1000.0<<" km"<<std::endl;
  std::cout<<"Benefit: "<<utility - cost<<std::endl;
  std::cout<<"Data: "<<dataLoad/1024.0/1024.0<<" MB"<<std::endl;
  Simulator::Stop();
}