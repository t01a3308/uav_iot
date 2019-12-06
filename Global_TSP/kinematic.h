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

extern double dataLoad;
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
extern std::map<Ptr<Node>, double> sensorData[NUM_CELL]; // all data
//extern std::map<Ptr<Node>, double> highData[NUM_CELL]; // all high data
//extern std::map<Ptr<Node>, double> taskToDo[NUM_CELL][NUM_UAV]; // all tasks of a uav
//extern std::queue<std::map<Ptr<Node>, double>> q[NUM_CELL][NUM_UAV];// sequence to do tasks
int appear[MAX_SITE_PER_CELL+1];
double dist[MAX_SITE_PER_CELL+1][MAX_SITE_PER_CELL+1];
int x[MAX_SITE_PER_CELL+1];
int path[MAX_SITE_PER_CELL+1];
double dmin;
double result;
double MIN;
int idNextSite = 0;
uint16_t idCurrentSite[NUM_CELL];
int uavState[NUM_CELL][NUM_UAV];
SiteList cell_site_list[NUM_CELL];
int finish[NUM_CELL];
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
void GenerateSensorData(int cellId, double threshold);
//functions to solve TSP
void TSP(int cellId);
int check(int v, int k);
void solution(int n);
void TRY(int k, int n);
//
void Execute(int cellId);
void SiteAssignment(int cellId, int uavId);
int CalculateNumberOfSites(int cellId, int uavId);
void DoTask(int cellId, int uavId, int idFirstSite, int idLastSite);
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
    SetPosition(uav[cellId].Get(i), Vector(X[cellId], Y[cellId] + CELL_RADIUS, 100));
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
    SetPosition(gw[cellId].Get(i), Vector(X[i], Y[i] + CELL_RADIUS, 0));
  }
    //std::cout<<GetPosition(gw.Get(3))<<std::endl;
}
void GenerateSensorData(int cellId, double threshold)
{

  std::cout<<"generate sensor data cell "<<cellId<<std::endl;
  finish[cellId] = 0;
  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
  for(int i = 0; i < NUM_SENSOR; i++)
  {
    double data = rd -> GetValue(0.0, 100.0);
    sensorData[cellId][sensor[cellId].Get(i)] = data;
    if(data > threshold)
    {
      Ptr<SITE> s = CreateObject<SITE>(GetPosition(sensor[cellId].Get(i)), data);
      cell_site_list[cellId].Add(s);
    }
  }
}
void TSP(int cellId)
{
  /* Calculate distance matrix for TSP. Sites to visit are determined 
  according to algorithm "back tracking" */
  std::cout<<"TSP cell "<<cellId<<std::endl;
  int n = cell_site_list[cellId].GetSize();
  if(n == 0)
  {
    return ;
  }
  std::cout<<"total sites: "<<n<<std::endl;
  Vector pos[n];  
  for (int i = 0; i < n; i ++)
  {
    pos[i] = cell_site_list[cellId].Get(i)->GetSitePosition();
  }
  dmin = 999999;
  result = 0;
  MIN = 9999999;
  dist[0][0] = 0;
  for(int k1 = 1; k1 <= n; k1++)
  {
    dist[0][k1] = dist[k1][0] = CalculateDistance(pos[k1-1], GetPosition(gw[cellId].Get(0)));
    for(int k2 = 1; k2 <= n; k2++)
    {
      dist[k1][k2] = CalculateDistance(pos[k1-1], pos[k2-1]);
     // std::cout<<"dist["<<k1<<"]["<<k2<<" = "<<dist[k1][k2]<<std::endl;
      if(dist[k1][k2] < dmin && dist[k1][k2] > 0)
      {
        dmin = dist[k1][k2];
      }
    }
    appear[k1] = 0;
  }
  //Solve TSP
  std::cout<<"solve tsp"<<std::endl;
  TRY(1, n);  
  std::cout<<"path: "<<std::endl;
  
  for(int i = 1; i <= n; i++)
  {
    std::cout<<path[i]<<" ";
  }
  std::cout<<std::endl;
  std::cout<<"length : "<<MIN<<std::endl;
  return ;
}
int check(int v, int k)
{
  return !appear[v]; //still not visited by UAV in TRY
}
void solution(int n)
{
	/* check if the current path is shorter than the previous, given
	the same sites */
  double rs = result + dist[x[n]][0];
  //std::cout<<"rs = "<<rs<<std::endl;
  if(rs < MIN)
  {
    MIN = rs;
    for(int i = 0; i <= n; i++)
    {
      path[i] = x[i] - 1;
    }
  }
}
void TRY(int k, int n)
{
	/*k: next site to visit; n: total sites */
  for(int v = 1; v <= n; v++)
  {   
    if(check(v, k)) //is v still not visited?
    {     
      x[k] = v; // store list of sites to visit
      result += dist[x[k-1]][x[k]];
      appear[v] = 1; // yes, already visited
      if(k == n )
      {
        solution(n);
      }
      else
      {
        if(result + dmin*(n-k+1) < MIN)
        {       
          TRY(k + 1, n);
        }
      }
      appear[v] = 0;
      result -= dist[x[k-1]][x[k]];
    }
  }
}
void Execute(int cellId)
{ 
  std::cout<<"execute cell "<<cellId<<std::endl;
  for(int i = 0; i < NUM_UAV; i++)
  {
    uavState[cellId][i] = 0;
  }
  idCurrentSite[cellId] = 0;
  for(int i = 0 ; i < NUM_UAV; i++)
  {
    SiteAssignment(cellId, i);
  }  
}
void SiteAssignment(int cellId, int uavId)
{
  int totalSite = cell_site_list[cellId].GetSize();
  int numSite = CalculateNumberOfSites(cellId, uavId);
  int firstSite = idCurrentSite[cellId];
  int lastSite = idCurrentSite[cellId] + numSite - 1;
  if(lastSite > (totalSite - 1))
  {
    lastSite = totalSite - 1;
  }
  uavState[cellId][uavId] = 1;
  idCurrentSite[cellId] += numSite;
  DoTask(cellId, uavId, firstSite, lastSite);  
}
int CalculateNumberOfSites(int cellId, int uavId)
{
  return 3;
}
void DoTask(int cellId, int uavId, int idFirstSite, int idLastSite)
{
  //std::cout<<GetNow()<<": goi ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
  Ptr<UAV> u = uav[cellId].GetUav(uavId);
  if(idFirstSite > idLastSite)
  {
    std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    double flightTime = Goto(uav[cellId].Get(uavId), GetPosition(gw[cellId].Get(0)));    
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    Simulator::Schedule(Seconds(flightTime), &NextRound, cellId, uavId);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  Ptr<SITE> firstSite = cell_site_list[cellId].Get(path[idFirstSite + 1]);
  std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<path[idFirstSite + 1]<<std::endl;
  double flightTime = Goto(uav[cellId].Get(uavId), firstSite -> GetSitePosition());
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime + firstSite -> GetVisitedTime()), &DoTask, cellId, uavId, idFirstSite + 1, idLastSite);
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
    Simulator::Schedule(Seconds(60*5), &SiteAssignment, cellId, uavId);
  }
}
int IsFinish()
{
  for(int i =0 ; i < NUM_CELL; i++)
  {
    if(finish[i] == 0)
    {
      return 0;
    }
  }
  return 1;
}
int IsFinish(int cellId)
{
  //std::cout<<"goi ham IsFinish"<<std::endl;
  if(idCurrentSite[cellId] < (cell_site_list[cellId].GetSize() - 1))
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
  for(int i = 0; i < NUM_CELL; i++)
  {
    energy += uav[i].CalculateEnergyConsumption();
    fliedDistance += uav[i].CalculateFliedDistance();
    utility += cell_site_list[i].GetUtility();
  }
  double cost = fliedDistance * Rd;
  std::cout<<"Flight time: "<<GetNow()<<" s"<<std::endl;
  std::cout<<"Energy: "<<energy/1000000.0<<" MJ"<<std::endl;
  std::cout<<"Flied distance: "<<fliedDistance/1000.0<<" km"<<std::endl;
  std::cout<<"Benefit: "<<utility - cost<<std::endl;
  std::cout<<"Data: "<<dataLoad/1024.0/1024.0<<" MB"<<std::endl;
  Simulator::Stop();
}