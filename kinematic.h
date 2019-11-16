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

using namespace ns3;

extern double X[maxNumCell], Y[maxNumCell];//cell center 's position
extern NodeContainer uav[numCell], sensor[numCell], gw[numCell], allNodes[numCell];
extern std::map<Ptr<Node>, double> sensorData[numCell]; // all data
extern std::map<Ptr<Node>, double> highData[numCell]; // all high data
extern std::map<Ptr<Node>, double> taskToDo[numCell][numUav]; // all tasks of a uav
extern std::queue<std::map<Ptr<Node>, double>> q[numCell][numUav];// sequence to do tasks
extern int appear[maxTaskPerUav+1];
extern double distance[maxTaskPerUav+1][maxTaskPerUav+1];
extern int x[maxTaskPerUav+1];
extern int path[maxTaskPerUav+1];
extern double dmin;
extern double result;
extern double MIN;
extern int uavNext[numCell];

Vector GetPosition(Ptr<Node> node);
void SetPosition(Ptr<Node> node, Vector pos);
Vector GetVelocity(Ptr<Node> node);
void SetVelocity(Ptr<Node> node, Vector v);
void Goto(Ptr<Node> node, Vector dest);
double doixung(double t1, double t2);
void CalculateCellCenter();
void SetupSensorPosition(int cellId);
void SetupGwPosition(int cellId);
void SetupUavPosition(int cellId);

//functions to solve TSP
void TSP(int cellId, int uavId);
int check(int v, int k);
void solution(int n);
void TRY(int k, int n);
/*Function implementation goes here*/

void SetPosition(Ptr<Node> node, Vector pos)
{
  node -> GetObject<MobilityModel>()->SetPosition(pos);
}
void SetupGwPosition(int cellId)
{
  MobilityHelper m;
    m.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    m.Install (gw[cellId]);
    for(int i = 0; i < numGw; i++)
    {
      SetPosition(gw[cellId].Get(i), Vector(X[i], Y[i] + r, 0));
    }
    //std::cout<<GetPosition(gw.Get(3))<<std::endl;
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
void Goto(Ptr<Node> node, Vector dest)
{
  Vector pos = GetPosition(node);
  double vx = dest.x - pos.x;
  double vy = dest.y - pos.y;
  double length = std::sqrt(vx*vx + vy*vy);
  SetVelocity(node, Vector(vx/length*vuav, vy/length*vuav, 0));
  Simulator::Schedule(Seconds(length/vuav), &SetVelocity, node, Vector(0, 0, 0));
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
  X[0]=x0;
  Y[0]=y0;
  for(int i=1;i<n;i++)
  {
    X[start[i]]=x0;
    Y[start[i]]=y0+i*r*sqrt(3);
    int le=1,chan=2;
    for(int k=1;k<6*i;k++)
    {
      int m=start[i]+k;
      if(k<i+1)
      {
        X[m]=X[m-1]+1.5*r;
        Y[m]=Y[m-1]-0.5*r*sqrt(3);
      }
      else if(k<(1.5*i+1-0.5*(i%2)))
      {
        X[m]=X[m-1];
        Y[m]=Y[m-1]-r*sqrt(3);
      }
      else if(k<6*i/2)
      {
        if(i%2!=0)
        {
          X[m]=X[m-le];
          Y[m]=doixung(Y[m-le],y0);
          le+=2;
        }
        else
        {
          X[m]=X[m-chan];
          Y[m]=doixung(Y[m-chan],y0);
          chan+=2;
        }
      }
      else
      {
        X[m]=doixung(X[m-3*i], x0);
        Y[m]=doixung(Y[m-3*i], y0);
      }    
    }
  }
}
void SetupSensorPosition(int cellId)
{
  MobilityHelper m;
    m.SetPositionAllocator ("ns3::UniformDiscPositionAllocator",
                                 "rho", DoubleValue (r/2.0*std::sqrt(3)),
                                 "X", DoubleValue (X[cellId]),
                                 "Y", DoubleValue (Y[cellId]) );
    m.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    m.Install (sensor[cellId]);
}

void SetupUavPosition(int cellId)
{
  MobilityHelper m;
  m.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    m.Install(uav[cellId]);
    for(int i = 0; i < numUav; i++)
    {
      SetPosition(uav[cellId].Get(i), Vector(X[cellId], Y[cellId] + r, 100));
    }
    //std::cout<<GetPosition(uav.Get(2));
}
void TSP(int cellId, int uavId)
{
  //Calculate distance matrix for TSP
  int n = taskToDo[cellId][uavId].size();
  if(n == 0)
  {
    return;
  }
  Vector pos[n];  
  int i = 0;
  auto it1 = taskToDo[cellId][uavId].begin();
  while (it1 != taskToDo[cellId][uavId].end())
  {
    pos[i] = GetPosition(it1->first);
    it1++;
    i++;
  }
  dmin = 999999;
  result = 0;
  MIN = 9999999;
  distance[0][0] = 0;
  for(int k1 = 1; k1 <= n; k1++)
  {
    distance[0][k1] = distance[k1][0] = CalculateDistance(pos[k1-1], GetPosition(uav[cellId].Get(uavId)));
    for(int k2 = 1; k2 <= n; k2++)
    {
      distance[k1][k2] = CalculateDistance(pos[k1-1], pos[k2-1]);
      if(distance[k1][k2] < dmin && distance[k1][k2] > 0)
      {
        dmin = distance[k1][k2];
      }
    }
    appear[k1] = 0;
  }
  //Solve TSP
  TRY(1, n);  
  std::cout<<"path: "<<std::endl;
  /*
  for(int i = 1; i <= n; i++)
  {
    //std::cout<<path[i]<<" ";
    int s = 0;
    auto it = taskToDo[cellId][uavId].begin();
    while(it != taskToDo[cellId][uavId].end())
    {
      if(s == path[i] - 1)
      {
        q[cellId][uavId].push(it);
        break;
      }
    }
  }*/
  std::cout<<std::endl;
  std::cout<<"length : "<<MIN<<std::endl;
}
int check(int v, int k)
{
  return !appear[v];
}
void solution(int n)
{
  double rs = result + distance[x[n]][0];
  //std::cout<<"rs = "<<rs<<std::endl;
  if(rs < MIN)
  {
    MIN = rs;
    for(int i = 0; i <= n; i++)
    {
      path[i] = x[i];
    }
  }
}
void TRY(int k, int n)
{
  for(int v = 1; v <= n; v++)
  {   
    if(check(v, k))
    {     
      x[k] = v;
      result += distance[x[k-1]][x[k]];
      appear[v] = 1;
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
      result -= distance[x[k-1]][x[k]];
    }
  }
}
void Execute(int cellId)
{ 
  int ex = 1;
  for(int i = 0; i < maxTaskPerUav; i++)
  {
    if(highData[cellId].size() == 0)
    {
      ex = 0;
      break;
    }
    auto it = highData[cellId].begin();
    double maxData = 0;
    Ptr<Node> node;
    while (it != highData[cellId].end())
    {
      if(it->second > maxData)
      {
        node = it->first;
        maxData = it->second;
      }
      it++;
    }
    taskToDo[cellId][uavNext[cellId]][node] = maxData;
    highData[cellId].erase(node);
  }
  std::cout<<"TSP cell "<<cellId<<", uav "<<uavNext[cellId]<<std::endl;
  TSP(cellId, uavNext[cellId]);
  uavNext[cellId]++;
  if(ex)
  {
    Simulator::Schedule(Seconds(5), &Execute, cellId);
  }
}

