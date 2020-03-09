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
#include "ns3/gnuplot.h"
#include <iostream>
#include <math.h>
#include <queue>
#include <list>
#include "macro_param.h"
//#include "handle.h"
#include "communication.h"
using namespace ns3;
using namespace std;
extern AnimationInterface *anim;
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
typedef std::pair < double, std::vector<int> > myPairs;
std::map < std::pair<int, int>, std::vector<int> > myMap;
//
int finish[NUM_CELL];
int uavState[NUM_CELL][NUM_UAV];
SiteList cell_site_list[NUM_CELL];// all sites
queue < Ptr<SITE> > sortedSites[NUM_CELL]; // depend on urgency
SiteList sitesOfUav[NUM_CELL][NUM_UAV];// sites uav will visit
SiteList segment[NUM_CELL][MAX_SITE_PER_CELL];
int numSegment[NUM_CELL];
int mark[NUM_CELL][MAX_SITE_PER_CELL];
double segmentFlightTime[NUM_CELL][MAX_SITE_PER_CELL];
int numSite[NUM_CELL];
std::vector<int> segmentsOfUav[NUM_CELL][NUM_UAV];
std::vector<int> completedSites[NUM_CELL];
typedef std::pair<double, double> myPair;
typedef std::pair<int, std::string> myPair1;
typedef std::pair<myPair1, myPair> mp;
std::vector<mp> workInfor[NUM_CELL];
double length0 = 0;
//
Gnuplot2dDataset p[NUM_UAV];
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
myPairs TSP(int mask, int current, int n);
//
void Execute(int cellId);
void SortSite(int cellId); //depend on urgency
void DivideSitesIntoSegment(int cellId);
void FindSegment(int cellId, int uavId);
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
void AddPath(int uavId)
{
  Vector pos = GetPosition(uav[0].Get(uavId));
  p[uavId].Add(pos.x, pos.y);
}
void CreatePathPlot ()
{
  std::string fileNameWithNoExtension = "path_local_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = fileNameWithNoExtension + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  //std::string plotTitle               = "2-D Plot";
  std::string dataTitle               = "uav";
  // Instantiate the plot and set its title.
  Gnuplot plot (graphicsFileName);
 // plot.SetTitle (plotTitle);

  // Make the graphics file, which the plot file will create when it
  // is used with Gnuplot, be a PNG file.
  plot.SetTerminal ("png");

  // Rotate the plot 30 degrees around the x axis and then rotate the
  // plot 120 degrees around the new z axis.
  plot.AppendExtra ("set view 30, 120, 1.0, 1.0");

  // Make the zero for the z-axis be in the x-axis and y-axis plane.
  plot.AppendExtra ("set ticslevel 0");

  // Set the labels for each axis.
  plot.AppendExtra ("set xlabel \"Coordination X\"");
  plot.AppendExtra ("set ylabel \"Coordination Y\"");
  

  // Set the ranges for the x and y axis.


  // Instantiate the dataset, set its title, and make the points be
  // connected by lines.
  for(int i = 0; i < NUM_UAV; i++)
  {
    p[i].SetStyle (Gnuplot2dDataset::LINES);
    p[i].SetTitle (dataTitle+std::to_string(i));
    plot.AddDataset (p[i]);
  }
  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
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
    SetPosition(uav[cellId].Get(i), Vector(X[cellId], Y[cellId], height));
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
    SetPosition(gw[cellId].Get(i), Vector(X[cellId], Y[cellId], height));
  }
    //std::cout<<GetPosition(gw.Get(3))<<std::endl;
}
void CreateSite()
{
  // calculate number of sites for each cell
  
  if(TOTAL_SITE > 90)
  {
    std::cout<<"Max total site is "<<90<<std::endl;
    Simulator::Stop();
  }
  std::cout<<"Calculate number of sites for each cell"<<std::endl;
  int temp = TOTAL_SITE;
  int k = 0, id = 0;
  while(temp > 0)
  {
    if(k % 2 != 0)
    {
      if(id == 0)
      {
        numberOfSites[id] += 2;
        temp -= 2;
      }
      else
      {
        numberOfSites[id]++;
        temp--;
      }     
    }
    else
    {
      numberOfSites[id]++;
      temp --;
    }
    id++;
    if(id == NUM_CELL-1)
    {
      id = 0;
      k++;
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
      Ptr<SITE> s = CreateObject<SITE>(GetPosition(sensor[i].Get(j)), data, j);
      cell_site_list[i].Add(s);
      anim->UpdateNodeSize(sensor[i].Get(j)->GetId(), 100, 100);
      anim->UpdateNodeColor(sensor[i].Get(j)->GetId(), 0, 255, 0);
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
  myPairs m = TSP(1, 0, n);
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
myPairs TSP(int mask,int current, int n) // 
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
      myPairs m = TSP(newMask, k, n);
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
  numSite[cellId] = 0;
  SortSite(cellId);
  for(int i = 0; i < NUM_UAV; i++)
  {
    anim->UpdateNodeSize(uav[cellId].Get(i)->GetId(), 200, 200);
    anim->UpdateNodeColor(uav[cellId].Get(i)->GetId(), 0, 0, 255);
    uavState[cellId][i] = 0;
  }
  DivideSitesIntoSegment(cellId);
  for(int i = 0 ; i < NUM_UAV; i++)
  {
    FindSegment(cellId, i);
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

void CalculateSegmentFlightTime(int cellId)
{
  for(int i = 0; i < numSegment[cellId]; i++)
  {
    segmentFlightTime[cellId][i] = 0;
    int n = segment[cellId][i].GetSize();
    //
    Ptr<SITE> first = segment[cellId][i].Get(0);
    Vector pos = first->GetSitePosition();
    double dt = CalculateDistance(pos, GetPosition(gw[cellId].Get(0)));
    double t = dt/VUAV;
    segmentFlightTime[cellId][i] += t;
    //
    Ptr<SITE> last = segment[cellId][i].Get(n-1);
    Vector pos1 = last->GetSitePosition();
    double dt1 = CalculateDistance(pos1, GetPosition(gw[cellId].Get(0)));
    double t1= dt1/VUAV;
    segmentFlightTime[cellId][i] += t1;
    for(int j = 0; j < n; j++)
    {
      Ptr<SITE> s1 = segment[cellId][i].Get(j);
      double time = s1->GetVisitedTime();
      segmentFlightTime[cellId][i] += time;
      if(j != n-1)
      {
        Ptr<SITE> s2 = segment[cellId][i].Get(j+1);
        Vector p1 = s1->GetSitePosition();
        Vector p2 = s2->GetSitePosition();
        double d = CalculateDistance(p1, p2);
        double time1 = d/VUAV;
        segmentFlightTime[cellId][i] += time1;
      }
    }
    segmentFlightTime[cellId][i] /= 60.0;
  }
}
void PrintSegmentInformation(int cellId)
{
  std::cout<<"cell "<<cellId<<std::endl;
  for(int i = 0; i < numSegment[cellId]; i++)
  {
    std::cout<<"segment "<<i<<": ";
    for(int j = 0; j < (int)segment[cellId][i].GetSize(); j++)
    {
      Ptr<SITE> s = segment[cellId][i].Get(j);
      int id = s->GetId();
      std::cout<<id<<" ";
    }
    std::cout<<"\t flightTime: "<<segmentFlightTime[cellId][i]<<std::endl;
  }
}
void AllocateSegment(int cellId)
{
  if(numSegment[cellId] == 0)
  {
    return;
  }
  int n = numSegment[cellId];
  double time[NUM_UAV];
  for(int i = 0; i < NUM_UAV; i++)
  {
    time[i] = 0;
  }
  int mk[n];
  for(int i = 0; i < n; i++)
  {
    mk[i] = 0;
  }
  while(n > 0)
  {
    int idSegment = 999;
    double maxtime = 0;
    for(int i = 0; i < numSegment[cellId]; i++)
    {
      if(mk[i] == 0)
      {
        if(segmentFlightTime[cellId][i] > maxtime)
        {
          maxtime = segmentFlightTime[cellId][i];
          idSegment = i;
        }
      }
    }
    mk[idSegment] = 1;
    //
    int idmin = 0;
    double mintime = time[0];
    for(int i = 1; i < NUM_UAV; i++)
    {
      if(time[i] < mintime)
      {
        mintime = time[i];
        idmin = i;
      }
    }
    segmentsOfUav[cellId][idmin].push_back(idSegment);
    time[idmin] += segmentFlightTime[cellId][idSegment];
    n--;
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    std::cout<<"segments of uav "<<i<<": ";
    for(int j = 0; j < (int)segmentsOfUav[cellId][i].size(); j++)
    {
      std::cout<<segmentsOfUav[cellId][i][j]<<" ";
    }
    std::cout<<std::endl;
  }
}
void DivideSitesIntoSegment(int cellId)
{
  std::cout<<"divide site into segment cell "<<cellId<<std::endl;
  
  int totalSite = cell_site_list[cellId].GetSize();
  if(totalSite == 0)
  {
    std::cout<<"cell "<<cellId<<" has 0 segment"<<std::endl;
    numSegment[cellId] = 0;
    return;
  }
    std::list<int> list;
    int step;
    int remainder;
    SiteList sl[totalSite];
    std::cout<<"list: ";
    for(int i = 0; i < totalSite; i++)
    {
      int id = sortedSites[cellId].front()->GetId();
      list.push_back(id);
      sortedSites[cellId].pop();
      std::cout<<id<<" ";
    }
    std::cout<<std::endl;
    //////////////////
    int idSegment = 0;
    while(list.size() > 0)
    {
      int n = (int)list.size();
      step = n/NUM_UAV;
      remainder = n - step*NUM_UAV;
      if(step > 0)
      {
        //std::cout<<"first"<<std::endl;
        for(int i = 0; i < NUM_UAV; i++)
        {
          
          for(int j = 0; j < step; j++)
          {
            if(list.size() == 0)
            {
              break;
            }
            int siteId = GetValue<int>(list, 0);
            Ptr<SITE> s = cell_site_list[cellId].Get(siteId);
            if(sl[idSegment].GetResource() + s->GetResource() < MAX_RESOURCE_PER_UAV)
            {
              sl[idSegment].Add(s);
              list.pop_front();
              if(j == step - 1)
              {
                if(remainder > 0)
                {
                  if(list.size() == 0)
                  {
                    break;
                  }
                  int siteId1 = GetValue<int>(list, 0);
                  Ptr<SITE> s1 = cell_site_list[cellId].Get(siteId1);
                  //std::cout<<"siteid1 = "<<siteId1<<" i = "<<i<<"j = "<<j<<std::endl;
                  if(sl[idSegment].GetResource() + s1->GetResource() < MAX_RESOURCE_PER_UAV)
                  {
                    sl[idSegment].Add(s1);
                    list.pop_front();
                    remainder--;
                  }
                }
              }
            }
          }
          idSegment++;
        }
      }
      else
      {
       // std::cout<<"second"<<std::endl;
        for(int i = 0; i < remainder; i++)
        {
          if(list.size() == 0)
          {
            break;
          }
          for(int j = 0; j < remainder; j++)
          {
            if(list.size() == 0)
            {
              break;
            }
            int siteId = GetValue<int>(list, 0);
            Ptr<SITE> s = cell_site_list[cellId].Get(siteId);
            double r1 = sl[idSegment].GetResource();
            double r2 = s->GetResource();
            if(r1 + r2 < MAX_RESOURCE_PER_UAV)
            {
              sl[idSegment].Add(s);
              list.pop_front();
            }
          }
          idSegment++;
        }
      }
      
    }
    //////////////////////

    numSegment[cellId] = 0;
    for(int i = 0; i < totalSite; i++)
    {
      if(sl[i].GetSize() == 0)
      {
        break;
      }
      numSegment[cellId]++;
      for(int j = 0; j < (int)sl[i].GetSize(); j++)
      {
        Ptr<SITE> s = sl[i].Get(j);
        segment[cellId][i].Add(s);
      }
      sl[i].Clear();
    }
  
  for(int i = 0; i < numSegment[cellId]; i++)
  {
    mark[cellId][i] = 0;
  }
  std::cout<<"cell "<<cellId<<" has "<<numSegment[cellId]<<" segment"<<std::endl;
  CalculateSegmentFlightTime(cellId);
  PrintSegmentInformation(cellId);
  AllocateSegment(cellId);
}
void FindSegment(int cellId, int uavId)
{
  //std::cout<<"find segment cell "<<cellId<<" uav "<<uavId<<std::endl;
  if(segmentsOfUav[cellId][uavId].size() == 0)
  {
    return;
  }
  int id = segmentsOfUav[cellId][uavId][0];
  segmentsOfUav[cellId][uavId].erase(segmentsOfUav[cellId][uavId].begin());
  uavState[cellId][uavId] = 1;  
  for(int i = 0; i < (int)segment[cellId][id].GetSize(); i++)
  {
    Ptr<SITE> s = segment[cellId][id].Get(i);
    sitesOfUav[cellId][uavId].Add(s);
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
void ChangeColor(int cellId, int sensorId)
{
  anim->UpdateNodeColor(sensor[cellId].Get(sensorId)->GetId(), 255, 0, 0);
}
void DoTask(int cellId, int uavId)
{
  //std::cout<<GetNow()<<": goi ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
  Ptr<UAV> u = uav[cellId].GetUav(uavId);
  if(cellId == 0)
  {
    AddPath(uavId);
  }
  int n = u->GetSiteSize();
  if(n == 0)
  {
    sitesOfUav[cellId][uavId].Clear();
   // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    double flightTime = Goto(uav[cellId].Get(uavId), GetPosition(gw[cellId].Get(0)));    
    u -> UpdateFlightTime(flightTime);
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    if(cellId == 0)
    {
      length0 += VUAV*flightTime;
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
    }
    Simulator::Schedule(Seconds(flightTime), &NextRound, cellId, uavId);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  numSite[cellId]++;
  Ptr<SITE> s = u->GetSite();
  completedSites[cellId].push_back(s->GetId());
  u->RemoveSite();
  //std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<s->GetId()<<std::endl;
  double flightTime = Goto(uav[cellId].Get(uavId), s -> GetSitePosition());
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  double visitedTime = s -> GetVisitedTime();
  u->UpdateFlightTime(flightTime + visitedTime);
  myPair1 m1 = std::make_pair(s->GetId(), std::to_string(cellId) + std::to_string(uavId));
  myPair m2 = std::make_pair(GetNow() + flightTime, GetNow() + flightTime + visitedTime);
  workInfor[cellId].push_back(std::make_pair(m1, m2));
  if(cellId == 0)
  {
    length0 += VUAV*flightTime;
    Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
  }
  Simulator::Schedule(Seconds(flightTime+visitedTime), &ChangeColor, cellId, s->GetId());
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, cellId, uavId);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
  //std::cout<<"ket thuc ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
}
void NextRound(int cellId, int uavId)
{
  uavState[cellId][uavId] = 0;
  if(IsFinish(cellId))
  {
   // std::cout<<GetNow()<<"cell "<<cellId<<" xong"<<std::endl;
    finish[cellId] = 1;
    if(IsFinish())
    {
      StopSimulation();
    }
  }
  else
  {
   // std::cout<<GetNow()<<": next round cell "<<cellId<<", uav "<<uavId<<std::endl;
    Simulator::Schedule(Seconds(60*INTERVAL_BETWEEN_TWO_ROUNDS), &FindSegment, cellId, uavId);
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
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(segmentsOfUav[cellId][i].size() > 0)
    {
      return 0;
    }
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(uavState[cellId][i] == 1)
    {
      return 0;
    }
  }
  return 1;
}
void StopSimulation()
{
 // std::cout<<GetNow()<<": stop sim"<<std::endl;
  double energy = 0;
  double fliedDistance = 0;
  double utility = 0;
  double flightTime = 0;
  for(int i = 0; i < NUM_CELL; i++)
  {
    std::cout<<i<<": "<<numSite[i]<<std::endl;
    energy += uav[i].CalculateEnergyConsumption();
    fliedDistance += uav[i].CalculateFliedDistance();
    utility += cell_site_list[i].GetUtility();
    flightTime += uav[i].CalculateFlightTime();
  }
  // for(int i = 0; i < NUM_CELL; i++)
  // {
  //   std::cout<<"cell "<<i<<": ";
  //   for(int j = 0; j < (int)completedSites[i].size(); j++)
  //   {
  //     std::cout<<completedSites[i][j]<<" ";
  //   }
  //   std::cout<<std::endl;
  // }
  // for(int i = 0; i < NUM_CELL; i++)
  // {
  //   std::cout<<"cell "<<i<<std::endl;
  //   for(int j = 0; j < (int)workInfor[i].size(); j++)
  //   {
  //     mp m = workInfor[i][j];
  //     myPair1 id = m.first;
  //     myPair time = m.second;
  //     std::cout<<"site "<<id.first<<", by "<<id.second<<", start = "<<time.first<<", stop = "<<time.second<<", pos "<<cell_site_list[i].Get(id.first)->GetSitePosition()<<std::endl;
  //   }
  // }
  std::cout<<"length0 = "<<length0<<std::endl;
  double cost = CalculateCost(fliedDistance);
  std::cout<<"Local TSP R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
  std::cout<<"Spanning time: "<<GetNow()/60.0<<" m"<<std::endl;
  std::cout<<"Flight time: "<<flightTime/3600.0<<" h"<<std::endl;
  std::cout<<"Energy: "<<energy/1000000.0<<" MJ"<<std::endl;
  std::cout<<"Flied distance: "<<fliedDistance/1000.0<<" km"<<std::endl;
  std::cout<<"Benefit: "<<utility - cost<<std::endl;
  std::cout<<"Data: "<<dataLoad/1024.0/1024.0<<" MB"<<std::endl;
  CreatePathPlot();
  Simulator::Stop();
}