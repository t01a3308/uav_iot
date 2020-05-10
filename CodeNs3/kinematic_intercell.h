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
#include <list>
#include "macro_param.h"
#include "communication.h"
#include "vrp_capacity.h"
using namespace ns3;
extern AnimationInterface *anim;
extern double dataLoad;
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
extern std::map<Ptr<Node>, double> sensorData[NUM_CELL]; // all data
//

int numberOfSites[NUM_CELL-1];
//

//
int uavState[NUM_CELL][NUM_UAV];
int siteState[NUM_CELL][MAX_SITE_PER_CELL]; // 0 - not a site, 1 - new site, 2 - assigned to UAV, 3 - being handled, 4 - done
SiteList cell_site_list[NUM_CELL];
int numSegment[NUM_CELL];
int finish[NUM_CELL];
std::vector<EventId> event;
//
// cvrp variables
std::vector<std::vector<int64>> dist_matrix;
std::vector<int64> site_resource;
std::vector<int64> uav_resource;
int nUAV;
//
int numScenario = 0;
double Tbegin = 0;
double Tend = 0;
double cumulativeSpanningTime = 0;
double cumulativeFlightTime = 0;
double cumulativeFlightDistance = 0;
double cumulativeEnergy = 0;
double cumulativeUtility = 0;
double cumulativeBenefit = 0;
double cumulativeData = 0;
double uti = 0;
//
Gnuplot2dDataset p[NUM_UAV], p1[NUM_UAV];
Gnuplot2dDataset utidts, uavdts;
//
//variables for TSP
double dt_matrix[21][21];
std::vector <int> path;
int VISITED_ALL; // 
double dp[2097152][21]; //
typedef std::pair < double, std::vector<int> > myPairs;
std::map < std::pair<int, int>, std::vector<int> > myMap;
//
//functions to solve TSP
myPairs TSP(int mask, int current, int n);
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
Ptr<SITE> NewSite();
void CreateSite();
void ChangeSiteState(int cellId, int siteId, int state);
//functions to solve TSP

//
double CalculateCost(double distance);
void Execute(int cellId);
std::vector<std::vector<int>> SolveCVRP(int cellId, Ptr<UAV> u);
void InterCell(int cellId);
void FindSegment(int cellId, Ptr<UAV> u);
void AllocateSegment(Ptr<UAV> u, SiteList sl);
void FreeSite(Ptr<UAV> u);
void DoTask(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);
int IsFinish();
int SiteCheck(int cellId);
int IsFinish(int cellId);
void EndScenario();
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
void AddPath(int uavId, int cellId)
{
  Vector pos = GetPosition(uav[cellId].Get(uavId));
  if(cellId == 0)
  {
    p[uavId].Add(pos.x, pos.y);
  }
  else if(cellId == 6)
  {
    p1[uavId].Add(pos.x, pos.y);
  }

}
void UpdateUtility(double begintime, double endtime, double delta)
{
  double now = GetNow();
  if(now > endtime)
  {
    return;
  }
  else
  {
    if(now < begintime)
    {

    }
    else
    {
      uti += delta;
    }
    Simulator::Schedule(Seconds(0.1), &UpdateUtility, begintime, endtime, delta);
  }
}
void AddUtilityPlot()
{
  if(numScenario > 0)
  {
    return;
  }
  utidts.Add(GetNow(), uti);
  Simulator::Schedule(Seconds(0.5), &AddUtilityPlot);
}
void NumUAVPlot()
{
  if(numScenario > 0)
  {
    return;
  }
  int n = 0;
  for(int i = 0; i < NUM_CELL; i++)
  {
    for(int j = 0; j < NUM_UAV; j++)
    {
      if(uavState[i][j] == 1)
      {
        n++;
      }
      else
      {

      }
    }
  }
  uavdts.Add(GetNow(), n);
  Simulator::Schedule(Seconds(0.5), &NumUAVPlot);
}
void CreateUtilityPlot ()
{
  std::string fileNameWithNoExtension = "utility/intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  //std::string plotTitle               = "2-D Plot";
  std::string dataTitle               = "utility";
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

    utidts.SetStyle (Gnuplot2dDataset::LINES);
    utidts.SetTitle (dataTitle);
    plot.AddDataset (utidts);
  
  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
}
void CreateNumUAVPlot ()
{
  std::string fileNameWithNoExtension = "numUAV/intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
  std::string plotFileName            = fileNameWithNoExtension + ".plt";
  //std::string plotTitle               = "2-D Plot";
  std::string dataTitle               = "numUAV";
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

    uavdts.SetStyle (Gnuplot2dDataset::LINES);
    uavdts.SetTitle (dataTitle);
    plot.AddDataset (uavdts);
  
  // Open the plot file.
  std::ofstream plotFile (plotFileName.c_str());

  // Write the plot file.
  plot.GenerateOutput (plotFile);

  // Close the plot file.
  plotFile.close ();
}
void CreatePathPlot ()
{
  std::string fileNameWithNoExtension = "path/intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "intercell_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
  for(int i = 0; i < NUM_UAV; i++)
  {
    p1[i].SetStyle (Gnuplot2dDataset::LINES);
    p1[i].SetTitle (dataTitle+std::to_string(i));
    plot.AddDataset (p1[i]);
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
}
SiteList LocalTSP(SiteList sl, Vector depot)
{
  SiteList res;
  int n = sl.GetSize();
  path.clear();
  for(int i = 0; i < (1<<(n+1)); i++)
  {
    for(int j = 0; j < n+1; j++)
    {
      dp[i][j] = -1;
    }
   }
  VISITED_ALL = (1<<(n+1)) - 1;
  Vector pos[n];  
  for (int i = 0; i < n; i++)
  {
    Ptr<SITE> s = sl.Get(i);
    pos[i] = s->GetSitePosition();
  }
  dt_matrix[0][0] = 0;
  for(int k1 = 1; k1 <= n; k1++)
  {
    dt_matrix[0][k1] = dt_matrix[k1][0] = CalculateDistance(pos[k1-1], depot);
    for(int k2 = 1; k2 <= n; k2++)
    {
      dt_matrix[k1][k2] = CalculateDistance(pos[k1-1], pos[k2-1]);
    }
  }
  //Solve TSP
  myPairs m = TSP(1, 0, n);
  path = m.second; 
 // std::cout<<"path: "<<std::endl;
  for(int i = 1; i <= n; i++)
  {
    Ptr<SITE> s = sl.Get(path[i]);
    res.Add(s);
   // std::cout<<s->GetId()<<" ";
  }
 // std::cout<<std::endl;
 // std::cout<<"length : "<<m.first<<std::endl;
  myMap.clear();
  return res;
}
myPairs TSP(int mask,int current, int n) // 
{   
  std::vector <int> v;
  v.push_back(current-1);
  if(mask==VISITED_ALL)
  {
    return std::make_pair(dt_matrix[current][0], v);
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
      double newAns = dt_matrix[current][k] + m.first;           
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
void CalculateNumberOfSites()
{
  // std::cout<<"Calculate number of sites for each cell"<<std::endl;
  for(int i = 0; i < NUM_CELL-1; i++)
  {
    numberOfSites[i] = 0;
  }
  int t = TOTAL_SITE;
  int k = 0, id = 0;
  while(t > 0)
  {
    if(k % 2 != 0)
    {
      if(id == 0)
      {
        numberOfSites[id] += 2;
        t -= 2;
      }
      else
      {
        numberOfSites[id]++;
        t--;
      }     
    }
    else
    {
      numberOfSites[id]++;
      t --;
    }
    id++;
    if(id == NUM_CELL-1)
    {
      id = 0;
      k++;
    }
  }
}
void ChangeSiteState(int cellId, int siteId, int state)
{
  if(finish[cellId] == 1)
  {
    return;
  }
  siteState[cellId][siteId] = state;
  // if(state == 0)
  // {
  //   anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 1, 1);
  //   anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 0);
  // }
  // else if(state == 1)
  // {
  //   anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
  //   anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 0);
  // }
  // else if(state == 2)
  // {
  //   anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
  //   anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 255, 255, 0);
  // }
  // else if(state == 3)
  // {
  //   anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
  //   anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 255, 0, 0);
  // }
  // else
  // {
  //   anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 1, 1);
  //   anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 255);
  // }
}

Ptr<SITE> NewSite(int cellId, int siteId)
{
  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
  int data = (int) rd -> GetValue(MIN_VALUE, MAX_VALUE);
  double radius = rd -> GetValue(0.0, CELL_RADIUS/2.0*std::sqrt(3));
  double theta = rd -> GetValue(0.0, 2*PI);
  double x = X[cellId] + radius*std::cos(theta);
  double y = Y[cellId] + radius*std::sin(theta);
  Vector pos = Vector(x, y, height);
  SetPosition(sensor[cellId].Get(siteId), pos);
  int change_resource = 0;
  double delta_appear_time = 0;
  int tenPercent = (int)numberOfSites[cellId]/10;
  if(siteId < tenPercent)
  {
    change_resource = 1;
  }
  if(siteId > (numberOfSites[cellId] - tenPercent))
  {
    delta_appear_time = rd -> GetValue(MIN_TIME, MAX_TIME);
  }
  Ptr<SITE> s = CreateObject<SITE>(pos, data, siteId, cellId, change_resource, delta_appear_time);
  Simulator::Schedule(Seconds(delta_appear_time), &ChangeSiteState, cellId, siteId, 1);
  EventId ev = Simulator::Schedule(Seconds(delta_appear_time), &ChangeSiteState, cellId, siteId, 1);
  if(delta_appear_time > 0)
  {
    event.push_back(ev);
  }
  return s;
}
void CreateSite()
{
  //create site
 // std::cout<<"Creating sites"<<std::endl;
  finish[NUM_CELL-1] = 1;
  
  for(int i = 0; i < NUM_CELL - 1; i++)
  {
   // std::cout<<"cell "<<i<<" has "<<numberOfSites[i]<<" sites"<<std::endl;
    finish[i] = 0;
    int tenPercent = (int)numberOfSites[i]/10;
    for(int j = 0; j < numberOfSites[i]; j++)
    {
      Ptr<SITE> s = NewSite(i, j);
      cell_site_list[i].Add(s);    
    }
  }
}
void NewScenario()
{
  for(int i = 0; i < NUM_CELL; i++)
  {
    for(int j = 0; j < numberOfSites[i]; j++)
    {
      ChangeSiteState(i, j, 0); 
    }
    for(int j = 0; j < NUM_UAV; j++)
    {
      uavState[i][j] = 0;
      // anim->UpdateNodeSize(uav[i].Get(j)->GetId(), 200, 200);
      // anim->UpdateNodeColor(uav[i].Get(j)->GetId(), 0, 0, 255);
    }
  }
  Tbegin = GetNow();
  CreateSite();
  for(int i = 0; i < NUM_CELL-1; i++)
  {
    Execute(i);
  }
}
double CalculateCost(double distance)
{
  return distance*Rd;
}

void Execute(int cellId)
{ 
 // std::cout<<"execute cell "<<cellId<<std::endl;
  if(cellId == 0)
  {
    Simulator::Schedule(Seconds(0.001), &InterCell, cellId);
    for(int i = 0 ; i < NUM_UAV; i++)
    {
      Simulator::Schedule(Seconds(0.01*i + 0.1), &FindSegment, cellId, uav[cellId].GetUav(i));     
    }
  }
  else if(cellId == 6)
  {

  }
  else
  {
    for(int i = 0 ; i < NUM_UAV; i++)
    {
      Simulator::Schedule(Seconds(0.01*i+0.1), &FindSegment, cellId, uav[cellId].GetUav(i));     
    }  
  }
}
// void InterCell(int cellId)
// {
//     //std::cout<<"intercell "<<cellId<<std::endl;
//   // send to other cells to find free cells
//   dataLoad += 6 * 1 * UAV_PACKET_SIZE;
//   // reply
//   dataLoad += 6 * 1 * UAV_PACKET_SIZE;
//   // only 1 free cell
//   // send data to  free cell;
//   dataLoad += 10 * UAV_PACKET_SIZE;
//   // reply result
//   dataLoad += 3 * UAV_PACKET_SIZE;
//   std::vector<std::vector<int>> sm = SolveCVRP(cellId, uav[cellId].GetUav(0));
//   int n = sm.size();
//   // std::cout<<"num row = "<<n<<std::endl;
//   if(n <= NUM_UAV)
//   {
//     return;
//   }
//   int k = n - NUM_UAV;
//   if(k > NUM_UAV)
//   {
//     k = NUM_UAV;
//   }
//   for(int i = 0; i < k; i++)
//   {
//     Simulator::Schedule(Seconds(0.001*i), &FindSegment, cellId, uav[6].GetUav(i));
//   }
// }
void InterCell(int cellId)
{
  //std::cout<<"intercell "<<cellId<<std::endl;
  vector < Ptr<UAV> > neighbor;
  vector <int> segmentId;
  vector < pair<Ptr<UAV>, int> > result;
  // send to other cells to find free cells
  dataLoad += 6 * 1 * UAV_PACKET_SIZE;
  // reply
  dataLoad += 6 * 1 * UAV_PACKET_SIZE;
  // only 1 free cell
  // send data to  free cell;
  dataLoad += 10 * UAV_PACKET_SIZE;
  // reply result
  dataLoad += 3 * UAV_PACKET_SIZE;
  std::vector<std::vector<int>> sm = SolveCVRP(cellId, uav[cellId].GetUav(0));
  int n = sm.size();
  // std::cout<<"num row = "<<n<<std::endl;
  if(n == 0)
  {
    return;
  }
  int n1 = n - NUM_UAV;//number of segments to be bidded
  if(n1 <= 0)
  {
    return;
  }
  SiteList sitelist[n];
   // std::cout<<"result cell "<<cellId<<" uav "<<u->GetUavId()<<std::endl;
  for(int i = 0; i < n; i++)
  {
    std::vector<int> v = sm[i];
    for(int j = 0; j < (int)v.size(); j++)
    {
      int id = v[j];
      sitelist[i].Add(cell_site_list[cellId].Get(id));
    }
  }
  SiteList sl[n];
  for(int i = 0; i < n; i++)
  {
    segmentId.push_back(i);
    int m = sitelist[i].GetSize();
    if(m > 20)
    {
      std::cout<<"segment "<<i<<" size > 20"<<std::endl;
    }
    sl[i] = LocalTSP(sitelist[i], Vector(X[6], Y[6], height));
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    neighbor.push_back(uav[6].GetUav(i));
  }
  int n2 = neighbor.size();
  if(n1 > n2)
  {
    n1 = n2;
  }
  //std::cout<<"begin while  "<<std::endl;
  while(n1 > 0)
  {
    //std::cout<<"n1 = "<<n1<<std::endl;
    int n3 = neighbor.size(); // 
    int size = (int)segmentId.size();
    double benefit[n3][MAX_SITE_PER_CELL];
    for(int i = 0; i < n3; i++)
    {
      Vector pos = GetPosition(neighbor[i]);
      for(int j = 0; j < size; j++)
      {
        double utility = sl[segmentId[j]].GetExpectedUtility(6);
        double length = sl[segmentId[j]].GetLength(pos);
        double cost = CalculateCost(length);
        double benef = utility - cost;
        benefit[i][j] = benef;
      }
    }
    pair<int, int> rs = FindLargestElement(benefit, n3, size);
    Ptr<UAV> u = neighbor[rs.first];
    int id = segmentId[rs.second];
    result.push_back(make_pair(u, id));
    RemoveValueInVector<Ptr<UAV>>(neighbor, u);
    RemoveValueInVector<int>(segmentId, id);
    //std::cout<<"end n1 = "<<n1<<std::endl;
    n1--;
  }
 // std::cout<<"end while"<<std::endl;
  for(int i = 0; i < (int)result.size(); i++)
  {
    //std::cout<<"i = "<<i<<std::endl;
    Ptr<UAV> u = result[i].first;
    int id = result[i].second;
   // std::cout<<"uav "<<u->GetUavId()<<" segment "<<id<<" size = "<<(int)sl[id].GetSize()<<std::endl;
    AllocateSegment(u, sl[id]);
   // std::cout<<"end allo"<<std::endl;
    int id1 = u->GetCellId();
    int id2 = u->GetUavId();
   // std::cout<<"id1 = "<<id1<<" id2 = "<<id2<<std::endl;
    uavState[id1][id2] = 1;
    DoTask(u);
  }
 // std::cout<<"ket thuc intercell"<<std::endl;
}
void AddSegment(int cellId)
{
  // if(cellId != 0)
  // {
  //   return;
  // }
  //    // find largest segment
  // int idSegment = 999;
  // double maxtime = 0;
  // for(int i = 0; i < numSegment[cellId]; i++)
  // {
  //   if(segmentFlightTime[cellId][i] > maxtime)
  //   {
  //     maxtime = segmentFlightTime[cellId][i];
  //     idSegment = i;
  //   }
  // }
  // int size = segment[cellId][idSegment].GetSize();
  // int half = (int) size/2;
  // if(half > 0)
  // {
  //   int n = numSegment[cellId];
  //   for(int i = 0; i < half; i++)
  //   {
  //     Ptr<SITE> si = segment[cellId][idSegment].Get(0);
  //     segment[cellId][n].Add(si);
  //     segment[cellId][idSegment].Pop();
  //   }
  //   CalculateSegmentFlightTime(cellId, idSegment);
  //   CalculateSegmentFlightTime(cellId, n);
  //   numSegment[cellId]++;
  // }
  // else
  // {
  //   return;
  // } 
}
std::vector<std::vector<int>>
SolveCVRP(int cellId, Ptr<UAV> u)
{
  //std::cout<<"SolveCVRP cell "<<cellId<<" uav "<<uavId<<std::endl;
  std::vector<std::vector<int>> temp;
  int n1 = cell_site_list[cellId].GetSize();
 // std::cout<<"n1 = "<<n1<<std::endl;
  if(n1 == 0)
  {
    return temp;
  }
  std::vector<int> siteId;
  std::vector<Vector> pos;
  std::vector<int64> res;
  res.push_back(0);
  for(int i = 0; i < numberOfSites[cellId]; i++)
  {
    if(siteState[cellId][i] == 1)
    {
      siteId.push_back(i);
      Ptr<SITE> s = cell_site_list[cellId].Get(i);
      Vector p = s -> GetSitePosition();
      pos.push_back(p);
      int r = s -> GetResource();
      res.push_back(r);
    }
  }
  int n = siteId.size();
  if(n == 0)
  {
    return temp;
  }
 // std::cout<<"n = "<<n<<std::endl;
  Vector depotPosition = GetPosition(u);
  std::vector<std::vector<int64>> dist;
  for(int i = 0; i <= n; i++)
  {
    Vector p1;
    if(i == 0)
    {
      p1 = depotPosition;
    }
    else
    {
      p1 = pos[i-1];
    }
    std::vector<int64> row;
    for(int j = 0; j <= n; j++)
    {
      Vector p2;
      if(j == 0)
      {
        p2 = depotPosition;
      }
      else
      {
        p2 = pos[j-1];
      }
      int64 d = (int64)CalculateDistance(p1, p2);
      row.push_back(d);
    }
    dist.push_back(row);
  }
  dist_matrix = dist;
  site_resource = res;
  uav_resource = std::vector<int64>(numberOfSites[cellId], MAX_RESOURCE_PER_UAV);
  nUAV = numberOfSites[cellId];
  temp = operations_research::VrpCapacity();
  dist_matrix.clear();
  site_resource.clear();
  uav_resource.clear();
  std::vector<std::vector<int>> result;
  for(int i = 0; i < (int)temp.size(); i++)
  {
    std::vector<int> row = temp[i];
    std::vector<int> row_result;
    for(int j = 0; j < (int)row.size(); j++)
    {
      int id = row[j];
      int idSite = siteId[id];
      row_result.push_back(idSite);
    }
    result.push_back(row_result);
  }
  return result;
}
void FindSegment(int cellId, Ptr<UAV> u)
{
 // std::cout<<"find segment cell "<<cellId<<" uav "<<uavId<<std::endl;
  std::vector< std::vector<int> > sm = SolveCVRP(cellId, u);
    int n = sm.size();
   // std::cout<<"num row = "<<n<<std::endl;
    if(n == 0)
    {
      numSegment[cellId] = 0;
      return;
    }
    int sz = cell_site_list[cellId].GetSize();
    SiteList sl[sz];
   // std::cout<<"result cell "<<cellId<<" uav "<<u->GetUavId()<<std::endl;
    for(int i = 0; i < n; i++)
    {
      std::vector<int> v = sm[i];
        for(int j = 0; j < (int)v.size(); j++)
        {
          int id = v[j];
         // std::cout<<id<<" ";
          sl[i].Add(cell_site_list[cellId].Get(id));
        }
       // std::cout<<std::endl;
    }
    numSegment[cellId] = n; 
    int id = 0;
    double maxUti = sl[0].GetExpectedUtility(cellId);
    for(int i = 1; i < n; i++)
    {
      double expectedUti = sl[i].GetExpectedUtility(cellId);
      if(expectedUti > maxUti)
      {
        maxUti = expectedUti;
        id = i;
      }
    }
    //
    AllocateSegment(u, sl[id]);
    DoTask(u); 
}
void AllocateSegment(Ptr<UAV> u, SiteList sl)
{
  int uavId = u -> GetUavId();
  int cellId = u -> GetCellId();
  uavState[cellId][uavId] = 1;
 // std::cout<<GetNow()<<": allocate segment "<<" for uav "<<uavId<<" cell "<<cellId<<std::endl;
  double uti1 = sl.GetExpectedUtility(cellId);
  double uti2 = sl.GetReverseUtility(cellId);
  int size = (int)sl.GetSize();
 // std::cout<<"befor"<<std::endl;
  if(uti1 > uti2)
  { 
    for(int i = 0; i < size; i++)
    {
      Ptr<SITE> s = sl.Get(i);
      u -> AddSite(s);
      int id = s -> GetId();
      ChangeSiteState(s->GetCellId(), id, 2);
    }
  }
  else
  {
    for(int i = size-1; i >= 0; i--) 
    {
      Ptr<SITE> s = sl.Get(i);
      u -> AddSite(s);
      int id = s -> GetId();
      ChangeSiteState(s->GetCellId(), id, 2);
    }
  }
 // std::cout<<"after"<<std::endl;
  //sl.Clear();
}
void FreeSite(Ptr<UAV> u)
{
 //std::cout<<"free site cell "<<u->GetCellId()<<" uav "<<u->GetUavId()<<std::endl;
  int n = u -> GetSiteSize();
  for(int i = 0; i < n; i++)
  {
    Ptr<SITE> s = u -> GetSite();
    u -> RemoveSite();
    int siteId = s -> GetId();
    int cellId = s -> GetCellId();
    ChangeSiteState(cellId, siteId, 1);
  }
}
void DoTask(Ptr<UAV> u)
{
  int uavId = u -> GetUavId();
  int cellId = u -> GetCellId();
 // std::cout<<GetNow()<<": dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
  if((cellId == 0 || cellId == 6) && numScenario == 0)
  {
    AddPath(uavId, cellId);
  }
  int uavResource = u -> GetResource();
  if(u->GetSiteSize() == 0 || uavResource == 0)
  {
   // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    if(uavResource == 0)
    {
      FreeSite(u);
    }
    double flightTime = Goto(u, GetPosition(gw[cellId].Get(0)));    
    u -> UpdateFlightTime(flightTime);
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    if((cellId == 0 || cellId == 6) && numScenario == 0)
    {
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId, cellId);
    }
    Simulator::Schedule(Seconds(flightTime), &CheckCellFinish, u);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  Ptr<SITE> s = u->GetSite();
  u->RemoveSite();
 // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<s->GetId()<<std::endl;  
  double flightTime = Goto(u, s -> GetSitePosition());
  ChangeSiteState(u->GetCellId(), s->GetId(), 3);
  int resource = s -> GetRealResource(); 
  int deltaResource = s -> GetDeltaResource();
  double visitedTime = s -> GetVisitedTime();
  double rate;
  int nextState = 4;
  if(uavResource < resource)
  {
    nextState = 1;
    u -> SetResource(0);
    rate = (double)uavResource/resource;
    s -> SetResource(resource - uavResource);
    s -> SetVisitedTime(visitedTime*(1.0 - rate));
    visitedTime *= rate;
  }
  else
  {
    u -> SetResource(uavResource - resource);
  }
  if(deltaResource > 0)
  {
    s -> SetDeltaResource(0);
    SendPacket(u, gw[cellId].Get(0), UAV_NUM_PACKET, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
  }
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  u->UpdateFlightTime(flightTime + visitedTime);
  double begintime = GetNow() + flightTime;
  double endtime = GetNow() + flightTime + visitedTime;
  s->SetBeginTime(begintime);
  s->SetEndTime(endtime);
  double utility = s->GetRealUtility();
  if(deltaResource > 0 && (uavResource < resource))
  {
    utility *= rate;
    s -> SetUtility(s -> GetUtility() * (1.0 - rate));
  }
  UpdateUtility(begintime, endtime, utility/10.0/(endtime-begintime));
  if((cellId == 0 || cellId == 6) && numScenario == 0)
  {
    Simulator::Schedule(Seconds(flightTime), &AddPath, uavId, cellId);
  }
  Simulator::Schedule(Seconds(flightTime+visitedTime), &ChangeSiteState, s->GetCellId(), s->GetId(), nextState);
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
}
void NextRound(Ptr<UAV> u)
{
  if(SiteCheck(u->GetCellId()) == 0)
  {
    u -> SetResource(MAX_RESOURCE_PER_UAV);
    FindSegment(u->GetCellId(), u);
  }
}
void CheckCellFinish(Ptr<UAV> u)
{
  int cellId = u -> GetCellId();
  int uavId = u -> GetUavId();
 // std::cout<<GetNow()<<": next round cell "<<cellId<<", uav "<<uavId<<std::endl;
  uavState[cellId][uavId] = 0;
  if(IsFinish(cellId))
  {
    finish[cellId] = 1;
    if(IsFinish())
    {
      EndScenario();
    }
  }
  else
  {
    EventId ev = Simulator::Schedule(Seconds(60*INTERVAL_BETWEEN_TWO_ROUNDS), &NextRound, u);
    event.push_back(ev);
  }
}
int IsFinish()
{
  for(int i =0 ; i < NUM_CELL; i++)
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
     // std::cout<<"cell "<<i<<" chua xong"<<std::endl;
      return 0;
    }
  }
  return 1;
}
int SiteCheck(int cellId)
{
  for(int i = 0; i < numberOfSites[cellId]; i++)
  {
    if(siteState[cellId][i] == 1)
    {
      return 0;
    }
    else if(siteState[cellId][i] == 2)
    {
      return 0;
    }
    else if(siteState[cellId][i] == 3)
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
    if(uavState[cellId][i] == 1)
    {
      return 0;
    }
  }
  return SiteCheck(cellId);
}
void Reset()
{
  for(int i = 0; i < NUM_CELL; i++)
  {
    if(i == NUM_CELL-1)
    {
      finish[i] = 1;
    }
    else
    {
      finish[i] = 0;
    }
    cell_site_list[i].Clear();
    dataLoad = 0;
    uti = 0;
    for(int j = 0; j < NUM_UAV; j++)
    {
      Ptr<UAV> u = uav[i].GetUav(j);
      double energy = u -> GetConsumedEnergy();
      u -> IncreaseEnergy(-energy);
      double flightTime = u -> GetFlightTime();
      u -> UpdateFlightTime(-flightTime);
      double distance = u -> GetFliedDistance();
      u -> UpdateFliedDistance (-distance);
      u -> SetResource(MAX_RESOURCE_PER_UAV);
    }
  }
}
void EndScenario()
{
  std::cout<<"end scenario "<<numScenario<<std::endl;
  for(int i = 0; i < (int)event.size(); i++)
  {
    EventId ev = event[i];
    if(!Simulator::IsExpired(ev))
    {
      Simulator::Cancel(ev);
    }
  }
  event.clear();
  double Tend = GetNow();
  double energy = 0;
  double fliedDistance = 0;
  double flightTime = 0;
  for(int i = 0; i < NUM_CELL; i++)
  {
    energy += uav[i].CalculateEnergyConsumption()/1000000.0;
    fliedDistance += uav[i].CalculateFliedDistance()/1000.0;
    flightTime += uav[i].CalculateFlightTime()/3600.0;
  }

  double cost = CalculateCost(fliedDistance);
  cumulativeSpanningTime += (Tend - Tbegin)/60.0;
 // std::cout<<"spanning time = "<<
  cumulativeFlightTime += flightTime;
  cumulativeEnergy += energy;
  cumulativeFlightDistance += fliedDistance;
  cumulativeUtility += uti;
  cumulativeBenefit += uti - cost;
  cumulativeData += dataLoad/1024.0/1024.0;
  std::cout<<"spanning time = "<<(Tend - Tbegin)/60.0<<std::endl;
  std::cout<<"flightTime = "<<flightTime<<std::endl;
  std::cout<<"energy = "<<energy<<std::endl;
  std::cout<<"fliedDistance = "<<fliedDistance<<std::endl;
  std::cout<<"benefit = "<<uti - cost<<std::endl;
  std::cout<<"data = "<<dataLoad/1024.0/1024.0<<std::endl;
  numScenario++;
  if(numScenario == NUM_SCENARIO)
  {
    StopSimulation();
    return;
  }
  else
  {
    Reset();
    NewScenario();
  }
}
void StopSimulation()
{
  std::cout<<"Intercell R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
  std::cout<<"Spanning time: "<<cumulativeSpanningTime/NUM_SCENARIO<<" m"<<std::endl;
  std::cout<<"Flight time: "<<cumulativeFlightTime/NUM_SCENARIO<<" h"<<std::endl;
  std::cout<<"Energy: "<<cumulativeEnergy/NUM_SCENARIO<<" MJ"<<std::endl;
  std::cout<<"Flied distance: "<<cumulativeFlightDistance/NUM_SCENARIO<<" km"<<std::endl;
  std::cout<<"Benefit: "<<cumulativeBenefit/NUM_SCENARIO<<std::endl;
  std::cout<<"Data: "<<cumulativeData/NUM_SCENARIO<<" MB"<<std::endl;
  CreateUtilityPlot();
  CreateNumUAVPlot();
  CreatePathPlot();
  Simulator::Stop();
}