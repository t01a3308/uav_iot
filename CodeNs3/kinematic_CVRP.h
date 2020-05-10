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
//#include "handle.h"
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
//
int numberOfSites[NUM_CELL-1];
// TSP 
//
int uavState[NUM_CELL][NUM_UAV];
int siteState[NUM_CELL][MAX_SITE_PER_CELL]; // 0 - not a site, 1 - new site, 2 - assigned to UAV, 3 - being handled, 4 - done
SiteList cell_site_list[NUM_CELL];
double segmentFlightTime[NUM_CELL][MAX_SITE_PER_CELL];
double segmentDistance[NUM_CELL][MAX_SITE_PER_CELL];
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
double cumulativeBenefit = 0;
double cumulativeData = 0;
double uti = 0;
//
Gnuplot2dDataset p[NUM_UAV];
Gnuplot2dDataset utidts, uavdts;
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
void ChangeSiteState(int cellId, int siteId, int state);

Ptr<SITE> NewSite();
void CreateSite();
void EndScenario();
void StopSimulation();
void Reset();
void NewScenario();
//functions to solve TSP

//
double CalculateCost(double distance);
void Execute(int cellId);
std::vector<std::vector<int>> SolveCVRP(int cellId, Ptr<UAV> u);
void FindSegment(int cellId, Ptr<UAV> u);
void AllocateSegment(Ptr<UAV> u, SiteList sl);
void FreeSite(Ptr<UAV> u);
void DoTask(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);
int IsFinish();
int SiteCheck(int cellId);
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
    }
  }
  uavdts.Add(GetNow(), n);
  Simulator::Schedule(Seconds(0.5), &NumUAVPlot);
}
void CreateUtilityPlot ()
{
  std::string fileNameWithNoExtension = "utility/CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
  std::string fileNameWithNoExtension = "numUAV/CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
  std::string fileNameWithNoExtension = "path/path_CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "CVRP_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
      //anim->UpdateNodeSize(uav[cellId].Get(i)->GetId(), 200, 200);
    //anim->UpdateNodeColor(uav[cellId].Get(i)->GetId(), 0, 0, 255);
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
  //std::cout<<"execute cell "<<cellId<<std::endl;
  for(int i = 0; i < NUM_UAV; i++)
  {
    Simulator::Schedule(Seconds(0.01*i), &FindSegment, cellId, uav[cellId].GetUav(i)); 
  }
}
void CalculateSegmentFlightTime(int cellId, int i) // i is segmentId
{
	// segmentFlightTime[cellId][i] = 0;
 //  segmentDistance[cellId][i] = 0;
 //    int n = segment[cellId][i].GetSize();
 //    //
 //    Ptr<SITE> first = segment[cellId][i].Get(0);
 //    Vector pos = first->GetSitePosition();
 //    double dt = CalculateDistance(pos, GetPosition(gw[cellId].Get(0)));
 //    segmentDistance[cellId][i] += dt;
 //    double t = dt/VUAV;
 //    segmentFlightTime[cellId][i] += t;
 //    //
 //    Ptr<SITE> last = segment[cellId][i].Get(n-1);
 //    Vector pos1 = last->GetSitePosition();
 //    double dt1 = CalculateDistance(pos1, GetPosition(gw[cellId].Get(0)));
 //    segmentDistance[cellId][i] += dt1;
 //    double t1= dt1/VUAV;
 //    segmentFlightTime[cellId][i] += t1;
 //    for(int j = 0; j < n; j++)
 //    {
 //      Ptr<SITE> s1 = segment[cellId][i].Get(j);
 //      double time = s1->GetVisitedTime();
 //      segmentFlightTime[cellId][i] += time;
 //      if(j != n-1)
 //      {
 //        Ptr<SITE> s2 = segment[cellId][i].Get(j+1);
 //        Vector p1 = s1->GetSitePosition();
 //        Vector p2 = s2->GetSitePosition();
 //        double d = CalculateDistance(p1, p2);
 //        segmentDistance[cellId][i] += d;
 //        double time1 = d/VUAV;
 //        segmentFlightTime[cellId][i] += time1;
 //      }
 //    }
 //    segmentFlightTime[cellId][i] /= 60.0;
 //    segmentDistance[cellId][i] /= 1000.0;
}
void CalculateSegmentFlightTime(int cellId)
{
  for(int i = 0; i < numSegment[cellId]; i++)
  {
    CalculateSegmentFlightTime(cellId, i);
  }
}
void PrintSegmentInformation(int cellId)
{
  // std::cout<<"cell "<<cellId<<std::endl;
  // double d = 0;
  // for(int i = 0; i < numSegment[cellId]; i++)
  // {
  //   std::cout<<"segment "<<i<<": ";
  //   for(int j = 0; j < (int)segment[cellId][i].GetSize(); j++)
  //   {
  //     Ptr<SITE> s = segment[cellId][i].Get(j);
  //     int id = s->GetId();
  //     std::cout<<id<<" ";
  //   }
  //   d += segmentDistance[cellId][i];
  //   std::cout<<" flightTime: "<<segmentFlightTime[cellId][i]<<" m,"<<" distance: "<<segmentDistance[cellId][i]<<" km"<<std::endl;
  // }
  // std::cout<<"total distance: "<<d<<std::endl;
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
void AllocateSegment(Ptr<UAV> u, SiteList sl)
{
  int uavId = u -> GetUavId();
  int cellId = u -> GetCellId();
  uavState[cellId][uavId] = 1;
 // std::cout<<"allocate segment "<<id<<" for uav "<<uavId<<std::endl;
  double uti1 = sl.GetExpectedUtility(cellId);
  double uti2 = sl.GetReverseUtility(cellId);
  int size = (int)sl.GetSize();
  if(uti1 > uti2)
  {
    for(int i = size-1; i >= 0; i--)  
    {
      Ptr<SITE> s = sl.Get(i);
      u -> AddSite(s);
      int id = s -> GetId();
      ChangeSiteState(s->GetCellId(), id, 2);
    }
  }
  else
  {
    for(int i = 0; i < size; i++)
    {
      Ptr<SITE> s = sl.Get(i);
      u -> AddSite(s);
      int id = s -> GetId();
      ChangeSiteState(s->GetCellId(), id, 2);
    }
  }
  sl.Clear();
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
    SiteList sl[n];
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
  if(cellId == 0 && numScenario == 0)
  {
    AddPath(uavId);
  }
  int uavResource = u -> GetResource();
  if(u -> GetSiteSize() == 0 || uavResource == 0)
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
    if(cellId == 0 && numScenario == 0)
    {
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
    }

    Simulator::Schedule(Seconds(flightTime), &CheckCellFinish, u);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  Ptr<SITE> s = u->GetSite();
  u->RemoveSite();
  //std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<s->GetId()<<std::endl; 
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
  if(cellId == 0 && numScenario == 0)
  {
    Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
  }
  Simulator::Schedule(Seconds(flightTime + visitedTime), &ChangeSiteState, s->GetCellId(), s->GetId(), nextState);
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
   // utility += cell_site_list[i].GetUtility();
    flightTime += uav[i].CalculateFlightTime()/3600.0;
  }
  double cost = CalculateCost(fliedDistance);
  cumulativeSpanningTime += (Tend - Tbegin)/60.0;
 // std::cout<<"spanning time = "<<
  cumulativeFlightTime += flightTime;
  cumulativeEnergy += energy;
  cumulativeFlightDistance += fliedDistance;
  cumulativeBenefit += uti - cost;
  cumulativeData += dataLoad/1024.0/1024.0;
  std::cout<<"spanning time = "<<(Tend - Tbegin)/60.0<<std::endl;
  std::cout<<"flightTime = "<<flightTime<<std::endl;
  std::cout<<"energy = "<<energy<<std::endl;
  std::cout<<"fliedDistance = "<<fliedDistance<<std::endl;
  std::cout<<"benefit = "<<uti - cost<<std::endl;
  std::cout<<"data = "<<dataLoad/1024.0/1024.0<<std::endl;
  Reset();
  numScenario++;
  if(numScenario == NUM_SCENARIO)
  {
    StopSimulation();
    return;
  }
  else
  {
    NewScenario();
  }
}
void StopSimulation()
{
  std::cout<<"CVRP R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
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