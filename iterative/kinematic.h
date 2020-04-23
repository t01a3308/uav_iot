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
#include "macro_param.h"
//#include "handle.h"
#include "communication.h"
using namespace ns3;
extern AnimationInterface *anim;
extern double dataLoad;
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
//
int siteData[MAX_SITE_PER_CELL*NUM_CELL];
int numberOfSites[NUM_CELL-1];
//
int uavState[NUM_CELL][NUM_UAV];
SiteList cell_site_list[NUM_CELL], temp[NUM_CELL];
int finish[NUM_CELL];
int numSite[NUM_CELL];
std::vector<int> completedSites[NUM_CELL];
typedef std::pair<double, double> myPair;
typedef std::pair<int, std::string> myPair1;
typedef std::pair<myPair1, myPair> mp;
//std::vector<mp> workInfor[NUM_CELL];
double length0 = 0;
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
void CreateSite();
//
double CalculateCost(double distance);
void Execute(int cellId);
void FindSite(Ptr<UAV> u);
int CalculateNumberOfSites(int cellId, int uavId);
void DoTask(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);
int IsFinish();
int IsFinish(int cellId);
void EndScenario();
void StopSimulation();
void Reset();
void NewScenario();
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
  std::string fileNameWithNoExtension = "utility/iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
  std::string fileNameWithNoExtension = "numUAV/iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
  std::string fileNameWithNoExtension = "path/iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = "iterative_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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
void CreateSite()
{
  //create site
 // std::cout<<"Creating sites"<<std::endl;
  finish[NUM_CELL-1] = 1;
  Ptr<UniformRandomVariable> rd = CreateObject<UniformRandomVariable>();
  for(int i = 0; i < NUM_CELL - 1; i++)
  {
   // std::cout<<"cell "<<i<<" has "<<numberOfSites[i]<<" sites"<<std::endl;
    finish[i] = 0;
    for(int j = 0; j < numberOfSites[i]; j++)
    {
      int data = (int) rd -> GetValue(MIN_VALUE, MAX_VALUE);
      double radius = rd -> GetValue(0.0, CELL_RADIUS/2.0*std::sqrt(3));
      double theta = rd -> GetValue(0.0, 2*PI);
      double x = X[i] + radius*std::cos(theta);
      double y = Y[i] + radius*std::sin(theta);
      Vector pos = Vector(x, y, height);
      SetPosition(sensor[i].Get(j), pos);
      Ptr<SITE> s = CreateObject<SITE>(pos, data, j);
      cell_site_list[i].Add(s);
      temp[i].Add(s);
    //  anim->UpdateNodeSize(sensor[i].Get(j)->GetId(), 100, 100);
     // anim->UpdateNodeColor(sensor[i].Get(j)->GetId(), 0, 255, 0);
    }
  }
  for(int cellId = 0; cellId < NUM_CELL-1; cellId++)
  {
    int n = cell_site_list[cellId].GetSize();
    if(n == 0)
    {
      continue ;
    }
    Vector pos[n];  
    for (int i = 0; i < n; i ++)
    {
      pos[i] = cell_site_list[cellId].Get(i)->GetSitePosition();
    }
  	double dist[numberOfSites[cellId]+1][numberOfSites[cellId]+1];
    dist[0][0] = 0;
    for(int k1 = 1; k1 <= n; k1++)
    {
      dist[0][k1] = dist[k1][0] = CalculateDistance(pos[k1-1], GetPosition(gw[cellId].Get(0)));
      for(int k2 = 1; k2 <= n; k2++)
      {
        dist[k1][k2] = CalculateDistance(pos[k1-1], pos[k2-1]);
      }
    }
  // write dist matrix and resource of sites 
    std::string filename = "input_cvrp/dist_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    ofstream myfile;
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0; i <= n; i++)
    {
      for(int j = 0; j <= n; j++)
      {
        int d = (int)dist[i][j];   
        if(j == n)
        {
          myfile<<d<<"\n";
        }
        else
        {
          myfile<<d<<" ";
        }
      }
    }
    myfile.close();
    filename = "input_cvrp/demand_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    myfile<<0<<" ";
    for(int i = 0 ; i < n; i++)
    {
      double resource = cell_site_list[cellId].Get(i)->GetResource();
      myfile<<(int)resource<<" ";
    }
    myfile.close();

    // scenario to file
    filename = "scenario/posX_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double posX = cell_site_list[cellId].Get(i)->GetSitePosition().x;
      myfile<<posX<<" ";
    }
    myfile.close();

    filename = "scenario/posY_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double posY = cell_site_list[cellId].Get(i)->GetSitePosition().y;
      myfile<<posY<<" ";
    }
    myfile.close();

    filename = "scenario/visitedtime_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double time = cell_site_list[cellId].Get(i)->GetVisitedTime();
      myfile<<time<<" ";
    }
    myfile.close();

    filename = "scenario/resource_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double resource = cell_site_list[cellId].Get(i)->GetResource();
      myfile<<resource<<" ";
    }
    myfile.close();

    filename = "scenario/utility_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double utility = cell_site_list[cellId].Get(i)->GetUtility();
      myfile<<utility<<" ";
    }
    myfile.close();

    filename = "scenario/urgency_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
    myfile.open(filename, std::ios::out | std::ios::trunc);
    for(int i = 0 ; i < n; i++)
    {
      double urgency = cell_site_list[cellId].Get(i)->GetUrgency();
      myfile<<urgency<<" ";
    }
    myfile.close();
  }
}
void ReadScenario()
{
	for(int cellId = 0; cellId < NUM_CELL-1; cellId++)
	{
		std::vector<double> posX;
		std::string filename = "scenario/posX_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, posX);

		std::vector<double> posY;
		filename = "scenario/posY_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, posY);

		std::vector<double> visitedtime;
		filename = "scenario/visitedtime_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, visitedtime);

		std::vector<double> resource;
		filename = "scenario/resource_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, resource);
		
		std::vector<double> utility;
		filename = "scenario/utility_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, utility);

		std::vector<double> urgency;
		filename = "scenario/urgency_"+std::to_string(cellId)+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV)+"_"+std::to_string(numScenario)+".txt";
		ReadData(filename, urgency);
		//std::cout<<"xong read"<<std::endl;

		for(int i = 0; i < numberOfSites[cellId]; i++)
		{
			Vector pos = Vector(posX[i], posY[i], height);
      SetPosition(sensor[cellId].Get(i), pos);
      Ptr<SITE> s = CreateObject<SITE>(pos, visitedtime[i], (int)resource[i], utility[i], urgency[i], i);
      cell_site_list[cellId].Add(s);
      temp[cellId].Add(s);
		}
	//	std::cout<<"cell "<<cellId<<" has "<<numberOfSites[cellId]<<" sites"<<std::endl;
		posX.clear();
		posY.clear();
		visitedtime.clear();
		resource.clear();
		utility.clear();
		urgency.clear();
	}
}
void NewScenario()
{
  
  Tbegin = GetNow();
  //CreateSite();
  ReadScenario();
  for(int i = 0; i < NUM_CELL; i++)
  {
    Execute(i);
  }
}
double CalculateCost(double distance)
{
  return Rd*distance;
}
void Execute(int cellId)
{ 
 // std::cout<<"execute cell "<<cellId<<std::endl;

  numSite[cellId] = 0;
  for(int i = 0; i < NUM_UAV; i++)
  {
   // anim->UpdateNodeSize(uav[cellId].Get(i)->GetId(), 200, 200);
   // anim->UpdateNodeColor(uav[cellId].Get(i)->GetId(), 0, 0, 255);
    uavState[cellId][i] = 0;
  }
  for(int i = 0 ; i < NUM_UAV; i++)
  {
    Simulator::Schedule(Seconds(0.01*i), &DoTask, uav[cellId].GetUav(i));
  }  
}
void FindSite(Ptr<UAV> u)
{
  int cellId = u -> GetCellId();
 // int uavId = u -> GetUavId();
 // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" find site "<<std::endl;
  int n = temp[cellId].GetSize();
  if(n == 0)
  {
  //  std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" khong co site "<<std::endl;
    return;
  }
  Vector pos = GetPosition(u);
  double distance[n];
  for(int i = 0; i < n; i++)
  {
    Ptr<SITE> s = temp[cellId].Get(i);
    Vector p = s->GetSitePosition();
    distance[i] = CalculateDistance(pos, p);
  }
  double currentResource = u -> GetResource();
  int rank = 0;
  int flag = 0;
  while(flag == 0)
  {
    if(rank >= n)
    {
     // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" khong tim duoc site "<<std::endl;
      return;
    }
    int id = FindId(distance, n, rank);
    Ptr<SITE> s = temp[cellId].Get(id);
    if(s -> GetResource() > currentResource)
    {
     // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" het resource"<<std::endl;
      return;
    }
    // uav send to site
    dataLoad += 3*UAV_PACKET_SIZE;
    u -> IncreaseEnergy(3*ENERGY_SEND);
    //site reply uav
    dataLoad += 3*UAV_PACKET_SIZE;
    u -> IncreaseEnergy(3*ENERGY_RECEIVE);
    int status = s->GetStatus();
    if(status == 0)
    {
     // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" select site "<<s->GetId()<<std::endl;
      s -> AddBidder(u, distance[id]);
      s -> SetStatus(1);
      u -> AddSite(s);
      flag = 1;      
      return;
    }
    else
    {
      if(s -> CheckPrice(u, distance[id]))
      {
       // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" win site "<<s->GetId();
        Ptr<UAV> bd = s -> GetBidder();
       // std::cout<<" from uav "<<bd->GetUavId()<<std::endl;
        // site send to loser
        dataLoad += 3*UAV_PACKET_SIZE;
        bd -> IncreaseEnergy(3*ENERGY_RECEIVE);
        //
        s -> RemoveBidder();
        s -> AddBidder(u, distance[id]);
        bd -> RemoveSite();
        u -> AddSite(s);
        flag = 1;        
        return;
      }
      else
      {
        rank++;
      }
    }
    
  }
}
void ChangeColor(int cellId, int sensorId)
{
  //anim->UpdateNodeColor(sensor[cellId].Get(sensorId)->GetId(), 255, 0, 0);
}
void DoTask(Ptr<UAV> u)
{
  int cellId = u -> GetCellId();
  int uavId = u -> GetUavId();
  if(cellId == 0)
  {
    AddPath(uavId);
  }
  if(u -> GetSiteSize() == 0)
  {
    FindSite(u);
  }
  if(u->GetSiteSize() == 0)
  {
    //std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    double flightTime = Goto(u, GetPosition(gw[cellId].Get(0)));  
 
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    u -> UpdateFlightTime(flightTime);
    if(cellId == 0)
    {
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
      length0 += VUAV*flightTime;
    }
    Simulator::Schedule(Seconds(flightTime), &NextRound, u);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  numSite[cellId]++;
  uavState[cellId][uavId] = 1;
  Ptr<SITE> s = u->GetSite();
  completedSites[cellId].push_back(s->GetId());
  double newResource = u -> GetResource() - s -> GetResource();
  if(newResource < 0)
  {
    std::cout<<"error stopsim"<<std::endl;
    Simulator::Stop();
  }
  u -> SetResource(newResource);
 // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" pos = "<<GetPosition(u)<<" go to site "<<s->GetId()<<" pos = "<<s->GetSitePosition()<<std::endl;
  double flightTime = Goto(u, s -> GetSitePosition());
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  u->RemoveSite();
  temp[cellId].Remove(s);
  
  double visitedTime = s -> GetVisitedTime();
  u->UpdateFlightTime(flightTime + visitedTime);
  double begintime = GetNow() + flightTime;
  double endtime = GetNow() + flightTime + visitedTime;
  s->SetBeginTime(begintime);
  s->SetEndTime(endtime);
  double utility = s->GetRealUtility();
  UpdateUtility(begintime, endtime, utility/10.0/(endtime-begintime));
 // myPair1 m1 = std::make_pair(s->GetId(), std::to_string(cellId) + std::to_string(uavId));
 // myPair m2 = std::make_pair(GetNow() + flightTime, GetNow() + flightTime + visitedTime);
  //workInfor[cellId].push_back(std::make_pair(m1, m2));
  if(cellId == 0)
  {
    Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
    length0 += VUAV*flightTime;
  }
  Simulator::Schedule(Seconds(flightTime+visitedTime), &ChangeColor, cellId, s->GetId());
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime), &FindSite, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
 // std::cout<<"ket thuc ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
}
void NextRound(Ptr<UAV> u)
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
    u -> SetResource(MAX_RESOURCE_PER_UAV);
    if(temp[cellId].GetSize())
    {
      Simulator::Schedule(Seconds(60*INTERVAL_BETWEEN_TWO_ROUNDS), &DoTask, u);
    }
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
int IsFinish(int cellId)
{
 // std::cout<<"goi ham IsFinish"<<std::endl;
  if(temp[cellId].GetSize() > 0)
  {
   // std::cout<<"size > 0"<<std::endl;
    return 0;
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(uavState[cellId][i] == 1)
    {
     // std::cout<<"uav "<<i<<" chua xong"<<std::endl;
      return 0;
    }
  }
  return 1;
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
   // workInfor[i].clear();
    completedSites[i].clear();
    cell_site_list[i].Clear();
    temp[i].Clear();
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

  double Tend = GetNow();
  double energy = 0;
  double fliedDistance = 0;
  double flightTime = 0;
  double utility = 0;
  for(int i = 0; i < NUM_CELL; i++)
  {
    //std::cout<<i<<": "<<numSite[i]<<std::endl;
    energy += uav[i].CalculateEnergyConsumption()/1000000.0;
    fliedDistance += uav[i].CalculateFliedDistance()/1000.0;
    utility += cell_site_list[i].GetRealUtility();
    flightTime += uav[i].CalculateFlightTime()/3600.0;
  }
  // for(int i = 0; i < NUM_CELL; i++)
  // {
  //   std::cout<<"cell "<<i<<" has "<<(int)completedSites[i].size()<<" sites: ";
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
  
  double cost = CalculateCost(fliedDistance);
  cumulativeSpanningTime += (Tend - Tbegin)/60.0;
 // std::cout<<"spanning time = "<<
  cumulativeFlightTime += flightTime;
  cumulativeEnergy += energy;
  cumulativeFlightDistance += fliedDistance;
  cumulativeUtility += uti;
  cumulativeBenefit += utility - cost;
  cumulativeData += dataLoad/1024.0/1024.0;
  std::cout<<"spanning time = "<<(Tend - Tbegin)/60.0<<std::endl;
  std::cout<<"flightTime = "<<flightTime<<std::endl;
  std::cout<<"energy = "<<energy<<std::endl;
  std::cout<<"fliedDistance = "<<fliedDistance<<std::endl;
  std::cout<<"utility = "<<utility<<std::endl;
  std::cout<<"benefit = "<<utility - cost<<std::endl;
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

  std::cout<<"Iterative R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
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
