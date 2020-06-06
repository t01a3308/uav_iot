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
#include "ns3/mobility-module.h"
#include "ns3/gnuplot.h"
#include <iostream>
#include "communication.h"

extern AnimationInterface *anim;
extern int useAnim;
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
extern std::string method;
extern void FindSegment(int, Ptr<UAV>);
extern void DoTask(Ptr<UAV>);
extern void Execute(int);
extern void InterCell();
double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
int finish[NUM_CELL];
std::vector<EventId> event;
SiteList cell_site_list[NUM_CELL];
int numberOfSites[NUM_CELL];
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
//
Gnuplot2dDataset p[NUM_UAV], p1[NUM_UAV];
Gnuplot2dDataset utidts, uavdts;
int uavState[NUM_CELL][NUM_UAV]; // 0 - free, 1 - busy, 2 - recharging
int siteState[NUM_CELL][MAX_SITE_PER_CELL]; // 0 - not a site, 1 - new site, 2 - assigned to UAV, 3 - being handled, 4 - done
// declare
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

double CalculateCost(double distance);
int IsFinish();
int SiteCheck(int cellId);
int IsFinish(int cellId);
void EndScenario();
void StopSimulation();
void CreateUtilityPlot();
void CreateNumUAVPlot();
void CreatePathPlot();
// definition
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
  for(int i = 1; i < NUM_CELL; i++)
  {
    numberOfSites[i] = 0;
  }
  int rate[NUM_CELL-1] = {17, 10, 10, 10, 10, 10};
  int b = 0;
  for(int i = 1; i < NUM_CELL; i++)
  {
  	b += rate[i-1];
  }
  int n = (int)TOTAL_SITE/b;
  for(int i = 1; i < NUM_CELL; i++)
  {
  	numberOfSites[i] += n*rate[i-1];
  }
  int remainder = TOTAL_SITE - n*b;
  if(remainder == 0)
  {
  	return;
  }
  int id = 1;
  while(remainder > 0)
  {
  	numberOfSites[id]++;
  	remainder--;
  	id++;
  	if(id == NUM_CELL)
  	{
  		id = 1;
  	}
  }
  for(int i = 1; i < NUM_CELL; i++)
  {
  	std::cout<<i<<": "<<numberOfSites[i]<<std::endl;
  }
}
void ChangeUAVState(int cellId, int uavId, int state)
{
  uavState[cellId][uavId] = state;

}
void ChangeSiteState(int cellId, int siteId, int state)
{
  if(finish[cellId] == 1)
  {
    return;
  }
  siteState[cellId][siteId] = state;
  //if(method == "intercell" || method == "CVRP")
  {
  	if(state == 1 && (GetNow() > Tbegin + 10))
  	{
   // std::cout<<GetNow()<<" newsite cell "<<cellId<<" site "<<siteId<<std::endl;
    	for(int i = 0; i < NUM_UAV; i++)
    	{
      	if(uavState[cellId][i] == 0)
      	{
          if(method == "iterative")
          {
            DoTask(uav[cellId].GetUav(i));
           //std::cout<<"dr uav "<<i<<std::endl;
          }
          else
          {
        	  FindSegment(cellId, uav[cellId].GetUav(i));
          }
        	break;
      	}
    	}
  	}
  }
  if(useAnim == 0)
  {
  	return;
  }
  if(state == 0)
  {
    anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 1, 1);
    anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 0);
  }
  else if(state == 1)
  {
    anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
    anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 0);
  }
  else if(state == 2)
  {
    anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
    anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 255, 255, 0);
  }
  else if(state == 3)
  {
    anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 100, 100);
    anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 255, 0, 0);
  }
  else
  {
    anim->UpdateNodeSize(sensor[cellId].Get(siteId)->GetId(), 1, 1);
    anim->UpdateNodeColor(sensor[cellId].Get(siteId)->GetId(), 0, 255, 255);
  }

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
  EventId ev = Simulator::Schedule(Seconds(delta_appear_time), &ChangeSiteState, cellId, siteId, 1);
  if(delta_appear_time > 0)
  {
    event.push_back(ev);
  }
  return s;
}
void CreateSite()
{

  finish[0] = 1;
  
  for(int i = 1; i < NUM_CELL; i++)
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
      if(useAnim == 1)
      {
      	int red = 0, green = 0, blue = 255;
      	if(i == 0)
      	{
      		red = 255;
      		green = 255;
      		blue = 0;
      	}
      	anim->UpdateNodeSize(uav[i].Get(j)->GetId(), 200, 200);
      	anim->UpdateNodeColor(uav[i].Get(j)->GetId(), red, green, blue);
      }
    }
  }
  Tbegin = GetNow();
  CreateSite();
  Simulator::Schedule(Seconds(0.0001), &InterCell);
  for(int i = 1; i < NUM_CELL; i++)
  {
    Simulator::Schedule(Seconds(0.001*i + 0.001), &Execute, i);
  }
}
double CalculateCost(double distance)
{
  return distance*Rd;
}
int IsFinish()
{
  for(int i = 0 ; i < NUM_CELL; i++)
  {
    if(finish[i] == 0)
    {
      if(IsFinish(i) == 0)
      {
        return 0;
      }
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
  int rs = SiteCheck(cellId);
  if(rs == 1)
  {
    finish[cellId] = 1;
  }
  return rs;
}
void Reset()
{
  for(int i = 0; i < NUM_CELL; i++)
  {
    finish[i] = 0;
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
     // std::cout<<"cancel "<<std::endl;
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
  std::cout<<method<<" R0 = "<<MAX_RESOURCE_PER_UAV<<", total site = "<<TOTAL_SITE<<std::endl;
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
void CreateUtilityPlot ()
{

  std::string fileNameWithNoExtension = "utility/"+method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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

  std::string fileNameWithNoExtension = "numUAV/"+method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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

  std::string fileNameWithNoExtension = "path/"+method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV);
  std::string graphicsFileName        = method+"_"+std::to_string(TOTAL_SITE)+"_"+std::to_string((int)MAX_RESOURCE_PER_UAV) + ".png";
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