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
#include "ns3/netanim-module.h"
#include "vrp_capacity.h"
#include "scenario.h"
using namespace ns3;
extern AnimationInterface *anim;
extern double dataLoad;
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
//


//
// cvrp variables
std::vector<std::vector<int64>> dist_matrix;
std::vector<int64> site_resource;
std::vector<int64> uav_resource;
int nUAV;
//

void Execute(int cellId);
std::vector<std::vector<int>> SolveCVRP(int cellId, Vector depotPosition);
void FindSegment(int cellId, Ptr<UAV> u);
void AllocateSegment(Ptr<UAV> u, SiteList sl);
void FreeSite(Ptr<UAV> u);
void DoTask(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);

/*Function implementation goes here*/
void InterCell()
{
  
}

void AddPath(int uavId)
{
  Vector pos = GetPosition(uav[1].Get(uavId));
  p[uavId].Add(pos.x, pos.y);
}

void Execute(int cellId)
{ 
  //std::cout<<"execute cell "<<cellId<<std::endl;
  for(int i = 0; i < NUM_UAV; i++)
  {
    Simulator::Schedule(Seconds(0.01*i+0.01), &FindSegment, cellId, uav[cellId].GetUav(i)); 
  }
}
std::vector<std::vector<int>>
SolveCVRP(int cellId, Vector depotPosition)
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
  pos.clear();
  dist_matrix = dist;
  dist.clear();
  site_resource = res;
  res.clear();
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
  temp.clear();
  siteId.clear();

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
  DoTask(u);
}
void FindSegment(int cellId, Ptr<UAV> u)
{
 // std::cout<<"find segment cell "<<cellId<<" uav "<<uavId<<std::endl;
  std::vector< std::vector<int> > sm = SolveCVRP(cellId, Vector(X[u->GetCellId()], Y[u->GetUavId()], height));
    int n = sm.size();
   // std::cout<<"num row = "<<n<<std::endl;
    if(n == 0)
    {
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
    sm.clear();
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
    for(int i = 0; i < n; i++)
    {
      sl[i].Clear();
    }
     
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
  if(cellId == 1 && numScenario == 0)
  {
    AddPath(uavId);
  }
  int uavResource = u -> GetResource();
  if(u -> GetSiteSize() == 0 || uavResource == 0)
  {    
    if(uavResource == 0)
    {     
      FreeSite(u);
    }
    else
    {
       
    }
    double flightTime = Goto(u, GetPosition(gw[cellId].Get(0)));    
    u -> UpdateFlightTime(flightTime);
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    if(cellId == 1 && numScenario == 0)
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
  if(cellId == 1 && numScenario == 0)
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
  uavState[u->GetCellId()][u->GetUavId()] = 0;
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
  uavState[cellId][uavId] = 2;
  if(IsFinish(cellId))
  {
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