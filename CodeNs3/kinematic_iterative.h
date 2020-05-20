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
#include "scenario.h"
using namespace ns3;
extern AnimationInterface *anim;
extern double dataLoad;
extern double X[MAX_NUM_CELL], Y[MAX_NUM_CELL];//cell center 's position
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];

void FindSegment(int, Ptr<UAV>);
void Execute(int cellId);
void FindSite(Ptr<UAV> u);
int CalculateNumberOfSites(int cellId, int uavId);
void DoTask(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);

/*Function implementation goes here*/

void FindSegment(int, Ptr<UAV>)
{
  
}
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
 // std::cout<<"execute cell "<<cellId<<std::endl;

  for(int i = 0; i < NUM_UAV; i++)
  {
    uavState[cellId][i] = 0;
  }
  for(int i = 0 ; i < NUM_UAV; i++)
  {
    Simulator::Schedule(Seconds(0.01*i+0.01), &DoTask, uav[cellId].GetUav(i));
  }  
}
void FindSite(Ptr<UAV> u)
{
  int cellId = u -> GetCellId();
 // int uavId = u -> GetUavId();
  std::vector<int> siteId;
  std::vector<Vector> pos;
  for(int i = 0; i < numberOfSites[cellId]; i++)
  {
    if(siteState[cellId][i] == 1)
    {
      siteId.push_back(i);
      Ptr<SITE> s = cell_site_list[cellId].Get(i);
      Vector p = s -> GetSitePosition();
      pos.push_back(p);
    }
  }
  int n = siteId.size();
  if(n == 0)
  {
    return;
  }
  Vector p = GetPosition(u);
  double distance[n];
  for(int i = 0; i < n; i++)
  {
    distance[i] = CalculateDistance(pos[i], p);
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
    int idrank = FindId(distance, n, rank);
    int id = siteId[idrank];
    Ptr<SITE> s = cell_site_list[cellId].Get(id);
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
      s -> AddBidder(u, distance[idrank]);
      s -> SetStatus(1);
      u -> AddSite(s);
      flag = 1;      
      return;
    }
    else
    {
      if(s -> CheckPrice(u, distance[idrank]))
      {
       // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" win site "<<s->GetId();
        Ptr<UAV> bd = s -> GetBidder();
       // std::cout<<" from uav "<<bd->GetUavId()<<std::endl;
        // site send to loser
        dataLoad += 3*UAV_PACKET_SIZE;
        bd -> IncreaseEnergy(3*ENERGY_RECEIVE);
        //
        s -> RemoveBidder();
        s -> AddBidder(u, distance[idrank]);
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
void RemoveBidder(Ptr<SITE> s)
{
  s -> RemoveBidder();
  s -> SetStatus(0);
}
void DoTask(Ptr<UAV> u)
{
  int cellId = u -> GetCellId();
  int uavId = u -> GetUavId();
  if(cellId == 1 && numScenario == 0)
  {
    AddPath(uavId);
  }
  if(u -> GetSiteSize() == 0)
  {
    FindSite(u);
  }
  if(u->GetSiteSize() == 0)
  {
   // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go back"<<std::endl;
    double flightTime = Goto(u, GetPosition(gw[cellId].Get(0)));  
 
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    u -> UpdateFlightTime(flightTime);
    if(cellId == 0 && numScenario == 0)
    {
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId);
    }
    Simulator::Schedule(Seconds(flightTime), &CheckCellFinish, u);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }

  uavState[cellId][uavId] = 1;
  int nextstate = 4;
  double rate;
  Ptr<SITE> s = u->GetSite();
  ChangeSiteState(s->GetCellId(), s->GetId(), 3);
  int uavResource = u -> GetResource();
  int resource = s -> GetRealResource();
  int deltaResource = s -> GetDeltaResource();
  int newResource = uavResource - resource;
  double visitedTime = s -> GetVisitedTime();
  if(newResource < 0)
  {
    nextstate = 1;
    u -> SetResource(0);
    rate = (double)uavResource/resource;
    s -> SetResource(resource - uavResource);
    s -> SetVisitedTime(visitedTime*(1.0 - rate));
    visitedTime *= rate;
  }
  else
  {
    u -> SetResource(newResource);
  }
  if(deltaResource > 0)
  {
    s -> SetDeltaResource(0);
    SendPacket(u, gw[cellId].Get(0), UAV_NUM_PACKET, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
  }
 // std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" pos = "<<GetPosition(u)<<" go to site "<<s->GetId()<<" pos = "<<s->GetSitePosition()<<std::endl;
  double flightTime = Goto(u, s -> GetSitePosition());
  u -> UpdateEnergy(FLYING);
  u -> UpdateFliedDistance(VUAV*flightTime);
  u->RemoveSite();
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
  if(uavResource < resource)
  {
    Simulator::Schedule(Seconds(flightTime + visitedTime), &RemoveBidder, s);
  }
  Simulator::Schedule(Seconds(flightTime + visitedTime), &ChangeSiteState, s->GetCellId(), s->GetId(), nextstate);
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime), &FindSite, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
 // std::cout<<"ket thuc ham dotask cell "<<cellId<<" uav "<<uavId<<std::endl;
}
void NextRound(Ptr<UAV> u)
{
  if(SiteCheck(u->GetCellId()) == 0)
  {
    u -> SetResource(MAX_RESOURCE_PER_UAV);
    DoTask(u);
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
