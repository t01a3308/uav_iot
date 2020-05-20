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
int numSegment[NUM_CELL];
//
// cvrp variables
std::vector<std::vector<int64>> dist_matrix;
std::vector<int64> site_resource;
std::vector<int64> uav_resource;
int nUAV;
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
SiteList LocalTSP(SiteList sl, Vector depot);
//
void Execute(int cellId);
std::vector<std::vector<int>> SolveCVRP(int cellId, Vector depotPosition);
void AddSegment(int cellId);
std::pair<SiteList, SiteList> AddSegment(SiteList largeSegment);
void InterCell();
void InterCell(int cellId);
void FindSegment(int cellId, Ptr<UAV> u);
void AllocateSegment(Ptr<UAV> u, SiteList sl);
void FreeSite(Ptr<UAV> u);
void DoTask(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);

/*Function implementation goes here*/

void AddPath(int uavId, int cellId)
{

  Vector pos = GetPosition(uav[cellId].Get(uavId));
  if(cellId == 1)
  {
    p[uavId].Add(pos.x, pos.y);
  }
  else if(cellId == 0)
  {
    p1[uavId].Add(pos.x, pos.y);
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
  sl.Clear();
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
void Execute(int cellId)
{ 

  std::vector<std::vector<int>> sm = SolveCVRP(cellId, Vector(X[cellId], Y[cellId], height));
  int n = sm.size();
  //std::cout<<"n = "<<n<<std::endl;
  if(n >= NUM_UAV)
  {
    sm.clear();   
      for(int i = 0 ; i < NUM_UAV; i++)
      {
        Simulator::Schedule(Seconds(0.01*i + 0.01), &FindSegment, cellId, uav[cellId].GetUav(i));     
      }
  }
  else
  {
   // std::cout<<"thua resource n = "<<n<<std::endl;
    sm.clear();
    AddSegment(cellId);
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
void InterCell(int cellId, Ptr<UAV> u)
{

  std::vector<std::vector<int>> sm = SolveCVRP(cellId, Vector(X[cellId], Y[cellId], height));
  int n = sm.size();
  if(n == 0)
  {
   // std::cout<<"no segment"<<std::endl;
    return;
  }
  int numFreeUav = 0;
  for(int i = 0; i < NUM_UAV; i++)
  {
    if(uavState[cellId][i] != 1)
    {
      numFreeUav++;
    }
  }
  int n1 = n - numFreeUav;//number of segments to be bidded
  if(n1 <= 1)
  {
    //std::cout<<"numSegment < numuav"<<std::endl;
    return;
  }
  // send data to  free cell;
  dataLoad += 5 * UAV_PACKET_SIZE;
  // reply result
  dataLoad += 1 * UAV_PACKET_SIZE;
  SiteList sitelist[n];
  for(int i = 0; i < n; i++)
  {
    std::vector<int> v = sm[i];
    for(int j = 0; j < (int)v.size(); j++)
    {
      int id = v[j];
      sitelist[i].Add(cell_site_list[cellId].Get(id));
    }
  }
  sm.clear();
  // find path for each segment
  SiteList sl[n];
  vector <int> segmentId;
  for(int i = 0; i < n; i++)
  {
    segmentId.push_back(i);
    int m = sitelist[i].GetSize();
    if(m > 20)
    {
      std::cout<<"segment "<<i<<" size > 20"<<std::endl;
    }
    sl[i] = LocalTSP(sitelist[i], Vector(X[0], Y[0], height));
    sitelist[i].Clear();
  }
  // find segment having max benefit
  double benefit[n];
  Vector pos = GetPosition(u);
  for(int j = 0; j < n; j++)
  {
    double utility = sl[segmentId[j]].GetExpectedUtility(0);
    double length = sl[segmentId[j]].GetLength(pos);
    double cost = CalculateCost(length);
    double benef = utility - cost;
    benefit[j] = benef;
  }
  int id = IdMax(benefit, n);
  //std::cout<<"uav "<<u->GetUavId()<<" segment "<<id<<" size = "<<(int)sl[id].GetSize()<<std::endl;
  AllocateSegment(u, sl[id]);
 //sl[id].Print();
  
}
void InterCell()
{

  // send to other cells to find  cells which need help
  dataLoad += 6 * 1 * UAV_PACKET_SIZE;
  // reply
  dataLoad += 6 * 1 * UAV_PACKET_SIZE;
  
  std::vector<std::vector<int>> sm[NUM_CELL];
  int n[NUM_CELL];
  for(int i = 1; i < NUM_CELL; i++)
  {
    sm[i] = SolveCVRP(i, Vector(X[i], Y[i], height));
    n[i] = sm[i].size();
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    //find cell having max number of segment
    Ptr<UAV> u = uav[0].GetUav(i);
    int uavId = u->GetId();
    if(uavState[0][uavId] != 0)
    {
      continue;
    }
    int cellId = 1;
    int maxSegment = n[1];
    for(int k = 2; k < NUM_CELL; k++)
    {
      if(n[k] > maxSegment)
      {
        maxSegment = n[k];
        cellId = k;
      }
    }
    if(n[cellId] < 2)
    {
      return;
    }
    InterCell(cellId, u);
    // sm[cellId].clear();
    // sm[cellId] = SolveCVRP(cellId, Vector(X[cellId], Y[cellId], height));
    // n[cellId] = sm[cellId].size();
    n[cellId]--;
  }
}
std::pair<SiteList, SiteList> AddSegment(SiteList largeSegment)
{

  SiteList sl[2];
  int n = largeSegment.GetSize();
  for(int i = 0; i < n; i++)
  {
    double t1 = sl[0].GetFlightTime();
    double t2 = sl[1].GetFlightTime();
    if(t1 < t2)
    {
      Ptr<SITE> s = largeSegment.Get(0);
      sl[0].Add(s);
      largeSegment.Pop();
    }
    else
    {
      int k = largeSegment.GetSize();
      Ptr<SITE> s = largeSegment.Get(k-1);
      sl[1].Add(s);
      largeSegment.Pop_Back();
    }
  }
  return std::make_pair(sl[0], sl[1]);
}
void AddSegment(int cellId)
{

  std::vector< std::vector<int> > sm = SolveCVRP(cellId, Vector(X[cellId], Y[cellId], height));
  int n = sm.size();
   // std::cout<<"num row = "<<n<<std::endl;
  if(n == 0)
  {
    numSegment[cellId] = 0;
    return;
  }
  int sz = cell_site_list[cellId].GetSize();
  SiteList sl[sz];
  for(int i = 0; i < n; i++)
  {
    std::vector<int> v = sm[i];
    for(int j = 0; j < (int)v.size(); j++)
    {
      int id = v[j];
      sl[i].Add(cell_site_list[cellId].Get(id));
    }
  }
  sm.clear();
  int numSegment = n;
  int k = NUM_UAV - n;
  for(int i = 0; i < k; i++)
  {
     // find largest segment
    int idSegment = 0;
    double maxtime = sl[0].GetFlightTime();
    for(int i = 1; i < numSegment; i++)
    {
      double t = sl[i].GetFlightTime();
      if(t > maxtime)
      {
        maxtime = t;
        idSegment = i;
      }
    }
    if(sl[idSegment].GetSize() < 2)
    {
      //std::cout<<"large segment < 2"<<std::endl;
      continue;
    }
   // std::cout<<"largest segment: "<<idSegment<<std::endl;
   // sl[idSegment].Print();
    std::pair<SiteList, SiteList> newSegments = AddSegment(sl[idSegment]);
    sl[idSegment] = newSegments.first;
    sl[numSegment] = newSegments.second;
    numSegment++;
  }
  if(numSegment != NUM_UAV)
  {
    std::cout<<"numSegment != NUM_UAV"<<std::endl;
  }
  for(int i = 0; i < NUM_UAV; i++)
  {
    //std::cout<<"segment "<<i<<std::endl;
    //sl[i].Print();
    AllocateSegment(uav[cellId].GetUav(i), sl[i]);
    sl[i].Clear();
  }
}
std::vector<std::vector<int>>
SolveCVRP(int cellId, Vector depotPosition)
{

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
void FindSegment(int cellId, Ptr<UAV> u)
{

    std::vector< std::vector<int> > sm = SolveCVRP(cellId, GetPosition(u));
    int n = sm.size();

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
    sm.clear();
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
    AllocateSegment(u, sl[id]);
    for(int i = 0; i < n; i++)
    {
      sl[i].Clear();
    }
}
void AllocateSegment(Ptr<UAV> u, SiteList sl)
{
  int uavId = u -> GetUavId();
  int cellId = u -> GetCellId();
  uavState[cellId][uavId] = 1;

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
  DoTask(u);
}
void FreeSite(Ptr<UAV> u)
{

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

  if((cellId == 0 || cellId == 1) && numScenario == 0)
  {
    AddPath(uavId, cellId);
  }
  int uavResource = u -> GetResource();
  if(u->GetSiteSize() == 0 || uavResource == 0)
  {

    if(uavResource == 0)
    {
      FreeSite(u);
    }
    double flightTime = Goto(u, GetPosition(gw[cellId].Get(0)));    
    u -> UpdateFlightTime(flightTime);
    u -> UpdateEnergy(FLYING);
    u -> UpdateFliedDistance(VUAV*flightTime);
    if((cellId == 0 || cellId == 1) && numScenario == 0)
    {
      Simulator::Schedule(Seconds(flightTime), &AddPath, uavId, cellId);
    }
    Simulator::Schedule(Seconds(flightTime), &CheckCellFinish, u);   
    Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, STOP); 
    return;
  }
  Ptr<SITE> s = u->GetSite();

  u->RemoveSite();
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
  if((cellId == 0 || cellId == 1) && numScenario == 0)
  {
    Simulator::Schedule(Seconds(flightTime), &AddPath, uavId, cellId);
  }
  Simulator::Schedule(Seconds(flightTime + visitedTime), &ChangeSiteState, s->GetCellId(), s->GetId(), nextState);
  Simulator::Schedule(Seconds(flightTime), &UAV::UpdateEnergy, u, HANDLING);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &DoTask, u);
  Simulator::Schedule(Seconds(flightTime + visitedTime), &SendPacket, u, gw[cellId].Get(0), NUM_PACKET_SITE, UAV_PACKET_SIZE, INTERVAL_BETWEEN_TWO_PACKETS);
}
void NextRound(Ptr<UAV> u)
{
  int cellId = u->GetCellId();
  int uavId = u->GetUavId();

  uavState[cellId][uavId] = 0;
  if(cellId == 0)
  {
    InterCell();
    return;
  }
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
