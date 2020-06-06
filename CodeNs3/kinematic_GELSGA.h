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
#include "vrp_capacity.h"
using namespace ns3;
extern AnimationInterface *anim;
extern double dataLoad;
extern UAVContainer uav[NUM_CELL];
extern SensorContainer sensor[NUM_CELL];
extern GwContainer gw[NUM_CELL];
extern NodeContainer allNodes[NUM_CELL];
#define NUM_CHROMOSOME 50
int NUM_ITERATOR = 10000;

//GELSGA
double dist[MAX_SITE_PER_CELL][MAX_SITE_PER_CELL];
double velocity[MAX_SITE_PER_CELL][MAX_SITE_PER_CELL];
double mass[MAX_SITE_PER_CELL][MAX_SITE_PER_CELL];
std::vector<int> chromosome[NUM_CHROMOSOME];
std::vector<int> children;
int value777[NUM_CHROMOSOME][MAX_SITE_PER_CELL-1];
int value777_children[MAX_SITE_PER_CELL-1];
double fitness[NUM_CHROMOSOME];
int numIterator;
std::vector<int> gelsga_out[NUM_CELL];
std::vector<int> path[NUM_CELL];
//
int num_vehicles[NUM_CELL];
std::vector<int> siteId;
std::vector<Vector> pos;
std::vector<int> res;
int numberOfFreeSites[NUM_CELL];
//
// cvrp variables
std::vector<std::vector<int64>> dist_matrix;
std::vector<int64> site_resource;
std::vector<int64> uav_resource;
int nUAV;
//
//

std::vector<int> CreateChromosome(int);
void CreatePopulation(int cellId);
double CalculateFitness(std::vector<int> &chro, int cellId);
void CalculateFitness(int cellId);
void CrossOverType1(int father, int mother, int cellId);
void CrossOverType2(int father, int mother, int cellId);
void CrossOver(int);
void MutationType1(int);
void MutationType2(int);
void Mutation(int);
void Terminate(int cellId);
void GELS(int cellId);
void GELSGA(int cellId);
//
double CalculateCost(double distance);
void Execute(int cellId);
void FindSegment(int cellId, Ptr<UAV> u);
void AllocateSegment(Ptr<UAV> u, SiteList sl);
void DoTask(Ptr<UAV> u);
void NextRound(Ptr<UAV> u);
void CheckCellFinish(Ptr<UAV> u);
/*Function implementation goes here*/
void InterCell()
{
  
}
void AddPath(int uavId)
{
  Vector pos = GetPosition(uav[1].Get(uavId));
  p[uavId].Add(pos.x, pos.y);
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
std::vector<int> CreateChromosome(int cellId)
{
  std::vector<int> v;
  for(int i = 0; i < num_vehicles[cellId]-1; i++)
  {
    v.push_back(777);
  }
  for(int i = 0; i < numberOfFreeSites[cellId]; i++)
  {
    v.push_back(i);
  }
  std::vector<int> chro;
  for(int i = 0; i < numberOfFreeSites[cellId] + num_vehicles[cellId]-1; i++)
  {
    int size = v.size();
    if(size < 2)
    {
      chro.push_back(v[0]);
    }
    else
    {
      int id = rand() % size;
      chro.push_back(v[id]);
      v.erase(v.begin()+id);
    }
  }
  return chro;
}
void CreatePopulation(int cellId)
{
 // std::cout<<"CreatePopulation"<<std::endl;
  for(int i = 0; i < NUM_CHROMOSOME; i++)
  {
    chromosome[i].clear();
  }
  for(int i = 0; i < NUM_CHROMOSOME; i++)
  { 
    chromosome[i] = CreateChromosome(cellId);
    // std::cout<<"chromosome "<<i<<": ";
    // for(int j = 0; j < (int)chromosome[i].size(); j++)
    // {
    //   std::cout<<chromosome[i][j]<<" ";
    // }
    // std::cout<<std::endl;
  }
  CalculateFitness(cellId);
}
void Update777Value()
{
  for(int k = 0; k < NUM_CHROMOSOME; k++)
  {
    int id = 0;
    for(int i = 0; i < (int)chromosome[k].size(); i++)
    {
      if(chromosome[k][i] == 777)
      {
        value777[k][id] = i;
        id++;
      }
    }
  }
}
double CalculateFitness(std::vector<int> &chro, int cellId)
{
  int id = 0;
  int vl777[num_vehicles[cellId]-1];
  for(int i = 0; i < (int)chro.size(); i++)
  {
    if(chro[i] == 777)
    {
      vl777[id] = i;
      id++;
    }
  }
  double fit = 0;
  double load[num_vehicles[cellId]];
  int begin, end;
  for(int i = 0; i < num_vehicles[cellId]; i++)
  {
    if(i == 0)
    {
      begin = 0;
      end = vl777[0] - 1;
    }
    else if(i == num_vehicles[cellId]-1)
    {
      begin = vl777[num_vehicles[cellId]-2] + 1;
      end = (int)chro.size() - 1;
    }
    else
    {
      begin = vl777[i-1] + 1;
      end = vl777[i] - 1;
    }
    if(begin > end)
    {
      //std::cout<<"begin > end (fitness)"<<std::endl;
      continue;
    }
    load[i] = 0;
    fit += CalculateDistance(Vector(X[cellId], Y[cellId], 0), cell_site_list[cellId].Get(siteId[chro[begin]])->GetSitePosition());
    fit += CalculateDistance(Vector(X[cellId], Y[cellId], 0), cell_site_list[cellId].Get(siteId[chro[end]])->GetSitePosition());
    for(int j = begin; j <= end; j++)
    {
      Ptr<SITE> s = cell_site_list[cellId].Get(siteId[chro[j]]);
      load[i] += s -> GetResource();
      if(j != end)
      {
        fit += CalculateDistance(cell_site_list[cellId].Get(siteId[chro[j+1]])->GetSitePosition(), s->GetSitePosition());
      }
    }
    //std::cout<<"load "<<i<<" = "<<load[i]<<std::endl;
    double o = load[i] - MAX_RESOURCE_PER_UAV;
    if(o > 0)
    {
      fit += 1000*o;    
    }
  }
  return fit;
}
void CalculateFitness(int cellId)
{
//  std::cout<<"CalculateFitness"<<std::endl;
  for(int k = 0; k < NUM_CHROMOSOME; k++)
  {
    //std::cout<<"chro "<<k<<std::endl;
    // find id of 777
    int start = 0;
    int num = 0;
    while(num < num_vehicles[cellId]-1)
    {
      for(int i = start; i < (int)chromosome[k].size(); i++)
      {
        if(chromosome[k][i] == 777)
        {
          value777[k][num] = i;
          start = i + 1;
          num++;
          break;
        }
      }
    }
    // std::cout<<"777 pos: ";
    // for(int i = 0; i < NUM_UAV-1; i++)
    // {
    //  std::cout<<value777[k][i]<<" ";
    // }
    // std::cout<<std::endl;
    //
    fitness[k] = CalculateFitness(chromosome[k], cellId);
   //std::cout<<"fitness["<<k<<"] = "<<fitness[k]<<std::endl;
  }
  CrossOver(cellId);
}
void CrossOverType1(int father, int mother, int cellId)
{
 // std::cout<<"CrossOverType1"<<std::endl;
  int sequence = father;
  int number = mother;
  int ran = rand() % 10;
  if(ran % 2 == 0)
  {
    sequence = mother;
    number = father;
  }
  int id = 0;
  int idx = 0;
  for(int i = 0; i < (int)chromosome[sequence].size(); i++)
  {
    if(i == value777[number][id])
    {
      children.push_back(777);
      value777_children[id] = i;
      if(id < num_vehicles[cellId]-2)
      {
        id++;
      }
      if(chromosome[sequence][idx] == 777)
      {
        idx++;
      }
      else
      {
        children.push_back(chromosome[sequence][idx]);
        idx++;
      }
    }
    else
    {
      if(chromosome[sequence][idx] == 777)
      {
        idx++;
      }
      else
      {
        children.push_back(chromosome[sequence][idx]);
        idx++;
      }
    }
  }
  // for(int i = 0; i < (int)children.size(); i++)
  // {
  //   std::cout<<children[i]<<" ";
  // }
  // std::cout<<std::endl;
}
void CrossOverType2(int father, int mother, int cellId)
{
 // std::cout<<"CrossOverType2"<<std::endl;
  int size = (int)chromosome[father].size();
  int pos = rand() % size;
 // std::cout<<"pos = "<<pos<<std::endl;
  for(int i = 0; i <= pos; i++)
  {
    children.push_back(chromosome[father][i]);
  }
  for(int i = pos + 1; i < size; i++)
  {
    int id = chromosome[mother][i];
    children.push_back(id);
  }
  // std::cout<<"children: ";
  // for(int i = 0; i < (int)children.size(); i++)
  // {
  //   std::cout<<children[i]<<" ";
  // }
  // std::cout<<std::endl;
  std::vector<int> deletedPosition;
  //std::vector<int> idDelete;
  for(int i = pos+1; i < size; i++)
  {
    int id = children[i];
    for(int j = 0; j <= pos; j++)
    {
      if(children[j] == id)
      {
        if(id != 777)
        {
          deletedPosition.push_back(i);
          break;
        }
        else
        {
          int c = 0;
          for(int k = 0; k < i; k++)
          {
            if(children[k] == 777)
            {
              c++;
            }
          }
          if(c < num_vehicles[cellId]-1)
          {

          }
          else
          {
            deletedPosition.push_back(i);
            break;
          }
        }
      }
    }
  }
  // std::cout<<"deletedPosition: ";
  // for(int i = 0; i < (int)deletedPosition.size(); i++)
  // {
  //   std::cout<<deletedPosition[i]<<" ";
  // }
  // std::cout<<std::endl;
  std::vector<int> v;
  for(int i = 0; i <= pos; i++)
  {
    v.push_back(chromosome[mother][i]);
  }
  for(int i = pos+1; i < (int)chromosome[father].size(); i++)
  {
    v.push_back(chromosome[father][i]);
  }
  // std::cout<<"v: ";
  // for(int i = 0; i < (int)v.size(); i++)
  // {
  //   std::cout<<v[i]<<" ";
  // }
  // std::cout<<std::endl;
  int count777 = 0;
  for(int i = 0; i < (int)children.size(); i++)
  {
    if(children[i] == 777)
    {
      count777++;
    }
  }
  //std::cout<<"count777 = "<<count777<<std::endl;
  for(int i = (int)v.size()-1; i >=0 ; i--)
  {
    if(v[i] == 777)
    {
      if(count777 < num_vehicles[cellId]-1)
      {
        count777++;
      }
      else
      {
        v.erase(v.begin()+i);
      }
    }
  }
  // std::cout<<"v after 777: ";
  // for(int i = 0; i < (int)v.size(); i++)
  // {
  //   std::cout<<v[i]<<" ";
  // }
  // std::cout<<std::endl;
  for(int i = (int)v.size()-1; i >=0 ; i--)
  {
    int id = v[i];
    if(id == 777)
    {
      continue;
    }
    for(int j = 0; j < i; j++)
    {
      if(v[j] == id)
      {
        v.erase(v.begin()+i);
      }
    }
  }
  // std::cout<<"v after oth: ";
  // for(int i = 0; i < (int)v.size(); i++)
  // {
  //   std::cout<<v[i]<<" ";
  // }
  // std::cout<<std::endl;
  std::vector<int> idInsert;
  for(int i = 0; i < (int)v.size(); i++)
  {
    int id = v[i];
    int appear = 0;
    for(int j = 0; j < (int)children.size(); j++)
    {
      if(children[j] == id)
      {
        int isDel = 0;
        for(int k = 0; k < (int)deletedPosition.size(); k++)
        {
          if(j == deletedPosition[k])
          {
            isDel = 1;
            break;
          }
        }
        if(isDel == 1)
        {
          continue;
        }
        if(id != 777)
        {
          appear = 1;
        }
        else
        {
          int c = 0;
          for(int k = 0; k < i; k++)
          {
            if(children[k] == 777)
            {
              c++;
            }
          }
          for(int k = 0; k < (int)idInsert.size(); k++)
          {
            //if()
          }
          if(c < num_vehicles[cellId]-1)
          {
            break;
          }
          else
          {
            appear = 1;
          }

        }
      }
    }
    if(appear == 0)
    {
      idInsert.push_back(id);
    }
  }
  // std::cout<<"idInsert: ";
  // for(int i = 0; i < (int)idInsert.size(); i++)
  // {
  //   std::cout<<idInsert[i]<<" ";
  // }
  // std::cout<<std::endl;

  if(idInsert.size() != deletedPosition.size())
  {
    std::cout<<"insert and delete size are diff"<<std::endl;
    return;
  }
  for(int i = 0; i < (int)deletedPosition.size(); i++)
  {
    int p = deletedPosition[i];
    children[p] = idInsert[0];
    idInsert.erase(idInsert.begin());
  }
  // for(int i = 0; i < (int)children.size(); i++)
  // {
  //   std::cout<<children[i]<<" ";
  // }
  // std::cout<<std::endl;
}
void CrossOver(int cellId)
{
  // min fitness
  int idMin = 0;
  double minFitness = fitness[0];
  for(int i = 1; i < NUM_CHROMOSOME; i++)
  {
    if(fitness[i] < minFitness)
    {
      minFitness = fitness[i];
      idMin = i;
    }
  }
  // second smallest fitness
  int second = 0;
  double secondFitness = 9999999999;
  for(int i = 0; i < NUM_CHROMOSOME; i++)
  {
    if(i == idMin)
    {
      continue;
    }
    if(fitness[i] < secondFitness)
    {
      secondFitness = fitness[i];
      second = i;
    }
  }
  // std::cout<<"parent "<<idMin<<": "<<std::endl;
  // for(int i = 0; i < (int)chromosome[idMin].size(); i++)
  // {
  //   std::cout<<chromosome[idMin][i]<<" ";
  // }
  // std::cout<<std::endl;
  // std::cout<<"parent "<<second<<": "<<std::endl;
  // for(int i = 0; i < (int)chromosome[second].size(); i++)
  // {
  //   std::cout<<chromosome[second][i]<<" ";
  // }
  // std::cout<<std::endl;
  Update777Value();
  children.clear();
  int ran = rand() % 10;
  if(ran % 2 == 0)
  {
    CrossOverType1(idMin, second, cellId);
  }
  else
  {
    CrossOverType2(idMin, second, cellId);
  }
  int id = 0;
    for(int i = 0; i < (int)children.size(); i++)
    {
      if(children[i] == 777)
      {
        value777_children[id] = i;
        id++;
      }
    }
  Mutation(cellId);
}
void MutationType1(int cellId)
{
 // std::cout<<"MutationType1"<<std::endl;
  while(1)
  {
    int id1 = rand() % (children.size());
    int id2 = rand() % (children.size());
    if(id1 != id2)
    {
      int temp = children[id1];
      children[id1] = children[id2];
      children[id2] = temp;
      if(children[id1] == 777 || children[id2] == 777)
      {
        int id = 0;
        for(int i = 0; i < (int)children.size(); i++)
        {
          if(children[i] == 777)
          {
            value777_children[id] = i;
            id++;
          }
        }
      }
      break;
    }
  }
  // for(int i = 0; i < (int)children.size(); i++)
  // {
  //   std::cout<<children[i]<<" ";
  // }
  // std::cout<<std::endl;
}
void MutationType2(int cellId)
{
// std::cout<<"MutationType2"<<std::endl;
  int id = rand() % num_vehicles[cellId];
 // std::cout<<"id = "<<id<<std::endl;
  int begin, end;
  //std::cout<<"777 0 = "<<value777_children[0]<<std::endl;
  if(id == 0)
  {
    begin = 0;
    end = value777_children[0] - 1;
  }
  else if(id == num_vehicles[cellId]-1)
  {
    begin = value777_children[num_vehicles[cellId]-2] + 1;
    end = (int)children.size() - 1;
  }
  else
  {
    begin = value777_children[id-1] + 1;
    end = value777_children[id] - 1;
  }
 // std::cout<<"begin = "<<begin<<" end = "<<end<<std::endl;
  if(begin > end)
  {
   // std::cout<<"begin > end (mutation)"<<std::endl;
    return;
  }
  int pivot = rand() % (end - begin + 1) + begin;
  std::vector<int> mt;
  for(int i = pivot + 1; i <= end; i++)
  {
    mt.push_back(children[i]);
  }
  mt.push_back(children[pivot]);
  for(int i = begin; i < pivot; i++)
  {
    mt.push_back(children[i]);
  }
  for(int i = end; i >= begin; i--)
  {
    children[i] = mt.back();
    mt.pop_back();
  }
  // for(int i = 0; i < (int)children.size(); i++)
  // {
  //   std::cout<<children[i]<<" ";
  // }
  // std::cout<<std::endl;
}
void Mutation(int cellId)
{
  int ran = rand() % 10;
  if(ran % 2 == 0)
  {
    MutationType1(cellId);
  }
  else
  {
    MutationType2(cellId);
  }
  Terminate(cellId);
}
void Terminate(int cellId)
{
 // std::cout<<"Terminate scenario "<<numScenario<<" cell "<<cellId<<" iterator "<<numIterator<<std::endl;
  double childrenFitness = CalculateFitness(children, cellId);
 // std::cout<<"children fitness = "<<childrenFitness<<std::endl;
  int idMax = 0;
  double maxFitness = fitness[0];
  for(int i = 1; i < NUM_CHROMOSOME; i++)
  {
    if(fitness[i] > maxFitness)
    {
      maxFitness = fitness[i];
      idMax = i;
    }
  }
 // std::cout<<"IdMaxFitness: "<<idMax<<": "<<maxFitness<<std::endl;
  if(childrenFitness < maxFitness)
  {
    chromosome[idMax].clear();
    chromosome[idMax] = children;
    fitness[idMax] = childrenFitness;
    for(int i = 0; i < num_vehicles[cellId]-1; i++)
    {
      value777[idMax][i] = value777_children[i];
    }
  }
  if(numIterator == NUM_ITERATOR-1)
  {
   // std::cout<<"Cell "<<cellId<<std::endl;
    for(int i = 0; i < NUM_CHROMOSOME; i++)
    {
      fitness[i] = CalculateFitness(chromosome[i], cellId);
    }
    int idMin = 0;
    double minFitness = fitness[0];
    for(int i = 1; i < NUM_CHROMOSOME; i++)
    {
      if(fitness[i] < minFitness)
      {
        minFitness = fitness[i];
        idMin = i;
      }
    }
  //  std::cout<<"idmin = "<<idMin<<" fitness = "<<fitness[idMin]<<std::endl;
    gelsga_out[cellId].clear();
    gelsga_out[cellId] = chromosome[idMin];
   //  std::cout<<"output: ";
   //  for(int i = 0; i < (int)gelsga_out[cellId].size(); i++)
   //  {
   //    int id = gelsga_out[cellId][i];
   //    std::cout<<id<<" ";
   //  }
   // std::cout<<std::endl;
    path[cellId].clear();
    path[cellId] = gelsga_out[cellId];
    
    return;
  }
  else
  {
    numIterator++;
   // std::cout<<"iterator "<<numIterator<<std::endl;
    CrossOver(cellId);
    //GELS(cellId);
  }
}
void GELS(int cellId)
{
 // std::cout<<"GELS"<<std::endl;
  for(int i = 0; i < NUM_CHROMOSOME; i++)
  {
   // std::cout<<"chromosome "<<i<<std::endl;
    int size = (int)chromosome[i].size();
    for(int i = 0; i < numberOfFreeSites[cellId]; i++)
    {
      for(int j = 0; j < numberOfFreeSites[cellId]; j++)
      {
        if(i == j)
        {
          velocity[i][j] = 0;
          mass[i][j] = 0;
        }
        else
        {
          velocity[i][j] = 100;
          mass[i][j] = dist[i][j]/velocity[i][j]*60;
        }
      }
    }
    for(int n = 0; n < size-2; n++) //i
    {
      if(chromosome[i][n] == 777)
      {
        continue;
      }
      
      for(int j = n+1; j < size-1; j++)//j
      {
        if(chromosome[i][j] == 777)
        {
          continue;
        }
        std::vector<int> ca;
        for(int k = 0; k <= j; k++)
        {
          ca.push_back(chromosome[i][k]); //before j
        }
        std::vector<int> id;
        for(int k = j+1; k < size; k++)
        {
          id.push_back(chromosome[i][k]); // after j
        }
        std::vector<double> m;    
        for(int k = j+1; k < size; k++)
        {
          int a = chromosome[i][k];
          if(a == 777)
          {
            m.push_back(99999);
          }
          else
          {
            m.push_back(mass[chromosome[i][n]][a]);
          }
        }
        // std::cout<<"before arrange: "<<std::endl;
        // std::cout<<"id: ";
        // for(int k = 0; k < (int)id.size(); k++)
        // {
        //  std::cout<<id[k]<<" ";
        // }
        // std::cout<<std::endl;
        // std::cout<<"mass: ";
        // for(int k = 0; k < (int)m.size(); k++)
        // {
        //  std::cout<<m[k]<<" ";
        // }
        // std::cout<<std::endl;
        // arrange according to mass
        int s = (int)m.size();
        for(int g = 0; g < s-1; g++)
        {
          if(m[g] == 99999)
          {
            continue;
          }
          for(int h = g+1; h < s; h++)
          {
            if(m[h] == 99999)
            {
              continue;
            }
            if(m[g] > m[h])
            {
              double t = m[g];
              m[g] = m[h];
              m[h] = t;
              int tp = id[g];
              id[g] = id[h];
              id[h] = tp;
            }
          }
        }
        // std::cout<<"after arrange: "<<std::endl;
        // std::cout<<"id: ";
        // for(int k = 0; k < (int)id.size(); k++)
        // {
        //  std::cout<<id[k]<<" ";
        // }
        // std::cout<<std::endl;
        // std::cout<<"mass: ";
        // for(int k = 0; k < (int)m.size(); k++)
        // {
        //  std::cout<<m[k]<<" ";
        // }
        // std::cout<<std::endl;
        
        for(int k = 0; k < (int)id.size(); k++)
        {
          ca.push_back(id[k]);
        }
        // std::cout<<"CA: ";
        // for(int k = 0; k < (int)ca.size(); k++)
        // {
        //  std::cout<<ca[k]<<" ";
        // }
        // std::cout<<std::endl;
        double CAfitness = CalculateFitness(ca, cellId);
        double CUfitness = CalculateFitness(chromosome[i], cellId);
        if(CAfitness < CUfitness)
        {
          //std::cout<<"update CU"<<std::endl;
          for(int k = 0; k < (int)ca.size(); k++)
          {
            chromosome[i][k] = ca[k];
          }
          fitness[i] = CAfitness;
          int idx;
          for(int k = j-1; k >= 0; k--)
          {
            if(chromosome[i][k] == 777)
            {
              continue;
            }
            else
            {
              idx = k;
              break;
            }
          }
          //std::cout<<"idx = "<<idx<<std::endl;
          double d = dist[idx][j];
          double F = 6.672*(CUfitness - CAfitness)/d/d;
          velocity[idx][j] += F;
          mass[idx][j] = dist[idx][j]/velocity[idx][j];
        }
      }
    }
  }
  CrossOver(cellId);
}
void GELSGA(int cellId)
{
  numIterator = 0;
  gelsga_out[cellId].clear();
  siteId.clear();
  pos.clear();
  res.clear();
  for(int i = 0; i < NUM_CHROMOSOME; i++)
  {
    chromosome[i].clear();
  }
  children.clear();
  gelsga_out[cellId].clear();
  path[cellId].clear();
 // int allResource = 0;
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
      //allResource += r;
    }

  }
  // num_vehicles[cellId] = allResource/MAX_RESOURCE_PER_UAV + 1;
  numberOfFreeSites[cellId] = res.size();
  if(numberOfFreeSites[cellId] == 0)
  {
    return;
  }
  if(num_vehicles[cellId] == 1)
  {
    for(int i = 0; i < numberOfFreeSites[cellId]; i++)
    {
      path[cellId].push_back(i);
    }
    return;
  }
  CreatePopulation(cellId);
}
void Execute(int cellId)
{ 
  //std::cout<<"execute cell "<<cellId<<std::endl;
  for(int i = 0; i < NUM_UAV; i++)
  {
    Simulator::Schedule(Seconds(0.01*i+0.01), &FindSegment, cellId, uav[cellId].GetUav(i));
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
  sl.Clear();
  DoTask(u);
}

void FindSegment(int cellId, Ptr<UAV> u)
{
 // std::cout<<GetNow()<<"find segment cell "<<u->GetCellId()<<" uav "<<u->GetUavId()<<std::endl;
  std::vector< std::vector<int> > sm = SolveCVRP(cellId, Vector(X[u->GetCellId()], Y[u->GetUavId()], height));
  int sz = sm.size();
  if(sz == 0)
  {
   // std::cout<<"sz=0"<<std::endl;
    return;
  }
  if(sz > NUM_UAV)
  {
    num_vehicles[cellId] = sz+1;
  }
  else
  {
    num_vehicles[cellId] = sz;
  }
  GELSGA(cellId);
  SiteList sl[num_vehicles[cellId]];
  int idSegment = 0;
  int size = (int)path[cellId].size();
  for(int i = 0; i < size; i++)
  {
    if(path[cellId][i] == 777)
    {
      idSegment++;
    }
    else
    {
      int id = path[cellId][i];
      Ptr<SITE> s = cell_site_list[cellId].Get(siteId[id]);
      sl[idSegment].Add(s);
    }
  }
    int n = num_vehicles[cellId];
   // std::cout<<"num row = "<<n<<std::endl;
    if(n == 0)
    {
      return;
    }

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
//  std::cout<<GetNow()<<": cell "<<cellId<<", uav "<<uavId<<" go to site "<<s->GetId()<<std::endl; 
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