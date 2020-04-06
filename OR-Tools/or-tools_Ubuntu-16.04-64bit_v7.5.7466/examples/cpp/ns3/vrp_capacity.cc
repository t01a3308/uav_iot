// Copyright 2010-2018 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// [START program]
// [START import]
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
// [END import]
using namespace std;
std::string cellId;
std::string totalSite;
std::string maxresource;
std::string scenario;
void ReadDistance(std::string filename, std::vector< std::vector<int64> > &arr)
{
  //
  std::string line;
  ifstream myfile(filename.c_str());
  if(myfile.is_open())
  {
    int row = 0;
    while(getline(myfile, line))
    {
      int size = line.size();
      if(line.size() == 0)
      {
        continue;
      }
      //std::cout<<"line "<<row<<" size = "<<line.size()<<std::endl;
      row++;
      std::string temp;
      std::vector<int64> r;
      for(int i = 0; i < size; i++)
      {
        if(line[i] == ' ')
        {
          r.push_back(atoi(temp.c_str()));
          temp.clear();
        }
        else if(i == size-1)
        {
          temp.push_back(line[i]);
          r.push_back(atoi(temp.c_str()));
        }
        else
        {
          temp.push_back(line[i]);
        }
      }
      arr.push_back(r);
    }
    // std::cout<<"num row = "<<arr.size()<<std::endl;
    // for(int i = 0; i < (int)arr.size(); i++)
    // {
    //   std::vector<int64> v = arr[i];
    //   int s = v.size();
    //   for(int j = 0; j < s; j++)
    //   {
    //     std::cout<<arr[i][j]<<" ";
    //   }
    //   std::cout<<std::endl;
    // }
  }
  else
  {
    std::cout<<"cannot open file "<<filename<<std::endl;
  }
}
void ReadDemand(std::string filename, std::vector<int64> &arr)
{
  //
  std::string line;
  ifstream myfile(filename.c_str());
  if(myfile.is_open())
  {
    int row = 0;
    while(getline(myfile, line))
    {
      int size = line.size();
      if(line.size() == 0)
      {
        continue;
      }
      //std::cout<<"line "<<row<<" size = "<<line.size()<<std::endl;
      row++;
      std::string temp;
      for(int i = 0; i < size; i++)
      {
        if(line[i] == ' ')
        {
          arr.push_back(atoi(temp.c_str()));
          temp.clear();
        }
        else if(i == size-1)
        {
          temp.push_back(line[i]);
          arr.push_back(atoi(temp.c_str()));
          temp.clear();
        }
        else
        {
          temp.push_back(line[i]);
        }
      }
    }
    //std::cout<<"demand size = "<<arr.size()<<std::endl;
    // for(int i = 0; i < (int)arr.size(); i++)
    // {
    //     std::cout<<arr[i]<<" ";
    // }
    // std::cout<<std::endl;
  }
  else
  {
    std::cout<<"cannot open file "<<filename<<std::endl;
  }
}
int numVehicle = 115;
std::vector<int64> temp;
namespace operations_research 
{
// [START data_model]
struct DataModel 
{
//   const std::vector<std::vector<int64>> distance_matrix{
// {0, 476, 769, 676, 405, 819, 647, 162, 446, 402, 857, 858},
// {476, 0, 582, 974, 77, 405, 315, 583, 765, 775, 1148, 995},
// {769, 582, 0, 782, 562, 402, 884, 745, 693, 1170, 1613, 600},
// {676, 974, 782, 0, 900, 1101, 1251, 517, 231, 953, 1392, 345},
// {405, 77, 562, 900, 0, 444, 369, 507, 690, 725, 1116, 935},
// {819, 405, 402, 1101, 444, 0, 611, 874, 948, 1168, 1554, 986},
// {647, 315, 884, 1251, 369, 611, 0, 794, 1029, 798, 1059, 1305},
// {162, 583, 745, 517, 507, 874, 794, 0, 289, 486, 948, 726},
// {446, 765, 693, 231, 690, 948, 1029, 289, 0, 750, 1202, 465},
// {402, 775, 1170, 953, 725, 1168, 798, 486, 750, 0, 462, 1208},
// {857, 1148, 1613, 1392, 1116, 1554, 1059, 948, 1202, 462, 0, 1665},
// {858, 995, 600, 345, 935, 986, 1305, 726, 465, 1208, 1665, 0},
//   };
//   // [START demands_capacities]
//   const std::vector<int64> demands{
//       0, 68, 85, 83, 91, 80, 94, 90, 78, 97, 51, 99,
//   };
  
  std::vector<std::vector<int64>> distance_matrix;
  std::vector<int64> demands;
  const int num_vehicles = numVehicle;
  
  const std::vector<int64> vehicle_capacities = temp;
  // [END demands_capacities]
  
  const RoutingIndexManager::NodeIndex depot{0};
};
// [END data_model]

// [START solution_printer]
//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) 
{
  int64 total_distance{0};
  int64 total_load{0};
  std::string filename = "output_cvrp/output_" + cellId + "_" + totalSite + "_" + maxresource + "_" + scenario + ".txt";
  std::ofstream myfile;
  myfile.open(filename, std::ios::out | std::ios::trunc);
  std::cout<<"cell "<<cellId<<", totalSite = "<<totalSite<<", resource = "<<maxresource<<", scenario "<<scenario<<std::endl;
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) 
  {
    int64 index = routing.Start(vehicle_id);
   // LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64 route_distance{0};
    int64 route_load{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false) 
    {
      int64 node_index = manager.IndexToNode(index).value();
      if(node_index != 0)
      {
        myfile<<node_index-1<<" ";
      }
      route_load += data.demands[node_index];
      route << node_index << " Load(" << route_load << ") -> ";
      int64 previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64{vehicle_id});
    }
    myfile<<"\n";
    // LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    // LOG(INFO) << "Distance of the route: " << route_distance << "m";
    // LOG(INFO) << "Load of the route: " << route_load;
    total_distance += route_distance;
    total_load += route_load;
  }
  myfile.close();
  // LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  // LOG(INFO) << "Total load of all routes: " << total_load;
  // LOG(INFO) << "";
  // LOG(INFO) << "Advanced usage:";
  // LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}
// [END solution_printer]

void VrpCapacity() {
  std::vector<std::vector<int64>> dist;
  std::vector<int64> dem;
  std::string distname = "input_cvrp/dist_" + cellId + "_" + totalSite + "_" + maxresource + "_" + scenario + ".txt";
  std::string demandname = "input_cvrp/demand_" + cellId + "_" + totalSite + "_" + maxresource + "_" + scenario + ".txt";
  ReadDistance(distname, dist);
  ReadDemand(demandname, dem);
  // Instantiate the data problem.
  // [START data]
  DataModel data;
  data.distance_matrix = dist;
  data.demands = dem;
  // [END data]

  // Create Routing Index Manager
  // [START index_manager]
  RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
                              data.depot);
  // [END index_manager]

  // Create Routing Model.
  // [START routing_model]
  RoutingModel routing(manager);
  // [END routing_model]

  // Create and register a transit callback.
  // [START transit_callback]
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        int from_node = manager.IndexToNode(from_index).value();
        int to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });
  // [END transit_callback]

  // Define cost of each arc.
  // [START arc_cost]
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  // [END arc_cost]

  // Add Capacity constraint.
  // [START capacity_constraint]
  const int demand_callback_index = routing.RegisterUnaryTransitCallback(
      [&data, &manager](int64 from_index) -> int64 {
        // Convert from routing variable Index to demand NodeIndex.
        int from_node = manager.IndexToNode(from_index).value();
        return data.demands[from_node];
      });
  routing.AddDimensionWithVehicleCapacity(
      demand_callback_index,    // transit callback index
      int64{0},                 // null capacity slack
      data.vehicle_capacities,  // vehicle maximum capacities
      true,                     // start cumul to zero
      "Capacity");
  // [END capacity_constraint]

  // Setting first solution heuristic.
  // [START parameters]
  RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
  searchParameters.set_first_solution_strategy(
      FirstSolutionStrategy::PATH_CHEAPEST_ARC);
  // [END parameters]

  // Solve the problem.
  // [START solve]
  const Assignment* solution = routing.SolveWithParameters(searchParameters);
  // [END solve]

  // Print solution on console.
  // [START print_solution]
  PrintSolution(data, manager, routing, *solution);
  // [END print_solution]
}
}  // namespace operations_research

int main(int argc, char** argv) 
{
  for(int i = 0; i < 6; i++)
  {
    cellId = std::to_string(i);
    for(int j = 500; j <= 500; j += 10)
    {
      totalSite = std::to_string(j);
      for(int k = 150; k <= 650; k += 50)
      {
        maxresource = std::to_string(k);
        int64 maxdemand = std::atoi(maxresource.c_str());
        for(int m = 0; m < 100; m++)
        {
          temp.clear();
          temp = std::vector<int64>(numVehicle, maxdemand);
          scenario = std::to_string(m);
          operations_research::VrpCapacity();
        }
      }
    }
  }
  return EXIT_SUCCESS;
}
// [END program]
