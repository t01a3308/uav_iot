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
#include <cmath>

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
namespace operations_research {
// [START data_model]
struct DataModel {
  // const std::vector<std::vector<int64>> distance_matrix{
  //     {0, 2451, 713, 1018, 1631, 1374, 2408, 213, 2571, 875, 1420, 2145, 1972},
  //     {2451, 0, 1745, 1524, 831, 1240, 959, 2596, 403, 1589, 1374, 357, 579},
  //     {713, 1745, 0, 355, 920, 803, 1737, 851, 1858, 262, 940, 1453, 1260},
  //     {1018, 1524, 355, 0, 700, 862, 1395, 1123, 1584, 466, 1056, 1280, 987},
  //     {1631, 831, 920, 700, 0, 663, 1021, 1769, 949, 796, 879, 586, 371},
  //     {1374, 1240, 803, 862, 663, 0, 1681, 1551, 1765, 547, 225, 887, 999},
  //     {2408, 959, 1737, 1395, 1021, 1681, 0, 2493, 678, 1724, 1891, 1114, 701},
  //     {213, 2596, 851, 1123, 1769, 1551, 2493, 0, 2699, 1038, 1605, 2300, 2099},
  //     {2571, 403, 1858, 1584, 949, 1765, 678, 2699, 0, 1744, 1645, 653, 600},
  //     {875, 1589, 262, 466, 796, 547, 1724, 1038, 1744, 0, 679, 1272, 1162},
  //     {1420, 1374, 940, 1056, 879, 225, 1891, 1605, 1645, 679, 0, 1017, 1200},
  //     {2145, 357, 1453, 1280, 586, 887, 1114, 2300, 653, 1272, 1017, 0, 504},
  //     {1972, 579, 1260, 987, 371, 999, 701, 2099, 600, 1162, 1200, 504, 0},
  // };
  std::vector<std::vector<int64>> distance_matrix;
  const int num_vehicles = 1;
  const RoutingIndexManager::NodeIndex depot{0};
};
// [END data_model]

// [START solution_printer]
//! @brief Print the solution.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
void PrintSolution(const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) 
{
  std::cout<<"cell "<<cellId<<", totalSite = "<<totalSite<<", resource = "<<maxresource<<", scenario "<<scenario<<std::endl;
  std::string filename = "output_global_tsp/output_" + cellId + "_" + totalSite + "_" + maxresource + "_" + scenario + ".txt";
  std::ofstream myfile;
  myfile.open(filename, std::ios::out | std::ios::trunc);
  
  // Inspect solution.
 // LOG(INFO) << "Objective: " << solution.ObjectiveValue() << " miles";
  int64 index = routing.Start(0);
 // LOG(INFO) << "Route:";
  int64 distance{0};
  std::stringstream route;
  while (routing.IsEnd(index) == false) 
  {
    if(index != 0)
    {
      myfile<<index-1<<" ";
    }
    route << manager.IndexToNode(index).value() << " -> ";
    int64 previous_index = index;
    index = solution.Value(routing.NextVar(index));
    distance += routing.GetArcCostForVehicle(previous_index, index, int64{0});
  }
  myfile.close();
  // LOG(INFO) << route.str() << manager.IndexToNode(index).value();
  // LOG(INFO) << "Route distance: " << distance << "miles";
  // LOG(INFO) << "";
  // LOG(INFO) << "Advanced usage:";
  // LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}
// [END solution_printer]

void Tsp() 
{
  std::vector<std::vector<int64>> dist;
  std::string distname = "input_cvrp/dist_" + cellId + "_" + totalSite + "_" + maxresource + "_" + scenario + ".txt";
  ReadDistance(distname, dist);
  // Instantiate the data problem.
  // [START data]
  DataModel data;
  data.distance_matrix = dist;
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

  // [START transit_callback]
  const int transit_callback_index = routing.RegisterTransitCallback(
      [&data, &manager](int64 from_index, int64 to_index) -> int64 {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = manager.IndexToNode(from_index).value();
        auto to_node = manager.IndexToNode(to_index).value();
        return data.distance_matrix[from_node][to_node];
      });
  // [END transit_callback]

  // Define cost of each arc.
  // [START arc_cost]
  routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);
  // [END arc_cost]

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
  PrintSolution(manager, routing, *solution);
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
        for(int m = 0; m < 100; m++)
        {
          scenario = std::to_string(m);
          operations_research::Tsp();
        }
      }
    }
  }
  
  return EXIT_SUCCESS;
}
// [END program]
