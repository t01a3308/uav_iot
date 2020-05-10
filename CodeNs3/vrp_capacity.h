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

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"
// [END import]
extern std::vector<std::vector<int64>> dist_matrix;
extern std::vector<int64> site_resource;
extern std::vector<int64> uav_resource;
extern int nUAV;

namespace operations_research 
{
// [START data_model]
struct DataModel 
{
  const std::vector<std::vector<int64>> distance_matrix = dist_matrix;
  // [START demands_capacities]
  const std::vector<int64> demands = site_resource;
  const std::vector<int64> vehicle_capacities = uav_resource;
  // [END demands_capacities]
  const int num_vehicles = nUAV;
  const RoutingIndexManager::NodeIndex depot{0};
};
// [END data_model]

// [START solution_printer]
//! @brief Print the solution.
//! @param[in] data Data of the problem.
//! @param[in] manager Index manager used.
//! @param[in] routing Routing solver used.
//! @param[in] solution Solution found by the solver.
std::vector<std::vector<int>> 
PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
                   const RoutingModel& routing, const Assignment& solution) 
{
  int64 total_distance{0};
  int64 total_load{0};
  std::vector<std::vector<int>> result;
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) 
  {
    std::vector<int> row;
    int64 index = routing.Start(vehicle_id);
  //  std::cout << "Route for Vehicle " << vehicle_id << ":"<<std::endl;
    int64 route_distance{0};
    int64 route_load{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false)
    {
      int64 node_index = manager.IndexToNode(index).value();
      if(node_index != 0)
      {
        row.push_back((int)node_index-1);
      }
      route_load += data.demands[node_index];
     // route << node_index << " Load(" << route_load << ") -> ";
      int64 previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64{vehicle_id});
    }
   // std::cout << route.str() << manager.IndexToNode(index).value();
    //std::cout << "Distance of the route: " << route_distance << "m"<<std::endl;
   // std::cout << "Load of the route: " << route_load<<std::endl;
    total_distance += route_distance;
    total_load += route_load;
    if(row.size() > 0)
    {
      result.push_back(row);
    }
  }
 // std::cout << "Total distance of all routes: " << total_distance << "m"<<std::endl;
 // std::cout << "Total load of all routes: " << total_load<<std::endl;
 // std::cout << "Problem solved in " << routing.solver()->wall_time() << "ms"<<std::endl;
  return result;
}
// [END solution_printer]

std::vector<std::vector<int>> VrpCapacity() 
{
  // Instantiate the data problem.
  // [START data]
  DataModel data;
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
  return PrintSolution(data, manager, routing, *solution);
  // [END print_solution]
}
}  // namespace operations_research

