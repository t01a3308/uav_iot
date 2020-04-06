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

namespace operations_research {
// [START data_model]
struct DataModel {
  const std::vector<std::vector<int64>> distance_matrix{
{0, 575, 369, 804, 842, 169, 256, 190, 776, 570, 630, 811, 861, 344, 33, 839, 511},
{575, 0, 353, 970, 268, 441, 636, 717, 479, 1066, 1206, 1125, 677, 277, 580, 367, 1077},
{369, 353, 0, 1056, 601, 201, 564, 557, 763, 939, 955, 774, 924, 104, 354, 491, 794},
{804, 970, 1056, 0, 1112, 900, 549, 680, 665, 608, 1034, 1602, 518, 967, 837, 1338, 1078},
{842, 268, 601, 1112, 0, 709, 874, 973, 493, 1305, 1471, 1355, 705, 540, 847, 349, 1345},
{169, 441, 201, 900, 709, 0, 376, 356, 735, 737, 780, 786, 859, 182, 157, 673, 637},
{256, 636, 564, 549, 874, 376, 0, 166, 656, 431, 671, 1057, 684, 498, 289, 967, 626},
{190, 717, 557, 680, 973, 356, 166, 0, 811, 381, 520, 924, 849, 520, 214, 1013, 460},
{776, 479, 763, 665, 493, 735, 656, 811, 0, 1031, 1327, 1515, 212, 661, 799, 800, 1264},
{570, 1066, 939, 608, 1305, 737, 431, 381, 1031, 0, 438, 1179, 991, 898, 591, 1385, 535},
{630, 1206, 955, 1034, 1471, 780, 671, 520, 1327, 438, 0, 875, 1341, 962, 628, 1446, 201},
{811, 1125, 774, 1602, 1355, 786, 1057, 924, 1515, 1179, 875, 0, 1644, 866, 779, 1126, 684},
{861, 677, 924, 518, 705, 859, 684, 849, 212, 991, 1341, 1644, 0, 820, 889, 1010, 1310},
{344, 277, 104, 967, 540, 182, 498, 520, 661, 898, 962, 866, 820, 0, 338, 494, 816},
{33, 580, 354, 837, 847, 157, 289, 214, 799, 591, 628, 779, 889, 338, 0, 831, 500},
{839, 367, 491, 1338, 349, 673, 967, 1013, 800, 1385, 1446, 1126, 1010, 494, 831, 0, 1285},
{511, 1077, 794, 1078, 1345, 637, 626, 460, 1264, 535, 201, 684, 1310, 816, 500, 1285, 0},
  };
  // [START demands_capacities]
  const std::vector<int64> demands{
      0, 54, 63, 74, 72, 80, 92, 80, 60, 82, 67, 73, 78, 96, 52, 97, 65,
  };
  const std::vector<int64> vehicle_capacities{300, 300, 300, 300, 300, 300, 300};
  // [END demands_capacities]
  const int num_vehicles = 7;
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
  for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) 
  {
    int64 index = routing.Start(vehicle_id);
    LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
    int64 route_distance{0};
    int64 route_load{0};
    std::stringstream route;
    while (routing.IsEnd(index) == false) 
    {
      int64 node_index = manager.IndexToNode(index).value();
      route_load += data.demands[node_index];
      route << node_index << " Load(" << route_load << ") -> ";
      int64 previous_index = index;
      index = solution.Value(routing.NextVar(index));
      route_distance += routing.GetArcCostForVehicle(previous_index, index,
                                                     int64{vehicle_id});
    }
    LOG(INFO) << route.str() << manager.IndexToNode(index).value();
    LOG(INFO) << "Distance of the route: " << route_distance << "m";
    LOG(INFO) << "Load of the route: " << route_load;
    total_distance += route_distance;
    total_load += route_load;
  }
  LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
  LOG(INFO) << "Total load of all routes: " << total_load;
  LOG(INFO) << "";
  LOG(INFO) << "Advanced usage:";
  LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
}
// [END solution_printer]

void VrpCapacity() {
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
  PrintSolution(data, manager, routing, *solution);
  // [END print_solution]
}
}  // namespace operations_research

int main(int argc, char** argv) {
  operations_research::VrpCapacity();
  return EXIT_SUCCESS;
}
// [END program]
