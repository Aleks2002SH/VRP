// Googlevrp.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//
#include <cstdint>
#include <vector>
#include <fstream>
#include <filesystem>
#include <string>


#include "google/protobuf/duration.pb.h"
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace std;

struct point {
    double x;
    double y;
};
double dist(point p1, point p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}
namespace  fs = std::filesystem;
vector<string> filesindir(const fs::path& dir) {
    vector<string> files;
    for (auto& p : fs::recursive_directory_iterator(dir)) {
        if (fs::is_regular_file(p)) {
            files.push_back(p.path().string());
        }
    }
    return files;
}
void fill_result(double& total_distance, ofstream& out, string& file_name) {
    out << file_name << " ";
    out << total_distance << '\n';
}

namespace operations_research {
    struct DataModel {
        vector<vector<double>> distance_matrix;
        vector<int64_t> demands;
        vector<int64_t> vehicle_capacities;
        int64_t num_vehicles;
        const RoutingIndexManager::NodeIndex depot{ 0 };
    };

    //! @brief Print the solution.
    //! @param[in] data Data of the problem.
    //! @param[in] manager Index manager used.
    //! @param[in] routing Routing solver used.
    //! @param[in] solution Solution found by the solver.
    double PrintSolution(const DataModel& data, const RoutingIndexManager& manager,
        const RoutingModel& routing, const Assignment& solution) {
        int64_t total_distance{ 0 };
        int64_t total_load{ 0 };
        for (int vehicle_id = 0; vehicle_id < data.num_vehicles; ++vehicle_id) {
            int64_t index = routing.Start(vehicle_id);
            //LOG(INFO) << "Route for Vehicle " << vehicle_id << ":";
            int64_t  route_distance{ 0 };
            int64_t route_load{ 0 };
            stringstream route;
            while (routing.IsEnd(index) == false) {
                int64_t node_index = manager.IndexToNode(index).value();
                route_load += data.demands[node_index];
                route << node_index << " Load(" << route_load << ") -> ";
                int64_t previous_index = index;
                index = solution.Value(routing.NextVar(index));
                route_distance += routing.GetArcCostForVehicle(previous_index, index,
                    int64_t{ vehicle_id });
            }
            //LOG(INFO) << route.str() << manager.IndexToNode(index).value();
            //LOG(INFO) << "Distance of the route: " << route_distance << "m";
            //LOG(INFO) << "Load of the route: " << route_load;
            total_distance += route_distance;
            total_load += route_load;
        }
        LOG(INFO) << "Total distance of all routes: " << total_distance << "m";
        //LOG(INFO) << "Total load of all routes: " << total_load;
        //LOG(INFO) << "";
        //LOG(INFO) << "Advanced usage:";
        LOG(INFO) << "Problem solved in " << routing.solver()->wall_time() << "ms";
        return total_distance;
    }

    double VrpCapacity(DataModel& data) {
        // Create Routing Index Manager
        RoutingIndexManager manager(data.distance_matrix.size(), data.num_vehicles,
            data.depot);

        // Create Routing Model.
        RoutingModel routing(manager);

        // Create and register a transit callback.
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&data, &manager](int64_t from_index, int64_t to_index) -> int64_t {
                // Convert from routing variable Index to distance matrix NodeIndex.
                int from_node = manager.IndexToNode(from_index).value();
                int to_node = manager.IndexToNode(to_index).value();
                return data.distance_matrix[from_node][to_node];
            });

        // Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        // Add Capacity constraint.
        const int demand_callback_index = routing.RegisterUnaryTransitCallback(
            [&data, &manager](int64_t from_index) -> int64_t {
                // Convert from routing variable Index to demand NodeIndex.
                int from_node = manager.IndexToNode(from_index).value();
                return data.demands[from_node];
            });
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,    // transit callback index
            int64_t{ 0 },               // null capacity slack
            data.vehicle_capacities,  // vehicle maximum capacities
            true,                     // start cumul to zero
            "Capacity");

        // Setting first solution heuristic.
        RoutingSearchParameters search_parameters = DefaultRoutingSearchParameters();
        search_parameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        search_parameters.set_local_search_metaheuristic(
            LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
        search_parameters.mutable_time_limit()->set_seconds(300); //for all tests except vrp_151_14_2 cause it gives more value than it is acceptable
        // for vrp_31_9_1 300 for vrp_200_16_2 700

        // Solve the problem.
        const Assignment* solution = routing.SolveWithParameters(search_parameters);

        // Print solution on console.
        return PrintSolution(data, manager, routing, *solution);
    }
}  // namespace operations_research

int main(int argc, char** argv) {
    vector<string> files;
    auto cur_p = fs::current_path();
    fs::path dir{ "data" };
    fs::path pathtodata = cur_p / dir;
    files = filesindir(pathtodata);
    ofstream out("result.txt");
    for (int file = 0; file < files.size(); file++) {
        ifstream in(files[file]);
        int N;
        int64_t V, c, dem;
        double x, y;
        in >> N >> V >> c;
        vector<point> customers;
        vector<int64_t> demands(N);
        vector<int64_t> vehicle_capacities(V, c);
        for (int64_t i = 0; i < N; i++) {
            in >> dem >> x >> y;
            demands[i] = dem;
            point c = { x,y };
            customers.push_back(c);
        }
        vector<vector<double>> distance_matrix(N);
        for (int i = 0; i < N; i++) {
            point v1 = customers[i];
            for (int j = 0; j < N; j++) {
                if (i != j) {
                    point v2 = customers[j];
                    distance_matrix[i].push_back(dist(v1, v2));
                }
                else {
                    distance_matrix[i].push_back(0);
                }
            }
        }
        customers.clear();
        operations_research::DataModel data;
        data.demands = demands;
        data.distance_matrix = distance_matrix;
        data.num_vehicles = V;
        data.vehicle_capacities = vehicle_capacities;
        double total_distance = operations_research::VrpCapacity(data);
        fill_result(total_distance, out, files[file]);
        in.close();
    }
    out.close();
    return EXIT_SUCCESS;
}

// Запуск программы: CTRL+F5 или меню "Отладка" > "Запуск без отладки"
// Отладка программы: F5 или меню "Отладка" > "Запустить отладку"

// Советы по началу работы 
//   1. В окне обозревателя решений можно добавлять файлы и управлять ими.
//   2. В окне Team Explorer можно подключиться к системе управления версиями.
//   3. В окне "Выходные данные" можно просматривать выходные данные сборки и другие сообщения.
//   4. В окне "Список ошибок" можно просматривать ошибки.
//   5. Последовательно выберите пункты меню "Проект" > "Добавить новый элемент", чтобы создать файлы кода, или "Проект" > "Добавить существующий элемент", чтобы добавить в проект существующие файлы кода.
//   6. Чтобы снова открыть этот проект позже, выберите пункты меню "Файл" > "Открыть" > "Проект" и выберите SLN-файл.
