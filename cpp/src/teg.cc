#include <cassert>
#include <limits>
#include <iostream>
#include <fstream>
#include <zlib.h>
#include <algorithm>
#include <vector>
#include <set>
#include <unordered_set>
#include <cstdarg>
#include <cmath>
#include <string>
#include <unordered_map>
#include <map>
#include <tuple>

#include "file_util.hh"
#include "mgraph.hh"

// 78756
// 90900
// 90900
// 26136
// 85

static std::map<int, std::pair<int, int>> vertices;

static mgraph<int, int> *
time_expanded_graph(const std::vector<std::vector<std::string>>& stop_times,
                    const std::vector<std::vector<std::string>>* transfers,
                    const bool transfer_metric)
{
    // stop_id -> event -> (vertex_id, trips)
    std::map<int, std::map<int, std::pair<int, std::vector<int>>>> stop_event;
    std::vector<unit::graph::edge> edges;
    int index = 1;
    int old_trip_id = -1; int old_dep = -1, old_dep_time, old_transfer = -1;
    std::cout << "Building tables…" << std::endl;

    for (const auto& row : stop_times) {
        std::vector<int> values;
        std::transform(row.begin(), row.end(), std::back_inserter(values),
                       [](const std::string& str) { return std::stoi(str); });
        const int& trip_id = values[0], arrival_time = values[1],
            departure_time = values[2], stop_id = values[3],
            stop_sequence = values[4];

        int arr_vertex = 0, dep_vertex = 0;
        if (stop_event[stop_id][arrival_time].first == 0) {
            stop_event[stop_id][arrival_time].first = index;
            stop_event[stop_id][arrival_time].second.push_back(trip_id);
            vertices[index] = std::make_pair<>(stop_id, arrival_time);
            arr_vertex = index;
            ++index;
        }

        if (arrival_time != departure_time) {
            if (stop_event[stop_id][departure_time].first == 0) {
                stop_event[stop_id][departure_time].first = index;
                stop_event[stop_id][arrival_time].second.push_back(trip_id);
                vertices[index] = std::make_pair<>(stop_id, departure_time);
                dep_vertex = index;
                ++index;
            }
        } else {
            dep_vertex = arr_vertex;
        }

        // Connection arcs
        if (trip_id == old_trip_id) {
            if (transfer_metric) {
                edges.push_back(unit::graph::edge(old_dep, index, 0));
                edges.push_back(unit::graph::edge(index, arr_vertex, 1));
                vertices[index] = std::make_pair<>(stop_id, -1);
                if (old_transfer != -1) {
                    edges.push_back(unit::graph::edge(old_transfer, old_dep, 0));
                }
                old_transfer = index;
                index++;
            } else {
                edges.push_back(unit::graph::edge(old_dep, arr_vertex,
                                                  arrival_time - old_dep_time));
            }
        } else {
            old_trip_id = trip_id;
            std::cout << "here" << std::endl;
            old_transfer = -1;
        }
        old_dep = dep_vertex;
        old_dep_time = departure_time;
    }

    // Waiting arcs
    std::cout << "Waiting arcs…" << std::endl;
    for (const auto& stop_id : stop_event) {
        for (auto event = stop_id.second.begin();
             event != std::prev(stop_id.second.end(), 1); event++) {
            int dep = event->second.first,
                arr = std::next(event, 1)->second.first;
            int duration = std::next(event, 1)->first - event->first;
            edges.push_back(unit::graph::edge(dep, arr, transfer_metric ? 0 : duration));
        }
    }

    // Footpath transfers
    if (transfers == nullptr) {
        auto graph = new mgraph<int, int>(index  + 1, edges);
        return graph;
    }

    std::cout << "Footpath transfers…" << std::endl;
    std::map<int, int> transfer_vertices;
    for (const auto& row : *transfers) {
        std::vector<int> values;
        std::transform(row.begin(), row.end(), std::back_inserter(values),
                       [](const std::string& str) { return std::stoi(str); });
        int from_stop_id = values[0], to_stop_id = values[1],
            min_transfer_time = values[2];

        // the two stops may be stations or pseudo-stops
        int from_vertex = -1, to_vertex = -1;
        auto ittransfer = transfer_vertices.find(from_stop_id);
        const auto& itfrom = stop_event.find(from_stop_id);
        if (itfrom == stop_event.end() && ittransfer == transfer_vertices.end()) {
            from_vertex = index++;
            transfer_vertices[from_stop_id] = from_vertex;
        }
        ittransfer = transfer_vertices.find(from_stop_id);
        const auto& itto = stop_event.find(to_stop_id);
        if (itto == stop_event.end() && ittransfer == transfer_vertices.end()) {
            to_vertex = index++;
            transfer_vertices[to_stop_id] = to_vertex;
        }

        // For now, we ignore transfers with pseudo-stops
        if (from_vertex != -1 || to_vertex != -1)
            continue;

        for (const auto& from : stop_event[from_stop_id]) {
            for (const auto& to : stop_event[to_stop_id]) {
                if (to.first >= from.first + min_transfer_time) {
                    edges.push_back(unit::graph::edge(from.second.first,
                                                      to.second.first,
                                                      transfer_metric ? 0 : min_transfer_time));
                    break;
                }
            }
        }
    }

    auto graph = new mgraph<int, int>(index  + 1, edges);
    return graph;
}

static void
stats(const std::vector<std::vector<std::string>> stop_times)
{
    std::vector<int> maxs({0, 0, 0, 0, 0});
    for (const auto& row : stop_times) {
        std::vector<int> values;
        std::transform(row.begin(), row.end(), std::back_inserter(values),
                       [](const std::string& str) { return std::stoul(str); });
        for (int i = 0; i < 5; ++i) {
            if (values[i] > maxs[i])
                maxs[i] = values[i];
        }
    }

    for (int i = 0; i < 5; ++i)
        std::cout << maxs[i] << std::endl;
}

static void
graph_to_dot(const mgraph<int, int>& graph)
{
    std::ofstream dot;
    dot.open("graph.dot");
    dot << "digraph g {" << std::endl;
    dot << "  rankdir=\"LR\";" << std::endl;
    for (const auto& u : graph) {
        int time = vertices[u].second;
        if (time == -1)
            dot << "  " << u << "[shape=point];" << std::endl;
        else
            dot << "  " << u << "[label=\"" << vertices[u].first << "@"
                << time << "\"];" << std::endl;
        for (const auto& e : graph[u]) {
            dot << "  " << u << " -> " << e.dst
                << " [label=\"" << e.wgt << "\"];" << std::endl;
        }
    }
    dot << "}" << std::endl;
    dot.close();
}

int
main(int argc, char *argv[])
{
    int opt;
    bool transfer_metric = false;
    char *transfers_file = nullptr;
    while ((opt = getopt(argc, argv, "mt:")) != -1) {
        switch (opt) {
        case 'm':
            transfer_metric = true;
            break;
        case 't':
            transfers_file = optarg;
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " [-m] [-t <transfers>] <stop_times>" << std::endl;
            return -1;
        }
    }

    if (optind != argc - 1) {
        std::cerr << "Usage: " << argv[0] << " [-m] [-t <transfers>] <stop_times>" << std::endl;
        return 1;
    }

    mgraph<int, int> *graph;

    auto stop_times =
        read_csv(std::string(argv[optind]), 5, "trip_id", "arrival_time",
                 "departure_time", "stop_id", "stop_sequence");
    std::vector<std::vector<std::string>> transfers;
    if (transfers_file == nullptr) {
        graph = time_expanded_graph(stop_times, nullptr, transfer_metric);
    } else {
        transfers = read_csv(std::string(transfers_file), 3, "from_stop_id", "to_stop_id",
                             "min_transfer_time");
        graph = time_expanded_graph(stop_times, &transfers, transfer_metric);
    }

    graph_to_dot(*graph);

    delete graph;
    return 0;
};
