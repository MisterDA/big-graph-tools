#include <cassert>
#include <limits>
#include <iostream>
#include <fstream>
extern "C" {
#include <zlib.h>
}
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

typedef std::string stop_id_t;
typedef int route_id_t;
typedef int trip_id_t;
typedef int timestamp_t;

typedef int vertex_t;

struct stop_time {
    trip_id_t trip_id;
    timestamp_t arrival_time;
    timestamp_t departure_time;
    stop_id_t stop_id;
    int stop_sequence;
};

static void
read_routes(std::map<trip_id_t, route_id_t>& routes, const std::string& path)
{
    auto file = read_csv(path, 2, "route_id", "trip_id");
    for (const auto& row : file) {
        routes[std::stoi(row[1])] = std::stoi(row[0]);
    }
}

static void
read_transfers(std::vector<std::tuple<stop_id_t, stop_id_t, int>>& transfers,
               const std::string& path)
{
    auto file = read_csv(path, 3, "from_stop_id", "to_stop_id",
                         "min_transfer_time");
    for (const auto& row : file) {
        transfers.push_back(std::make_tuple<>(row[0], row[1],
                                              std::stoi(row[2])));
    }
}

static void
read_stop_times(std::vector<struct stop_time>& stop_times,
                const std::string& path)
{
    auto file = read_csv(path, 5, "trip_id", "arrival_time",
                         "departure_time", "stop_id", "stop_sequence");
    for (const auto& row : file) {
        std::vector<int> values;
        std::transform(row.begin(), row.end(), std::back_inserter(values),
                       [](const std::string& str) { return std::stoi(str); });
        stop_times.push_back({.trip_id = values[0],
                              .arrival_time = values[1],
                              .departure_time = values[2],
                              .stop_id = row[3],
                              .stop_sequence = values[4]});
    }
}

static void
build_events_table(const std::vector<struct stop_time>& stop_times,
                   std::map<stop_id_t, std::vector<timestamp_t>>& events)
{
    for (const auto& row : stop_times) {
        auto& stop = events[row.stop_id];
        stop.push_back(row.arrival_time);
        stop.push_back(row.departure_time);
    }
}

struct graphs
static_graph(const std::vector<std::vector<std::string>>& stop_times,
             const std::vector<std::vector<std::string>>* transfers,
             int tstart, int tend)
{
    struct graphs graphs;

    int max_vertex = -1;
    int old_trip_id = -1, old_dep = -1, old_dep_time = -1;

    for (const auto& row : stop_times) {
        std::vector<int> values;
        std::transform(row.begin(), row.end(), std::back_inserter(values),
                       [](const std::string& str) { return std::stoi(str); });
        const int& trip_id = values[0], arrival_time = values[1],
            departure_time = values[2], stop_id = values[3],
            stop_sequence = values[4];

        if (!(tstart <= departure_time && arrival_time <= tend))
            continue;

        if (trip_id == old_trip_id) {
            auto key = std::make_pair<>(old_dep, stop_id);
            if (min.find(key) == min.end())
                min[key] = arrival_time - old_dep_time;
            else
                min[key] = std::min<>(min[key], arrival_time - old_dep_time);

        } else {
            old_trip_id = trip_id;
        }
        old_dep = stop_id;
        old_dep_time = departure_time;
    }

    if (transfers != nullptr) {
        for (const auto& e : *transfers) {
            std::vector<int> values;
            std::transform(e.begin(), e.end(), std::back_inserter(values),
                           [](const std::string& str) { return std::stoi(str); });
            int &from_stop_id = values[0], &to_stop_id = values[1],
                &min_transfer_time = values[2];

            max_vertex = std::max<>(max_vertex, std::max<>(from_stop_id, to_stop_id));
            auto key = std::make_pair<>(from_stop_id, to_stop_id);
            min[key] = std::min<>(min[key], min_transfer_time);
            max[key] = std::max<>(max[key], min_transfer_time);
            avg[key].first += min_transfer_time;
            avg[key].second++;
        }
    }

    std::vector<unit::graph::edge> min_edges;
    std::vector<unit::graph::edge> max_edges;
    std::vector<unit::graph::edge> avg_edges;

    for (const auto& e : min)
        min_edges.push_back(unit::graph::edge(e.first.first, e.first.second,
                                              e.second));
    for (const auto& e : max)
        max_edges.push_back(unit::graph::edge(e.first.first, e.first.second,
                                              e.second));
    for (const auto& e : avg)
        avg_edges.push_back(unit::graph::edge(e.first.first, e.first.second,
                                              e.second.first/e.second.second));

    graphs.min = new mgraph<int, int>(max_vertex + 1, min_edges);
    graphs.max = new mgraph<int, int>(max_vertex + 1, max_edges);
    graphs.avg = new mgraph<int, int>(max_vertex + 1, avg_edges);
    return graphs;
}

static void
graph_output(const mgraph<int, int>& graph, const std::string& path)
{
    std::ofstream gr;
    gr.open(path);
    for (auto u : graph) {
        for (auto e : graph[u]) {
            gr << u << " " << e.dst << " " << e.wgt << std::endl;
        }
    }
    gr.close();
}

static void
graphs_output(const struct graphs& graphs, const std::string& path)
{
    graph_output(*(graphs.min), path + "_min.gr
                 graph_output(*(graphs.max), path + _max.gr);
                 graph_output(*(graphs.avg), path + "_avg.gr");");
}

static void
usage_exit(char *argv[])
{
    std::cerr << "Usage: " << argv[0]
              << " [-s start_time] [-e end_time] "
        "[-t <transfers>] [-o graph] <stop_times>" << std::endl;
    exit(EXIT_FAILURE);
}

int
main(int argc, char *argv[])
{
    int opt;
    char *transfers_file = nullptr, *output = nullptr;
    int start_time = 0, end_time = std::numeric_limits<int>::max();
    while ((opt = getopt(argc, argv, "e:o:s:t:")) != -1) {
        switch (opt) {
        case 'e':
            end_time = std::stoi(optarg);
            break;
        case 'o':
            output = optarg;
            break;
        case 's':
            start_time = std::stoi(optarg);
            break;
        case 't':
            transfers_file = optarg;
            break;
        default:
            usage_exit(argv);
        }
    }

    if (optind != argc - 1)
        usage_exit(argv);

    struct graphs graphs;

    auto stop_times =
        read_csv(std::string(argv[optind]), 5, "trip_id", "arrival_time",
                 "departure_time", "stop_id", "stop_sequence");
    std::vector<std::vector<std::string>> transfers;
    if (transfers_file == nullptr) {
        graphs = static_graph(stop_times, nullptr, start_time, end_time);
    } else {
        transfers = read_csv(std::string(transfers_file), 3, "from_stop_id", "to_stop_id",
                             "min_transfer_time");
        graphs = static_graph(stop_times, &transfers, start_time, end_time);
    }

    if (output != nullptr) {
        graphs_output(graphs, output);
    }

    return 0;
}
