#include <fstream>

#include "mgraph.hh"
#include "timetable.hh"
#include "traversal.hh"
#include "pruned_landmark_labeling.hh"

typedef int V;
typedef int64_t W;
typedef mgraph<V, W> graph_t;
typedef pruned_landmark_labeling<graph_t> pl_lab;
typedef mgraph<int, pl_lab::hubinfo> graphL;

static std::tuple<size_t, size_t, size_t>
min_waiting_time(timetable& ttbl, std::vector<size_t>& waiting_times)
{
    size_t min = std::numeric_limits<size_t>::max(),
        max = std::numeric_limits<size_t>::min(),
        avg = 0, n = 0;

    waiting_times.reserve(ttbl.stop_departures.size());
    for (size_t s = 0; s < ttbl.stop_departures.size(); ++s) {
        assert(ttbl.stop_departures[s].size() == ttbl.stop_arrivals[s].size());
        waiting_times[s] = std::numeric_limits<timetable::T>::max();
        for (size_t e = 0; e < ttbl.stop_departures[s].size(); ++e) {
            auto t = ttbl.stop_departures[s][e] - ttbl.stop_arrivals[s][e];
            auto m = std::min(waiting_times[s], (size_t)t);
            waiting_times[s] = m;
            min = std::min(min, m);
            max = std::max(max, m);
            avg += m;
            ++n;
        }
    }
    return std::make_tuple<>(min, avg / n, max);
}

static std::pair<size_t, size_t>
avg_degree(const graph_t& graph)
{
    size_t in = 0, out = 0, n = 0;
    std::vector<int> in_degree;
    for (const auto& u : graph) {
        ++n;
        out += graph.degree(u);
        for (const auto& e : graph[u]) {
            if (in_degree.size() < (unsigned long)e.dst)
                in_degree.resize(e.dst);
            in_degree[e.dst] += 1;
        }
    }
    in_degree.resize(n);
    for (const auto& d : in_degree)
        in += d;

    return std::make_pair<>(in/n, out/n);
}

typedef std::map<timetable::S, std::pair<V, V>> stops_vertices_t;  // (arrival, departure)
typedef std::map<timetable::ST, V> stations_vertices_t;
typedef std::vector<timetable::S> vertices_stops_t;
typedef std::vector<timetable::ST> vertices_stations_t;

struct min_graph_t {
    graph_t *graph;
    stops_vertices_t stops_vertices;
    stations_vertices_t stations_vertices;
    vertices_stops_t vertices_stops;
    vertices_stations_t vertices_stations;
};

static void
add_min_edge(std::map<std::pair<V, V>, W>& graph,
             const V& src, const V& dst, const W& wgt)
{
    auto key = std::make_pair<>(src, dst);
    auto git = graph.find(key);
    if (git != graph.end()) {
        graph[key] = std::min(graph[key], wgt);
    } else {
        graph[key] = wgt;
    }
}

static struct min_graph_t
min_graph(timetable& ttbl, int ma = 30, int mb = 30)
{
    V max_vertex = 0;
    stops_vertices_t stops_vertices;
    stations_vertices_t stations_vertices;
    vertices_stops_t vertices_stops;
    vertices_stations_t vertices_stations;

    std::map<std::pair<V, V>, W> graph;

    for (size_t r = 0; r < ttbl.trips_of.size(); ++r) {
        const auto& route = ttbl.trips_of[r];
        for (size_t t = 0; t < route.size(); ++t) {
            const auto& trip = route[t];
            for (size_t s = 0; s < trip.size() - 1; ++s) {
                const auto& stop = trip[s];
                auto w = stop.second - stop.first;
                auto e = trip[s+1].first - stop.second;
                auto dep_stop_id = ttbl.route_stops[r][s],
                    arr_stop_id = ttbl.route_stops[r][s + 1];

                auto it = stops_vertices.find(dep_stop_id);
                if (it == stops_vertices.end()) {
                    stops_vertices[dep_stop_id] = std::make_pair<>(max_vertex,
                                                             max_vertex+1);
                    vertices_stops.push_back(dep_stop_id);
                    vertices_stops.push_back(dep_stop_id);
                    max_vertex += 2;
                }
                it = stops_vertices.find(arr_stop_id);
                if (it == stops_vertices.end()) {
                    stops_vertices[arr_stop_id] = std::make_pair<>(max_vertex,
                                                             max_vertex+1);
                    vertices_stops.push_back(arr_stop_id);
                    vertices_stops.push_back(arr_stop_id);
                    max_vertex += 2;
                }

                add_min_edge(graph,
                             stops_vertices[dep_stop_id].first,
                             stops_vertices[dep_stop_id].second,
                             w);

                add_min_edge(graph,
                             stops_vertices[dep_stop_id].second,
                             stops_vertices[arr_stop_id].first,
                             e);
            }
            auto end_stop = ttbl.route_stops[r][trip.size() - 1];
            add_min_edge(graph,
                         stops_vertices[end_stop].first,
                         stops_vertices[end_stop].second,
                         trip.back().second - trip.back().first);
        }
    }

    for (size_t st = 0; st < ttbl.station_stops.size(); ++st) {
        const auto& station = ttbl.station_stops[st];
        stations_vertices[st] = max_vertex;
        vertices_stations.push_back(st);
        for (size_t s = 0; s < station.size(); ++s) {
            const auto& stop = stops_vertices[station[s]];
            add_min_edge(graph, stop.first, max_vertex, ma);
            add_min_edge(graph, max_vertex, stop.second, mb);
        }
        ++max_vertex;
    }

    // Is the transfer graph symmetric?
    for (const auto& u : ttbl.transfers) {
        for (const auto& e : ttbl.transfers[u]) {
            if (stations_vertices[u] != stations_vertices[e.dst])
                add_min_edge(graph,
                             stations_vertices[u],
                             stations_vertices[e.dst],
                             e.wgt);
        }
    }


    std::vector<unit::graph::edge> edges;
    for (const auto& e : graph) {
        edges.push_back(unit::graph::edge(e.first.first,
                                          e.first.second,
                                          e.second));
    }
    auto mg = new mgraph<V, int>(max_vertex, edges);

    return {mg, stops_vertices, stations_vertices,
            vertices_stops, vertices_stations};
}

static void
graphviz(const min_graph_t& graph,
         const std::string& path)
{
    std::ofstream dot;
    dot.open(path);
    dot << "digraph g {" << std::endl;
    dot << "  rankdir=\"LR\";" << std::endl;
    for (const auto& u : *graph.graph) {
        if (u < graph.vertices_stops.size())
            dot << "  " << u << ";" << std::endl;
        else
            dot << "  " << u << "[shape=point];" << std::endl;
        for (const auto& e : (*graph.graph)[u])
            dot << "  " << u << " -> " << e.dst
                << " [label=\"" << e.wgt << "\"];" << std::endl;
    }
    dot << "}" << std::endl;
    dot.close();
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
usage_exit(void)
{
    std::cerr << "Usage: comparison <stop_times.csv> <transfers.csv>" << std::endl;
    exit(1);
}

static void
timeprofile(std::vector<std::pair<int, int>>& profile, const min_graph_t& mg,
            const timetable& ttbl, const pl_lab& hl,
            const V src, const V dst, const int departure_time)
{
    // coming soon
}

int main(int argc, char *argv[]) {
    timetable ttbl(argv[1], argv[2]);
    min_graph_t mg = min_graph(ttbl);

    std::vector<bool> is_sel(mg.graph->n(), true);
    std::vector<pl_lab::edgeL> edg;
    pl_lab hl(*mg.graph);
    edg = hl.in_hub_edges(is_sel, is_sel);
    auto queries =
        read_csv(std::string(argv[3]), 6, "source","destination",
                 "departure_time","log2_of_station_rank","station_rank","walk_time");
    for (const auto& query : queries) {
        V src = std::stoi(query[0]), dst = std::stoi(query[1]);
        int departure_time = std::stoi(query[2]);
        auto src_station = mg.stations_vertices[src],
            dst_station = mg.stations_vertices[dst];
    }


        // graphviz(mg, "test.gv");



//


    // std::cout << "Minimum waiting time (secs): " << std::endl;
    // std::vector<size_t> waiting_times;
    // size_t min, max, avg;
    // std::tie(min, avg, max) = min_waiting_time(ttbl, waiting_times);
    // std::cout << "  {.min = " << min << ", .avg = " << avg << ", .max = "
    //           << max << "}" << std::endl;



    // size_t in_degree, out_degree;
    // std::tie(in_degree, out_degree) = avg_degree(*mg.graph);
    // std::cout << "Average degree of minimum static graph: "
    //           << in_degree << ", " << out_degree << std::endl;

}
