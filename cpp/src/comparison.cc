#include "timetable.hh"

typedef int V;
typedef mgraph<V, int> graph;

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
        for (size_t e = 0; e < ttbl.stop_departures[s].size() - 1; ++e) {
            auto t = ttbl.stop_arrivals[s][e + 1] - ttbl.stop_departures[s][e];
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

static size_t
avg_degree(const graph& graph)
{
    size_t avg = 0, n = 0;
    for (const auto& u : graph) {
        ++n;
        avg += graph.degree(u);
    }
    return avg / n;
}

static graph *
min_graph(timetable& ttbl, int ma = 30, int mb = 30)
{
    std::vector<unit::graph::edge> edges;
    V max_vertex = 0;
    std::map<timetable::S, std::pair<V, V>> vertices; // (arrival, departure)

    for (size_t r = 0; r < ttbl.trips_of.size(); ++r) {
        const auto& route = ttbl.trips_of[r];
        for (size_t t = 0; t < route.size(); ++t) {
            const auto& trip = route[t];
            for (size_t s = 0; s < trip.size() - 1; ++s) {
                const auto& stop = trip[s];
                auto w = stop.first - stop.second;
                auto e = trip[s+1].first - stop.second;
                auto dep_stop_id = ttbl.route_stops[r][s],
                    arr_stop_id = ttbl.route_stops[r][s + 1];

                auto it = vertices.find(dep_stop_id);
                if (it == vertices.end()) {
                    vertices[dep_stop_id] = std::make_pair<>(max_vertex,
                                                             max_vertex+1);
                    max_vertex += 2;
                }
                it = vertices.find(arr_stop_id);
                if (it == vertices.end()) {
                    vertices[arr_stop_id] = std::make_pair<>(max_vertex,
                                                             max_vertex+1);
                    max_vertex += 2;
                }

                edges.push_back(unit::graph::edge(vertices[dep_stop_id].first,
                                                  vertices[dep_stop_id].second,
                                                  w));
                edges.push_back(unit::graph::edge(vertices[dep_stop_id].second,
                                                  vertices[arr_stop_id].first,
                                                  e));
            }
        }
    }

    std::map<timetable::ST, V> stations_vertices;
    for (size_t st = 0; st < ttbl.station_stops.size(); ++st) {
        const auto& station = ttbl.station_stops[st];
        stations_vertices[st] = max_vertex;

        for (size_t s = 0; s < station.size(); ++s) {
            const auto& stop = vertices[station[s]];
            edges.push_back(unit::graph::edge(stop.first, max_vertex, ma));
            edges.push_back(unit::graph::edge(max_vertex, stop.second, mb));
        }
        ++max_vertex;
    }

    // Is the transfer graph symmetric?
    for (const auto& u : ttbl.transfers) {
        for (const auto& e : ttbl.transfers[u]) {
            edges.push_back(unit::graph::edge(stations_vertices[u],
                                              stations_vertices[e.dst],
                                              e.wgt));
        }
    }

    auto graph = new mgraph<V, int>(max_vertex, edges);
    return graph;
}

static void
usage(void)
{
    std::cerr << "Usage: comparison <stop_times.csv> <transfers.csv>" << std::endl;
}

int main(int argc, char *argv[]) {
    timetable ttbl(argv[1], argv[2]);

    std::cout << "Minimum waiting time (secs): " << std::endl;
    std::vector<size_t> waiting_times;
    size_t min, max, avg;
    std::tie(min, avg, max) = min_waiting_time(ttbl, waiting_times);
    std::cout << "  {.min = " << min << ", .avg = " << avg << ", .max = "
              << max << "}" << std::endl;

    graph *graph = min_graph(ttbl);
    std::cout << "Average degree of minimum static graph: "
              << avg_degree(*graph) << std::endl;
}
