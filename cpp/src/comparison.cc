#include "timetable.hh"

// static void
// min_waiting_time(timetable& ttbl, std::vector<timetable::T>& waiting_times)
// {
//     waiting_times.reserve(ttbl.stop_departures.size());
//     for (size_t i = 0; i < ttbl.stop_departures.size(); ++i) {
//         assert(ttbl.stop_departures[i].size() == ttbl.stop_arrivals[i].size());
//         waiting_times[i] = std::numeric_limits<timetable::T>::max();
//         for (size_t j = 0; j < ttbl.stop_departures[i].size() - 1; ++j) {
//             auto t = ttbl.stop_arrivals[i][j + 1] - ttbl.stop_departures[i][j];
//             waiting_times[i] = std::min(waiting_times[i], t);
//         }
//     }
// }

typedef int V;
typedef mgraph<V, int> graph;

static graph *
min_graph(timetable& ttbl, int ma = 30, int mb = 30) // TODO: transfers
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

    auto graph = new mgraph<V, int>(max_vertex, edges);
    return graph;
}

int main(int argc, char *argv[]) {
    timetable ttbl(argv[1], argv[2]);

    // std::vector<timetable::T> waiting_times;
    // min_waiting_time(ttbl, waiting_times);

    graph *graph = min_graph(ttbl);
}
