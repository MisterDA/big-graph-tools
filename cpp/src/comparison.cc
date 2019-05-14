#include <iostream>
#include <fstream>
#include <list>

#include "mgraph.hh"
#include "timetable.hh"
#include "traversal.hh"
#include "pruned_landmark_labeling.hh"

namespace {

    using V = int;
    using W = int;
    using graph_t = mgraph<V, W>;
    using pl_lab = pruned_landmark_labeling<graph_t>;
    using graphL = mgraph<int, pl_lab::hubinfo>;

    using stops_vertices_t = std::map<timetable::S, std::pair<V, V>>;  // (arrival, departure)
    using stations_vertices_t = std::map<timetable::ST, V>;
    using vertices_stops_t = std::vector<timetable::S>;
    using vertices_stations_t = std::vector<timetable::ST>;

    struct min_graph_t {
        graph_t *min_graph;
        stops_vertices_t stops_vertices;
        stations_vertices_t stations_vertices;
        vertices_stops_t vertices_stops;
        vertices_stations_t vertices_stations;
    };

    struct label {
        V hub, next_hop, length;
    };

    using hl_t = std::map<V, std::vector<struct label>>;

    struct timeprofile_t {
        int tdep;
        int tarr;
    };

    struct trip_stop {
        V vertex;
        bool is_stop;
        timetable::ST station;
        std::vector<std::pair<timetable::T, timetable::T>> schedule;
    };

    void
    graph_output(const mgraph<int, int> &graph, const std::string &path)
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

    void
    graph_input(mgraph<int, int> &graph, const std::string &path)
    {
        std::vector<unit::graph::edge> edges;
        std::ifstream f(path);
        std::string line;
        getline(f, line);           // header
        while (!f.eof()) {
            int u, v, wgt;
            f >> u >> v >> wgt;
            edges.push_back(unit::graph::edge(u, v, wgt));
        }
        graph.set_edges(edges);
    }


    void
    graphviz(const min_graph_t &graph,
             const std::string &path)
    {
        std::ofstream dot;
        dot.open(path);
        dot << "digraph g {" << std::endl;
        dot << "  rankdir=\"LR\";" << std::endl;
        for (const auto &u : *graph.min_graph) {
            if (u < graph.vertices_stops.size())
                dot << "  " << u << ";" << std::endl;
            else
                dot << "  " << u << "[shape=point];" << std::endl;
            for (const auto &e : (*graph.min_graph)[u])
                dot << "  " << u << " -> " << e.dst
                    << " [label=\"" << e.wgt << "\"];" << std::endl;
        }
        dot << "}" << std::endl;
        dot.close();
    }

    void
    hl_output(pl_lab &hl, std::ostream &f)
    {
        std::vector<pl_lab::edgeL> edg;
        edg = hl.in_hub_edges();
        for (const pl_lab::edgeL &e : edg) {
            f << "i " << e.wgt.hub << " " << e.wgt.next_hop
              << " "  << e.dst << " " << e.wgt.dist << std::endl;
        }
        edg = hl.out_hub_edges();
        for (const pl_lab::edgeL &e : edg) {
            f << "o " << e.src << " " << e.wgt.next_hop
              << " " << e.wgt.hub << " " << e.wgt.dist << std::endl;
        }
    }

    void
    hl_input(std::istream &s, hl_t &outhubs, hl_t &inhubs)
    {
        char t;
        V vertex, next_hop, hub;
        int length;
        while (!s.eof()) {
            s >> t;
            if (t == 'i') {
                s >> hub >> next_hop >> vertex >> length;
                inhubs[vertex].push_back({.hub = hub,
                                          .next_hop = next_hop,
                                          .length = length});
            } else if (t == 'o') {
                s >> vertex >> next_hop >> hub >> length;
                outhubs[vertex].push_back({.hub = hub,
                                           .next_hop = next_hop,
                                           .length = length});
            } else {
                s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
    }

    std::tuple<size_t, size_t, size_t>
    min_waiting_time(timetable &ttbl, std::vector<size_t> &waiting_times)
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

    std::pair<size_t, size_t>
    avg_degree(const graph_t &graph)
    {
        size_t in = 0, out = 0, n = 0;
        std::vector<int> in_degree;
        for (const auto &u : graph) {
            ++n;
            out += graph.degree(u);
            for (const auto &e : graph[u]) {
                if (in_degree.size() < (unsigned long)e.dst)
                    in_degree.resize(e.dst);
                in_degree[e.dst] += 1;
            }
        }
        in_degree.resize(n);
        for (const auto &d : in_degree)
            in += d;

        return std::make_pair<>(in/n, out/n);
    }


    void
    add_min_edge(std::map<std::pair<V, V>, W> &graph,
                 const V &src, const V &dst, const W &wgt)
    {
        auto key = std::make_pair<>(src, dst);
        auto git = graph.find(key);
        if (git != graph.end()) {
            graph[key] = std::min(graph[key], wgt);
        } else {
            graph[key] = wgt;
        }
    }

    struct min_graph_t
    min_graph(timetable &ttbl, int ma = 30, int mb = 30)
    {
        V max_vertex = 0;
        stops_vertices_t stops_vertices;
        stations_vertices_t stations_vertices;
        vertices_stops_t vertices_stops;
        vertices_stations_t vertices_stations;

        std::map<std::pair<V, V>, W> graph;

        for (size_t r = 0; r < ttbl.trips_of.size(); ++r) {
            const auto &route = ttbl.trips_of[r];
            for (size_t t = 0; t < route.size(); ++t) {
                const auto &trip = route[t];
                for (size_t s = 0; s < trip.size() - 1; ++s) {
                    const auto &stop = trip[s];
                    auto w = stop.second - stop.first;
                    auto e = trip[s+1].first - stop.second;
                    auto dep_stop_id = ttbl.route_stops[r][s],
                        arr_stop_id = ttbl.route_stops[r][s + 1];

                    auto it = stops_vertices.find(dep_stop_id);
                    if (it == stops_vertices.end()) {
                        stops_vertices[dep_stop_id] =
                            std::make_pair<>(max_vertex, max_vertex+1);
                        vertices_stops.push_back(dep_stop_id);
                        vertices_stops.push_back(dep_stop_id);
                        max_vertex += 2;
                    }
                    it = stops_vertices.find(arr_stop_id);
                    if (it == stops_vertices.end()) {
                        stops_vertices[arr_stop_id] =
                            std::make_pair<>(max_vertex, max_vertex+1);
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
            const auto &station = ttbl.station_stops[st];
            stations_vertices[st] = max_vertex;
            vertices_stations.push_back(st);
            for (size_t s = 0; s < station.size(); ++s) {
                const auto &stop = stops_vertices[station[s]];
                add_min_edge(graph, stop.first, max_vertex, ma);
                add_min_edge(graph, max_vertex, stop.second, mb);
            }
            ++max_vertex;
        }

        // Is the transfer graph symmetric?
        for (const auto &u : ttbl.transfers) {
            for (const auto &e : ttbl.transfers[u]) {
                if (stations_vertices[u] != stations_vertices[e.dst])
                    add_min_edge(graph,
                                 stations_vertices[u],
                                 stations_vertices[e.dst],
                                 e.wgt);
            }
        }


        std::vector<unit::graph::edge> edges;
        for (const auto &e : graph) {
            edges.push_back(unit::graph::edge(e.first.first,
                                              e.first.second,
                                              e.second));
        }
        auto mg = new mgraph<V, int>(max_vertex, edges);

        return {mg, stops_vertices, stations_vertices,
                vertices_stops, vertices_stations};
    }


    std::pair<label *, label *>
    reachability(hl_t &outhubs, hl_t &inhubs, const V &src, const V &dst)
    {
        // first find the common hub with least distance
        // this is O(n^2), is linear sweep possible?
        label *outlbl = nullptr, *inlbl = nullptr;
        int length = std::numeric_limits<int>::max();
        for (auto &outlabel : outhubs[src]) {
            for (auto &inlabel : inhubs[dst]) {
                if (outlabel.hub == inlabel.hub
                    && outlabel.length + inlabel.length < length) {
                    length = outlabel.length + inlabel.length;
                    inlbl = &inlabel;
                    outlbl = &outlabel;
                }
            }
        }

        if (!outlbl || !inlbl) {
            std::cerr << "Did not find common hub for " << src << " and " << dst
                      << "." << std::endl;
            exit(1);
        }

        return std::make_pair<>(outlbl, inlbl);
    }

    std::list<V>
    buildpath_rec(hl_t &outhubs, hl_t &inhubs, const V &src, const V &dst)
    {
        label *outlbl, *inlbl;
        std::tie(outlbl, inlbl) = reachability(outhubs, inhubs, src, dst);
        V &hub = outlbl->hub;

        std::list<V> path;
        if (outlbl->next_hop != hub)
            path.splice(path.end(),
                        buildpath_rec(outhubs, inhubs, outlbl->next_hop, hub));
        if (inlbl->hub != inlbl->next_hop)
            path.splice(path.end(),
                        buildpath_rec(outhubs, inhubs, hub, inlbl->next_hop));

        path.push_front(src);
        path.push_back(dst);
        return path;
    }

    std::list<V>
    buildpath(hl_t &outhubs, hl_t &inhubs, const V &src, const V &dst) {
        auto path = buildpath_rec(outhubs, inhubs, src, dst);
        path.unique();
        return path;
    }

    void
    retrieve_schedule(const min_graph_t &mg, const std::list<V> &path,
                      const timetable &ttbl, std::vector<trip_stop> &tripstops)
    {
        auto max_stop = mg.vertices_stops.size();
        auto max_station = mg.vertices_stops.size() + mg.vertices_stations.size();

        for (const auto &v : path) {
            if (v < max_stop) { // it's a stop
                timetable::S stop = mg.vertices_stops[v];
                auto &route = ttbl.stop_route[stop];
                trip_stop s = {.vertex = v,
                               .is_stop = true,
                               .station = ttbl.stop_station[stop],
                               .schedule = ttbl.trips_of[route.first][route.second]};
                tripstops.push_back(s);
            } else if (v < max_station) { // it's a station
                trip_stop s = {.vertex = v,
                               .is_stop = false,
                               .station = mg.vertices_stations[v - max_stop],
                               .schedule = std::vector<std::pair<timetable::T, timetable::T>>()};
                tripstops.push_back(s);
            } else {
                std::cerr << v << " is neither a stop[" << max_stop <<
                    "] nor a station[" << max_station << "]." << std::endl;
                exit(1);
            }
        }

        #if 0
        // retrieve distances between stations
        for (size_t i = 0; i < tripstops.size() - 1; ++i) {
            auto &s = tripstops[i];
            auto &t = tripstops[i+1];
            if ((s.is_stop && !t.is_stop) || (!s.is_stop && t.is_stop)) {
                s.dist = mg.min_graph->edge_weight(s.vertex, t.vertex);
            } else if (!s.is_stop && !t.is_stop) {
                s.dist = ttbl.transfers.edge_weight(s.station,
                                                    tripstops[i+1].station);
            }
        }
        #endif
    }

    timetable::T
    find_eat(std::vector<trip_stop> &tripstops,
             const min_graph_t &mg,
             const timetable::T deptime)
    {
        timetable::T arrtime = deptime;
        for (size_t i = 0; i < tripstops.size() - 1; i++) {
            auto &u = tripstops[i], &v = tripstops[i+1];
            if (u.is_stop && v.is_stop) {
                for (size_t j = 0; j < u.schedule.size() - 1; j++) {
                    auto &sched = u.schedule[i];
                    if (arrtime < sched.second) {
                        arrtime = v.schedule[i].first;
                        break;
                    }
                }
            } else {
                arrtime = mg.min_graph->edge_weight(u.vertex, v.vertex);
            }
        }
        return arrtime;
    }

    // We assume that there is only one route between two stops.
    void
    timeprofile(std::vector<timeprofile_t> &profile,
                const min_graph_t &mg,
                timetable &ttbl,
                hl_t &outhubs, hl_t &inhubs,
                const timetable::ST src, const timetable::ST dst,
                const int departure_time = 0)
    {
        // find the source and the destination in the static graph
        V vsrc, vdst;
        auto it = mg.stations_vertices.find(src);
        if (it == mg.stations_vertices.end()) {
            std::cerr << "Could not find source station vertex " << src
                      << "." << std::endl;
            exit(1);
        }
        vsrc = it->second;
        it = mg.stations_vertices.find(dst);
        if (it == mg.stations_vertices.end()) {
            std::cerr << "Could not find destination station vertex " << src
                      << "." << std::endl;
            exit(1);
        }
        vdst = it->second;

        // build the path with the hub labeling
        std::list<V> path = buildpath(outhubs, inhubs, vsrc, vdst);

        // retrieve the time schedule
        std::vector<trip_stop> forward;
        retrieve_schedule(mg, path, ttbl, forward);
        std::vector<trip_stop> backward;
        ttbl.reverse_time();
        retrieve_schedule(mg, path, ttbl, backward);

        std::vector<timeprofile_t> timeprofiles;
        timeprofile_t tp = {0, 0};
        timetable::T t = tp.tarr;
        for (const auto &node : tripstops) {
            if (node.is_stop) {
                const auto &adt = find_arrdeptime(node.schedule, t);

            }
        }
    }

    void usage_exit (char **argv) {
        auto paragraph = [](std::string s, int width=80) -> std::string {
                             std::string acc;
                             while (s.size() > 0) {
                                 int pos = s.size();
                                 if (pos > width) pos = s.rfind(' ', width);
                                 std::string line = s.substr(0, pos);
                                 acc += line + "\n";
                                 s = s.substr(pos);
                             }
                             return acc;
                         };

        std::cerr <<"Usage: "<< argv[0] <<" <command> [params]\n"
                  << paragraph (
                                "Compares a dynamic graph and its static minimum projection.")
                  << paragraph (
                                "With command 'min-hubs-next-hop <stop_times.csv> <transfers.csv>', "
                                "it computes a static minimum graph and outputs a hub-labeling on the "
                                "graph, with the next-hop." )
                  << paragraph (
                                "With command 'comparison <stop_times.csv> <transfers.csv> "
                                "<min_graph.hl> <queries.csv>', it compaires the minimum static graph "
                                "with the dynamic graph." );
        exit(1);
    }

}

int main(int argc, char *argv[]) {
    logging main_log("--");
    double t = main_log.lap();

    std::string cmd(argc >= 2 ? argv[1] : "");
    if (argc < 3 || (cmd != "min-hubs-next-hop"
                     && cmd != "comparison")) {
        usage_exit(argv);
    }

    std::string stop_times(argv[2]);
    std::string transfers(argv[3]);

    main_log.cerr(t) << "Building timetable…" << std::endl;
    timetable ttbl(stop_times, transfers);
    t = main_log.lap();

    main_log.cerr(t) << "Building min graph…" << std::endl;
    min_graph_t mg = min_graph(ttbl);

    t = main_log.lap();
    if (cmd == "min-hubs-next-hop") {
        main_log.cerr(t) << "Building hub labelling on min graph…" << std::endl;
        std::ofstream hlf;
        pl_lab hl(*mg.min_graph);
        hlf.open("min_graph.hl");
        hl_output(hl, hlf);
        hlf.close();
    } else if (cmd == "comparison") {
        auto queries =
            read_csv(std::string(argv[5]), 6, "source","destination",
                     "departure_time","log2_of_station_rank","station_rank","walk_time");
        for (const auto &query : queries) {
            V src = std::stoi(query[0]), dst = std::stoi(query[1]);
            int departure_time = std::stoi(query[2]);
            auto src_station = mg.stations_vertices[src],
                dst_station = mg.stations_vertices[dst];
        }

        hl_t outhubs, inhubs;
        std::ifstream hlf;
        hlf.open(argv[4]);
        hl_input(hlf, outhubs, inhubs);

    }

    t = main_log.lap();
    main_log.cerr(t) << "Done!" << std::endl;


    // graphviz(mg, "test.gv");

    // std::cout << "Minimum waiting time (secs): " << std::endl;
    // std::vector<size_t> waiting_times;
    // size_t min, max, avg;
    // std::tie(min, avg, max) = min_waiting_time(ttbl, waiting_times);
    // std::cout << "  {.min = " << min << ", .avg = " << avg << ", .max = "
    //           << max << "}" << std::endl;



    // size_t in_degree, out_degree;
    // std::tie(in_degree, out_degree) = avg_degree(*mg.graph);
    // std::cout << "Average degree of minimum graph: "
    //           << in_degree << ", " << out_degree << std::endl;
}
