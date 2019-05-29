#ifndef STATIC_MIN_GRAPH_HH
#define STATIC_MIN_GRAPH_HH

#include <cassert>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "timetable.hh"

class static_min_graph {
public:
    using V = int;         // vertices
    using W = int;         // edges weigth
    using T = timetable::T;
    using R = timetable::R;
    using id = timetable::id;

private:
    using static_graph = std::map<std::pair<V, V>, W>;

    struct vertex { V arr, dep; };

    mgraph<V, W> graph;

    V max_stop; // max_stop is valid iff loaded from a timetable
    V max_vertex;

    friend std::ostream& operator<<(std::ostream &out,
                                    const static_min_graph &gr);

public:
    std::map<id, std::map<R, vertex>> id_to_vertex;
    std::map<id, V> id_to_station;
    std::vector<id> node_to_id;

    static_min_graph(const timetable &ttbl, const W ma = 30, const W mb = 30)
        : max_stop(-1), max_vertex(0),
          id_to_vertex(), id_to_station(), node_to_id()
    {
        static_graph min_graph;

        auto station_id_from_ttbl_stop =
            [&ttbl](size_t r, size_t s) {
                auto ttbl_stop = ttbl.route_stops[r][s];
                auto ttbl_station = ttbl.stop_station[ttbl_stop];
                auto &station_id = ttbl.station_id[ttbl_station];
                return station_id;
            };

        auto add_min_edge =
            [&min_graph](std::pair<V, V> arc, W wgt) {
                auto it = min_graph.find(arc);
                if (it == min_graph.end())
                    min_graph[arc] = wgt;
                else
                    it->second = std::min<W>(it->second, wgt);
            };

        auto find_add_vertex =
            [this, &station_id_from_ttbl_stop]
            (size_t r, size_t s) {
                auto id = station_id_from_ttbl_stop(r, s);
                auto itid = id_to_vertex.find(id);
                auto save =
                    [this, &id, &r]() {
                        vertex v = {.arr = max_vertex, .dep = max_vertex + 1};
                        max_vertex += 2;
                        auto str = id + "_" + std::to_string(r);
                        node_to_id.push_back(str + "_arr");
                        node_to_id.push_back(str + "_dep");
                        return v;
                    };

                if (itid == id_to_vertex.end()) {
                    return id_to_vertex[id][r] = save();
                } else {
                    auto itr = itid->second.find(r);
                    if (itr == itid->second.end()) {
                        return itid->second[r] = save();
                    } else {
                        return itr->second;
                    }
                }
            };

        // add trips arcs v1.arr -> v1.dep -> v2.arr -> v2.dep
        for (size_t r = 0; r < ttbl.trips_of.size(); ++r) {    // for each route
            const auto &route = ttbl.trips_of[r];
            for (const auto &trip : route) {                   // for each trip
                assert(trip.size() >= 2);
                vertex v1 = find_add_vertex(r, 0);
                add_min_edge(std::make_pair(v1.arr, v1.dep),
                             trip[0].second - trip[0].first);
                for (size_t s = 1; s < trip.size(); ++s) {     // for each arc
                    const auto &prev = trip[s-1], &stop = trip[s];
                    vertex v2 = find_add_vertex(r, s);
                    add_min_edge(std::make_pair(v2.arr, v2.dep),
                                 stop.second - stop.first);
                    add_min_edge(std::make_pair(v1.dep, v2.arr),
                                 stop.first - prev.second);
                    v1 = v2;
                }
            }
        }

        max_stop = max_vertex - 1;

        // add intra-station arcs
        for (size_t st = 0; st < ttbl.station_stops.size(); ++st) {
            V station = max_vertex++;
            auto id = ttbl.station_id[st];
            id_to_station[id] = station;
            node_to_id.push_back(id);
            for (const auto &stop : id_to_vertex[id]) {
                add_min_edge(std::make_pair(stop.second.arr, station), ma);
                add_min_edge(std::make_pair(station, stop.second.dep), mb);
            }
        }

        // add transfer arcs
        for (const auto &u : ttbl.transfers) {
            auto stu = id_to_station[ttbl.station_id[u]];
            for (const auto &e : ttbl.transfers[u]) {
                if (u == e.dst) continue;
                auto stv = id_to_station[ttbl.station_id[e.dst]];
                add_min_edge(std::make_pair(stu, stv), e.wgt);
                add_min_edge(std::make_pair(stv, stu), e.wgt);
            }
        }

        std::vector<unit::graph::edge> edges;
        for (const auto &e : min_graph)
            edges.push_back(unit::graph::edge(e.first.first,
                                              e.first.second,
                                              e.second));
        auto mg = mgraph<V, W>(max_vertex, edges);
        graph = mg.reverse().reverse();
    }

    static_min_graph(std::istream &f)
        : id_to_vertex(), id_to_station(), node_to_id(),
          max_stop(-1), max_vertex(0)
    {
        std::vector<std::string> elems(3);
        auto split =
            [&elems](const std::string &s) {
                std::stringstream ss(s);
                std::string item;
                elems.clear();
                while(std::getline(ss, item, '_'))
                    elems.push_back(item);
            };

        auto find_add_node =
            [this, &elems, &split](std::string &id) {
                split(id);
                if (elems.size() == 1) { // station
                    auto it = id_to_station.find(elems[0]);
                    V st;
                    if (it == id_to_station.end()) {
                        st = max_vertex++;
                        node_to_id.push_back(elems[0]);
                        id_to_station[elems[0]] = st;
                    } else {
                        st = it->second;
                    }
                    return st;
                } else if (elems.size() == 3) {
                    bool is_arr = elems[2] == "arr";
                    assert(is_arr || elems[2] == "dep");
                    auto save =
                        [this, &elems, &id, is_arr](vertex &v) {
                            *(is_arr ? &v.arr : &v.dep) = max_vertex++;
                            id_to_vertex[elems[0]][std::stoi(elems[1])] = v;
                            if (is_arr) {
                                id.replace(id.end()-3, id.end(), "arr");
                                std::cout << "here: " << v.arr << " " << id
                                          << std::endl;
                                node_to_id.push_back(id);
                                return v.arr;
                            } else {
                                id.replace(id.end()-3, id.end(), "dep");
                                node_to_id.push_back(id);
                                return v.dep;
                            }
                        };
                    auto itid = id_to_vertex.find(elems[0]);
                    vertex v = {.arr = -1, .dep = -1};
                    if (itid == id_to_vertex.end()) {
                        return save(v);
                    } else {
                        auto r = std::stoi(elems[1]);
                        auto itr = itid->second.find(r);
                        if (itr == itid->second.end())
                            return save(v);
                        else if ((is_arr && itr->second.arr == -1)
                                 || (!is_arr && itr->second.dep == -1))
                            return save(itr->second);
                        else
                            return is_arr ? itr->second.arr : itr->second.dep;
                    }
                } else {
                    std::cout << id << ": ";
                    for (const auto &e : elems)
                        std::cout << e << "|";
                    std::cout << std::endl;
                    assert(false);
                }
            };

        std::vector<unit::graph::edge> edges;
        while (!f.eof()) {
            std::string ssrc, sdst;
            W wgt;
            f >> ssrc >> sdst >> wgt;
            if (ssrc == "" || sdst == "") break; // FIXME
            split(ssrc);
            V src = find_add_node(ssrc);
            split(sdst);
            V dst = find_add_node(sdst);
            edges.push_back(unit::graph::edge(src, dst, wgt));
        }

        std::cout << "max_vertex: " << max_vertex << std::endl;

        auto mg = mgraph<V, W>(max_vertex, edges);
        graph = mg.reverse().reverse();
    }

    void
    graphviz_output(std::ostream &f) const
    {
        f << "digraph g {\nrankdir=\"LR\";\n";
        for (const auto &u : graph) {
            if (max_stop < u)
                f << u << "[shape=square];\n";
            for (const auto &e : graph[u]) {
                f << u << " -> " << e.dst << " [label=\"" << e.wgt << "\"";
                if (max_stop < u && max_stop < e.dst)
                    f << ",style=dashed];\n";
                else if (max_stop < u || max_stop < e.dst)
                    f << ",style=dotted];\n";
                else
                    f << "];\n";
            }
        }
        f << "}\n";
    }
};

std::ostream &
operator <<(std::ostream &out, const static_min_graph &gr)
{
    for (const auto &u : gr.graph) {
        for (const auto &e : gr.graph[u]) {
            const auto &uid = gr.node_to_id[u], &vid = gr.node_to_id[e.dst];
            out << uid << " " << vid << " " << e.wgt << std::endl;
        }
    }
    return out;
};

#endif
