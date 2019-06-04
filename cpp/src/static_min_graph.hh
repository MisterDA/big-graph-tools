#ifndef STATIC_MIN_GRAPH_HH
#define STATIC_MIN_GRAPH_HH

#include <cassert>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

#include "timetable.hh"

/* A static minimum graph (SMG) is a graph composed of vertices V and
 * arcs V×V labeled by W. To each vertex in the SMG corresponds an
 * unique stop or station in the timetable (TTBL).
 *
 * The *index* is the integer representation of a vertex. The *id* is
 * the textual representation of a vertex.
 *
 * If the GTFS id is "chatelet" then the station id is "chatelet". For
 * each route r at TTBL station "chatelet", I add SMG vertex id
 * "chatelet_r_arr" and "chatelet_r_dep". Those two vertices belong to
 * a SMG *stop*. I add one more vertex for each TTBL station with the
 * same id, that I call SMG *station*.
 */

template<typename V, // vertex-number (index) type
         typename W> // weight type

class static_min_graph {
public:
    using T = timetable::T;
    using R = timetable::R;
    using id = timetable::id;

    // used when loading the graph from a file
    struct vertex {
        bool is_stop;
        enum stop_type {arr, dep};
        stop_type type;
        int route;
        std::string station_id;    // the id of the station, always available

        explicit vertex(const std::string &stid)
            : is_stop(false), station_id(stid) {}
        vertex(const stop_type &tp, const int r, const std::string &stid)
            : is_stop(true), type(tp), route(r), station_id(stid) {}
    };

    struct stop { V arr, dep; };

    // maps a SMG station vertex to its stops
    std::map<V, std::map<R, stop>> station_stops;
    std::map<id, V> id_to_station;
    std::vector<id> index_to_id;

private:
    using static_graph = std::map<std::pair<V, V>, W>;

    mgraph<V, W> graph;

    template <typename VV, typename WW>
    friend std::ostream& operator<<(std::ostream &out,
                                    const static_min_graph<VV, WW> &gr);

public:
    explicit static_min_graph(const timetable &ttbl,
                              const W ma = 30, const W mb = 30)
        : station_stops(), id_to_station(), index_to_id()
    {
        static_graph min_graph;
        V max_index(0);        // == index_to_id.size();

        auto add_min_edge = [&min_graph](V arr, V dep, W wgt) {
            std::pair<V, V> arc = std::make_pair(arr, dep);
            auto it = min_graph.find(arc);
            if (it == min_graph.end())
                min_graph[arc] = wgt;
            else
                it->second = std::min<W>(it->second, wgt);
        };

        auto find_or_add =
            [this, &ttbl, ma, mb, &max_index, &add_min_edge]
            (timetable::R route, size_t seq)
        {
            // retrieve or create the station
            id station_id = station_id_from_ttbl_stop(ttbl, route, seq);
            V station_idx = find_or_add_station(station_id, max_index);
            max_index = std::max<>(max_index, station_idx);
            // add intra-station arcs
            auto f = [ma, mb, station_idx, &add_min_edge](const stop &s) {
                add_min_edge(s.arr, station_idx, ma);
                add_min_edge(station_idx, s.dep, mb);
                return s;
            };
            return find_or_add_stop(station_id, station_idx,
                                    route, max_index, f);
        };

        // add arcs s1.arr -> s1.dep -> s2.arr -> s2.dep
        for (size_t r = 0; r < ttbl.trips_of.size(); ++r) {    // for each route
            const auto &route = ttbl.trips_of[r];
            for (const auto &trip : route) {                   // for each trip
                assert(trip.size() >= 2);
                stop s1 = find_or_add(r, 0);
                add_min_edge(s1.arr, s1.dep, trip[0].second - trip[0].first);
                for (size_t s = 1; s < trip.size(); ++s) {     // for each arc
                    const auto &prev = trip[s-1], &curr = trip[s];
                    stop s2 = find_or_add(r, s);
                    add_min_edge(s1.dep, s2.arr, curr.first - prev.second);
                    add_min_edge(s2.arr, s2.dep, curr.second - curr.first);
                    s1 = s2;
                }
            }
        }

        // add transfer arcs
        for (const auto &u : ttbl.transfers) {
            auto stu = id_to_station[ttbl.station_id[u]];
            for (const auto &e : ttbl.transfers[u]) {
                if (u == e.dst) continue;
                auto stv = id_to_station[ttbl.station_id[e.dst]];
                add_min_edge(stu, stv, e.wgt);
                add_min_edge(stv, stu, e.wgt);
            }
        }

        std::vector<unit::graph::edge> edges;
        for (const auto &e : min_graph)
            edges.push_back(unit::graph::edge(e.first.first,
                                              e.first.second,
                                              e.second));
        assert(max_index == index_to_id.size());
        auto mg = mgraph<V, W>(max_index, edges);
        graph = mg.reverse().reverse();
    }

    explicit static_min_graph(std::istream &f)
        : station_stops(), id_to_station(), index_to_id()
    {
        V max_index(0);        // == index_to_id.size();

        auto find_or_add = [this, &max_index](const id &id) {
            vertex &v = id_to_vertex(id);
            V station_idx = find_or_add_station(v.station, max_index);
            if (!v.is_stop) return station_idx;
            auto f = [&v](const stop &s) {
                         return v.stop_type == stop::arr ? s.arr : s.dep; };
            return find_or_add_stop(v.station, station_idx,
                                    v.route, max_index, f);
        };

        std::vector<unit::graph::edge> edges;
        while (!f.eof()) {
            std::string ssrc, sdst;
            W wgt;
            f >> ssrc >> sdst >> wgt;
            if (ssrc == "" || sdst == "") break; // FIXME
            V src = find_or_add(ssrc), dst = find_or_add(sdst);
            edges.push_back(unit::graph::edge(src, dst, wgt));
        }

        assert(max_index == index_to_id.size());
        auto mg = mgraph<V, W>(max_index, edges);
        graph = mg.reverse().reverse();
    }

    W
    edge_weight(const V &src, const V &dst) const
    {
        return graph.edge_weight(src, dst);
    }

    bool
    has_edge(const V &u, const V &v) const
    {
        return graph.has_edge(u, v);
    }

    void
    graphviz_output(std::ostream &f) const
    {
        f << "digraph g {\nrankdir=\"LR\";\n";
        for (const auto &u : graph) {
            bool ustat = station_stops.find(u) != station_stops.end();
            f << u << "[label=\"" << index_to_id[u]
              << (ustat ? "\",shape=square];\n" : "\"];\n");
            for (const auto &e : graph[u]) {
                bool vstat = station_stops.find(e.dst) != station_stops.end();
                f << u << " -> " << e.dst << " [label=\"" << e.wgt << "\"";
                if (ustat && vstat)
                    f << ",style=dashed];\n";
                else if (ustat || vstat)
                    f << ",style=dotted];\n";
                else
                    f << "];\n";
            }
        }
        f << "}\n";
    }

private:
    V
    find_or_add_station(const id &station_id, V &max_index)
    {
        const auto &it = id_to_station.find(station_id);
        if (it == id_to_station.end()) {
            id_to_station[station_id] = max_index;
            index_to_id.push_back(station_id);
            return max_index++;
        }
        return it->second;
    }

    template<typename F>
    stop
    find_or_add_stop(const id &station_id, const V station_idx,
                     const timetable::R route, V &max_index,
                     F &f)
    {
        auto &station = station_stops[station_idx];
        const auto &it = station.find(route);
        if (it == station.end()) {
            id arr_id, dep_id;
            std::tie(arr_id, dep_id) = build_stop_ids(station_id, route);
            stop s = {.arr = max_index, .dep = max_index + 1};
            max_index += 2;
            station[route] = s;
            index_to_id.push_back(arr_id);
            index_to_id.push_back(dep_id);
            return f(s);
        }
        return it->second;
    }

    const id
    station_id_from_ttbl_stop(const timetable &ttbl,
                              timetable::R route, size_t seq) const
    {
        const auto ttbl_stop = ttbl.route_stops[route][seq];
        const auto ttbl_station = ttbl.stop_station[ttbl_stop];
        return ttbl.station_id[ttbl_station];
    }

    static const std::pair<id, id>
    build_stop_ids(const id &station, timetable::R route)
    {
        id id(station + "_" + std::to_string(route));
        return std::make_pair(id + "_arr", id + "_dep");
    }

    // Converts an id read from a file to a vertex struct, but does
    // not add the vertex to the graph.
    struct vertex
    id_to_vertex(const id &id) const
    {
        if (id.size() < sizeof("1_3_567") - 1)
            return vertex(id);
        typename vertex::stop_type type;
        auto end = id.substr(id.size() - 3);
        if (end == "arr")
            type = vertex::stop_type::arr;
        else if (end == "dep")
            type = vertex::stop_type::dep;
        else
            return vertex(id);
        auto i = id.rfind('_', id.size() - 5);
        if (i == std::string::npos)
            return vertex(id);
        auto _id = id.substr(0, i);
        auto route = id.substr(i+1, id.size() - i - 5);
        int r;
        try { r = std::stoi(route); }
        catch (std::invalid_argument &e) { return vertex(id); }
        return vertex(type, r, _id);
    }

#if 0 // Maybe depreciated
    bool
    is_station_id(const id &id) const
    {
        auto it = id_to_station.find(id);
        if (it == id_to_station.end()) {
            auto it = id_to_vertex.find(id);
            // crash if the id doesn’t exist at all
            assert(it != id_to_vertex.end());
            return false;
        }
        return true;
    }

    V
    id_to_index(const std::string &id) const
    {
        node n = id_to_node(id);
        if (n.is_stop && n.type == node::arr) {
            return id_to_vertex.at(n.station).at(n.route).arr;
        } else if (n.is_stop && n.type == node::dep) {
            return id_to_vertex.at(n.station).at(n.route).dep;
        } else {
            return id_to_station.at(n.station);
        }
    }

    struct node
    index_to_node(const V &v) const
    {
        return id_to_node(node_to_id[v]);
    }
#endif
};

template <typename V, typename W>
std::ostream &
operator <<(std::ostream &out, const static_min_graph<V, W> &gr)
{
    for (const auto &u : gr.graph) {
        for (const auto &e : gr.graph[u]) {
            const auto &uid = gr.index_to_id[u], &vid = gr.index_to_id[e.dst];
            out << uid << " " << vid << " " << e.wgt << std::endl;
        }
    }
    return out;
};

#endif
