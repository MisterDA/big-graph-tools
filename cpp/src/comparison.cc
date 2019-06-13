#include <cstring>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <type_traits>
#include <chrono>

#include "mgraph.hh"
#include "timetable.hh"
#include "traversal.hh"
#include "pruned_landmark_labeling.hh"
#include "static_min_graph.hh"

namespace {
    void
    usage_exit(const char *name)
    {
        std::cout << name << " static_min_graph <stop_times.csv> "
            "<transfers.csv> <output.gr> [output.gv]" << std::endl
                  << "  Generates a static minimum graph usable for hub-"
            "labeling." << std::endl
                  << std::endl
                  << name << " cmp <stop_times.csv> <transfers.csv> "
            "<static_min_graph.gr> <static_min_graph.hl> <queries.csv> "
            "<output.csv>" << std::endl
                  << "  Compares the static minimum graph against RAPTOR on "
            "given queries." << std::endl;
        exit(EXIT_SUCCESS);
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    void
    log(const std::string &str)
    {
        end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dur = end-start;
        std::cerr << dur.count() << "s: " << str << std::endl;
    }
};

class comparison {
private:
    using V = int;
    using W = int;
    using T = timetable::T;

    using pl_lab = pruned_landmark_labeling<mgraph<V, W>>;
    struct label {
        V hub, next_hop;
        W length;
    };
    using hl = std::map<V, std::vector<label>>;
    hl inhubs, outhubs;

    const timetable *ttbl, *ttbl_rev;
    const static_min_graph<V, W> *mg;

    struct stop_schedule {
        V index;
        enum {station, arr, dep} type;
        timetable::R route; // only if type is arr or dep
        const std::vector<T> *events, *events_rev; // only if type is arr or dep
    };

public:
    struct tp { T tdep, tarr; };

    comparison(const static_min_graph<V, W> &_mg, const timetable &_ttbl,
               const timetable &_ttbl_rev, std::istream &hubs)
        : inhubs(), outhubs(), ttbl(&_ttbl), ttbl_rev(&_ttbl_rev), mg(&_mg)
    {
        assert((void *)ttbl != (void *)ttbl_rev);
        hl_input(hubs);
    }

    template <typename f>
    void
    timeprofiles(f &lambda,
                 const std::vector<std::vector<std::string>> queries,
                 const size_t limit,
                 std::ostream &out) const
    {
        std::vector<tp> values;
        values.reserve(128);
        out << "query,departure,arrival" << std::endl;
        for (size_t q = 0; q < limit; ++q) {
            values.clear();
            const auto &query = queries[q];
            V src = mg->id_to_station.at(query[0]),
                dst = mg->id_to_station.at(query[1]);
            int deptime = std::stoi(query[2]);
            lambda(q, timeprofile(src, dst, values, deptime), out);
            log("query " + std::to_string(q) + " done.");
        }
    }

    // Find the timeprofile of the journey from src at a given
    // departure time to dst.
    const std::vector<tp> &
    timeprofile(const V &src, const V &dst,
                std::vector<tp> &timeprofiles,
                const int deptime = 0) const
    {
        // build the path with the hub labeling
        std::list<V> path = buildpath(src, dst);
        std::vector<stop_schedule> journey;
        transfers_schedule_from_path(path, journey);

        tp tp = {.tdep = deptime, .tarr = 0};
        while (true) {
            tp.tarr = earliest_arrival_time(journey, tp.tdep);
            // std::cout << "EAT(" << tp.tdep << ") = " << tp.tarr << std::endl;
            if (tp.tarr == -1)
                break;
            tp.tdep = -earliest_arrival_time_rev(journey, -tp.tarr);
            // std::cout << "REAT(" << tp.tarr << ") = " << tp.tdep << std::endl;
            if (tp.tdep == -1)
                break;
            timeprofiles.push_back(tp);
            ++tp.tdep;
        }

        return timeprofiles;
    }

private:
    // Return the first common hub of least distance
    std::pair<const label *, const label *>
    reachability(const V &src, const V &dst) const
    {
        // this is O(n^2), is linear sweep possible?
        const label *outlbl = nullptr, *inlbl = nullptr;
        int length = std::numeric_limits<int>::max();
        for (auto &outlabel : outhubs.at(src)) {
            for (auto &inlabel : inhubs.at(dst)) {
                if (outlabel.hub == inlabel.hub
                    && outlabel.length + inlabel.length < length) {
                    length = outlabel.length + inlabel.length;
                    inlbl = &inlabel;
                    outlbl = &outlabel;
                }
            }
        }

        if (!outlbl || !inlbl) {
            std::cerr << "No common hub for " << src << " and " << dst
                      << "." << std::endl;
            exit(1);
        }

        return std::make_pair<>(outlbl, inlbl);
    }

    std::list<V>
    buildpath_rec(const V &src, const V &dst) const
    {
        const label *outlbl, *inlbl;
        std::tie(outlbl, inlbl) = reachability(src, dst);
        const V &hub = outlbl->hub;

        std::list<V> path;
        if (outlbl->next_hop != hub)
            path.splice(path.end(), buildpath_rec(outlbl->next_hop, hub));
        if (inlbl->hub != inlbl->next_hop)
            path.splice(path.end(), buildpath_rec(hub, inlbl->next_hop));

        path.push_front(src);
        path.push_back(dst);
        return path;
    }

    // Returns the path between src and dst wrt the hub labeling
    std::list<V>
    buildpath(const V &src, const V &dst) const
    {
        auto path = buildpath_rec(src, dst);
        path.unique();
        return path;
    }

    void
    transfers_schedule_from_path(const std::list<V> &path,
                                 std::vector<stop_schedule> &journey) const
    {
        auto retrieve_schedule = [this](const V &v)
            -> stop_schedule
        {
            // fetch the id from the static_min_graph index
            const auto &vertex = mg->index_to_vertex(v);
            const timetable::ST &station_idx =
                ttbl->id_to_station.at(vertex.station_id);
            if (vertex.is_stop) {
                for (const auto &stop : ttbl->station_stops.at(station_idx)) {
                    if (ttbl->stop_route.at(stop).first == vertex.route) {
                        switch (vertex.type) {
                        case decltype(vertex.type)::arr:
                            return {v, stop_schedule::arr, vertex.route,
                                    &ttbl->stop_arrivals.at(stop),
                                    &ttbl_rev->stop_arrivals.at(stop)};
                        case decltype(vertex.type)::dep:
                            return {v, stop_schedule::dep, vertex.route,
                                    &ttbl->stop_departures.at(stop),
                                    &ttbl_rev->stop_arrivals.at(stop)};
                        }
                    }
                }
                std::cerr << "Could not find " << v << " in timetable."
                          << std::endl;
                assert(false);
            } else {
                return {v, stop_schedule::station, -1, nullptr, nullptr};
            }
        };
        auto it = path.begin();
        assert(it != path.end());
        auto curr = retrieve_schedule(*it);
        journey.push_back(curr);
        while (std::next(it) != path.end()) {
            auto next = retrieve_schedule(*std::next(it));
            // if one is a station, it counts as a transfer
            if (curr.type == stop_schedule::station
                || next.type == stop_schedule::station) {
                if (journey.back().index != curr.index)
                    journey.push_back(curr);
                journey.push_back(next);
            }
            curr = next;
            ++it;
        }
    }

    timetable::T
    earliest_arrival_time(const std::vector<stop_schedule> &journey,
                          const timetable::T deptime) const
    {
        timetable::T arrtime = deptime;
        size_t j = 0;
        for (size_t i = 0; i < journey.size() - 1; ++i) {
            const auto &u = journey[i], &v = journey[i+1];
            if (u.type == stop_schedule::dep &&
                v.type == stop_schedule::arr) {
                auto it = std::lower_bound(u.events->begin() + j,
                                          u.events->end(), arrtime);
                if (it == u.events->end())
                    return -1;
                arrtime = (*v.events)[it - u.events->begin()];
            } else if (u.type == stop_schedule::arr &&
                       v.type == stop_schedule::dep) {
                auto it = std::lower_bound(v.events->begin() + j,
                                           v.events->end(),
                                           arrtime);
                if (it == v.events->end())
                    return -1;
                arrtime = *it;
            } else {
                try {
                    arrtime += mg->edge_weight(u.index, v.index);
                } catch (const std::invalid_argument &e) {
                    return -1;
                }
            }
        }
        return arrtime;
    }

    timetable::T
    earliest_arrival_time_rev(const std::vector<stop_schedule> &journey,
                              const timetable::T arrtime) const
    {
        timetable::T deptime = arrtime;
        size_t j = 0;
        for (size_t i = journey.size() - 1; i > 0; --i) {
            auto &u = journey[i-1], &v = journey[i];
            if (u.type == stop_schedule::dep &&
                v.type == stop_schedule::arr) {
                // find the first departure time from u after arrtime
                auto it = std::lower_bound(v.events_rev->begin() + j,
                                           v.events_rev->end(),
                                           deptime);
                if (it == v.events_rev->end())
                    return 1;
                deptime = (*u.events_rev)[it - v.events_rev->begin()];
            } else if (u.type == stop_schedule::arr &&
                       v.type == stop_schedule::dep) {
                auto it = std::lower_bound(v.events_rev->begin() + j,
                                           v.events_rev->end(),
                                           deptime);
                if (it == v.events_rev->end())
                    return 1;
                deptime = (*u.events_rev)[it - v.events_rev->begin()];
            } else {
                try {
                    deptime += mg->edge_weight(u.index, v.index);
                } catch (const std::invalid_argument &e) {
                    return 1;
                }
            }
        }
        return deptime;
    }

    void
    hl_input(std::istream &s)
    {
        char t;
        std::string vertex_id, next_hop_id, hub_id;
        V vertex, next_hop, hub;
        int length;

        while (!s.eof()) {
            s >> t;
            if (t == 'i') {
                s >> hub_id >> next_hop_id >> vertex_id >> length;
                hub = mg->id_to_index(hub_id);
                next_hop = mg->id_to_index(next_hop_id);
                vertex = mg->id_to_index(vertex_id);
                inhubs[vertex].push_back({.hub = hub,
                                          .next_hop = next_hop,
                                          .length = length});
            } else if (t == 'o') {
                s >> vertex_id >> next_hop_id >> hub_id >> length;
                hub = mg->id_to_index(hub_id);
                next_hop = mg->id_to_index(next_hop_id);
                vertex = mg->id_to_index(vertex_id);
                outhubs[vertex].push_back({.hub = hub,
                                           .next_hop = next_hop,
                                           .length = length});
            } else {
                s.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
        }
    }
};

namespace {
    void
    f(const size_t query, const std::vector<comparison::tp> &tps,
      std::ostream &out) {
        for (const auto &tp : tps)
            out << query << "," << tp.tdep << "," << tp.tarr << std::endl;
    }

};

int
main(int argc, const char *argv[])
{
    const std::pair<std::string, int> commands[] =
        { std::make_pair<>("static_min_graph", 3),
          std::make_pair<>("cmp", 6) };
    if (argc < 2)
        usage_exit(argv[0]);


    size_t _limit = 10;

    if (argv[1] == commands[0].first && argc >= commands[0].second + 2) {
        timetable ttbl(argv[2], argv[3]);
        static_min_graph<int, int> mg(ttbl);
        std::ofstream f(argv[4]);
        if (f.bad()) {
            perror(argv[4]);
            exit(EXIT_FAILURE);
        }
        f << mg;
        f.close();

        if (argc == 6) {
            std::ofstream f(argv[5]);
            if (f.bad()) {
                perror(argv[5]);
                exit(EXIT_FAILURE);
            }
            mg.graphviz_output(f);
            f.close();
        }
    } else if (argv[1] == commands[1].first
               && (argc == commands[1].second + 1
                   || argc == commands[1].second + 2)) {
        timetable ttbl(argv[2], argv[3]);
        timetable ttbl_rev(ttbl);
        ttbl_rev.check();
        ttbl_rev.reverse_time();
        log("loaded timetables.");

        std::ifstream gr(argv[4]);
        if (gr.bad()) {
            perror(argv[4]);
            exit(EXIT_FAILURE);
        }
        static_min_graph<int, int> mg(gr);
        gr.close();
        log("loaded static_min_graph.");

        std::ifstream hl(argv[5]);
        if (hl.bad()) {
            perror(argv[5]);
            exit(EXIT_FAILURE);
        }
        log("loaded hub labeling");
        comparison cmp(mg, ttbl, ttbl_rev, hl);
        hl.close();
        log("loaded comparison.");

        auto queries(read_csv(argv[6], 6, "source","destination",
                              "departure_time","log2_of_station_rank",
                              "station_rank", "walk_time"));
        auto limit(_limit == 0 ? queries.size() :
                   std::min(queries.size(), _limit));
        log("loaded queries.");

        std::ostream *out = &std::cout;
        if (argc == 8) {
            out = new std::ofstream((argv[7]));
            if (out->bad()) {
                perror(argv[7]);
                exit(EXIT_FAILURE);
            }
        }
        cmp.timeprofiles(f, queries, limit, *out);
        if (out != &std::cout) {
            static_cast<std::ofstream *>(out)->close();
            delete out;
        }
    } else {
        usage_exit(argv[0]);
    }
}
