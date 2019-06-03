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
    auto end = std::chrono::system_clock::now();

    void
    log(const std::string &str)
    {
        end = std::chrono::system_clock::now();
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

    struct journey_stop {
        V node;
        bool is_stop;
        timetable::ST station;
        const std::vector<std::pair<T,T>> *sched;
    };

public:
    struct tp { T tdep, tarr; };

    comparison(const static_min_graph<V, W> &_mg, const timetable &_ttbl,
               const timetable &_ttbl_rev, std::istream &hubs)
        : inhubs(), outhubs(), ttbl(&_ttbl), ttbl_rev(&_ttbl_rev), mg(&_mg)
    {
        hl_input(hubs);
    }

    template <typename f>
    void
    timeprofiles(f &lambda,
                 const std::vector<std::vector<std::string>> queries,
                 const size_t limit,
                 std::ostream &out) const
    {
        std::vector<tp> values(128);
        for (std::remove_const<decltype(limit)>::type q = 0; q < limit; ++q) {
            const auto &query = queries[q];
            V src = mg->id_to_station.at(query[0]),
                dst = mg->id_to_station.at(query[1]);
            int deptime = std::stoi(query[2]);
            lambda(timeprofile(src, dst, values, deptime), out);
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

        std::cerr << "path: ";
        for (auto const &v : path)
            std::cerr << v << " ";
        std::cerr << std::endl;

        // retrieve the time schedule
        std::vector<journey_stop> forward;
        retrieve_schedule(path, ttbl, forward);
        std::vector<journey_stop> backward;
        retrieve_schedule(path, ttbl_rev, backward);

        tp tp = {.tdep = deptime, .tarr = 0};
        while (true) {
            tp.tarr = find_eat(forward, tp.tdep);
            std::cerr << "EAT(" << tp.tdep << ") = " << tp.tarr << std::endl;
            if (tp.tarr == -1)
                break;
            tp.tdep = -find_eat_rev(backward, -tp.tarr);
            std::cerr << "REAT(" << tp.tarr << ") = " << tp.tdep << std::endl;
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

    // Retrieve the schedule at each node in the path, following the routes
    void
    retrieve_schedule(const std::list<V> &path,
                      const timetable *ttbl,
                      std::vector<journey_stop> &tripstops) const
    {
        for (const auto &v : path) {
            // fetch the id from the static_min_graph index
            const auto &node = mg->index_to_node(v);
            const timetable::ST &station_idx =
                ttbl->id_to_station.at(node.station);
            if (node.is_stop) {
                // fetch the timetable index corresponding to this node
                bool found(false);
                for (const auto &st : ttbl->station_stops.at(station_idx)) {
                    const auto &route = ttbl->stop_route.at(st);
                    if (route.first == node.route) {
                        const auto *sched1 = &ttbl->trips_of.at(route.first);
                        const auto *sched2 = &sched1->at(route.second);
                        tripstops.push_back({v, node.is_stop, station_idx,
                                             sched2});
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    std::cerr << "Could not find timetable index for node " << v
                              << std::endl;
                    assert(false);
                }
            } else {
                tripstops.push_back({v, node.is_stop, station_idx, nullptr});
            }
        }
    }

    timetable::T
    find_eat(const std::vector<journey_stop> &tripstops,
             const timetable::T deptime) const
    {
        timetable::T arrtime = deptime;
        for (size_t i = 0; i < tripstops.size() - 1; ++i) {
            auto &u = tripstops[i], &v = tripstops[i+1];
            if (u.is_stop && v.is_stop) {
                bool found = false;
                // const auto
                //     &usched = ttbl->trips_of.at(u.route.first)
                //                             .at(u.route.second),
                //     &vsched = ttbl->trips_of.at(v.route.first)
                //                             .at(v.route.second);
                const auto &usched = u.sched, &vsched = v.sched;


                std::cout << std::endl;
                std::cout << "u: " << u.node << ", " << usched->size() << " "
                          << "v: " << v.node << ", " << vsched->size() << std::endl;
                for (const auto &e : *usched)
                    std::cout << e.first << " " << e.second << std::endl;
                std::cout << std::endl;
                for (const auto &e : *vsched)
                    std::cout << e.first << " " << e.second << std::endl;
                std::cout << std::endl;

                assert(usched->size() == vsched->size());
                for (size_t j = 0; j < usched->size() - 1; ++j) {
                    if (arrtime < usched->at(j).second) {
                        arrtime = vsched->at(j).first;
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return -1;
            } else {
                try {
                    arrtime += mg->edge_weight(u.node, v.node);
                } catch (const std::invalid_argument &e) {
                    std::cerr << e.what() << " " << u.node << "->"<< v.node
                              << std::endl;
                    return -1;
                }
            }
        }
        return arrtime;
    }

    timetable::T
    find_eat_rev(const std::vector<journey_stop> &tripstops,
                 const timetable::T deptime) const
    {
        timetable::T arrtime = deptime;
        for (size_t i = tripstops.size() - 1; i > 0; --i) {
            auto &u = tripstops[i], &v = tripstops[i-1];
            if (u.is_stop && v.is_stop) {
                bool found = false;
                // const auto
                //     &usched = ttbl_rev->trips_of[u.route.first][u.route.second],
                //     &vsched = ttbl_rev->trips_of[v.route.first][v.route.second];
                const auto &usched = u.sched, &vsched = v.sched;
                assert(usched->size() == vsched->size());
                for (size_t j = 0; j < usched->size() - 1; ++j) {
                    if (arrtime < usched->at(j).second) {
                        arrtime = vsched->at(j).first;
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return -1;
            } else {
                try {
                    arrtime -= mg->edge_weight(u.node, v.node);
                } catch (const std::invalid_argument &e) {
                    std::cerr << e.what() << " " << u.node << "->" << v.node
                              << std::endl;
                    return -1;
                }
            }
        }
        return arrtime;
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
    f(const std::vector<comparison::tp> &tps, std::ostream &out) {
        out << "arrival,departure" << std::endl;
        for (const auto &tp : tps)
            out << tp.tarr << "," << tp.tdep << std::endl;
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
    } else if (argv[1] == commands[1].first && argc == commands[1].second + 2) {
        timetable ttbl(argv[2], argv[3]), ttbl_rev(argv[2], argv[3]);
        ttbl_rev.reverse_time();
        log("loaded timetable.");

        std::ifstream gr(argv[4]);
        if (gr.bad()) {
            perror(argv[4]);
            exit(EXIT_FAILURE);
        }
        static_min_graph<int, int> mg(gr);
        log("loaded static_min_graph.");

        std::ifstream hl(argv[5]);
        if (hl.bad()) {
            perror(argv[5]);
            exit(EXIT_FAILURE);
        }
        log(argv[5]);
        comparison cmp(mg, ttbl, ttbl_rev, hl);
        log("loaded comparison.");

        auto queries(read_csv(argv[6], 6, "source","destination",
                              "departure_time","log2_of_station_rank",
                              "station_rank", "walk_time"));
        auto limit(_limit == 0 ? queries.size() :
                   std::min(queries.size(), _limit));
        log("loaded queries.");

        cmp.timeprofiles(f, queries, limit, std::cout);
    } else {
        usage_exit(argv[0]);
    }
}
