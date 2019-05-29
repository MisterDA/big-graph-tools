#include <cstring>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <type_traits>

#include "mgraph.hh"
#include "timetable.hh"
#include "traversal.hh"
#include "pruned_landmark_labeling.hh"
#include "static_min_graph.hh"

class comparison {
private:
    using V = static_min_graph::V;
    using W = static_min_graph::W;
    using T = static_min_graph::T;

    using pl_lab = pruned_landmark_labeling<mgraph<V, W>>;
    struct label {
        V hub, next_hop;
        W length;
    };
    using hl = std::map<V, std::vector<label>>;
    hl inhubs, outhubs;

    const timetable *ttbl;
    const static_min_graph *mg;

    struct tp { T tdep, tarr; };

    const std::vector<std::vector<std::string>> queries;
    const decltype(queries)::size_type limit;

public:
    comparison(const static_min_graph &_mg, const timetable &_ttbl,
               std::istream &hubs,
               const std::string &queries_path, const size_t _limit = 0)
        : inhubs(), outhubs(), ttbl(&_ttbl), mg(&_mg),
          queries(read_csv(queries_path, 6, "source","destination",
                           "departure_time","log2_of_station_rank",
                           "station_rank", "walk_time")),
          limit(_limit == 0 ? queries.size() : std::min(queries.size(), _limit))
    {
        hl_input(hubs);
    }

    template <typename f>
    void
    timeprofiles(f &lambda) const
    {
        for (std::remove_const<decltype(limit)>::type q = 0; q < limit; ++q) {
            const auto &query = queries[q];
            V src = mg->id_to_station.at(query[0]),
                dst = mg->id_to_station.at(query[1]);
            int deptime = std::stoi(query[2]);
            lambda(timeprofile(src, dst, deptime));
        }
    }

    const std::vector<tp>
    timeprofile(const V src, const V dst, const T deptime) const
    {
        return std::vector<tp>();
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
    hl_input(std::istream &s)
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
};

int
main(void)
{
    std::cout << "timetable" << std::endl;
    timetable ttbl("stop_times.csv", "transfers.csv");

    std::cout << "min graph" << std::endl;
    static_min_graph mg1(ttbl);

    std::ofstream f1("min_graph1.gr");
    f1 << mg1;
    f1.close();

    std::ifstream f2("min_graph1.gr");
    std::ofstream f3("min_graph2.gr");
    static_min_graph mg2(f2);
    f3 << mg2;
    f2.close();
    f3.close();
}
