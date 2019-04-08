#include <cassert>
#include <limits>
#include <iostream>
#include <fstream>
#include <zlib.h>
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

struct label_entry {
    int next_hop;
    int hub;
    int dist;
};

typedef std::map<int, std::vector<struct label_entry>> hubset_t;

static bool
label_intersect(std::vector<struct label_entry>& hubs_a,
                std::vector<struct label_entry>& hubs_b)
{
    for (size_t i = 0; i < hubs_a.size(); ++i)
        for (size_t j = i + 1; j < hubs_b.size(); ++j)
            if (hubs_a[i].hub == hubs_b[j].hub)
                return true;
    return false;
}

static void
profile_query(// stop_id -> event -> (vertex_id, trips)
              std::map<int, std::map<int, std::pair<int, std::vector<int>>>>& stop_event,
              hubset_t& outhubs, hubset_t& inhubs,
              int s, int t)
{
    auto event_s = stop_event[s].begin();
    auto event_t = stop_event[t].begin();
    bool prev_intersect;
    std::vector<std::pair<int, int>> journeys;
begin:
    prev_intersect = false;
    for (; event_t != stop_event[t].end(); event_t++) {
        int vs = event_s->second.first,
            vt = event_t->second.first;
        if (label_intersect(inhubs[vs], outhubs[vt])) {
            if (!prev_intersect)
                break;
            prev_intersect = true;
        } else {
            prev_intersect = false;
        }
    }

    for (event_s++; event_s != stop_event[s].end(); event_s++) {
        int vs = event_s->second.first,
            vt = event_t->second.first;
        if (!label_intersect(inhubs[vs], outhubs[vt])) {
            journeys.push_back(std::make_pair<>(std::prev(event_s, 1)->first,
                                                event_t->first));
            goto begin;
        }
    }
}

static void
parse_hubs(const std::string& file,
           hubset_t& outhubs,
           hubset_t& inhubs)
{
    std::ifstream f(file);
    int vertex;
    struct label_entry e;
    char type;
    while (f >> type >> vertex >> e.next_hop >> e.hub >> e.dist) {
        if (type == 'i') {
            inhubs[vertex].push_back(e);
        } else if (type == 'o') {
            outhubs[vertex].push_back(e);
        } else if (type == 'c') {
            // do nothing
        }
    }
    f.close();
}

static mgraph<int, int> *
parse_graph(const std::string& file)
{
    std::ifstream f(file);
    std::vector<unit::graph::edge> edges;
    int max_vector = 0;
    int s, t, w;

    while (f >> s >> t >> w) {
        edges.push_back(unit::graph::edge(s, t, w));
        max_vector = std::max<>(max_vector, std::max<>(s, t));
    }

    auto graph = new mgraph<int, int>(max_vector + 1, edges);
    return graph;
}

static void
compare(mgraph<int, int>& dynamic,
        std::pair<hubset_t, hubset_t> hubs,
        int s, int t)
{

}

int
main(int argc, char *argv[])
{
    auto *dynamic = parse_graph(std::string(argv[1]));
    std::vector<std::pair<hubset_t, hubset_t>> hubs;
    hubs.reserve(argc - 2);
    for (int i = 0; i < argc - 2; ++i) {
        parse_hubs(argv[i+2], hubs[i].first, hubs[i].second);
    }

}
