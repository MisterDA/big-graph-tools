#ifndef MGRAPH_HH
#define MGRAPH_HH

#include <assert.h>
#include <stdio.h>
#include <limits>
#include <vector>
#include <utility>
#include <functional>

#include "edge.hh"

/**
 * Minimalist graph implementation when vertices are ints from 0 to n-1.
 *
 * Example:
 *   typedef mgraph<int> graph;
 *   std:vector<graph::edge> edges;
 *   edges.push_back(graph::edge(1,2,100)); // edge 1 --> 2 with weight 100
 *   edges.push_back(graph::edge(2,3,200));
 *   graph g(4, edges); // 0 is also a vertex
 *   for (int u : g)
 *      for (auto e : g[u]) 
 *         std::cout << u << " " << e.dst << " " << e.wgt << std::endl;
 *
 *   // ignoring weights :
 *   for (int u : g)
 *      for (int v : g[u]) 
 *         std::cout << u << " " << v << std::endl;
 */


template<typename V, // vertex-number type
         typename W, // weight type
         V nb_not_vertex = std::numeric_limits<V>::max()>

class mgraph {
    // Minimalist graph as a flat array of dst,wgt pairs.

public:
    static const V not_vertex = nb_not_vertex;
    typedef V vertex;
    typedef W weight;
    typedef edge::dst_wgt<V,W> edge_head;  // A dst,wgt pair.
    
private:
    V n_;            // number of vertices
    size_t m_;       // number of edges
    size_t *sdeg;    // prefix sum of degrees
    edge_head *adj;  // neighbors of u are at index sdeg[u]
    
public:

    V n() const { return n_; }
    size_t m() const { return m_; }

    ~mgraph() {
        if (sdeg != nullptr) delete[] sdeg;
        if (adj != nullptr) delete[] adj;
    }


    typedef edge::src_dst_wgt<V,W> edge;

    // n must be greater than any vertex number (src or dst) in edg.
    mgraph(V n, const std::vector<edge> &edg) {
        init_from_edges(n, edg);
    }
    
    mgraph(const std::vector<edge> &edg) : sdeg(nullptr), adj(nullptr) {
        set_edges(edg);
    }

    void set_edges(const std::vector<edge> &edg) {
        if (sdeg != nullptr) delete[] sdeg;
        if (adj != nullptr) delete[] adj;
        V n = 0;
        for (auto const e : edg)
            n = std::max(n, std::max(e.src, e.dst) + 1);
        init_from_edges(n, edg);
    }

    mgraph() {
        static std::vector<edge> edg;
        init_from_edges(0, edg);
    }

    mgraph(mgraph&& o) noexcept : n_(o.n_), m_(o.m_), sdeg(o.sdeg), adj(o.adj) {
        o.sdeg = nullptr;
        o.adj = nullptr;
        o.n_ = 0;
        o.m_ = 0;
    }
    
    mgraph(const mgraph& o) : n_(0), m_(0), sdeg(nullptr), adj(nullptr) {
        sdeg = new size_t[o.n_+1];
        adj = new edge_head[o.m_];
        std::copy(o.sdeg, o.sdeg + o.n_ + 1, sdeg);
        std::copy(o.adj, o.adj + o.m_, adj);
        n_ = o.n_;
        m_ = o.m_;
    }
    
    //TODO mgraph(mgraph &&g) n_(g.n_), m_(g.m_), sdeg(g.sdeg), adj(g.adj){}

    V degree(V u) const { return (int) sdeg[u+1] - sdeg[u]; }

    size_t degree_sum(V u) const { return sdeg[u]; }
    
    mgraph reverse() const {
        std::vector<edge> edg;
        edg.reserve(m_);
        for (V u = 0; u < n_; ++u) {
            for (size_t e = sdeg[u]; e < sdeg[u+1]; ++e) {
                edg.push_back(edge(adj[e].dst, u, adj[e].wgt));
            }
        }
        return mgraph(n_, edg);
    }

    // asserts sorted adjacency lists (use g.reverse() or g.reverse().reverse())
    bool has_edge(V u, V v) {
        size_t e1 = sdeg[u], e2 = sdeg[u+1];
        // is v in adj[e1 .. e2-1] ?
        while (e1 < e2) {
            size_t m = (e1 + e2) / 2;
            V w = adj[m].dst;
            if (w == v) return true;
            if (v < w) e2 = m; // in adj[e1 .. m-1]
            else e1 = m + 1; // in adj[m+1 .. e2-1]
        }
        return false;
    }
    
    // asserts sorted adjacency lists (use g.reverse() or g.reverse().reverse())
    W edge_weight(V u, V v) {
        size_t e1 = sdeg[u], e2 = sdeg[u+1];
        // is v in adj[e1 .. e2-1] ?
        while (e1 < e2) {
            size_t m = (e1 + e2) / 2;
            V w = adj[m].dst;
            if (w == v) return adj[m].wgt;
            if (v < w) e2 = m; // in adj[e1 .. m-1]
            else e1 = m + 1; // in adj[m+1 .. e2-1]
        }
        throw std::invalid_argument("mgraph.edge_weight(): edge not found");
    }
    
    static W aggregate_min (W w, W x) { return std::min(w, x); }
    static W aggregate_sum (W w, W x) { return w + x; }
    
    mgraph simple(std::function<W(W,W)> aggr = aggregate_min) const {
        mgraph g(reverse().reverse()); // sort adjacencies
        std::vector<edge> edg;
        edg.reserve(m_);
        for (V u = 0; u < g.n_; ++u) {
            for (size_t e = g.sdeg[u]; e < g.sdeg[u+1]; ++e) {
                W w = g.adj[e].wgt;
                V v = g.adj[e].dst;
                while (e+1 < g.sdeg[u+1] && g.adj[e+1].dst == v) {
                    ++e;
                    w = aggr(w, g.adj[e].wgt);
                }
                edg.push_back(edge(u, v, w));
            }
        }
        return mgraph(n_, edg);
    }

    std::vector<edge> edges() const {
        std::vector<edge> edg(m_);
        size_t i = 0;
        for (V u = 0; u < n_; ++u) {
            for (size_t e = sdeg[u]; e < sdeg[u+1]; ++e) {
                edg[i++] = edge(u, adj[e].dst, adj[e].wgt);
            }
        }
        return edg;
    }

    std::pair<mgraph<V, W, nb_not_vertex>, std::vector<V> >
    subgraph(std::function<bool(V)> is_in, const std::vector<V> &vtx = {})
        const {
        bool use_vtx = vtx.size() > 0;
        if (use_vtx) assert(vtx.size() == n_);

        std::vector<V> vtx_sub, vtx_inv(n_, nb_not_vertex);
        V n_sub = 0;
        for (V u = 0; u < n_; ++u) { if (is_in(u)) ++n_sub; }
        vtx_sub.reserve(n_sub);
        for (V u = 0; u < n_; ++u) {
            if (is_in(u)) {
                vtx_inv[u] = vtx_sub.size();
                vtx_sub.push_back(use_vtx ? vtx[u] : u);
            }
        }
        
        std::vector<edge> edg;
        for (V u = 0; u < n_; ++u) {
            if (is_in(u)) {
                for (size_t e = sdeg[u]; e < sdeg[u+1]; ++e) {
                    V v = adj[e].dst;
                    if (is_in(v)) {
                        edg.push_back(edge(vtx_inv[u], vtx_inv[v], adj[e].wgt));
                    }
                }
            }
        }

        return std::pair<mgraph<V, W, nb_not_vertex>,
                         std::vector<V> >(mgraph(n_sub, edg),
                                          vtx_sub);
    }

    
    //  --------------------- iterators : -----------------------
    //
    // for (int u : g)
    //     for (auto e : g[u])
    //         f(u, e.dst, e.wgt);
    //

    class vtx_iterator {
        V u;
    public:
        vtx_iterator(V u) : u(u) {}
        vtx_iterator(vtx_iterator &&v) : u(v.u) {}
        V operator*() const { return u; }
        vtx_iterator &operator++() { ++u; return (*this); }
        bool operator!=(const vtx_iterator& v) { return u != v.u; }
    };

    vtx_iterator begin() const { return vtx_iterator(0); }
    vtx_iterator end() const { return vtx_iterator(n_); }
    
    class ngb_iterator {
        const mgraph &g;
        const V u;
    public:
        ngb_iterator(const mgraph &g, V u) : g(g), u(u) {}
        typedef const edge_head *eh_iterator;
        eh_iterator begin() const { return g.adj + g.sdeg[u]; }
        eh_iterator end() const { return g.adj + g.sdeg[u+1]; }
    };

    ngb_iterator operator[](V u) const {
        if (u < 0 || u >= n_)
            throw std::invalid_argument("mgraph: not a vertex index: "
                                        + std::to_string(u));
        return ngb_iterator(*this, u);
    }

    // copy assignment:

    mgraph& operator=(const mgraph& other) {
        if (this != &other) { // self-assignment check expected
            if (other.n_ != n_) {
                delete[] sdeg;
                n_ = 0; sdeg = nullptr;       // in case next line throws
                sdeg = new size_t[other.n_+1];
            } 
            if (other.m_ != m_) {
                delete[] adj;
                m_ = 0; adj = nullptr;       // in case next line throws
                adj = new edge_head[other.m_];
            } 
            std::copy(other.adj, other.adj + other.m_, adj);
            std::copy(other.sdeg, other.sdeg + other.n_ + 1, sdeg);
            n_ = other.n_;
            m_ = other.m_;
        }
        return *this;
    }

    // move assignment:

    mgraph& operator=(mgraph&& other) noexcept {
        if(this != &other) {
            delete[] sdeg;
            n_ = 0; sdeg = nullptr;
            delete[] adj;
            m_ = 0; adj = nullptr;
            std::swap(sdeg, other.sdeg);
            std::swap(n_, other.n_);
            std::swap(adj, other.adj);
            std::swap(m_, other.m_);
        }
        return *this;
    }
    
private:

    void init_from_edges(V n, const std::vector<edge> &edg) {
        n_ = n;
        m_ = edg.size();
        
        // degrees:
        sdeg = new size_t [n_+1];
        for (V u = 0; u <= n_; ++u) sdeg[u] = 0;
        for (size_t i = 0; i < m_; ++i) {
            assert(0 <= edg[i].src && edg[i].src < n_);
            assert(0 <= edg[i].dst && edg[i].dst < n_);
            sdeg[edg[i].src] += 1;
        }
        for (V u = 1; u <= n_; ++u) {
            sdeg[u] += sdeg[u-1];
        }
        
        // adjacencies:
        adj = new edge_head [m_+1] ;
        for (size_t i = m_; i > 0; ) {
            --i;
            size_t u = edg[i].src;
            size_t e = sdeg[u] - 1;
            adj[e].wgt = edg[i].wgt;
            adj[e].dst = edg[i].dst;
            sdeg[u] = e;
        }
    }

    
}; // mgraph


namespace unit {

    typedef mgraph<int, int> graph;
    
    void mgraph_test(int n, int deg) {
        std::vector<graph::edge> edg;
        for (int u = 0; u < n; ++u) {
            for (int d = 0; d < deg; ++d) {
                edg.push_back(graph::edge(u, rand() % n, 1));
            }
        }
        std::cerr << "mgraph_test: ";
        graph g(n, edg);
        for (int u : g) {
            for (int v : g[u]) std::cerr << u <<","<< v <<" ";
        }
        std::cerr << "\n";

        // subgraph:
        auto sub = g.subgraph([](int u) { return u % 2 ==0; });
        graph h = sub.first;
        std::vector<int> vtx = sub.second;
        std::cerr << "mgraph_test: ";
        for (int u : h) {
            for (int v : h[u]) std::cerr << vtx[u] <<","<< vtx[v] <<" ";
        }
        std::cerr << "\n";
        g = g.reverse().reverse();
        int hm = 0;
        for (int u : g) {
            for (int v : g[u])
                if (u % 2 == 0 && v % 2 == 0) ++hm;
        }
        assert(h.m() == hm);
        for (int u : h) {
            for (int v : h[u])
                assert(g.has_edge(vtx[u], vtx[v]));
        }

        // subgraph of subgraph:
        sub = h.subgraph([&vtx](int u) { return vtx[u] % 4 == 0; }, vtx);
        h = sub.first;
        vtx = sub.second;
        std::cerr << "mgraph_test: ";
        for (int u : h) {
            for (int v : h[u]) std::cerr << vtx[u] <<","<< vtx[v] <<" ";
        }
        std::cerr << "\n";
        hm = 0;
        for (int u : g) {
            for (int v : g[u])
                if (u % 4 == 0 && v % 4 == 0) ++hm;
        }
        assert(h.m() == hm);
        for (int u : h) {
            for (int v : h[u])
                assert(g.has_edge(vtx[u], vtx[v]));
        }
    }
    
}
    


#endif // MGRAPH_HH
