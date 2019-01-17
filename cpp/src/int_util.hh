#ifndef INT_UTIL_HH
#define INT_UTIL_HH


template<typename I>
class int_iterator {
    I i;
    bool incr;
public:
    int_iterator(I i, bool incr=true) : i(i), incr(incr) {}
    I operator*() const { return i; }
    int_iterator &operator++() { if(incr) ++i; else --i; return *this; }
    int_iterator &operator--() { assert( ! incr); --i; return *this; }
    bool operator!=(const int_iterator& o) { return i != o.i; }
};


template<typename I>
class range {
    const I from, to;
    bool incr;
public:
    range(I from, I to, bool incr) : from(from), to(to), incr(incr) {} 
    range(I from, I to) : range(from, to, true) { assert(from < to); } 
    int_iterator<I> begin() const { return int_iterator<I>(from, incr); }
    int_iterator<I> end() const { return int_iterator<I>(to, incr); }
};

typedef range<int> irange;

    
#endif // INT_UTIL_HH