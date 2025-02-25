#pragma once

#include <tuple>
#include <list>
#include <vector>
#include <set>
#include <ctime>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using std::vector;
using std::list;
using std::set;
// using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;

// #define NDEBUG 

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

struct PathEntry
{
	int location = -1;
	// bool single = false;
	int mdd_width;  // TODO:: Myabe this can be deleted as we always build/look for MDDs when we classify conflicts
  bool is_goal;
  int task = -1;

	bool is_single() const
	{
		return mdd_width == 1;
	}
	// PathEntry(int loc = -1) { location = loc; single = false; }
};

// typedef vector<PathEntry> Path;

struct Path{
  int begin_time = 0;
  int agent;
  int task;
  int end_time(){
    return begin_time + size() - 1;
  }

  vector<PathEntry> path;
  vector<int> timestamps;

  bool empty() const {return path.empty();}
  size_t size() const {return path.size();}
  PathEntry & back() {return path.back();}
  PathEntry & front() {return path.front();}
  const PathEntry & back() const {return path.back();}
  const PathEntry & front() const {return path.front();}
  PathEntry & at(int idx) {return path[idx];}

  PathEntry & operator[] (int idx) {return path[idx];}
  const PathEntry & operator[] (int idx) const {return path[idx];}

  Path(){};
  Path(int size): path(vector<PathEntry>(size)){}

};

std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
/*struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};*/

