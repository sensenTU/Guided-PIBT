
#ifndef search_hpp
#define search_hpp

#include "Types.h"
#include "utils.hpp"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.hpp"

namespace TrafficMAPF{

// Forward declaration to avoid circular dependency
#ifdef USE_BPR_HEURISTIC
class TrajLNS;
#endif
//a astar minimized the opposide traffic flow with existing traffic flow

s_node singleShortestPath(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht,std::vector<int>& traffic, Traj& traj,
    MemoryPool& mem, int start, int goal);

s_node aStarOF(SharedEnvironment* env,
#ifdef USE_BPR_HEURISTIC
    const TrajLNS& lns,  // BPR parameter (const reference)
#endif
    std::vector<Int4>& flow,
    HeuristicTable& ht,std::vector<int>& traffic, Traj& traj,
    MemoryPool& mem, int start, int goal);
}

#endif