#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"

#include <set>
#include <array>

namespace TrafficMAPF{
enum ADAPTIVE {RANDOM, CONGESTION, COUNT};

struct FlowHeuristic{
    HeuristicTable* h; 
    int target;
    int origin;
    pqueue_min_of open;
    MemoryPool mem;


    bool empty(){
        return mem.generated() == 0;
    }
    void reset(){
        // op_flows.clear();
        // depths.clear();
        // dists.clear();
        open.clear();
        mem.reset();
    }

};

class TrajLNS{
    public:
    SharedEnvironment* env;
    std::vector<int> tasks;

    TimePoint start_time;
    int t_ms=0;


    std::vector<Traj> trajs;
    std::vector<Int4> flow;
    std::vector<HeuristicTable> heuristics;
    std::vector<Dist2Path> traj_dists;
    std::vector<s_node> goal_nodes;// store the goal node of single agent search for each agent. contains all cost information.
    // DH dh;
    std::vector<FlowHeuristic> flow_heuristics;

    std::vector<double> weights; //weights for adaptive lns

    double decay_factor = 0.001;
    double reaction_factor = 0.1;

    int group_size = LNS_GROUP_SIZE;


    std::vector<std::set<int>> occupations;
    std::vector<bool> tabu_list;
    int num_in_tablu=0;

    int traj_inited = 0;
    int dist2path_inited = 0;
    int tdh_build = 0;

    int op_flow = 0;
    int vertex_flow = 0;
    int soc = 0;

    MemoryPool mem;

    // ========== BPR (Bureau of Public Roads) Cost Function ==========
    // Directional flow with EMA (Exponential Moving Average) for BPR cost calculation
    std::vector<std::array<double, 4>> directional_flow;

    // BPR parameters (fixed-point scaling for integer-based A*)
    static constexpr int COST_SCALE = 1000;           // Fixed-point scaling factor
    static constexpr int BPR_T0 = 1000;               // Free-flow time (1.0 * COST_SCALE)
    static constexpr double BPR_ALPHA = 0.15;         // BPR α parameter
    static constexpr double BPR_BETA = 4.0;           // BPR β parameter (fixed at 4)
    static constexpr double C_MAX = 1.0;              // Maximum capacity
    static constexpr double GAMMA = 0.8;              // Reverse flow impact coefficient
    static constexpr double EMA_ETA = 0.2;            // EMA smoothing coefficient
    static constexpr double MIN_CAPACITY = 0.01;      // Minimum capacity to prevent division by zero

    void init_mem(){
        mem.init(env->map.size());
    }

    void init_bpr_flow(){
        directional_flow.resize(env->map.size());
        for (auto& flow_array : directional_flow) {
            flow_array.fill(0.0);  // Initialize all directional flows to 0
        }
    }

    TrajLNS(SharedEnvironment* env):
        env(env),
        trajs(env->num_of_agents), tasks(env->num_of_agents),tabu_list(env->num_of_agents,false),
        flow(env->map.size(),Int4({0,0,0,0})), heuristics(env->map.size()),
        flow_heuristics(env->num_of_agents),
        traj_dists(env->num_of_agents),goal_nodes(env->num_of_agents),occupations(env->map.size()),
        directional_flow(env->map.size()){  // Initialize BPR directional flow
            weights.resize(ADAPTIVE::COUNT,1.0);
            for (auto& flow_array : directional_flow) {
                flow_array.fill(0.0);  // Initialize all directional flows to 0
            }
        };

    TrajLNS(){};
};
}
#endif