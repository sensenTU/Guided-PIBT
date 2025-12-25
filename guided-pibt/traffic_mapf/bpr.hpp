#ifndef BPR_HPP
#define BPR_HPP

#include "TrajLNS.h"
#include <algorithm>
#include <climits>

namespace TrafficMAPF {

// ========== BPR Cost Calculation Functions ==========

// Calculate BPR edge cost using fixed-point arithmetic
// Args:
//   f_co - co-directional flow (same direction as the agent)
//   f_reverse - reverse directional flow (opposite direction)
// Returns:
//   Integer cost scaled by COST_SCALE (1000)
inline int calculate_bpr_cost(double f_co, double f_reverse) {
    // Calculate effective capacity: C_eff = C_max - γ * f_reverse
    double c_eff = TrajLNS::C_MAX - TrajLNS::GAMMA * f_reverse;
    c_eff = std::max(c_eff, TrajLNS::MIN_CAPACITY);  // Prevent division by zero

    // BPR formula: t = t0 * [1 + α * (f/C)^β]
    // Optimization: β=4 is fixed, avoid std::pow, use direct multiplication
    double ratio = f_co / c_eff;
    double ratio2 = ratio * ratio;
    double ratio4 = ratio2 * ratio2;  // (f/C)^4

    double cost_double = TrajLNS::BPR_T0 * (1.0 + TrajLNS::BPR_ALPHA * ratio4);

    // DEBUG: Track unusually high costs
    static int high_cost_count = 0;
    if (cost_double > 10000 && high_cost_count < 10) {
        std::cerr << "BPR High Cost #" << (high_cost_count + 1)
                  << ": f_co=" << f_co << ", f_reverse=" << f_reverse
                  << ", c_eff=" << c_eff << ", ratio4=" << ratio4
                  << ", cost=" << cost_double << std::endl;
        high_cost_count++;
    }

    // Overflow protection: clamp to maximum safe integer value
    // Use INT_MAX/2 to leave room for further calculations
    constexpr int MAX_SAFE_COST = INT_MAX / 2;
    if (cost_double > MAX_SAFE_COST) {
        cost_double = MAX_SAFE_COST;
    }

    // Convert to fixed-point integer (round to nearest)
    int cost = static_cast<int>(cost_double + 0.5);

    return cost;
}

// Get BPR edge cost for edge (u -> v)
// Args:
//   lns - Trajectory LNS object (const reference)
//   u - source location
//   v - target location
// Returns:
//   Integer cost scaled by COST_SCALE (1000)
inline int get_bpr_edge_cost(const TrajLNS& lns, int u, int v) {
    // Boundary check to prevent segmentation fault
    int size = (int)lns.directional_flow.size();
    if (u < 0 || u >= size || v < 0 || v >= size) {
        // Invalid edge - return maximum penalty
        static int oob_error_count = 0;
        if (oob_error_count < 20) {
            std::cerr << "BPR OOB Error #" << (oob_error_count + 1)
                      << ": u=" << u << ", v=" << v << ", size=" << size
                      << " (u_valid=" << (u >= 0 && u < size)
                      << ", v_valid=" << (v >= 0 && v < size) << ")" << std::endl;
            oob_error_count++;
        }
        return 10000;
    }

    // Check if map locations are valid (not obstacles)
    if (u >= (int)lns.env->map.size() || v >= (int)lns.env->map.size()) {
        static int map_oob_count = 0;
        if (map_oob_count < 10) {
            std::cerr << "BPR Map OOB #" << (map_oob_count + 1)
                      << ": u=" << u << ", v=" << v << ", map.size()=" << lns.env->map.size() << std::endl;
            map_oob_count++;
        }
        return 10000;
    }

    // Check if locations are traversable (not obstacles)
    if (lns.env->map[u] == 1 || lns.env->map[v] == 1) {
        // One or both locations are obstacles
        static int obstacle_count = 0;
        if (obstacle_count < 10) {
            std::cerr << "BPR Obstacle #" << (obstacle_count + 1)
                      << ": u=" << u << "(map=" << (int)lns.env->map[u] << ")"
                      << ", v=" << v << "(map=" << (int)lns.env->map[v] << ")" << std::endl;
            obstacle_count++;
        }
        return 10000;
    }

    // Special case: wait action (u == v)
    if (u == v) {
        // No movement, return free-flow cost
        return TrajLNS::BPR_T0;
    }

    // Calculate direction using existing utility function
    int diff = v - u;
    int d = get_d(diff, lns.env);

    // Validate direction
    if (d < 0 || d >= 4) {
        static int invalid_dir_count = 0;
        if (invalid_dir_count < 10) {
            std::cerr << "BPR Invalid Dir #" << (invalid_dir_count + 1)
                      << ": u=" << u << ", v=" << v << ", diff=" << diff
                      << ", d=" << d << std::endl;
            invalid_dir_count++;
        }
        return 10000;
    }

    // Co-directional flow (u -> v)
    double f_co = lns.directional_flow[u][d];

    // Reverse directional flow (v -> u)
    double f_reverse = lns.directional_flow[v][(d + 2) % 4];

    // Calculate BPR cost with T0 = 1000 (already scaled by COST_SCALE)
    // NO normalization: keep full precision to preserve gradient information
    // Free-flow: ~1000, Light congestion: ~1050, Heavy congestion: ~2000+
    int cost = calculate_bpr_cost(f_co, f_reverse);

    // Clamp to prevent overflow in A* g_score accumulation
    // Maximum single edge cost capped at 1000000 (1000x extreme congestion)
    // This allows ~2000 steps before INT_MAX overflow
    constexpr int MAX_BPR_COST = 1000000;
    return std::min(cost, MAX_BPR_COST);
}

// ========== EMA Flow Update Functions ==========

// Update BPR flow using EMA (Exponential Moving Average) towards a target integer count
// Args:
//   lns - Trajectory LNS object
//   loc - location index
//   d - direction (0:East, 1:South, 2:West, 3:North)
//   target_count - target integer count from Int4 flow
inline void update_bpr_flow_ema_to_count(TrajLNS& lns, int loc, int d, int target_count);

// Synchronize BPR flow after adding a trajectory (called by add_traj)
// This should be called AFTER Int4 flow has been incremented
// Args:
//   lns - Trajectory LNS object
//   agent - agent index
void sync_bpr_after_add(TrajLNS& lns, int agent);

// Synchronize BPR flow after removing a trajectory (called by remove_traj)
// This should be called AFTER Int4 flow has been decremented
// Args:
//   lns - Trajectory LNS object
//   agent - agent index
void sync_bpr_after_remove(TrajLNS& lns, int agent);

// Batch initialize BPR flow from all existing trajectories
// Called during initialization to build the initial flow model
// Args:
//   lns - Trajectory LNS object
void init_bpr_from_all_trajs(TrajLNS& lns);

} // namespace TrafficMAPF

#endif // BPR_HPP
