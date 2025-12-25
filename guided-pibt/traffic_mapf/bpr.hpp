#ifndef BPR_HPP
#define BPR_HPP

#include "TrajLNS.h"
#include <algorithm>

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
    // Calculate direction using existing utility function
    int diff = v - u;
    int d = get_d(diff, lns.env);

    // Co-directional flow (u -> v)
    double f_co = lns.directional_flow[u][d];

    // Reverse directional flow (v -> u)
    double f_reverse = lns.directional_flow[v][(d + 2) % 4];

    return calculate_bpr_cost(f_co, f_reverse);
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
