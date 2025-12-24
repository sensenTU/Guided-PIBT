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

} // namespace TrafficMAPF

#endif // BPR_HPP
