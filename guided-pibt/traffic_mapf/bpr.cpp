#include "bpr.hpp"
#include "utils.hpp"
#include <cassert>

namespace TrafficMAPF {

// ========== EMA Flow Update Functions ==========

// Update BPR flow using EMA (Exponential Moving Average) towards a target integer count
// Args:
//   lns - Trajectory LNS object
//   loc - location index
//   d - direction (0:East, 1:South, 2:West, 3:North)
//   target_count - target integer count from Int4 flow
inline void update_bpr_flow_ema_to_count(TrajLNS& lns, int loc, int d, int target_count) {
    // Boundary check to prevent segmentation fault
    if (loc < 0 || loc >= (int)lns.directional_flow.size() || d < 0 || d >= 4) {
        return;  // Invalid location or direction, skip update
    }

    double& flow = lns.directional_flow[loc][d];
    double target_usage = static_cast<double>(target_count);
    flow = (1.0 - TrajLNS::EMA_ETA) * flow + TrajLNS::EMA_ETA * target_usage;
}

// Synchronize BPR flow after adding a trajectory (called by add_traj)
// This should be called AFTER Int4 flow has been incremented
// Args:
//   lns - Trajectory LNS object
//   agent - agent index
void sync_bpr_after_add(TrajLNS& lns, int agent) {
    // Check if agent index is valid
    if (agent < 0 || agent >= (int)lns.trajs.size()) {
        return;
    }

    const Traj& traj = lns.trajs[agent];

    if (traj.size() <= 1) {
        return;  // Single-point trajectory, no movement
    }

    for (size_t i = 1; i < traj.size(); i++) {
        int u = traj[i - 1];  // Source location
        int v = traj[i];      // Target location

        // Boundary check for locations
        if (u < 0 || u >= (int)lns.directional_flow.size() ||
            v < 0 || v >= (int)lns.directional_flow.size()) {
            continue;  // Skip invalid locations
        }

        // Calculate direction
        int diff = v - u;
        int d = get_d(diff, lns.env);

        // Int4 flow has already been updated: lns.flow[u].d[d]++
        // Use the new integer count as EMA target
        int current_count = lns.flow[u].d[d];
        update_bpr_flow_ema_to_count(lns, u, d, current_count);
    }
}

// Synchronize BPR flow after removing a trajectory (called by remove_traj)
// This should be called AFTER Int4 flow has been decremented
// Args:
//   lns - Trajectory LNS object
//   agent - agent index
void sync_bpr_after_remove(TrajLNS& lns, int agent) {
    // Check if agent index is valid
    if (agent < 0 || agent >= (int)lns.trajs.size()) {
        return;
    }

    const Traj& traj = lns.trajs[agent];

    if (traj.size() <= 1) {
        return;  // Single-point trajectory, no movement
    }

    for (size_t i = 1; i < traj.size(); i++) {
        int u = traj[i - 1];  // Source location
        int v = traj[i];      // Target location

        // Boundary check for locations
        if (u < 0 || u >= (int)lns.directional_flow.size() ||
            v < 0 || v >= (int)lns.directional_flow.size()) {
            continue;  // Skip invalid locations
        }

        // Calculate direction
        int diff = v - u;
        int d = get_d(diff, lns.env);

        // Int4 flow has already been updated: lns.flow[u].d[d]--
        // Use the new integer count as EMA target (may still be > 0)
        int current_count = lns.flow[u].d[d];
        update_bpr_flow_ema_to_count(lns, u, d, current_count);
    }
}

// Batch initialize BPR flow from all existing trajectories
// Called during initialization to build the initial flow model
// Args:
//   lns - Trajectory LNS object
void init_bpr_from_all_trajs(TrajLNS& lns) {
    for (int agent = 0; agent < lns.env->num_of_agents; agent++) {
        if (!lns.trajs[agent].empty()) {
            sync_bpr_after_add(lns, agent);
        }
    }
}

} // namespace TrafficMAPF
