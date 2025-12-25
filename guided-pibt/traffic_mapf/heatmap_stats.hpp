#ifndef HEATMAP_STATS_HPP
#define HEATMAP_STATS_HPP

#include "TrajLNS.h"
#include <string>
#include <vector>

namespace TrafficMAPF {

// Print comprehensive traffic statistics from flow data
// Args:
//   lns - Trajectory LNS object containing flow data
//   label - Label to print (e.g., "BASELINE" or "BPR")
void print_traffic_statistics(const TrajLNS& lns, const std::string& label);

} // namespace TrafficMAPF

#endif // HEATMAP_STATS_HPP
