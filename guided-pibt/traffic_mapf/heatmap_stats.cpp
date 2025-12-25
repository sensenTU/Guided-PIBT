#include "heatmap_stats.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

namespace TrafficMAPF {

void print_traffic_statistics(const TrajLNS& lns, const std::string& label) {
    std::cout << "\n========== TRAFFIC STATISTICS " << label << " ==========" << std::endl;

    // Collect all edge flows (4 directional edges per location)
    std::vector<int> edge_flows;
    int total_edges = 0;
    int max_flow = 0;
    int min_flow = INT_MAX;

    for (size_t loc = 0; loc < lns.flow.size(); loc++) {
        // Skip obstacles
        if (lns.env->map[loc] == 1) continue;

        for (int d = 0; d < 4; d++) {
            int flow = lns.flow[loc].d[d];
            if (flow > 0) {  // Only count edges with actual traffic
                edge_flows.push_back(flow);
                max_flow = std::max(max_flow, flow);
                min_flow = std::min(min_flow, flow);
            }
            total_edges++;
        }
    }

    if (edge_flows.empty()) {
        std::cout << "No traffic data available!" << std::endl;
        return;
    }

    // Sort for percentile calculations
    std::sort(edge_flows.begin(), edge_flows.end());

    // Basic statistics
    double sum = std::accumulate(edge_flows.begin(), edge_flows.end(), 0.0);
    double mean = sum / edge_flows.size();

    double sq_sum = 0.0;
    for (int flow : edge_flows) {
        sq_sum += (flow - mean) * (flow - mean);
    }
    double std_dev = std::sqrt(sq_sum / edge_flows.size());

    // Percentiles
    int p50_idx = edge_flows.size() * 50 / 100;
    int p90_idx = edge_flows.size() * 90 / 100;
    int p95_idx = edge_flows.size() * 95 / 100;
    int p99_idx = edge_flows.size() * 99 / 100;

    // Gini coefficient (0 = perfect equality, 1 = maximum inequality)
    double gini = 0.0;
    size_t n = edge_flows.size();
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            gini += std::abs(edge_flows[i] - edge_flows[j]);
        }
    }
    gini /= (2.0 * n * sum);

    // High congestion count
    int high_congestion_5 = 0;
    int high_congestion_10 = 0;
    int high_congestion_20 = 0;
    for (int flow : edge_flows) {
        if (flow > 5) high_congestion_5++;
        if (flow > 10) high_congestion_10++;
        if (flow > 20) high_congestion_20++;
    }

    // Print results
    std::cout << "Edges with traffic: " << edge_flows.size() << " / " << total_edges << std::endl;
    std::cout << "Max edge usage: " << max_flow << std::endl;
    std::cout << "Min edge usage: " << min_flow << std::endl;
    std::cout << "Mean edge usage: " << mean << std::endl;
    std::cout << "Std deviation: " << std_dev << std::endl;
    std::cout << "\nPercentiles:" << std::endl;
    std::cout << "  50th (median): " << edge_flows[p50_idx] << std::endl;
    std::cout << "  90th: " << edge_flows[p90_idx] << std::endl;
    std::cout << "  95th: " << edge_flows[p95_idx] << std::endl;
    std::cout << "  99th: " << edge_flows[p99_idx] << std::endl;
    std::cout << "\nHigh congestion edges:" << std::endl;
    std::cout << "  Flow > 5:  " << high_congestion_5 << " ("
              << (100.0 * high_congestion_5 / edge_flows.size()) << "%)" << std::endl;
    std::cout << "  Flow > 10: " << high_congestion_10 << " ("
              << (100.0 * high_congestion_10 / edge_flows.size()) << "%)" << std::endl;
    std::cout << "  Flow > 20: " << high_congestion_20 << " ("
              << (100.0 * high_congestion_20 / edge_flows.size()) << "%)" << std::endl;
    std::cout << "\nLoad balancing:" << std::endl;
    std::cout << "  Gini coefficient: " << gini << " (lower = better distribution)" << std::endl;
    std::cout << "  Coefficient of variation: " << (std_dev / mean) << std::endl;

    // Top 10 most congested edges
    std::cout << "\nTop 10 most congested edges:" << std::endl;
    int count = 0;
    for (auto it = edge_flows.rbegin(); it != edge_flows.rend() && count < 10; ++it, ++count) {
        std::cout << "  #" << count << ": flow = " << *it << std::endl;
    }

    std::cout << "====================================================\n" << std::endl;
}

} // namespace TrafficMAPF
