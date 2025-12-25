#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>
#include <cstdint>

#include "traffic_mapf/bpr.hpp"
#include "traffic_mapf/TrajLNS.h"

using namespace TrafficMAPF;

// Simple environment for testing
SharedEnvironment create_test_env() {
    SharedEnvironment env;
    env.rows = 10;
    env.cols = 10;
    env.map.resize(env.rows * env.cols, 0);
    env.num_of_agents = 5;

    // Initialize neighbors
    env.init_neighbor();

    return env;
}

void test_bpr_zero_flow() {
    std::cout << "=== Test 1: BPR with zero flow ===" << std::endl;

    // Test: When both co-flow and reverse-flow are zero, cost should be BPR_T0
    double f_co = 0.0;
    double f_reverse = 0.0;

    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Reverse-flow: " << f_reverse << std::endl;
    std::cout << "  Expected cost: " << TrajLNS::BPR_T0 << " (1000)" << std::endl;
    std::cout << "  Actual cost: " << cost << std::endl;

    // Check: cost should be exactly BPR_T0 (1000)
    bool passed = (cost == TrajLNS::BPR_T0);
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Zero flow should give cost = " << TrajLNS::BPR_T0 << std::endl;
    }
}

void test_bpr_high_flow() {
    std::cout << "\n=== Test 2: BPR with high flow ===" << std::endl;

    // Test: High co-flow should give significantly higher cost
    double f_co = 5.0;  // 5 agents on the edge
    double f_reverse = 0.0;

    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Reverse-flow: " << f_reverse << std::endl;
    std::cout << "  Cost: " << cost << std::endl;

    // Check: cost should be > BPR_T0
    bool passed = (cost > TrajLNS::BPR_T0);
    std::cout << "  Is cost > BPR_T0? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: High flow should increase cost above " << TrajLNS::BPR_T0 << std::endl;
    }
}

void test_bpr_reverse_flow_impact() {
    std::cout << "\n=== Test 3: BPR reverse flow impact ===" << std::endl;

    double f_co = 2.0;

    // Scenario 1: No reverse flow
    double f_reverse_1 = 0.0;
    int cost_1 = calculate_bpr_cost(f_co, f_reverse_1);

    // Scenario 2: High reverse flow (reduces effective capacity)
    double f_reverse_2 = 3.0;
    int cost_2 = calculate_bpr_cost(f_co, f_reverse_2);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Scenario 1 (no reverse):" << std::endl;
    std::cout << "    Reverse-flow: " << f_reverse_1 << ", Cost: " << cost_1 << std::endl;
    std::cout << "  Scenario 2 (high reverse):" << std::endl;
    std::cout << "    Reverse-flow: " << f_reverse_2 << ", Cost: " << cost_2 << std::endl;

    // Check: cost_2 should be > cost_1 (reverse flow reduces capacity, increases cost)
    bool passed = (cost_2 > cost_1);
    std::cout << "  Does reverse flow increase cost? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Reverse flow should reduce effective capacity and increase cost" << std::endl;
    }
}

void test_bpr_nonlinear() {
    std::cout << "\n=== Test 4: BPR nonlinearity (β=4) ===" << std::endl;

    double f_reverse = 0.0;

    // Test different flow levels
    std::vector<double> flows = {0.5, 1.0, 2.0, 3.0};
    std::vector<int> costs;

    for (double f_co : flows) {
        int cost = calculate_bpr_cost(f_co, f_reverse);
        costs.push_back(cost);
        std::cout << "  Flow " << f_co << " -> Cost " << cost << std::endl;
    }

    // Check: cost should increase nonlinearly (exponentially with β=4)
    bool passed = true;
    for (size_t i = 1; i < costs.size(); i++) {
        if (costs[i] <= costs[i-1]) {
            passed = false;
            std::cerr << "  ERROR: Cost should increase with flow" << std::endl;
            break;
        }
    }

    std::cout << "  Is cost increasing? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;
}

void test_ema_update() {
    std::cout << "\n=== Test 5: EMA flow update ===" << std::endl;

    SharedEnvironment env = create_test_env();
    TrajLNS lns(&env);
    lns.init_bpr_flow();
    lns.init_mem();

    // Test: EMA should move towards target count
    int loc = 5;
    int d = 0;  // East

    // Initial flow should be 0
    std::cout << "  Initial flow: " << lns.directional_flow[loc][d] << std::endl;

    // Update to target 1
    update_bpr_flow_ema_to_count(lns, loc, d, 1);
    double flow_1 = lns.directional_flow[loc][d];
    std::cout << "  After update to 1: " << flow_1 << std::endl;

    // Update to target 5
    update_bpr_flow_ema_to_count(lns, loc, d, 5);
    double flow_2 = lns.directional_flow[loc][d];
    std::cout << "  After update to 5: " << flow_2 << std::endl;

    // Check: flow should increase towards target
    bool passed = (flow_2 > flow_1);
    std::cout << "  Does EMA move flow towards target? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: EMA should increase flow when target increases" << std::endl;
    }
}

void test_fixed_point_rounding() {
    std::cout << "\n=== Test 6: Fixed-point rounding ===" << std::endl;

    // Test: Cost should be rounded to nearest integer
    double f_co = 1.0;
    double f_reverse = 0.0;

    // Calculate manually to check rounding
    double c_eff = TrajLNS::C_MAX - TrajLNS::GAMMA * f_reverse;
    double ratio = f_co / c_eff;
    double ratio2 = ratio * ratio;
    double ratio4 = ratio2 * ratio2;
    double cost_double = TrajLNS::BPR_T0 * (1.0 + TrajLNS::BPR_ALPHA * ratio4);

    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Exact cost (double): " << cost_double << std::endl;
    std::cout << "  Rounded cost (int): " << cost << std::endl;
    std::cout << "  Rounding method: +0.5 then truncate" << std::endl;

    // Check: rounding should be correct (difference < 1)
    double diff = std::abs(cost_double - cost);
    bool passed = (diff < 1.0);
    std::cout << "  Rounding error: " << diff << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Rounding error should be < 1.0" << std::endl;
    }
}

int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  BPR Implementation Test Suite" << std::endl;
    std::cout << "======================================" << std::endl;

    std::cout << "\nParameters:" << std::endl;
    std::cout << "  COST_SCALE: " << TrajLNS::COST_SCALE << std::endl;
    std::cout << "  BPR_T0: " << TrajLNS::BPR_T0 << std::endl;
    std::cout << "  BPR_ALPHA: " << TrajLNS::BPR_ALPHA << std::endl;
    std::cout << "  BPR_BETA: " << TrajLNS::BPR_BETA << std::endl;
    std::cout << "  C_MAX: " << TrajLNS::C_MAX << std::endl;
    std::cout << "  GAMMA: " << TrajLNS::GAMMA << std::endl;
    std::cout << "  EMA_ETA: " << TrajLNS::EMA_ETA << std::endl;
    std::cout << "  MIN_CAPACITY: " << TrajLNS::MIN_CAPACITY << std::endl;

    test_bpr_zero_flow();
    test_bpr_high_flow();
    test_bpr_reverse_flow_impact();
    test_bpr_nonlinear();
    test_ema_update();
    test_fixed_point_rounding();

    std::cout << "\n======================================" << std::endl;
    std::cout << "  All tests completed!" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
