#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <array>
#include <climits>

// Minimal definitions for testing
namespace TrafficMAPF {

// BPR Parameters (from TrajLNS)
struct BPRParams {
    static constexpr int COST_SCALE = 1000;
    static constexpr int BPR_T0 = 1000;
    static constexpr double BPR_ALPHA = 0.15;
    static constexpr double BPR_BETA = 4.0;
    static constexpr double C_MAX = 1.0;
    static constexpr double GAMMA = 0.8;
    static constexpr double MIN_CAPACITY = 0.01;
};

// Calculate BPR edge cost using fixed-point arithmetic
inline int calculate_bpr_cost(double f_co, double f_reverse) {
    // Calculate effective capacity: C_eff = C_max - γ * f_reverse
    double c_eff = BPRParams::C_MAX - BPRParams::GAMMA * f_reverse;
    c_eff = std::max(c_eff, BPRParams::MIN_CAPACITY);  // Prevent division by zero

    // BPR formula: t = t0 * [1 + α * (f/C)^β]
    // Optimization: β=4 is fixed, avoid std::pow, use direct multiplication
    double ratio = f_co / c_eff;
    double ratio2 = ratio * ratio;
    double ratio4 = ratio2 * ratio2;  // (f/C)^4

    double cost_double = BPRParams::BPR_T0 * (1.0 + BPRParams::BPR_ALPHA * ratio4);

    // Overflow protection: clamp to maximum safe integer value
    constexpr int MAX_SAFE_COST = INT_MAX / 2;
    if (cost_double > MAX_SAFE_COST) {
        cost_double = MAX_SAFE_COST;
    }

    // Convert to fixed-point integer (round to nearest)
    int cost = static_cast<int>(cost_double + 0.5);

    return cost;
}

} // namespace TrafficMAPF

using namespace TrafficMAPF;

void test_bpr_zero_flow() {
    std::cout << "=== Test 1: BPR with zero flow ===" << std::endl;

    double f_co = 0.0;
    double f_reverse = 0.0;
    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Reverse-flow: " << f_reverse << std::endl;
    std::cout << "  Expected cost: " << BPRParams::BPR_T0 << " (1000)" << std::endl;
    std::cout << "  Actual cost: " << cost << std::endl;

    bool passed = (cost == BPRParams::BPR_T0);
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Zero flow should give cost = " << BPRParams::BPR_T0 << std::endl;
    }
}

void test_bpr_high_flow() {
    std::cout << "\n=== Test 2: BPR with high flow ===" << std::endl;

    double f_co = 5.0;
    double f_reverse = 0.0;
    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Reverse-flow: " << f_reverse << std::endl;
    std::cout << "  Cost: " << cost << std::endl;

    bool passed = (cost > BPRParams::BPR_T0);
    std::cout << "  Is cost > BPR_T0? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: High flow should increase cost above " << BPRParams::BPR_T0 << std::endl;
    }
}

void test_bpr_reverse_flow_impact() {
    std::cout << "\n=== Test 3: BPR reverse flow impact ===" << std::endl;

    double f_co = 2.0;
    double f_reverse_1 = 0.0;
    int cost_1 = calculate_bpr_cost(f_co, f_reverse_1);

    double f_reverse_2 = 3.0;
    int cost_2 = calculate_bpr_cost(f_co, f_reverse_2);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Scenario 1 (no reverse):" << std::endl;
    std::cout << "    Reverse-flow: " << f_reverse_1 << ", Cost: " << cost_1 << std::endl;
    std::cout << "  Scenario 2 (high reverse):" << std::endl;
    std::cout << "    Reverse-flow: " << f_reverse_2 << ", Cost: " << cost_2 << std::endl;

    // With overflow protection, cost_2 should be >= cost_1
    // (or at maximum safe value)
    bool passed = (cost_2 >= cost_1) && (cost_2 > 0);
    std::cout << "  Does reverse flow increase cost (or clamp to max)? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Reverse flow should reduce effective capacity and increase cost" << std::endl;
    }
}

void test_bpr_nonlinear() {
    std::cout << "\n=== Test 4: BPR nonlinearity (β=4) ===" << std::endl;

    double f_reverse = 0.0;
    std::vector<double> flows = {0.5, 1.0, 2.0, 3.0};
    std::vector<int> costs;

    for (double f_co : flows) {
        int cost = calculate_bpr_cost(f_co, f_reverse);
        costs.push_back(cost);
        std::cout << "  Flow " << f_co << " -> Cost " << cost << std::endl;
    }

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

void test_fixed_point_rounding() {
    std::cout << "\n=== Test 5: Fixed-point rounding ===" << std::endl;

    double f_co = 1.0;
    double f_reverse = 0.0;

    double c_eff = BPRParams::C_MAX - BPRParams::GAMMA * f_reverse;
    double ratio = f_co / c_eff;
    double ratio2 = ratio * ratio;
    double ratio4 = ratio2 * ratio2;
    double cost_double = BPRParams::BPR_T0 * (1.0 + BPRParams::BPR_ALPHA * ratio4);

    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Exact cost (double): " << cost_double << std::endl;
    std::cout << "  Rounded cost (int): " << cost << std::endl;

    double diff = std::abs(cost_double - cost);
    bool passed = (diff < 1.0);
    std::cout << "  Rounding error: " << diff << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Rounding error should be < 1.0" << std::endl;
    }
}

void test_capacity_protection() {
    std::cout << "\n=== Test 6: Minimum capacity protection ===" << std::endl;

    double f_co = 10.0;
    double f_reverse = 2.0;  // Would make C_eff = 1.0 - 0.8*2.0 = -0.6 (negative!)

    int cost = calculate_bpr_cost(f_co, f_reverse);

    std::cout << "  Co-flow: " << f_co << std::endl;
    std::cout << "  Reverse-flow: " << f_reverse << std::endl;
    std::cout << "  Calculated cost: " << cost << std::endl;
    std::cout << "  (C_eff would be " << (BPRParams::C_MAX - BPRParams::GAMMA * f_reverse) << ", clamped to " << BPRParams::MIN_CAPACITY << ")" << std::endl;
    std::cout << "  (Cost clamped to INT_MAX/2 = " << (INT_MAX/2) << ")" << std::endl;

    bool passed = (cost > 0 && cost != INT_MAX);  // Should succeed with overflow protection
    std::cout << "  Did calculation complete without overflow? " << (passed ? "YES" : "NO") << std::endl;
    std::cout << "  Result: " << (passed ? "PASS ✓" : "FAIL ✗") << std::endl;

    if (!passed) {
        std::cerr << "  ERROR: Should protect against overflow" << std::endl;
    }
}

int main() {
    std::cout << "======================================" << std::endl;
    std::cout << "  BPR Implementation Test Suite" << std::endl;
    std::cout << "======================================" << std::endl;

    std::cout << "\nParameters:" << std::endl;
    std::cout << "  COST_SCALE: " << BPRParams::COST_SCALE << std::endl;
    std::cout << "  BPR_T0: " << BPRParams::BPR_T0 << std::endl;
    std::cout << "  BPR_ALPHA: " << BPRParams::BPR_ALPHA << std::endl;
    std::cout << "  BPR_BETA: " << BPRParams::BPR_BETA << std::endl;
    std::cout << "  C_MAX: " << BPRParams::C_MAX << std::endl;
    std::cout << "  GAMMA: " << BPRParams::GAMMA << std::endl;
    std::cout << "  MIN_CAPACITY: " << BPRParams::MIN_CAPACITY << std::endl;

    test_bpr_zero_flow();
    test_bpr_high_flow();
    test_bpr_reverse_flow_impact();
    test_bpr_nonlinear();
    test_fixed_point_rounding();
    test_capacity_protection();

    std::cout << "\n======================================" << std::endl;
    std::cout << "  All tests completed!" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
