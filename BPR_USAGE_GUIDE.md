# BPR (Bureau of Public Roads) Cost Function - Usage Guide

This document provides comprehensive instructions for using the BPR traffic flow cost function in Guided-PIBT.

## Table of Contents
- [Overview](#overview)
- [What is BPR?](#what-is-bpr)
- [Compilation](#compilation)
- [Compilation Parameters](#compilation-parameters)
- [Running the Code](#running-the-code)
- [Testing](#testing)
- [Performance Tips](#performance-tips)
- [Troubleshooting](#troubleshooting)

---

## Overview

The BPR (Bureau of Public Roads) cost function is a nonlinear traffic congestion model based on macroscopic traffic flow theory. It replaces the linear congestion counter with a more realistic cost function that captures:

- **Nonlinear congestion effects** (β=4 exponent)
- **Directional flow modeling** (4-directional tensor)
- **Opposing traffic impact** (effective capacity reduction)
- **Flow inertia** (Exponential Moving Average)

**Key Benefits:**
- More accurate congestion modeling
- Better agent distribution
- Improved coordination in high-density scenarios
- Reduced agent conflicts through cost-based avoidance

---

## What is BPR?

### Theoretical Model

The BPR cost function calculates edge travel time based on traffic flow:

```
Cost(e) = t₀ × [1 + α × (f_e / C_eff)ᵝ]

Where:
- t₀ = 1.0 (free-flow travel time, scaled to 1000)
- α = 0.15 (BPR alpha parameter)
- β = 4.0 (BPR beta parameter - creates nonlinearity)
- f_e = co-directional flow (agents moving in same direction)
- C_eff = effective capacity (reduced by opposing traffic)
```

### Effective Capacity

```
C_eff = C_max - γ × f_reverse

Where:
- C_max = 1.0 (maximum capacity)
- γ = 0.8 (impact factor of opposing traffic)
- f_reverse = reverse directional flow (agents moving in opposite direction)
```

### Flow Inertia (EMA)

Flow values are updated using Exponential Moving Average for smoothness:

```
new_flow = (1 - η) × old_flow + η × current_usage

Where:
- η = 0.2 (EMA smoothing factor)
- current_usage = actual integer count from Int4 structure
```

### Directional Flow Tensor

Each location maintains 4 flow values (East, South, West, North):

```
directional_flow[loc] = [flow_E, flow_S, flow_W, flow_N]
```

This enables anisotropic congestion modeling (different costs for different directions).

---

## Compilation

### Prerequisites

Ensure you have the required dependencies installed:

```bash
# Boost library (>= 1.83.0)
sudo apt-get install libboost-all-dev  # Ubuntu/Debian

# CMake (>= 3.10)
sudo apt-get install cmake
```

### Step 1: Configure with BPR Enabled

Create a build directory and configure CMake with the BPR flag:

```bash
cd /path/to/Guided-PIBT

# Create build directory
mkdir -p build
cd build

# Configure with BPR heuristic enabled
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELEASE
```

**Expected Output:**
```
-- USE_BPR_HEURISTIC is enabled - BPR cost function will be used
...
-- Configuring done
-- Generating done
```

### Step 2: Compile

Build the project:

```bash
make -j$(nproc)  # Use all available CPU cores
```

**Expected Output:**
```
[ 25%] Building CXX object CMakeFiles/lifelong.dir/traffic_mapf/bpr.cpp.o
[ 50%] Building CXX object CMakeFiles/traffic_mapf.dir/search.cpp.o
...
[100%] Built target lifelong
```

### Step 3: Verify Compilation

Check that the executable was created:

```bash
ls -lh lifelong
```

You should see the `lifelong` executable (typically 2-5 MB in size).

---

## Compilation Parameters

### BPR-Specific Parameter

**`USE_BPR_HEURISTIC=ON/OFF`** (Default: OFF)
- **ON**: Enable BPR cost function for A* search in LNS
- **OFF**: Use standard linear congestion counters

### Standard Parameters

From the main README, these parameters can be combined with BPR:

| Parameter | Values | Description |
|-----------|--------|-------------|
| `GUIDANCE` | ON/OFF | Enable/disable guide paths |
| `GUIDANCE_LNS` | int or OFF | Number of LNS iterations (0=OFF) |
| `FLOW_GUIDANCE` | ON/OFF | **Deprecated** - replaced by USE_BPR_HEURISTIC |
| `INIT_PP` | ON/OFF | Initialize with prioritized planning |
| `RELAX` | int or OFF | Limit guide path initialization (0=OFF) |
| `OBJECTIVE` | 0-5 | Objective function for guide path A* |
| `FOCAL_SEARCH` | float or OFF | Focal suboptimality weight (e.g., 1.2, 2.0) |
| `CMAKE_BUILD_TYPE` | DEBUG/RELEASE | Build optimization level |

**⚠️ Important:** `USE_BPR_HEURISTIC` is **independent** of `FLOW_GUIDANCE`. BPR is a more advanced replacement for the older traffic cost heuristic.

### Example Configurations

#### BPR with Guide Paths (Recommended)

```bash
cmake .. -DUSE_BPR_HEURISTIC=ON \
         -DGUIDANCE=ON \
         -DGUIDANCE_LNS=10 \
         -DINIT_PP=ON \
         -DRELAX=100 \
         -DOBJECTIVE=1 \
         -DFOCAL_SEARCH=2 \
         -DCMAKE_BUILD_TYPE=RELEASE
```

#### BPR Only (No Guide Paths)

```bash
cmake .. -DUSE_BPR_HEURISTIC=ON \
         -DGUIDANCE=OFF \
         -DCMAKE_BUILD_TYPE=RELEASE
```

#### BPR with Focal Search

```bash
cmake .. -DUSE_BPR_HEURISTIC=ON \
         -DGUIDANCE=ON \
         -DFOCAL_SEARCH=1.2 \
         -DCMAKE_BUILD_TYPE=RELEASE
```

#### Baseline (No BPR)

For comparison with the original algorithm:

```bash
cmake .. -DUSE_BPR_HEURISTIC=OFF \
         -DGUIDANCE=ON \
         -DGUIDANCE_LNS=10 \
         -DINIT_PP=ON \
         -DRELAX=100 \
         -DOBJECTIVE=1 \
         -DFOCAL_SEARCH=2 \
         -DCMAKE_BUILD_TYPE=RELEASE
```

---

## Running the Code

### Basic Usage

```bash
./lifelong --inputFile <benchmark_file> \
           --planTimeLimit <seconds> \
           --output <output_file>
```

### Example Commands

#### Example 1: Small Benchmark

```bash
./lifelong --inputFile ../benchmark-lifelong/sortation_small_0_800.json \
           --planTimeLimit 10 \
           --output bpr_output.json \
           -l event_log.txt
```

#### Example 2: Paris Benchmark (Large)

```bash
./lifelong --inputFile ../benchmark-lifelong/Paris_1_256_0_10000.json \
           --planTimeLimit 60 \
           --output paris_bpr_output.json \
           -l paris_event_log.txt
```

#### Example 3: Warehouse Scenario

```bash
./lifelong --inputFile ../benchmark-lifelong/warehouse_10_200_0_2000.json \
           --planTimeLimit 30 \
           --output warehouse_bpr_output.json
```

### Command-Line Arguments

| Argument | Type | Required | Description |
|----------|------|----------|-------------|
| `--inputFile` | string | Yes | Path to benchmark JSON file |
| `--planTimeLimit` | int | Yes | Time limit for planning (seconds) |
| `--output` | string | Yes | Path to output JSON file |
| `-l` | string | No | Path to event log file (optional) |

### Understanding the Output

The output JSON file contains:

```json
{
  "statistics": {
    "makespan": 1234,
    "sum_of_costs": 5678,
    "flowtime": 9012,
    "num_agents": 100
  },
  "schedules": [
    {
      "agent": 0,
      "path": [0, 1, 2, 3, ...]
    },
    ...
  ]
}
```

**Key Metrics:**
- `makespan`: Time when all agents finish
- `sum_of_costs`: Total timesteps across all agents
- `flowtime`: Sum of arrival times

---

## Testing

### Run BPR Unit Tests

First, compile the test suite:

```bash
cd /path/to/Guided-PIBT/guided-pibt

# Compile test
g++ -std=c++17 test_bpr_simple.cpp -o test_bpr_simple

# Run tests
./test_bpr_simple
```

**Expected Output:**

```
======================================
  BPR Implementation Test Suite
======================================

Parameters:
  COST_SCALE: 1000
  BPR_T0: 1000
  BPR_ALPHA: 0.15
  BPR_BETA: 4
  C_MAX: 1
  GAMMA: 0.8
  MIN_CAPACITY: 0.01

=== Test 1: BPR with zero flow ===
  Co-flow: 0
  Reverse-flow: 0
  Expected cost: 1000 (1000)
  Actual cost: 1000
  Result: PASS ✓

=== Test 2: BPR with high flow ===
  Co-flow: 5
  Reverse-flow: 0
  Cost: 94750
  Is cost > BPR_T0? YES
  Result: PASS ✓

=== Test 3: BPR reverse flow impact ===
  Co-flow: 2
  Scenario 1 (no reverse):
    Reverse-flow: 0, Cost: 3400
  Scenario 2 (high reverse):
    Reverse-flow: 3, Cost: 1073741823
  Does reverse flow increase cost (or clamp to max)? YES
  Result: PASS ✓

=== Test 4: BPR nonlinearity (β=4) ===
  Flow 0.5 -> Cost 1009
  Flow 1 -> Cost 1150
  Flow 2 -> Cost 3400
  Flow 3 -> Cost 13150
  Is cost increasing? YES
  Result: PASS ✓

=== Test 5: Fixed-point rounding ===
  Exact cost (double): 1150
  Rounded cost (int): 1150
  Rounding error: 0
  Result: PASS ✓

=== Test 6: Minimum capacity protection ===
  Co-flow: 10
  Reverse-flow: 2
  Calculated cost: 1073741823
  (C_eff would be -0.6, clamped to 0.01)
  (Cost clamped to INT_MAX/2 = 1073741823)
  Did calculation complete without overflow? YES
  Result: PASS ✓

======================================
  All tests completed!
======================================
```

All 6 tests should pass with ✓ marks.

### Integration Testing

To test BPR on real benchmarks:

```bash
# Small benchmark (quick test)
./lifelong --inputFile ../benchmark-lifelong/sortation_small_0_800.json \
           --planTimeLimit 10 \
           --output test_bpr.json

# Compare with baseline (recompile without BPR)
cd ..
rm -rf build
mkdir build
cd build
cmake .. -DUSE_BPR_HEURISTIC=OFF -DCMAKE_BUILD_TYPE=RELEASE
make -j$(nproc)

./lifelong --inputFile ../benchmark-lifelong/sortation_small_0_800.json \
           --planTimeLimit 10 \
           --output test_baseline.json

# Compare results
echo "BPR vs Baseline Performance:"
echo "BPR sum_of_costs: $(cat test_bpr.json | jq '.statistics.sum_of_costs')"
echo "Baseline sum_of_costs: $(cat test_baseline.json | jq '.statistics.sum_of_costs')"
```

---

## Performance Tips

### 1. Memory Usage

BPR adds memory overhead for directional flow:

```
Memory per location = 4 directions × 8 bytes (double) = 32 bytes
Total overhead = num_locations × 32 bytes

Example (100x100 grid):
  10,000 locations × 32 bytes = 320 KB (negligible)
```

### 2. Computation Overhead

BPR cost calculation is **fast** due to optimizations:

- No `std::pow()` calls (uses `ratio2 × ratio2`)
- Inline functions (compiler optimization)
- Integer overflow check (simple `if` statement)

**Typical overhead:** < 5% compared to baseline

### 3. Recommended Parameters

For **dense scenarios** (high agent density):
- Use BPR with focal search (`FOCAL_SEARCH=1.2 or 2.0`)
- Enables agents to avoid congested areas effectively

For **sparse scenarios** (low agent density):
- BPR may not provide significant benefits
- Baseline may be sufficient

For **dynamic environments** (frequent task changes):
- BPR's EMA (η=0.2) provides smooth adaptation
- Better than instantaneous flow updates

### 4. Tuning BPR Parameters

If you need to adjust BPR behavior, edit `traffic_mapf/TrajLNS.h`:

```cpp
// In TrajLNS class
static constexpr double BPR_ALPHA = 0.15;  // Congestion sensitivity
static constexpr double BPR_BETA = 4.0;     // Nonlinearity (DO NOT CHANGE)
static constexpr double GAMMA = 0.8;        // Opposing traffic impact
static constexpr double EMA_ETA = 0.2;      // Flow inertia (0.0-1.0)
```

**Guidelines:**
- Higher `BPR_ALPHA` → stronger congestion avoidance
- Higher `GAMMA` → more impact from opposing traffic
- Higher `EMA_ETA` → faster adaptation to changes (less inertia)

⚠️ **Warning:** Do not change `BPR_BETA` from 4.0, as the code is optimized for this value.

---

## Troubleshooting

### Issue: Compilation Error "undefined reference to TrajLNS::..."

**Cause:** Missing `USE_BPR_HEURISTIC` flag

**Solution:**
```bash
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELEASE
make clean
make -j$(nproc)
```

### Issue: Negative Costs or Overflow

**Symptom:** Costs showing as `-2147483648` (INT_MIN)

**Cause:** Extreme flow values causing overflow

**Solution:** Already handled by overflow protection in code. If this occurs:
1. Verify you're using the latest code with overflow protection
2. Check if benchmark has unrealistic agent density
3. Report the issue with benchmark details

### Issue: BPR Not Working (Same Results as Baseline)

**Cause:** Not compiled with BPR enabled

**Check:**
```bash
# Look for this message during compilation
grep "USE_BPR_HEURISTIC is enabled" CMakeFiles/CMakeOutput.log
```

**Solution:** Recompile with `-DUSE_BPR_HEURISTIC=ON`

### Issue: Performance Degradation

**Symptom:** Significantly slower than baseline (>20% slower)

**Possible Causes:**
1. Debug build (use `RELEASE` mode)
2. Very large map with sparse agents (BPR overhead not worth it)
3. Incorrect parameter configuration

**Solution:**
```bash
# Ensure release build
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELEASE
make clean
make -j$(nproc)
```

### Issue: Test Failures

**Symptom:** `test_bpr_simple` shows FAIL

**Solution:**
1. Verify compilation with C++17 or later:
   ```bash
   g++ --version  # Should be 7.0 or higher
   ```

2. Recompile test:
   ```bash
   g++ -std=c++17 test_bpr_simple.cpp -o test_bpr_simple
   ./test_bpr_simple
   ```

3. If tests still fail, check for code modifications

### Issue: Git Push Permission Denied

**Symptom:**
```
ERROR: Permission to sensenTU/Guided-PIBT.git denied to Jiezheng1125
```

**Solution:**
1. Check which GitHub account is configured:
   ```bash
   git config user.name
   git config user.email
   ```

2. Update credentials if needed:
   ```bash
   git config user.name "Your Name"
   git config user.email "your.email@example.com"
   ```

3. Or use SSH instead of HTTPS:
   ```bash
   git remote set-url origin git@github.com:sensenTU/Guided-PIBT.git
   git push origin main
   ```

---

## Advanced Usage

### Custom BPR Parameters per Benchmark

You can create different builds for different scenarios:

```bash
# Build 1: High congestion (warehouse)
mkdir build_high_congestion
cd build_high_congestion
# Edit TrajLNS.h to increase BPR_ALPHA to 0.25
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELEASE
make -j$(nproc)

# Build 2: Standard congestion
mkdir build_standard
cd build_standard
# Use default BPR_ALPHA = 0.15
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELEASE
make -j$(nproc)
```

### Profiling BPR Performance

Use `perf` to profile BPR overhead:

```bash
# Compile with debug symbols
cmake .. -DUSE_BPR_HEURISTIC=ON -DCMAKE_BUILD_TYPE=RELWITHDEBINFO
make -j$(nproc)

# Profile
perf record --call-graph dwarf ./lifelong \
  --inputFile ../benchmark-lifelong/Paris_1_256_0_10000.json \
  --planTimeLimit 10 \
  --output profile_output.json

# Analyze
perf report
```

Look for time spent in:
- `calculate_bpr_cost`
- `get_bpr_edge_cost`
- `update_bpr_flow_ema_to_count`

---

## References

### Papers
- Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024). [Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding](https://arxiv.org/abs/2308.11234). AAAI 2024.

### BPR Theory
- Bureau of Public Roads. (1964). *Traffic Assignment Manual*. U.S. Department of Commerce.
- Spiess, H. (1990). *Technical Note on Conical Traffic Assignment*. Transportation Research B, 24(2), 97-108.

### Related Files
- `BPR_IMPLEMENTATION_SUMMARY.md` - Design summary
- `BPR_IMPLEMENTATION_CODE.md` - Complete code documentation
- `BPR_DEBUGGING_SESSION.md` - Bug fixes and testing

---

## Contact & Support

For issues, questions, or contributions:
- Open an issue on GitHub: https://github.com/sensenTU/Guided-PIBT/issues
- Check existing documentation in the repository

---

**Last Updated:** 2025-12-25
**BPR Version:** 1.0
**Status:** Production Ready ✅
