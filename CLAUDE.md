# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Guided-PIBT** implements "Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding" (AAAI 2024). It provides lifelong MAPF algorithms with traffic flow optimization using macroscopic traffic flow theory.

## Build System

### Dependencies
- CMake >= 3.16
- Boost >= 1.49.0 (program_options, system, filesystem, log, log_setup)
- C++17 compiler

### Build Commands

**Create a new build:**
```bash
cmake -B build-dir ./guided-pibt [OPTIONS]
make -C build-dir -j4
```

**Common build configurations:**

Baseline (no BPR):
```bash
cmake -B build-baseline ./guided-pibt \
  -DCMAKE_BUILD_TYPE=RELEASE
```

BPR-enabled:
```bash
cmake -B build-bpr ./guided-pibt \
  -DUSE_BPR_HEURISTIC=ON \
  -DCMAKE_BUILD_TYPE=RELEASE
```

Full optimization (GP-R100-Re10-F2):
```bash
cmake -B build-optimized ./guided-pibt \
  -DGUIDANCE=ON -DGUIDANCE_LNS=10 \
  -DUSE_BPR_HEURISTIC=ON \
  -DINIT_PP=ON -DRELAX=100 -DOBJECTIVE=1 \
  -DFOCAL_SEARCH=2 -DCMAKE_BUILD_TYPE=RELEASE
```

### Key CMake Options

- `USE_BPR_HEURISTIC=ON/OFF` - Enable BPR (Bureau of Public Roads) cost function for nonlinear congestion
- `GUIDANCE=ON/OFF` - Enable guide path system
- `GUIDANCE_LNS=int` - LNS iterations (0=OFF)
- `INIT_PP=ON/OFF` - Initialize with prioritized planning
- `OBJECTIVE=0-5` - Objective function for guide path A*
  - 0: Constant cost
  - 1: Two-part (Contra Flow + Vertex Flow)
  - 2: Vertex Flow only
  - 3: Vertex Flow + Contra Flow
  - 4: SUI Cost to Come
  - 5: SUI Cost to Go
- `FOCAL_SEARCH=float` - Focal suboptimality weight (1.0-3.0)
- `LNS_GROUP_SIZE=int` - Agents per LNS iteration (default: 10)

## Running the Application

### Lifelong MAPF (Guided-PIBT)

**Executable:** `build-dir/lifelong`

```bash
./lifelong --inputFile <benchmark_file> \
           --planTimeLimit <seconds> \
           --output <output_file> \
           [-l <event_log_file>]
```

**Example:**
```bash
cd build-bpr
./lifelong --inputFile ../guided-pibt/benchmark-lifelong/sortation_small_0_800.json \
           --planTimeLimit 60 \
           --output output.json \
           -l event_log.txt
```

### Input Format (JSON)

Benchmark files specify map, agents, and tasks:
```json
{
    "mapFile": "maps/sortation_small.map",
    "agentFile": "agents/sortation_small_0_800.agents",
    "taskFile": "tasks/sortation_small_0.task",
    "teamSize": 800,
    "numTasksReveal": 1,
    "taskAssignmentStrategy": "roundrobin"
}
```

### Performance Testing

Quick performance comparison script:
```bash
cd /home/sentu/mxw/Guided-PIBT
./performance_analysis.sh
```

This tests multiple maps and compares Baseline vs BPR versions, reporting:
- Timesteps completed
- Runtime (seconds)
- Throughput (steps/second)
- Performance ratios

## Architecture

### Core Modules

**CompetitionSystem** (`guided-pibt/src/CompetitionSystem.cpp`)
- Main simulation orchestrator
- Manages agent lifecycle and task assignment
- Handles collision detection and movement validation
- Implements continuous task generation for lifelong scenarios

**TrafficMAPF** (`guided-pibt/traffic_mapf/`)
- Traffic-aware path planning algorithms
- Integration with macroscopic traffic flow models

### Traffic Flow Implementation

**BPR Cost Function** (`traffic_mapf/bpr.cpp`)
Nonlinear congestion model:
```
Cost = t₀ × [1 + α × (f_e / C_eff)ᵝ]

Parameters:
- t₀ = 1000 (free-flow travel time)
- α = 0.15 (congestion sensitivity)
- β = 4.0 (nonlinearity exponent)
- C_eff = C_max - γ × f_reverse (effective capacity)
  - C_max = 1.0 (maximum capacity)
  - γ = 0.8 (opposing traffic impact)
```

**Flow Tracking** (`traffic_mapf/flow.cpp`)
- 4-directional flow tensor (E, S, W, N)
- Exponential Moving Average (EMA) for smoothing: `f_new = η × f_current + (1-η) × f_old`
- η = 0.2 (flow inertia)

**Search** (`traffic_mapf/search.cpp`)
- A* with traffic-aware edge costs
- Focal search for bounded suboptimality
- Integrates BPR cost function into edge evaluation

### Key Classes

**TrajLNS** (`traffic_mapf/TrajLNS.h`)
- Large Neighborhood Search for guide path optimization
- Manages traffic flow state
- Implements BPR heuristic integration

**SharedEnvironment** (`traffic_mapf/Environment.h`)
- Environment model with traffic flow
- Maintains flow tensors and agent positions
- Provides edge cost queries

**HeuristicTable** (`traffic_mapf/heuristics.cpp`)
- Precomputed distance heuristics
- Supports different objective functions

## BPR Implementation Details

### Integration Points

1. **Flow Initialization** (`TrajLNS::init_bpr_flow`)
   - Initializes 4-directional flow tensors
   - Called during environment setup

2. **Cost Calculation** (`get_bpr_edge_cost` in `search.cpp:186`)
   - Replaces linear cost in A* expansion
   - Computes: `cost = BPR_T0 * (1.0 + BPR_ALPHA * ratio_beta)`
   - Uses current forward flow and reverse flow

3. **Flow Updates** (`update_bpr_flow` and `add/remove_traj`)
   - Updates directional flows when trajectories are added/removed
   - Applies EMA smoothing: `f = EMA_ETA × f + (1 - EMA_ETA) × f_old`
   - Triggered during LNS iterations

### Performance Characteristics

**Computation Overhead:** ~20-30x higher per edge vs baseline
- Baseline: 1 integer addition
- BPR: ~20-30 floating-point operations

**Actual Performance:** Context-dependent
- Small/dense maps (e.g., Room 64x64): BPR often faster (better congestion avoidance)
- Large/sparse maps (e.g., Denver 520d): BPR may be slower (computational overhead dominates)
- Average across benchmarks: ~20-40% faster or slower depending on scenario

**Memory Overhead:** ~32 bytes per grid location (negligible)

## Testing

### Unit Tests
```bash
cd guided-pibt
g++ -std=c++17 test_bpr_simple.cpp -o test_bpr_simple
./test_bpr_simple
```

### Benchmark Files
Located in `guided-pibt/benchmark-lifelong/`:
- **Small**: `sortation_small_0_800.json`, `room-64-64-8_0_1000.json` (< 30 seconds)
- **Medium**: `ost003d_0_2000.json`, `warehouse_10_200_0_2000.json` (1-5 minutes)
- **Large**: `den520d_0_12000.json`, `Paris_1_256_0_2000.json` (5-30 minutes)

## Development Guidelines

### Modifying BPR Parameters

Edit in `guided-pibt/traffic_mapf/TrajLNS.h:81-88`:
```cpp
constexpr double BPR_ALPHA = 0.15;  // Congestion sensitivity
constexpr double BPR_BETA = 2.0;    // Nonlinearity (WARNING: code uses ratio^4)
constexpr double GAMMA = 0.8;        // Opposing traffic impact
constexpr double EMA_ETA = 0.2;      // Flow smoothing
constexpr int BPR_T0 = 1000;         // Free-flow time
```

### Adding New Objective Functions

1. Define objective in `traffic_mapf/heuristics.cpp`
2. Register in `TrajLNS::get_edge_cost`
3. Update CMake OBJECTIVE options
4. Document in this file

### Debugging BPR Behavior

Enable debug output in `traffic_mapf/bpr.cpp`:
```cpp
#define DEBUG_BPR
```

Or use heatmap generation:
```bash
./lifelong --inputFile <benchmark> --output heatmap.json
python guided-pibt/scripts/visualize_heatmap.py heatmap.json
```

## Project Structure

```
Guided-PIBT/
├── guided-pibt/                    # Main lifelong planning implementation
│   ├── src/                        # Core simulation (CompetitionSystem)
│   ├── traffic_mapf/               # Traffic-aware algorithms
│   │   ├── bpr.cpp/hpp             # BPR cost function
│   │   ├── flow.cpp/hpp            # Flow tracking
│   │   ├── search.cpp/hpp          # A* with traffic costs
│   │   ├── heuristics.cpp          # Heuristic tables
│   │   ├── TrajLNS.h               # LNS with traffic optimization
│   │   └── test_bpr*.cpp           # BPR tests
│   ├── benchmark-lifelong/          # Test scenarios
│   │   ├── maps/                   # Grid maps
│   │   ├── agents/                 # Agent start positions
│   │   └── tasks/                  # Task definitions
│   └── CMakeLists.txt              # Build configuration
├── guided-lacam2/                  # One-shot planning (LaCAM2-based)
├── build-baseline/                 # Build without BPR
├── build-bpr/                      # Build with BPR enabled
├── performance_analysis.sh         # Multi-map performance comparison
└── BPR_*.md                        # BPR implementation documentation
```

## Common Issues

### Build Errors
- **Boost not found**: Install boost libraries: `sudo apt-get install libboost-all-dev`
- **CMake too old**: Upgrade to CMake >= 3.16

### Runtime Errors
- **Map file not found**: Check paths relative to build directory
- **Agent file format error**: Verify agent file matches map dimensions

### Performance Issues
- **BPR slower than expected**: Check map density - BPR benefits dense scenarios
- **Memory growth**: Ensure EMA_ETA is set correctly (default 0.2)
- **Poor convergence**: Adjust FOCAL_SEARCH or GUIDANCE_LNS parameters
