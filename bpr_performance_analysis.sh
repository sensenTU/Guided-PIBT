#!/bin/bash
# BPR 性能开销分析脚本

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   BPR 性能开销分析${NC}"
echo -e "${BLUE}========================================${NC}"

PROJECT_ROOT="/home/sentu/mxw/Guided-PIBT"
TEST_MAP="guided-pibt/benchmark-lifelong/den520d_0_2000.json"
TEST_TIME=3

cd "$PROJECT_ROOT"

echo ""
echo -e "${YELLOW}测试地图:${NC} $TEST_MAP"
echo -e "${YELLOW}测试时间:${NC} ${TEST_TIME} 秒"
echo ""

# ==========================================
# 1. 分析 A* 搜索扩展节点数
# ==========================================

echo -e "${YELLOW}============ 1. A* 搜索扩展节点分析 ============${NC}"
echo ""

# 添加调试代码到 search.cpp 计算扩展次数
if ! grep -q "EXPANSION_COUNT" guided-pibt/traffic_mapf/search.cpp; then
    echo -e "${RED}需要在 search.cpp 中添加扩展节点计数器${NC}"
    echo ""
    echo "在 search.cpp 中添加（约第 83 行后）:"
    echo "  static int expansion_count = 0;"
    echo "  expanded++; expansion_count++;"
    echo "  if (expanded % 10000 == 0) {"
    echo "    std::cerr << \"Expanded: \" << expansion_count << std::endl;"
    echo "  }"
    echo ""
    echo "测试完成后需要移除这些调试代码"
fi

# ==========================================
# 2. 分析 BPR 成本计算频率
# ==========================================

echo -e "${YELLOW}============ 2. BPR 成本计算频率分析 ============${NC}"
echo ""

# 检查 get_bpr_edge_cost 调用次数
echo "BPR 成本计算位置:"
echo "  - 文件: guided-pibt/traffic_mapf/search.cpp:186"
echo "  - 函数: get_bpr_edge_cost()"
echo "  - 调用频率: 每次 A* 扩展调用 4 次（4个邻居方向）"
echo ""

# 估算调用次数
echo "估算:"
echo "  假设每次搜索平均扩展 10,000 个节点"
echo "  每个节点检查 4 个邻居 → 40,000 次成本计算/搜索"
echo "  每个时间步可能搜索 100 次 → 4,000,000 次成本计算"
echo ""

# ==========================================
# 3. 性能profiling建议
# ==========================================

echo -e "${YELLOW}============ 3. 性能 Profiling 建议 ============${NC}"
echo ""

echo "使用 perf 进行性能分析:"
echo ""
echo "  # Baseline 性能分析"
echo "  cd build-baseline"
echo "  perf record -g ./lifelong --inputFile ../$TEST_MAP --simulationTime 5"
echo "  perf report"
echo ""
echo "  # BPR 性能分析"
echo "  cd ../build-bpr"
echo "  perf record -g ./lifelong --inputFile ../$TEST_MAP --simulationTime 5"
echo "  perf report"
echo ""

# ==========================================
# 4. BPR 计算复杂度分析
# ==========================================

echo -e "${YELLOW}============ 4. BPR 计算复杂度分析 ============${NC}"
echo ""

cat << 'EOF'
BPR 成本计算 (get_bpr_edge_cost) 复杂度分解:

  1. 边界检查: O(1)
     - 检查 u, v 是否在范围内
     - 检查 map[u], map[v] 是否可通行

  2. 方向计算: O(1)
     - get_d(diff, env) → 数组查找

  3. 流量读取: O(1)
     - lns.directional_flow[u][d] → 数组访问
     - lns.directional_flow[v][(d+2)%4] → 数组访问

  4. BPR 成本计算: O(1)
     - c_eff = C_max - γ * f_reverse
     - ratio = f_co / c_eff
     - ratio4 = ratio² * ratio² (两次乘法，避免 pow)
     - cost = BPR_T0 * (1 + α * ratio4)

  总计: O(1) 常数时间

  每次调用约: 10-15 次浮点运算 + 5-10 次内存访问

对比 Baseline 边成本:
  - Baseline: cost = curr->g + 1  (1 次整数加法)
  - BPR: cost = curr->g + edge_cost (约 20-30 次运算)

开销比例: 约 20-30 倍
EOF

echo ""

# ==========================================
# 5. 为什么有时 BPR 更快？
# ==========================================

echo -e "${YELLOW}============ 5. 性能悖论解释 ============${NC}"
echo ""

cat << 'EOF'
为什么 BPR 计算开销更大，但有时性能更好？

原因 1: 搜索空间缩小
┌─────────────────────────────────────┐
│ Baseline:                           │
│  - 搜索沿着最短路径                 │
│  - 容易陷入局部最优                 │
│  - A* 扩展更多节点                  │
│                                      │
│ BPR:                                │
│  - 避开拥塞区域                     │
│  - 引导到不同路径                   │
│  - 虽然边成本计算慢，但...          │
│  - 总扩展节点数可能更少！           │
└─────────────────────────────────────┘

原因 2: 拥塞避免带来的连锁效应
┌─────────────────────────────────────┐
│ Baseline:                           │
│  - 多个智能体竞争最短路径           │
│  - 频繁冲突 → 重规划 → 更多搜索     │
│                                      │
│ BPR:                                │
│  - 智能体自然分散                   │
│  - 冲突减少 → 重规划减少             │
│  - 总搜索次数减少                   │
└─────────────────────────────────────┘

原因 3: 地图特性
┌─────────────────────────────────────┐
│ Room-64-64-8 (小地图):              │
│  - 空间狭窄，拥塞严重               │
│  - BPR 的拥塞避免效果显著           │
│  → BPR 更快 (161 vs 293)           │
│                                      │
│ Denver-520d (大地图):               │
│  - 空间宽敞，拥塞较少               │
│  - BPR 计算开销大于收益             │
│  → BPR 稍慢 (20 vs 18)             │
│                                      │
│ Oslo-003d (中等地图):               │
│  - 介于两者之间                     │
│  → BPR 稍快 (34 vs 42)             │
└─────────────────────────────────────┘
EOF

echo ""

# ==========================================
# 6. 优化建议
# ==========================================

echo -e "${YELLOW}============ 6. 性能优化建议 ============${NC}"
echo ""

cat << 'EOF'
短期优化 (已实施):
✓ Release 模式编译 (-O3)
✓ 避免使用 std::pow (ratio4 = ratio2*ratio2)
✓ 使用 constexpr 参数
✓ 内联函数

中期优化 (可实施):
1. 缓存 BPR 成本计算结果
   - 对于相同的 (u,v) 对，缓存计算结果
   - 适用场景：流量变化不频繁时

2. 批量更新 BPR 流量
   - 减少频繁的 EMA 更新
   - 每 N 个时间步更新一次

3. 简化 BPR 公式
   - 当前: ratio⁴ (β=2.0, 但实际上用 ratio⁴)
   - 可尝试: ratio² (β=2.0)
   - 性能提升: ~2倍

长期优化 (需要重构):
1. 使用整数运算代替浮点
   - 预计算 BPR 查找表
   - 性能提升: ~3-5倍

2. 并行化成本计算
   - SIMD 指令优化
   - 性能提升: ~4-8倍

3. 自适应 BPR
   - 根据地图大小动态调整参数
   - 小地图: 弱化 BPR
   - 大地图: 强化 BPR
EOF

echo ""

exit 0
