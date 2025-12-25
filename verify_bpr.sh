#!/bin/bash
# BPR 热力图自动验证脚本
# 快速验证 BPR 交通流模型的定性行为改善

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   BPR 热力图自动验证脚本${NC}"
echo -e "${BLUE}========================================${NC}"

# 配置
MAP_FILE="../guided-pibt/benchmark-lifelong/den520d_0_2000.json"
SIM_TIME=30
PROJECT_ROOT="/home/sentu/mxw/Guided-PIBT"

# 检查目录
cd "$PROJECT_ROOT"
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}错误: 找不到测试文件 $MAP_FILE${NC}"
    exit 1
fi

# ==================== Step 1: Baseline ====================
echo ""
echo -e "${YELLOW}[1/5] 编译 Baseline 版本...${NC}"

if [ ! -d "build-baseline" ]; then
    mkdir -p build-baseline
    cd build-baseline
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DGUIDANCE=ON \
          -DGUIDANCE_LNS=10 \
          -DFLOW_GUIDANCE=OFF \
          -DINIT_PP=ON \
          -DRELAX=100 \
          -DOBJECTIVE=1 \
          -DFOCAL_SEARCH=2 \
          ../guided-pibt > /dev/null 2>&1
else
    cd build-baseline
fi

make -j4 > /dev/null 2>&1
echo -e "${GREEN}  ✓ Baseline 编译完成${NC}"

# ==================== Step 2: 运行 Baseline ====================
echo ""
echo -e "${YELLOW}[2/5] 运行 Baseline 测试 (${SIM_TIME} timesteps)...${NC}"

./lifelong --inputFile "$MAP_FILE" --simulationTime $SIM_TIME 2>&1 | \
    grep -A 40 "TRAFFIC STATISTICS" > /tmp/baseline_stats.txt

if [ ! -s /tmp/baseline_stats.txt ]; then
    echo -e "${RED}错误: Baseline 测试失败，未生成统计数据${NC}"
    exit 1
fi

echo -e "${GREEN}  ✓ Baseline 测试完成${NC}"

# ==================== Step 3: BPR ====================
echo ""
echo -e "${YELLOW}[3/5] 编译 BPR 版本...${NC}"

cd "$PROJECT_ROOT"

if [ ! -d "build-bpr" ]; then
    mkdir -p build-bpr
    cd build-bpr
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DUSE_BPR_HEURISTIC=ON \
          -DGUIDANCE=ON \
          -DGUIDANCE_LNS=10 \
          -DFLOW_GUIDANCE=OFF \
          -DINIT_PP=ON \
          -DRELAX=100 \
          -DOBJECTIVE=1 \
          -DFOCAL_SEARCH=2 \
          ../guided-pibt > /dev/null 2>&1
else
    cd build-bpr
fi

make -j4 > /dev/null 2>&1
echo -e "${GREEN}  ✓ BPR 编译完成${NC}"

# ==================== Step 4: 运行 BPR ====================
echo ""
echo -e "${YELLOW}[4/5] 运行 BPR 测试 (${SIM_TIME} timesteps)...${NC}"

./lifelong --inputFile "$MAP_FILE" --simulationTime $SIM_TIME 2>&1 | \
    grep -A 40 "TRAFFIC STATISTICS" > /tmp/bpr_stats.txt

if [ ! -s /tmp/bpr_stats.txt ]; then
    echo -e "${RED}错误: BPR 测试失败，未生成统计数据${NC}"
    exit 1
fi

echo -e "${GREEN}  ✓ BPR 测试完成${NC}"

# ==================== Step 5: 对比分析 ====================
echo ""
echo -e "${YELLOW}[5/5] 生成对比报告...${NC}"

python3 << 'PYTHON'
import re
import sys

def extract_stat(filename, pattern, dtype=float):
    """从统计文件中提取数值"""
    try:
        with open(filename) as f:
            content = f.read()
            m = re.search(pattern, content)
            if m:
                return dtype(m.group(1))
    except Exception as e:
        print(f"警告: 无法解析 {filename}: {e}")
    return None

def percent_improvement(base, new):
    """计算改进百分比"""
    if base is None or new is None or base == 0:
        return "N/A"
    return f"↓ {100 * (1 - new/base):.1f}%"

def percent_increase(base, new):
    """计算增长百分比"""
    if base is None or new is None or base == 0:
        return "N/A"
    return f"↑ {100 * (new/base - 1):.1f}%"

# 提取统计数据
baseline = {}
bpr = {}

try:
    baseline['max'] = extract_stat('/tmp/baseline_stats.txt', r'Max edge usage:\s+(\d+)', int)
    baseline['mean'] = extract_stat('/tmp/baseline_stats.txt', r'Mean edge usage:\s+([\d.]+)')
    baseline['std'] = extract_stat('/tmp/baseline_stats.txt', r'Std deviation:\s+([\d.]+)')
    baseline['gini'] = extract_stat('/tmp/baseline_stats.txt', r'Gini coefficient:\s+([\d.]+)')
    baseline['cv'] = extract_stat('/tmp/baseline_stats.txt', r'Coefficient of variation:\s+([\d.]+)')
    baseline['edges'] = extract_stat('/tmp/baseline_stats.txt', r'Edges with traffic:\s+(\d+)', int)
    baseline['gt5'] = extract_stat('/tmp/baseline_stats.txt', r'Flow > 5:.*?\(([\d.]+)%\)')
    baseline['gt10'] = extract_stat('/tmp/baseline_stats.txt', r'Flow > 10:.*?\(([\d.]+)%\)')
    baseline['gt20'] = extract_stat('/tmp/baseline_stats.txt', r'Flow > 20:.*?\(([\d.]+)%\)')

    bpr['max'] = extract_stat('/tmp/bpr_stats.txt', r'Max edge usage:\s+(\d+)', int)
    bpr['mean'] = extract_stat('/tmp/bpr_stats.txt', r'Mean edge usage:\s+([\d.]+)')
    bpr['std'] = extract_stat('/tmp/bpr_stats.txt', r'Std deviation:\s+([\d.]+)')
    bpr['gini'] = extract_stat('/tmp/bpr_stats.txt', r'Gini coefficient:\s+([\d.]+)')
    bpr['cv'] = extract_stat('/tmp/bpr_stats.txt', r'Coefficient of variation:\s+([\d.]+)')
    bpr['edges'] = extract_stat('/tmp/bpr_stats.txt', r'Edges with traffic:\s+(\d+)', int)
    bpr['gt5'] = extract_stat('/tmp/bpr_stats.txt', r'Flow > 5:.*?\(([\d.]+)%\)')
    bpr['gt10'] = extract_stat('/tmp/bpr_stats.txt', r'Flow > 10:.*?\(([\d.]+)%\)')
    bpr['gt20'] = extract_stat('/tmp/bpr_stats.txt', r'Flow > 20:.*?\(([\d.]+)%\)')
except Exception as e:
    print(f"错误: 无法提取统计数据: {e}")
    sys.exit(1)

# 打印对比表
print("\n" + "="*70)
print(" " * 20 + "BPR vs BASELINE 对比结果")
print("="*70)
print(f"{'指标':<30} {'BASELINE':>12} {'BPR':>12} {'改进':>12}")
print("-"*70)

# 峰值拥塞
print(f"{'峰值拥塞:':<30}")
print(f"  {'最大边流量 (Max)':<28} {baseline['max']:>12.0f} {bpr['max']:>12.0f} {percent_improvement(baseline['max'], bpr['max']):>12}")
print(f"  {'平均边流量 (Mean)':<28} {baseline['mean']:>12.2f} {bpr['mean']:>12.2f} {percent_improvement(baseline['mean'], bpr['mean']):>12}")
print(f"  {'标准差 (Std Dev)':<28} {baseline['std']:>12.2f} {bpr['std']:>12.2f} {percent_improvement(baseline['std'], bpr['std']):>12}")

print(f"\n{'高拥塞边占比:':<30}")
print(f"  {'Flow > 5':<28} {baseline['gt5']:>12.2f}% {bpr['gt5']:>12.2f}% {percent_improvement(baseline['gt5'], bpr['gt5']):>12}")
print(f"  {'Flow > 10':<28} {baseline['gt10']:>12.2f}% {bpr['gt10']:>12.2f}% {percent_improvement(baseline['gt10'], bpr['gt10']):>12}")
print(f"  {'Flow > 20':<28} {baseline['gt20']:>12.2f}% {bpr['gt20']:>12.2f}% {percent_improvement(baseline['gt20'], bpr['gt20']):>12}")

print(f"\n{'负载均衡:':<30}")
print(f"  {'基尼系数 (Gini)':<28} {baseline['gini']:>12.3f} {bpr['gini']:>12.3f} {percent_improvement(baseline['gini'], bpr['gini']):>12}")
print(f"  {'变异系数 (CV)':<28} {baseline['cv']:>12.3f} {bpr['cv']:>12.3f} {percent_improvement(baseline['cv'], bpr['cv']):>12}")

print(f"\n{'流量扩散:':<30}")
print(f"  {'使用边数量':<28} {baseline['edges']:>12,} {bpr['edges']:>12,} {percent_increase(baseline['edges'], bpr['edges']):>12}")

print("="*70)

# 验证结果
all_pass = True
fail_reasons = []

if bpr['max'] >= baseline['max']:
    all_pass = False
    fail_reasons.append(f"❌ 最大流量未降低 ({baseline['max']} -> {bpr['max']})")

if bpr['gini'] >= baseline['gini']:
    all_pass = False
    fail_reasons.append(f"❌ 基尼系数未改善 ({baseline['gini']:.3f} -> {bpr['gini']:.3f})")

if bpr['gt20'] >= baseline['gt20']:
    all_pass = False
    fail_reasons.append(f"❌ 极端拥塞边(>20)未减少 ({baseline['gt20']:.1f}% -> {bpr['gt20']:.1f}%)")

if bpr['edges'] <= baseline['edges']:
    all_pass = False
    fail_reasons.append(f"❌ 使用边数未增加 ({baseline['edges']} -> {bpr['edges']})")

# 打印验证结果
print("\n" + "="*70)
if all_pass:
    print(" " * 15 + "✅✅✅  验证通过!  ✅✅✅")
    print("\nBPR 成功实现:")
    print("  ✅ 削峰 (最大流量减少 " + percent_improvement(baseline['max'], bpr['max']) + ")")
    print("  ✅ 去热点 (极端拥塞边减少 " + percent_improvement(baseline['gt20'], bpr['gt20']) + ")")
    print("  ✅ 负载均衡 (基尼系数改善 " + percent_improvement(baseline['gini'], bpr['gini']) + ")")
    print("  ✅ 流量扩散 (利用边数增加 " + percent_increase(baseline['edges'], bpr['edges']) + ")")
    print("\n🎉 恭喜! BPR 交通流模型工作正常! 🎉")
else:
    print(" " * 20 + "❌  验证失败  ❌")
    print("\n未通过的指标:")
    for reason in fail_reasons:
        print(f"  {reason}")
    print("\n请检查:")
    print("  1. BPR 参数是否正确 (TrajLNS.h)")
    print("  2. init_bpr_flow() 是否被调用 (MAPFPlanner.cpp:49)")
    print("  3. 编译模式是否为 Release")
    print("  4. 仿真时间是否足够 (≥30 timesteps)")

print("="*70 + "\n")
PYTHON

# 保存完整日志
echo ""
echo -e "${BLUE}详细日志已保存:${NC}"
echo "  /tmp/baseline_stats.txt"
echo "  /tmp/bpr_stats.txt"
echo ""
echo -e "${BLUE}查看完整统计:${NC}"
echo "  cat /tmp/baseline_stats.txt"
echo "  cat /tmp/bpr_stats.txt"
echo ""

exit 0
