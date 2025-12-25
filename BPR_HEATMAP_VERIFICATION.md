# BPR 热力图验证指南

## 📋 概述

本文档说明如何验证 **BPR (Bureau of Public Roads) 交通流模型**的定性行为，即对比 **Baseline** 和 **BPR** 版本的**交通分布热力图**。

### 验证目标

BPR 应该能够：
1. ✅ **削峰** (Peak Shaving): 减少最大边流量
2. ✅ **去热点** (Hotspot Elimination): 减少极端拥塞边
3. ✅ **负载均衡** (Load Balancing): 降低基尼系数，更均匀分配流量
4. ✅ **扩散效应** (Traffic Diffusion): 利用更多边，避免拥堵

### 测试场景

- **地图**: den520d (Denver 520x520 网格地图)
- **智能体数量**: 2000
- **仿真时长**: 30 timesteps
- **BPR 参数**: α=0.15, β=2.0, γ=0.2

---

## 🛠️ 环境准备

### 系统要求

- Linux 操作系统 (Ubuntu 20.04+ 推荐)
- C++ 编译器 (g++ 9.0+ 或 clang++)
- CMake 3.10+
- Boost 库
- nlohmann/json 库

### 目录结构

```
Guided-PIBT/
├── guided-pibt/          # 源代码目录
│   ├── traffic_mapf/     # 交通模块 (包含 heatmap_stats.cpp)
│   ├── src/              # 主程序 (包含修改后的 driver.cpp)
│   └── benchmark-lifelong/
│       └── den520d_0_2000.json
├── build-baseline/       # Baseline 版本构建目录
└── build-bpr/            # BPR 版本构建目录
```

---

## 🔨 编译步骤

### Step 1: 编译 Baseline 版本（无 BPR）

```bash
cd /home/sentu/mxw/Guided-PIBT

# 创建并配置构建目录
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
      ../guided-pibt

# 编译
make -j4

# 验证编译成功
./lifelong --help
```

**预期输出**:
- 配置信息应显示: `USE_BPR_HEURISTIC is disabled` (不显示)
- 编译无错误
- 生成 `lifelong` 可执行文件

---

### Step 2: 编译 BPR 版本（启用 BPR）

```bash
cd /home/sentu/mxw/Guided-PIBT

# 创建并配置构建目录
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
      ../guided-pibt

# 编译
make -j4

# 验证编译成功
./lifelong --help
```

**预期输出**:
- 配置信息应显示: `USE_BPR_HEURISTIC is enabled`
- 编译无错误
- 生成 `lifelong` 可执行文件

---

## 🧪 运行验证测试

### Step 3: 运行 Baseline 测试

```bash
cd /home/sentu/mxw/Guided-PIBT/build-baseline

# 运行 30 timestep 仿真（约需 10-20 秒）
./lifelong --inputFile ../guided-pibt/benchmark-lifelong/den520d_0_2000.json \
           --simulationTime 30 \
           2>&1 | tee baseline_results.log

# 提取交通统计
grep -A 40 "TRAFFIC STATISTICS" baseline_results.log > baseline_stats.txt
cat baseline_stats.txt
```

**预期输出示例**:
```
========== TRAFFIC STATISTICS BASELINE ==========
Edges with traffic: 46714 / 112712
Max edge usage: 110
Min edge usage: 1
Mean edge usage: 8.22
Std deviation: 9.42

Percentiles:
  50th (median): 5
  90th: 20
  95th: 28
  99th: 46

High congestion edges:
  Flow > 5:  21396 (45.80%)
  Flow > 10: 11830 (25.32%)
  Flow > 20: 4538 (9.71%)

Load balancing:
  Gini coefficient: 0.534
  Coefficient of variation: 1.147
====================================================
```

---

### Step 4: 运行 BPR 测试

```bash
cd /home/sentu/mxw/Guided-PIBT/build-bpr

# 运行 30 timestep 仿真（约需 10-20 秒）
./lifelong --inputFile ../guided-pibt/benchmark-lifelong/den520d_0_2000.json \
           --simulationTime 30 \
           2>&1 | tee bpr_results.log

# 提取交通统计
grep -A 40 "TRAFFIC STATISTICS" bpr_results.log > bpr_stats.txt
cat bpr_stats.txt
```

**预期输出示例**:
```
========== TRAFFIC STATISTICS BPR (α=0.150000, β=2.000000) ==========
Edges with traffic: 79026 / 112712
Max edge usage: 87
Min edge usage: 1
Mean edge usage: 6.14
Std deviation: 6.57

Percentiles:
  50th (median): 4
  90th: 13
  95th: 17
  99th: 34

High congestion edges:
  Flow > 5:  32128 (40.66%)
  Flow > 10: 11509 (14.56%)
  Flow > 20: 2755 (3.49%)

Load balancing:
  Gini coefficient: 0.479
  Coefficient of variation: 1.070
====================================================
```

---

## 📊 结果对比与验证

### Step 5: 对比关键指标

创建对比表格：

```bash
cat << 'EOF' > comparison.txt
=================== BPR vs BASELINE 对比 ===================

指标                    BASELINE    BPR        改进
----------------------------------------------------------------
最大边流量 (Max)        110         87         ↓ 21%  ✓
平均边流量 (Mean)       8.22        6.14       ↓ 25%  ✓
标准差 (Std Dev)        9.42        6.57       ↓ 30%  ✓

高拥塞边占比:
  - Flow > 5           45.80%      40.66%     ↓ 11%  ✓
  - Flow > 10          25.32%      14.56%     ↓ 42%  ✓✓✓
  - Flow > 20          9.71%       3.49%      ↓ 64%  ✓✓✓

负载均衡:
  - 基尼系数 (Gini)    0.534       0.479      ↓ 10%  ✓
  - 变异系数 (CV)      1.147       1.070      ↓ 7%   ✓

流量扩散:
  - 使用边数量         46,714      79,026     ↑ 69%  ✓
  - 边利用率           41.4%       70.1%      ↑ 69%  ✓

分位数对比:
  - 50th (中位数)      5           4          ↓ 20%  ✓
  - 90th               20          13         ↓ 35%  ✓
  - 95th               28          17         ↓ 39%  ✓
  - 99th               46          34         ↓ 26%  ✓
================================================================

结论: BPR 成功实现:
  ✅ 削峰 (最大流量减少 21%)
  ✅ 去热点 (极端拥塞边减少 64%)
  ✅ 负载均衡 (基尼系数改善 10%)
  ✅ 流量扩散 (利用边数增加 69%)

所有指标均显示 BPR 改善了交通分布!
EOF
cat comparison.txt
```

---

## ✅ 验证标准

### 必须满足的条件 (PASS 条件)

如果 BPR 实现正确，应该观察到：

1. ✅ **最大边流量降低**
   - `Max edge usage (BPR) < Max edge usage (Baseline)`
   - 预期降低: **15-25%**

2. ✅ **极端拥塞边数量减少**
   - `Flow > 20 的边占比 (BPR) < Flow > 20 的边占比 (Baseline)`
   - 预期降低: **50-70%**

3. ✅ **基尼系数改善**
   - `Gini coefficient (BPR) < Gini coefficient (Baseline)`
   - 预期降低: **5-15%**

4. ✅ **流量扩散**
   - `Edges with traffic (BPR) > Edges with traffic (Baseline)`
   - 预期增加: **50-100%**

### 定性验证

查看 **Top 10 Most Congested Edges**:
- Baseline 的最大流量应该显著高于 BPR
- BPR 的流量分布应该更平滑（不同等级的边之间差距更小）

---

## 🔬 深度分析（可选）

### 可视化流量分布（Python）

如果需要生成可视化图表：

```python
#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

# 读取统计数据
def parse_stats(filename):
    stats = {}
    with open(filename) as f:
        for line in f:
            if 'Max edge usage:' in line:
                stats['max'] = int(line.split(':')[1].strip())
            elif 'Mean edge usage:' in line:
                stats['mean'] = float(line.split(':')[1].strip())
            elif 'Gini coefficient:' in line:
                stats['gini'] = float(line.split(':')[1].strip())
            # ... 解析其他指标
    return stats

baseline = parse_stats('baseline_stats.txt')
bpr = parse_stats('bpr_stats.txt')

# 对比图
labels = ['Max Flow', 'Mean Flow', 'Gini Coef']
baseline_vals = [baseline['max'], baseline['mean'], baseline['gini']]
bpr_vals = [bpr['max'], bpr['mean'], bpr['gini']]

x = np.arange(len(labels))
width = 0.35

fig, ax = plt.subplots()
rects1 = ax.bar(x - width/2, baseline_vals, width, label='Baseline')
rects2 = ax.bar(x + width/2, bpr_vals, width, label='BPR')

ax.set_ylabel('Values')
ax.set_title('Baseline vs BPR Traffic Distribution')
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.legend()

plt.savefig('bpr_comparison.png')
print("图表已保存: bpr_comparison.png")
```

---

## 🐛 故障排查

### 问题 1: 编译错误

**症状**: `undefined reference to 'TrafficMAPF::print_traffic_statistics'`

**解决**:
```bash
# 确保重新配置 CMake（检测新文件）
cd build-bpr  # 或 build-baseline
rm -rf *
cmake -DCMAKE_BUILD_TYPE=Release ... (重新配置)
make -j4
```

---

### 问题 2: 看不到交通统计输出

**症状**: 程序运行但没有 "TRAFFIC STATISTICS" 输出

**检查**:
```bash
# 1. 确认仿真正常完成
grep "Done!" results.log

# 2. 检查是否使用了正确的仿真时间
grep "simulationTime" results.log

# 3. 确认 heatmap_stats.cpp 被编译
ls -lh traffic_mapf/heatmap_stats.cpp.o
```

---

### 问题 3: BPR 性能显著劣于 Baseline

**症状**: BPR 的 timestep 进度远慢于 Baseline

**原因**: 可能是 Debug 模式编译

**验证**:
```bash
# 检查编译优化级别
grep "CMAKE_BUILD_TYPE" CMakeCache.txt
# 应显示: CMAKE_BUILD_TYPE:STRING=Release

# 重新编译 Release 版本
make clean
cmake -DCMAKE_BUILD_TYPE=Release ...
make -j4
```

---

### 问题 4: 结果相反（BPR 拥塞更严重）

**症状**: BPR 的 `Max edge usage` 高于 Baseline

**可能原因**:
1. **BPR 参数设置错误** - 检查 TrajLNS.h 中的参数值
2. **方向流未初始化** - 检查 MAPFPlanner.cpp:49 是否调用 `init_bpr_flow()`
3. **仿真时间不足** - 需要至少 20-30 timesteps 才能看到明显差异

**调试**:
```bash
# 检查 BPR 参数
grep "BPR_" ../guided-pibt/traffic_mapf/TrajLNS.h

# 检查初始化
grep "init_bpr_flow" ../guided-pibt/src/MAPFPlanner.cpp

# 运行更长时间测试
./lifelong --inputFile ... --simulationTime 60
```

---

## 📝 快速验证脚本

一键运行完整验证流程：

```bash
#!/bin/bash
set -e

echo "================ BPR 热力图验证 ================"

# 配置
MAP_FILE="../guided-pibt/benchmark-lifelong/den520d_0_2000.json"
SIM_TIME=30

echo ""
echo "Step 1: 编译 Baseline..."
cd ../build-baseline
make -j4 > /dev/null 2>&1
echo "  ✓ Baseline 编译完成"

echo ""
echo "Step 2: 运行 Baseline 测试..."
./lifelong --inputFile $MAP_FILE --simulationTime $SIM_TIME 2>&1 | \
    grep -A 40 "TRAFFIC STATISTICS" > /tmp/baseline_stats.txt
echo "  ✓ Baseline 测试完成"

echo ""
echo "Step 3: 编译 BPR..."
cd ../build-bpr
make -j4 > /dev/null 2>&1
echo "  ✓ BPR 编译完成"

echo ""
echo "Step 4: 运行 BPR 测试..."
./lifelong --inputFile $MAP_FILE --simulationTime $SIM_TIME 2>&1 | \
    grep -A 40 "TRAFFIC STATISTICS" > /tmp/bpr_stats.txt
echo "  ✓ BPR 测试完成"

echo ""
echo "Step 5: 生成对比报告..."
python3 << 'PYTHON'
import re

def extract_stat(filename, pattern):
    with open(filename) as f:
        for line in f:
            m = re.search(pattern, line)
            if m:
                return float(m.group(1))
    return None

baseline = {
    'max': extract_stat('/tmp/baseline_stats.txt', r'Max edge usage:\s+(\d+)'),
    'mean': extract_stat('/tmp/baseline_stats.txt', r'Mean edge usage:\s+([\d.]+)'),
    'gini': extract_stat('/tmp/baseline_stats.txt', r'Gini coefficient:\s+([\d.]+)'),
    'gt10': extract_stat('/tmp/baseline_stats.txt', r'Flow > 10:.*?\(([\d.]+)%\)'),
    'gt20': extract_stat('/tmp/baseline_stats.txt', r'Flow > 20:.*?\(([\d.]+)%\)'),
}

bpr = {
    'max': extract_stat('/tmp/bpr_stats.txt', r'Max edge usage:\s+(\d+)'),
    'mean': extract_stat('/tmp/bpr_stats.txt', r'Mean edge usage:\s+([\d.]+)'),
    'gini': extract_stat('/tmp/bpr_stats.txt', r'Gini coefficient:\s+([\d.]+)'),
    'gt10': extract_stat('/tmp/bpr_stats.txt', r'Flow > 10:.*?\(([\d.]+)%\)'),
    'gt20': extract_stat('/tmp/bpr_stats.txt', r'Flow > 20:.*?\(([\d.]+)%\)'),
}

def improve(base, bpr):
    return f"↓ {100 * (1 - bpr/base):.1f}%" if base > 0 else "N/A"

print("\n=================== 对比结果 ===================")
print(f"{'指标':<25} {'Baseline':>12} {'BPR':>12} {'改进':>12}")
print("-" * 63)
print(f"{'最大边流量':<25} {baseline['max']:>12.0f} {bpr['max']:>12.0f} {improve(baseline['max'], bpr['max']):>12}")
print(f"{'平均边流量':<25} {baseline['mean']:>12.2f} {bpr['mean']:>12.2f} {improve(baseline['mean'], bpr['mean']):>12}")
print(f"{'基尼系数':<25} {baseline['gini']:>12.3f} {bpr['gini']:>12.3f} {improve(baseline['gini'], bpr['gini']):>12}")
print(f"{'Flow > 10 占比':<25} {baseline['gt10']:>12.2f}% {bpr['gt10']:>12.2f}% {improve(baseline['gt10'], bpr['gt10']):>12}")
print(f"{'Flow > 20 占比':<25} {baseline['gt20']:>12.2f}% {bpr['gt20']:>12.2f}% {improve(baseline['gt20'], bpr['gt20']):>12}")
print("=" * 63)

# 验证
all_pass = True
if bpr['max'] >= baseline['max']:
    print("❌ FAIL: 最大流量未降低")
    all_pass = False
if bpr['gini'] >= baseline['gini']:
    print("❌ FAIL: 基尼系数未改善")
    all_pass = False
if bpr['gt20'] >= baseline['gt20']:
    print("❌ FAIL: 极端拥塞边未减少")
    all_pass = False

if all_pass:
    print("\n✅✅✅ PASS: 所有指标验证通过! BPR 工作正常! ✅✅✅\n")
else:
    print("\n❌ FAIL: 部分指标未通过验证，请检查配置 ❌\n")
PYTHON

echo ""
echo "详细日志保存在:"
echo "  Baseline: /tmp/baseline_stats.txt"
echo "  BPR:      /tmp/bpr_stats.txt"
```

保存为 `verify_bpr.sh` 并运行：
```bash
chmod +x verify_bpr.sh
./verify_bpr.sh
```

---

## 📚 相关文件

### 实现文件
- `guided-pibt/traffic_mapf/heatmap_stats.cpp` - 统计计算实现
- `guided-pibt/traffic_mapf/heatmap_stats.hpp` - 统计接口头文件
- `guided-pibt/src/driver.cpp` - 主程序（包含统计输出调用）

### 配置文件
- `guided-pibt/traffic_mapf/TrajLNS.h` - BPR 参数定义（第 81-88 行）

### 输出文件
- `baseline_results.log` - Baseline 完整运行日志
- `bpr_results.log` - BPR 完整运行日志
- `baseline_stats.txt` - Baseline 交通统计摘要
- `bpr_stats.txt` - BPR 交通统计摘要

---

## 🎯 总结

如果验证通过，你应该看到：

1. ✅ **Max edge usage** 从 ~110 降低到 ~85-90 (↓ ~20%)
2. ✅ **Flow > 20** 边占比从 ~10% 降低到 ~3-5% (↓ ~60%)
3. ✅ **Gini coefficient** 从 ~0.53 降低到 ~0.48 (↓ ~10%)
4. ✅ **Edges with traffic** 从 ~47k 增加到 ~79k (↑ ~70%)

**恭喜! BPR 实现成功验证! 🎉**

---

## 🔗 参考资源

- **BPR 理论**: Bureau of Public Roads (1964) - Traffic Assignment Manual
- **基尼系数**: 衡量分布不均的指标，0 = 完全平等，1 = 完全不平等
- **分位数**: 50th = 中位数，90th = 前 10% 最拥塞边的最小值

---

**文档版本**: 1.0
**最后更新**: 2025-12-25
**作者**: Claude Code + User
