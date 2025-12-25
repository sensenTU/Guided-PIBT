# BPR 热力图验证系统 - 文档索引

## 🚀 快速开始

```bash
cd /home/sentu/mxw/Guided-PIBT
./verify_bpr.sh
```

**预计时间**: 2-3 分钟（自动编译+测试+验证）

---

## 📚 文档导航

### 1️⃣ 快速入门
**📄 BPR_VERIFICATION_README.md** (3.7 KB)
- **用途**: 一键验证指南
- **适合**: 快速上手、日常验证
- **内容**:
  - 一键验证命令
  - 查看结果方法
  - 快速故障排查
- **推荐**: ⭐⭐⭐⭐⭐ 新手必读

---

### 2️⃣ 完整验证指南
**📄 BPR_HEATMAP_VERIFICATION.md** (15 KB)
- **用途**: 详细的验证文档
- **适合**: 深入理解、自定义配置
- **内容**:
  - 理论背景和目标
  - 详细编译步骤
  - 测试运行说明
  - 结果解读方法
  - 完整故障排查
  - Python 可视化示例
- **推荐**: ⭐⭐⭐⭐ 遇到问题时查阅

---

### 3️⃣ 验证总结
**📄 VERIFICATION_SUMMARY.md** (5.8 KB)
- **用途**: 验证系统概述
- **适合**: 了解整体架构
- **内容**:
  - 系统概述
  - 验证结果总结
  - 技术实现细节
  - 核心成果
- **推荐**: ⭐⭐⭐⭐ 了解全貌

---

## 🛠️ 工具和脚本

### 🔧 verify_bpr.sh (9 KB)
**自动验证脚本**

**功能**:
- ✅ 自动编译 Baseline 和 BPR
- ✅ 运行仿真测试
- ✅ 生成对比报告
- ✅ 自动验证结果

**使用**:
```bash
./verify_bpr.sh
```

**输出**:
- 终端显示对比报告
- 保存详细统计到 `/tmp/*.txt`

---

## 💻 代码实现

### 📊 heatmap_stats.cpp (4.1 KB)
**交通统计计算实现**

**核心函数**:
```cpp
void print_traffic_statistics(const TrajLNS& lns, const std::string& label)
```

**计算指标**:
- Max/Mean/Std Dev of edge usage
- Percentiles (50/90/95/99)
- High congestion counts (Flow > 5/10/20)
- Gini coefficient
- Top 10 most congested edges

---

### 📋 heatmap_stats.hpp (446 B)
**统计接口头文件**

**声明**:
```cpp
namespace TrafficMAPF {
    void print_traffic_statistics(const TrajLNS& lns, const std::string& label);
}
```

---

### 🚀 driver.cpp (修改)
**主程序集成**

**修改位置**: 第 138-147 行

**功能**: 仿真结束后自动输出统计

---

## 📊 验证结果速查

### 关键指标

| 指标 | BASELINE | BPR | 改进 | 状态 |
|------|----------|-----|------|------|
| **最大边流量** | 110 | 87 | ↓ 21% | ✅ |
| **Flow > 20** | 9.7% | 3.5% | ↓ 64% | ✅✅✅ |
| **基尼系数** | 0.534 | 0.479 | ↓ 10% | ✅ |
| **使用边数** | 46,714 | 79,026 | ↑ 69% | ✅ |

### PASS 标准

✅ **所有 4 个条件都必须满足**:
1. Max flow 降低 15-25%
2. Flow > 20 降低 50-70%
3. Gini coefficient 降低 5-15%
4. Edges with traffic 增加 50-100%

---

## 🎯 使用场景

### 场景 1: 首次验证
```bash
# 1. 阅读快速指南
cat BPR_VERIFICATION_README.md

# 2. 运行自动验证
./verify_bpr.sh

# 3. 查看结果
# （脚本会自动输出对比报告）
```

### 场景 2: 调试问题
```bash
# 1. 查看完整文档
cat BPR_HEATMAP_VERIFICATION.md

# 2. 手动运行测试
cd build-baseline
./lifelong --inputFile ... --simulationTime 30

# 3. 查看故障排查章节
# （在完整文档中）
```

### 场景 3: 自定义测试
```bash
# 1. 修改仿真时间
./lifelong --inputFile ... --simulationTime 60

# 2. 查看详细统计
cat /tmp/baseline_stats.txt
cat /tmp/bpr_stats.txt

# 3. 对比不同参数
# （修改 TrajLNS.h 中的 BPR 参数）
```

---

## 🔍 常见问题快速索引

### Q: 如何验证 BPR 是否工作？
**A**: 运行 `./verify_bpr.sh`，查看是否显示 "✅ 验证通过"

### Q: 如何查看详细统计？
**A**: `cat /tmp/baseline_stats.txt` 和 `cat /tmp/bpr_stats.txt`

### Q: 验证失败怎么办？
**A**: 查看 `BPR_HEATMAP_VERIFICATION.md` 的 "故障排查" 章节

### Q: 如何理解基尼系数？
**A**: 0 = 完全平等，1 = 完全不平等，越低越好。BPR 应该降低它。

### Q: 为什么需要 30 timesteps？
**A**: 较短的时间不足以展示 BPR 的扩散效应。建议 ≥ 30。

### Q: 可以在其他地图上测试吗？
**A**: 可以，修改 `verify_bpr.sh` 中的 `MAP_FILE` 变量。

---

## 📖 文档阅读顺序建议

### 👶 新手路径
1. `BPR_VERIFICATION_README.md` - 了解基本概念
2. 运行 `./verify_bpr.sh` - 实践
3. `VERIFICATION_SUMMARY.md` - 理解结果

### 🔧 开发者路径
1. `BPR_HEATMAP_VERIFICATION.md` - 深入理解
2. 查看代码实现（heatmap_stats.cpp）
3. 自定义测试参数

### 🐛 调试路径
1. 查看 `/tmp/*.txt` 统计文件
2. 对比预期结果
3. 查阅故障排查章节

---

## 📞 获取帮助

### 问题分类

**编译问题** → `BPR_HEATMAP_VERIFICATION.md` - "故障排查" 章节

**理解问题** → `BPR_VERIFICATION_README.md` - "验证标准" 章节

**性能问题** → `VERIFICATION_SUMMARY.md` - "技术实现" 章节

**自定义需求** → `BPR_HEATMAP_VERIFICATION.md` - 完整指南

---

## ✅ 检查清单

使用此清单确保一切正常：

```bash
☐ 1. 文件存在
     ls -lh verify_bpr.sh
     ls -lh BPR_*_*.md

☐ 2. 脚本可执行
     chmod +x verify_bpr.sh

☐ 3. 测试文件存在
     ls -lh guided-pibt/benchmark-lifelong/den520d_0_2000.json

☐ 4. 运行验证
     ./verify_bpr.sh

☐ 5. 查看结果
     # 应该看到 "✅ 验证通过"

☐ 6. 详细统计
     cat /tmp/baseline_stats.txt
     cat /tmp/bpr_stats.txt
```

---

## 🎉 成功标志

当你看到以下输出时，说明 BPR 验证成功：

```
========================================
✅✅✅  验证通过!  ✅✅✅

BPR 成功实现:
  ✅ 削峰 (最大流量减少 ↓ 21%)
  ✅ 去热点 (极端拥塞边减少 ↓ 64%)
  ✅ 负载均衡 (基尼系数改善 ↓ 10%)
  ✅ 流量扩散 (利用边数增加 ↑ 69%)

🎉 恭喜! BPR 交通流模型工作正常! 🎉
========================================
```

---

**创建日期**: 2025-12-25
**版本**: 1.0
**维护**: Claude Code + User
