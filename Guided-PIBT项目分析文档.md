# Guided-PIBT 项目分析文档

> 项目：Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding (AAAI 2024)
>
> 生成日期：2025-12-24

---

## 目录

1. [项目概述](#项目概述)
2. [术语表](#术语表)
   - [MAPF相关术语](#mapf相关术语)
   - [算法特定术语](#算法特定术语)
   - [代码关键类名和函数名](#代码关键类名和函数名)
   - [实验Benchmark命名规则](#实验benchmark命名规则)
3. [代码结构分析](#代码结构分析)
4. [算法基础](#算法基础)
5. [设计模式](#设计模式)
6. [架构组织](#架构组织)

---

## 项目概述

**Guided-PIBT** 是一个先进的多智能体路径规划（MAPF）系统，实现了两个主要模块：

- **Guided-PIBT**: 用于 Lifelong Planning（持续任务规划）
- **Guided-Lacam2**: 用于 One-shot Planning（单次规划）

### 核心创新

通过添加**引导路径系统**和**交通流优化**，将简单的优先级算法（PIBT）转变为适合终身 MAPF 的高性能系统。

### 技术栈

- **语言**: C++17
- **构建**: CMake 3.16+
- **依赖**: Boost >= 1.83.0, Google Test, nlohmann/json

---

## 术语表

### MAPF相关术语

| 术语 | 英文全称 | 解释 |
|------|---------|------|
| **MAPF** | Multi-Agent Path Finding | 多智能体路径规划问题，研究如何为多个智能体规划无碰撞的路径 |
| **Lifelong MAPF** | Lifelong Multi-Agent Path Finding | 终身MAPF，智能体完成任务后会接收新任务，持续工作 |
| **One-shot MAPF** | One-shot Multi-Agent Path Finding | 单次MAPF，所有智能体的任务预先给定，规划完成后即结束 |
| **PIBT** | Priority-based Iterative Best Response | 基于优先级的迭代最优响应算法 |
| **Causal PIBT** | Causal Priority-based Iterative Best Response | 因果PIBT，当高优先级智能体抢占位置时，递归地重新规划低优先级智能体 |
| **LNS** | Large Neighborhood Search | 大邻域搜索，一种迭代改进算法，通过破坏和重建部分解来优化 |
| **Focal Search** | Focal Search | 焦点搜索，在f值不超过最优值×权重的节点中搜索 |
| **Makespan** | Makespan | 所有智能体完成时间的最大值 |
| **SOC** | Sum of Costs | 总路径长度，所有智能体路径长度之和 |
| **SUI** | Sum of Individual Costs | 个体成本总和（与SOC类似） |
| **Flow** | Flow | 流量，描述智能体在地图上的移动模式 |
| **Vertex Conflict** | Vertex Conflict | 顶点冲突，两个智能体在同一时间步占据同一位置 |
| **Edge Conflict** | Edge Conflict | 边冲突，两个智能体在同一时间步交换位置 |
| **Opposing Flow** | Opposing Flow | 对向流，相反方向移动的智能体流 |
| **Heuristic** | Heuristic | 启发式函数，用于估计从当前位置到目标的代价 |
| **Manhattan Distance** | Manhattan Distance | 曼哈顿距离，网格中两点间的最短路径长度（不考虑障碍） |

---

### 算法特定术语

| 术语 | 解释 |
|------|------|
| **Guided-PIBT** | 带引导路径的PIBT算法，上层使用改进A*计算引导路径，下层PIBT使用引导路径作为启发式 |
| **Guided-Lacam2** | 带引导路径的LaCAM2算法，用于One-shot MAPF场景 |
| **Guidance Path** | 引导路径，预计算的优化路径，用于指导PIBT的决策 |
| **Traffic Flow Optimization** | 交通流优化，通过减少对向流和顶点冲突来优化整体交通 |
| **Opposing Flow** | 对向流，在边上相反方向移动的智能体对的期望数量 |
| **Vertex Flow** | 顶点流量，经过同一位置的智能体数量 |
| **Adaptive Selection** | 自适应选择，LNS中根据权重动态选择邻域策略（随机/拥堵感知） |
| **Tabu List** | 禁忌列表，记录最近被重新规划的智能体，避免重复选择 |
| **Label Mechanism** | 标签机制，内存池中用于快速批量清理节点的版本标记 |
| **Memory Pool** | 内存池，预分配搜索节点内存，避免动态分配开销 |
| **Priority Queue** | 优先级队列，用于A*搜索的开放列表，支持decrease_key操作 |
| **Relaxation** | 放松，限制每个时间步初始化的智能体数量，避免计算过载 |
| **Destroy-Improve** | 破坏-改进，LNS的核心循环：移除部分智能体的轨迹，重新规划它们 |

#### 编译选项术语

| 宏/选项 | 值 | 说明 |
|---------|---|------|
| **GUIDANCE** | ON/OFF | 启用/禁用引导路径系统 |
| **GUIDANCE_LNS** | int | LNS迭代次数（0表示关闭路径优化） |
| **FLOW_GUIDANCE** | ON/OFF | 启用/禁用流量代价启发式 |
| **INIT_PP** | ON/OFF | ON：像优先级规划一样逐个初始化（考虑拥堵）；OFF：使用独立最短路径 |
| **RELAX** | int | 限制引导路径/启发式表初始化的智能体数量 |
| **OBJECTIVE** | 0-5 | 优化目标函数（见下表） |
| **FOCAL_SEARCH** | float/OFF | 焦点搜索权重（>1.0启用，OFF关闭） |

#### OBJECTIVE 优化目标

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | NONE | 每步移动代价为1 |
| 1 | O_VC | 对向流 + 顶点流（两部分目标） |
| 2 | VC | 仅顶点流 |
| 3 | SUM_OVC | 对向流 + 顶点流总和 |
| 4 | SUI_TC | SUI Cost to Come（已付出的代价） |
| 5 | SUI_TG | SUI Cost to Go（未来代价） |

---

### 代码关键类名和函数名

#### 核心类

| 类名 | 文件 | 职责 |
|------|------|------|
| **MAPFPlanner** | `inc/MAPFPlanner.h` | 主规划器，协调所有组件 |
| **TrajLNS** | `traffic_mapf/TrajLNS.h` | 轨迹大邻域搜索模块 |
| **SharedEnvironment** | `inc/SharedEnv.h` | 共享环境，包含地图、智能体状态等 |
| **Grid** | `inc/Grid.h` | 网格地图 |
| **State** | `inc/States.h` | 智能体状态（位置、时间、方向） |
| **MemoryPool** | `traffic_mapf/Memory.h` | 搜索节点内存池 |
| **s_node** | `traffic_mapf/search_node.h` | A*搜索节点 |
| **HeuristicTable** | `traffic_mapf/Types.h` | Dijkstra预计算的启发式表 |
| **FlowHeuristic** | `traffic_mapf/TrajLNS.h` | 流量感知启发式 |
| **Dist2Path** | `traffic_mapf/TrajLNS.h` | 距离到路径表 |

#### 数据结构

| 类型 | 定义 | 说明 |
|------|------|------|
| **Traj** | `std::vector<int>` | 轨迹，位置序列 |
| **Path** | `std::vector<State>` | 路径，状态序列 |
| **Int4** | `struct Int4 { int d[4]; }` | 四个方向的流出流量 |
| **Action** | `enum Action { W, FW, CR, CCR }` | 动作类型：等待、前进、顺时针旋转、逆时针旋转 |

#### 核心函数

| 函数名 | 文件 | 功能 |
|--------|------|------|
| **causalPIBT** | `traffic_mapf/pibt.cpp` | 因果PIBT算法，带回溯机制 |
| **aStarOF** | `traffic_mapf/search.cpp` | 优化流量的A*算法，计算引导路径 |
| **singleShortestPath** | `traffic_mapf/search.cpp` | 简化路径，沿梯度下降 |
| **destory_improve** | `traffic_mapf/flow.cpp` | LNS破坏-改进循环 |
| **random_select** | `traffic_mapf/flow.cpp` | 随机邻域选择策略 |
| **select_most_expensive** | `traffic_mapf/flow.cpp` | 拥堵感知邻域选择策略 |
| **init_heuristic** | `traffic_mapf/heuristics.cpp` | 初始化Dijkstra启发式表 |
| **get_heuristic** | `traffic_mapf/heuristics.cpp` | 查询启发式值（增量计算） |
| **init_flow_heuristic** | `traffic_mapf/flow.cpp` | 初始化流量启发式 |
| **get_flow_heuristic** | `traffic_mapf/flow.cpp` | 查询流量启发式值 |
| **init_dist_2_path** | `traffic_mapf/flow.cpp` | 初始化距离-路径表 |
| **get_dist_2_path** | `traffic_mapf/flow.cpp` | 查询到引导路径的距离 |
| **remove_traj** | `traffic_mapf/flow.cpp` | 移除轨迹，更新流量统计 |
| **add_traj** | `traffic_mapf/flow.cpp` | 添加轨迹，更新流量统计 |
| **update_focal** | `traffic_mapf/search.cpp` | 更新focal列表（Focal Search） |
| **getAction** | `src/MAPFPlanner.cpp` | 根据状态变化生成动作 |
| **getNeighborLocs** | `traffic_mapf/utils.cpp` | 获取邻居位置列表 |
| **get_op_flow** | `traffic_mapf/flow.cpp` | 计算对向流 |
| **get_vertex_flow** | `traffic_mapf/flow.cpp` | 计算顶点流量 |

#### 优先级队列特化类型

| 类型名 | 定义 | 用途 |
|--------|------|------|
| **pqueue_min_f** | `pqueue<s_node, cmp_less_f, min_q>` | F值优先的优先级队列 |
| **pqueue_min_of** | `pqueue<s_node, cmp_less_of, min_q>` | 反向流优先的优先级队列 |
| **pqueue_min_jam** | `pqueue<s_node, cmp_less_jam, min_q>` | 拥堵感知优先的优先级队列 |

---

### 实验Benchmark命名规则

#### Lifelong MAPF Benchmark 命名

格式：`{地图名}_{智能体数量}_{任务集}_{智能体数量}_{任务数量}.json`

示例：`Paris_1_256_0_10000.json`

- **Paris**: 地图名称（巴黎仓库地图）
- **1**: 种子编号（随机种子）
- **256**: 智能体数量
- **0**: 任务集编号（0-24）
- **10000**: 任务总数

#### 地图类型

| 地图名 | 说明 |
|--------|------|
| **Paris** | 巴黎仓库地图 |
| **sortation_small** | 小型分拣中心地图 |

#### One-shot MAPF Benchmark 命名

地图文件格式：`{类型}-{行数}-{列数}-{障碍率}.map`

- **random**: 随机地图
- **warehouse**: 仓库地图

场景文件格式：`{地图名}-{种子}.scen`

示例：`random-32-32-10.map` - 32x32随机地图，10%障碍率
示例：`random-32-32-10-random-1.scen` - 使用种子1生成的场景

---

## 代码结构分析

### 整体目录结构

```
Guided-PIBT/
├── guided-pibt/          # Guided-PIBT 模块 (Lifelong Planning)
│   ├── src/              # 主要应用逻辑
│   ├── inc/              # 头文件
│   ├── traffic_mapf/     # 核心算法实现
│   ├── maps/             # 地图文件
│   └── benchmark-lifelong/  # 基准测试数据
├── guided-lacam2/        # Guided-Lacam2 模块 (One-shot Planning)
│   ├── lacam2/           # Lacam2 算法核心
│   ├── assets/           # 资源文件
│   └── scripts/          # 脚本和配置
├── LICENCE.txt
└── README.md
```

### Guided-PIBT 模块核心文件

| 文件路径 | 说明 |
|----------|------|
| `src/driver.cpp` | 主程序入口，处理命令行参数 |
| `src/MAPFPlanner.cpp` | 多智能体路径规划器实现 |
| `src/CompetitionSystem.cpp` | 竞赛系统核心实现 |
| `src/Grid.cpp` | 网格地图处理 |
| `src/States.cpp` | 状态管理 |
| `traffic_mapf/pibt.cpp` | PIBT 算法实现 |
| `traffic_mapf/search.cpp` | 搜索算法实现（改进A*） |
| `traffic_mapf/heuristics.cpp` | 启发式函数 |
| `traffic_mapf/flow.cpp` | 流量成本计算和LNS |
| `traffic_mapf/utils.cpp` | 工具函数 |

### Guided-Lacam2 模块核心文件

| 文件路径 | 说明 |
|----------|------|
| `main.cpp` | 主程序入口 |
| `lacam2/src/` | LaCAM2 算法核心实现 |
| `lacam2/include/` | LaCAM2 头文件 |

---

## 算法基础

### 基础框架

项目主要基于以下算法框架：

1. **PIBT (Priority-based Iterative Best Response)**
   - 基于优先级的迭代最优响应算法
   - 实现文件：`traffic_mapf/pibt.cpp`

2. **LaCAM2**
   - 基于论文："Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding" (IJCAI 2023)
   - 作者：Keisuke Okumura

3. **主论文**
   - "Traffic Flow Optimisation for Lifelong Multi-Agent Path Finding" (AAAI 2024)
   - 链接：https://arxiv.org/abs/2308.11234

### PIBT 与 Guided-PIBT 的关系

```
标准 PIBT (基础层)
    ↓
添加引导路径系统 ← 创新点
    ↓
Guided-PIBT (完整算法)
    ↓
可选 LNS 优化 ← 性能提升
```

**核心差异**：

| 特性 | 标准 PIBT | Guided-PIBT |
|------|-----------|-------------|
| 启发式 | Manhattan 距离 | 到引导路径的距离 |
| 全局优化 | 无 | 有（流量优化） |
| 预计算 | 无 | 有（引导路径） |
| LNS 支持 | 无 | 有（持续优化） |
| Focal Search | 无 | 可选 |

---

## 设计模式

### 1. 策略模式 (Strategy Pattern)

**应用场景**：优先级队列的比较器策略

```cpp
// 不同的比较器定义不同的优先级策略
struct re_f {  // F值优先
    inline bool operator()(const s_node& lhs, const s_node& rhs) const {
        return lhs.get_f() < rhs.get_f();
    }
};

struct re_of {  // 反向流优先
    inline bool operator()(const s_node& lhs, const s_node& rhs) const {
        if (lhs.get_op_flow() == rhs.get_op_flow())
            return lhs.get_f() < rhs.get_f();
        return lhs.get_op_flow() < rhs.get_op_flow();
    }
};

// 使用
template <class s_node, class Comparator, class QType>
class pqueue {
    Comparator* cmp_;  // 策略对象
};
```

### 2. 模板方法模式 (Template Method Pattern)

**应用场景**：A*搜索算法骨架

- **固定步骤**：初始化、主搜索循环、路径重构
- **可变部分**：代价计算（通过OBJECTIVE宏控制）、Focal Search（通过宏开关）

### 3. 工厂模式 - 对象池实现

**应用场景**：搜索节点的创建和管理

```cpp
class MemoryPool {
    s_node* generate_node(int id, int g, int h, ...) {
        // 使用预分配内存"构造"节点
        nodes[id].label = label;
        nodes[id].g = g;
        // ...
        return &(nodes[id]);
    }

    void reset() {
        label++;  // O(1)批量清理
    }
};
```

### 4. 观察者模式 (Observer Pattern)

**应用场景**：轨迹更新的级联传播

- **主题**：轨迹添加/移除
- **观察者**：流量统计、占用表、全局指标

### 5. 组合模式 (Composite Pattern)

**应用场景**：轨迹的层级管理

```
TrajLNS (全局)
    ├── trajs[i] (智能体层)
    │   └── positions[j] (位置层)
    ├── flow (流量模型)
    └── occupations (占用表)
```

---

## 架构组织

### A* 搜索的组织

#### 节点结构

```cpp
struct s_node {
    int id;                  // 位置ID（唯一标识）
    int g, h;                // 实际代价和启发式
    int op_flow;             // 反向流量
    int all_vertex_flow;     // 总顶点冲突
    bool closed;             // 是否已扩展
    s_node* parent;          // 父节点
};
```

#### 开放列表和关闭列表

- **开放列表**：模板化优先级队列，支持decrease_key
- **关闭列表**：通过节点的closed标志位和标签机制实现

#### 启发式函数集成

1. **基础启发式表**：Dijkstra预计算
2. **流量感知启发式**：考虑流量代价的A*搜索
3. **距离-路径启发式**：到引导路径的距离

### 路径规划模块组织

#### 规划器接口

```cpp
class MAPFPlanner {
    virtual void initialize(int preprocess_time_limit);
    virtual void plan(int time_limit, std::vector<Action>& plan);

    std::vector<int> decision;         // 当前时间步的决策
    std::vector<State> prev_states;    // 当前状态
    std::vector<State> next_states;    // 下一状态
    std::vector<double> p;             // 优先级权重
    TrajLNS trajLNS;                   // LNS模块
};
```

#### 主规划流程

1. **状态重置**：清理上一时间步的决策
2. **启发式初始化**：按需计算启发式表
3. **轨迹引导系统**：
   - 处理任务变更（移除旧轨迹）
   - LNS改进（破坏-优化循环）
   - 距离表更新
4. **按优先级排序智能体**
5. **因果PIBT规划**：带回溯机制的优先级规划
6. **生成动作**
7. **状态更新**

#### 智能体管理架构

**因果PIBT算法**：

1. **优先级机制**：按优先级顺序处理智能体
2. **回溯机制**：高优先级智能体抢占时，递归地重新规划低优先级智能体
3. **多源启发式**：支持流量引导、距离-路径表、预计算启发式

---

## 算法配置示例

### 典型配置

| 配置名 | 说明 |
|--------|------|
| **GP-R100-Re10-F2** | 引导路径 + 100%智能体参与LNS + 10次迭代 + Focal权重2.0 |
| **GP-R100** | 引导路径 + 无LNS + 无Focal |
| **GP-R100-F1.2** | 引导路径 + 无LNS + Focal权重1.2 |
| **SP-R100** | 简单路径引导（无LNS，无Focal） |
| **THv-R100** | 仅交通流启发式（无完整引导路径） |

### CMake配置示例

```bash
# GP-R100-Re10-F2
cmake -B guided-pibt-build ./guided-pibt \
  -DGUIDANCE=ON \
  -DGUIDANCE_LNS=10 \
  -DFLOW_GUIDANCE=OFF \
  -DINIT_PP=ON \
  -DRELAX=100 \
  -DOBJECTIVE=1 \
  -DFOCAL_SEARCH=2 \
  -DCMAKE_BUILD_TYPE=RELEASE

# 编译
make -C guided-pibt-build

# 运行
./guided-pibt-build/lifelong \
  --inputFile guided-pibt/benchmark-lifelong/Paris_1_256_0_10000.json \
  --planTimeLimit 10 \
  --output output.json \
  -l event_log.txt
```

---

## 架构优势

### 1. 模块化设计

- MAPFPlanner：高层协调
- TrajLNS：轨迹优化
- A*搜索：单智能体路径规划
- PIBT：在线冲突解决

### 2. 可扩展性

- 策略模式支持新的比较器和启发式
- 模板方法支持新的A*变体
- 虚函数接口支持新的规划算法

### 3. 性能优化

- 内存池避免动态分配
- 预计算的邻居和启发式
- 标签机制实现O(1)批量清理

### 4. 灵活性

- 编译时宏控制（FOCAL_SEARCH, OBJECTIVE）
- 运行时策略选择（LNS邻域选择方法）
- 多种引导机制的组合

---

## 设计哲学

1. **层次抽象**：从环境模型→搜索算法→规划系统，逐层封装
2. **关注点分离**：搜索算法与代价函数分离、数据结构与算法分离
3. **性能-正确性平衡**：使用标签机制、内存池等优化，同时保持代码清晰
4. **增量计算**：启发式、流量模型等按需计算，避免不必要的开销

---

## 总结

**Guided-PIBT** 是对 **PIBT** 的重大扩展，通过添加**引导路径系统**和**交通流优化**，将一个简单的优先级算法转变为适合终身 MAPF 的高性能系统。

### 核心创新

让 PIBT 不仅看目标，还看全局优化的引导路径，从而在保持在线响应速度的同时优化整体交通流量。

### 适用场景

- 仓储机器人路径规划
- 自动化物流系统
- 动态任务分配的智能体协调

这是一个经过深思熟虑的架构，展示了MAPF研究中算法工程的最佳实践，将学术算法的高效性与工程代码的可维护性完美结合。
