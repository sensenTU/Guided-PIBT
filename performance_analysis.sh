#!/bin/bash
# 多地图性能对比分析脚本

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   多地图性能对比分析${NC}"
echo -e "${BLUE}========================================${NC}"

PROJECT_ROOT="/home/sentu/mxw/Guided-PIBT"

cd "$PROJECT_ROOT"

# 测试地图列表（代表不同类型）
declare -a MAPS=(
    "guided-pibt/benchmark-lifelong/den520d_0_12000.json:Denver 520d (12000 agents)"
    "guided-pibt/benchmark-lifelong/ost003d_0_2000.json:Oslo 003d (2000 agents)"
    "guided-pibt/benchmark-lifelong/Paris_1_256_0_2000.json:Paris 256 (2000 agents)"
    "guided-pibt/benchmark-lifelong/room-64-64-8_0_1000.json:Room 64x64 (1000 agents)"
    "guided-pibt/benchmark-lifelong/sortation_small_0_800.json:sortation 64x64 (800 agents)"
    ""
)

echo ""
echo -e "${YELLOW}测试配置:${NC}"
echo "  测试模式: 完整运行每个地图"
echo "  地图数量: ${#MAPS[@]}"
echo ""

# 存储结果
declare -A BASELINE_RESULTS
declare -A BPR_RESULTS

echo -e "${YELLOW}==================== Baseline 测试 ====================${NC}"

for map_info in "${MAPS[@]}"; do
    IFS=':' read -r map_file map_name <<< "$map_info"

    # 跳过空行
    if [ -z "$map_name" ]; then
        continue
    fi

    if [ ! -f "$map_file" ]; then
        echo -e "${RED}跳过 $map_name (文件不存在)${NC}"
        continue
    fi

    echo ""
    echo -e "${BLUE}测试 Baseline: $map_name${NC}"

    cd build-baseline
    output=$(bash -c "time ./lifelong --inputFile \"../$map_file\" 2>&1" 2>&1)
    timesteps=$(echo "$output" | grep -oP "timestep=\K[0-9]+" | tail -1)
    cd ..

    if [ -n "$timesteps" ] && [ "$timesteps" -gt 0 ]; then
        BASELINE_RESULTS["$map_name"]=$timesteps
        echo -e "  完成的任务数: ${GREEN}$timesteps${NC}"
    else
        echo -e "  ${RED}测试失败${NC}"
        BASELINE_RESULTS["$map_name"]=0
    fi
done

echo ""
echo -e "${YELLOW}==================== BPR 测试 ====================${NC}"

for map_info in "${MAPS[@]}"; do
    IFS=':' read -r map_file map_name <<< "$map_info"

    # 跳过空行
    if [ -z "$map_name" ]; then
        continue
    fi

    if [ ! -f "$map_file" ]; then
        continue
    fi

    echo ""
    echo -e "${BLUE}测试 BPR: $map_name${NC}"

    cd build-bpr
    output=$(bash -c "time ./lifelong --inputFile \"../$map_file\" 2>&1" 2>&1)
    timesteps=$(echo "$output" | grep -oP "timestep=\K[0-9]+" | tail -1)
    cd ..

    if [ -n "$timesteps" ] && [ "$timesteps" -gt 0 ]; then
        BPR_RESULTS["$map_name"]=$timesteps
        echo -e "  完成的任务数: ${GREEN}$timesteps${NC}"
    else
        echo -e "  ${RED}测试失败${NC}"
        BPR_RESULTS["$map_name"]=0
    fi
done

# 生成对比报告
echo ""
echo -e "${YELLOW}==================== 性能对比报告 ====================${NC}"

echo ""
printf "%-35s | %12s | %12s | %12s\n" "地图" "Baseline" "BPR" "性能比率"
echo "--------------------------------------------------------------------------------"

for map_info in "${MAPS[@]}"; do
    IFS=':' read -r map_file map_name <<< "$map_info"

    # 跳过空行
    if [ -z "$map_name" ]; then
        continue
    fi

    base=${BASELINE_RESULTS[$map_name]}
    bpr=${BPR_RESULTS[$map_name]}

    if [ -n "$base" ] && [ -n "$bpr" ] && [ "$base" -gt 0 ] && [ "$bpr" -gt 0 ]; then
        ratio=$(echo "scale=2; $bpr * 100 / $base" | bc)
        ratio_str="$ratio%"

        # 判断性能
        if (( $(echo "$ratio > 95" | bc -l) )); then
            status="${GREEN}✓${NC}"
        elif (( $(echo "$ratio > 85" | bc -l) )); then
            status="${YELLOW}~${NC}"
        else
            status="${RED}✗${NC}"
        fi

        printf "%-35s | %12s | %12s | %12s %s\n" "$map_name" "$base" "$bpr" "$ratio_str" "$status"
    else
        printf "%-35s | %12s | %12s | %12s\n" "$map_name" "N/A" "N/A" "N/A"
    fi
done

echo "--------------------------------------------------------------------------------"
echo ""
echo -e "${BLUE}说明:${NC}"
echo "  ✓  性能比率 > 95%  (优秀)"
echo "  ~  性能比率 85-95% (可接受)"
echo "  ✗  性能比率 < 85%  (需要优化)"
echo ""
echo -e "${BLUE}性能指标:${NC}  完成的任务数 (timesteps)"
echo ""

# 计算平均性能
total_base=0
total_bpr=0
count=0

for map_info in "${MAPS[@]}"; do
    IFS=':' read -r map_file map_name <<< "$map_info"

    # 跳过空行
    if [ -z "$map_name" ]; then
        continue
    fi

    base=${BASELINE_RESULTS[$map_name]}
    bpr=${BPR_RESULTS[$map_name]}

    if [ -n "$base" ] && [ "$bpr" ] && [ "$base" -gt 0 ] && [ "$bpr" -gt 0 ]; then
        total_base=$((total_base + base))
        total_bpr=$((total_bpr + bpr))
        count=$((count + 1))
    fi
done

if [ $count -gt 0 ]; then
    avg_ratio=$(echo "scale=2; $total_bpr * 100 / $total_base" | bc)

    echo -e "${BLUE}总任务数:${NC} Baseline=$total_base, BPR=$total_bpr"
    echo -e "${BLUE}平均性能比率: ${avg_ratio}%${NC}"
    echo ""

    if (( $(echo "$avg_ratio > 95" | bc -l) )); then
        echo -e "${GREEN}✓ BPR 性能优秀!${NC}"
    elif (( $(echo "$avg_ratio > 85" | bc -l) )); then
        echo -e "${YELLOW}~ BPR 性能可接受${NC}"
    else
        echo -e "${RED}✗ BPR 性能需要优化${NC}"
    fi
fi

echo ""
exit 0
