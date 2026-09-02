#!/bin/bash
# =============================================================================
# F1 大小脑同机部署 — 资源观测与判定脚本
# 用途: 在主控上运行, 采集大脑(导航)/小脑(控制)同机运行时的系统资源数据,
#       结束后输出 PASS/FAIL 判定表, 回答"大脑资源是否够用"。
#
# 用法:
#   ./resource_monitor.sh [时长秒, 默认600] [输出目录, 默认~/f1_resource_logs/时间戳]
#   可选环境变量:
#     WITH_CYCLICTEST=1   需要 root + rt-tests, 同步跑 cyclictest (10万次)
#     ROS_SETUP=/opt/ros/humble/setup.bash   提供 ros2 则自动采集话题频率
#
# 依赖: sysstat (pidstat/mpstat), procps (vmstat), 默认 Ubuntu 22.04 均可装
# =============================================================================
set -u

DUR="${1:-600}"
OUT="${2:-$HOME/f1_resource_logs/$(date +%Y%m%d_%H%M%S)}"
INTERVAL=5            # 系统采样间隔(秒)
mkdir -p "$OUT"

log()  { echo "[$(date +%H:%M:%S)] $*" | tee -a "$OUT/monitor.log"; }
fail() { echo "MISSING" ; }

echo "=============================================="
echo " F1 同机部署资源观测  时长=${DUR}s 输出=$OUT"
echo "=============================================="

# ---------- 0. 工具检查 ----------
MISSING=""
for t in pidstat mpstat vmstat awk; do
  command -v "$t" >/dev/null 2>&1 || MISSING="$MISSING $t"
done
if [ -n "$MISSING" ]; then
  echo "缺少工具:$MISSING  (sudo apt install sysstat procps)"; exit 1
fi
if [ "${WITH_CYCLICTEST:-0}" = "1" ]; then
  command -v cyclictest >/dev/null 2>&1 || { echo "cyclictest 不存在: sudo apt install rt-tests"; exit 1; }
  [ "$(id -u)" = "0" ] || { echo "WITH_CYCLICTEST=1 需要 root"; exit 1; }
fi

# ---------- 1. 环境快照(判定前提, 不满足则后续判定无效) ----------
SNAP="$OUT/system_snapshot.txt"
{
  echo "=== 时间 ===";        date
  echo "=== 内核 ===";        uname -a; cat /sys/kernel/realtime 2>/dev/null || echo "非RT内核!"
  echo "=== isolcpus 隔离 ==="; grep -o 'isolcpus=[^ ]*' /proc/cmdline || echo "!!未配置 isolcpus"
  echo "=== governor ===";    cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
  echo "=== CPU 拓扑 ===";    lscpu -e=CPU,CORE,MAXMHZ,ONLINE 2>/dev/null
  echo "=== 内存/swap ===";   free -m
  echo "=== 实时进程线程审计 (aimrt_main) ==="
  pid=$(pidof aimrt_main 2>/dev/null)
  if [ -n "$pid" ]; then
    ps -T -p "$pid" -o tid,comm,cls,rtprio,psr,pcpu
  else
    echo "aimrt_main 未运行"
  fi
  echo "=== 导航相关进程线程落核审计 (检查是否有线程落在隔离核 CPU4/6) ==="
  for p in $(pgrep -f 'nav2|fast_lio|fastlio|open3d|controller_server|planner_server|livox' 2>/dev/null); do
    ps -T -p "$p" -o pid,tid,comm,psr,pcpu --no-headers 2>/dev/null \
      | awk -v pn="$p" -v nm="$(ps -p $p -o comm= 2>/dev/null)" '{printf "%s(%s) tid=%s %s psr=%s cpu=%s%%\n", pn,nm,$2,$3,$4,$5}'
  done
  echo "=== 网卡 IRQ 分布 ===";  grep -E 'igc|r8152|xhci' /proc/interrupts | head -20
  echo "=== 启动以来 OOM 记录 ==="; dmesg -T 2>/dev/null | grep -ci 'out of memory' || echo "0 (无权限读dmesg时为0)"
} > "$SNAP" 2>&1
log "环境快照 -> $SNAP"

# ---------- 2. 后台采集器 ----------
log "启动后台采集 (间隔 ${INTERVAL}s) ..."

# 2.1 每线程 CPU (含线程落核, 判定大脑各组件实际占用)
#     2026-09-02: 改有限次数 + LC_ALL=C —— 无限时长被 SIGTERM kill 会丢 stdio 缓冲且
#     sysstat 工具的 "Average:" 汇总块只在自然结束时输出(此前隔离核检查恒 NA 的根因)。
#     次数取 DUR/INTERVAL-1, 保证在主循环结束前自然退出并落盘汇总。
N1=$((DUR/INTERVAL - 1)); [ "$N1" -ge 1 ] || N1=1
LC_ALL=C pidstat -t -h -p ALL $INTERVAL $N1 > "$OUT/pidstat_threads.log" 2>&1 &
P1=$!
# 2.2 每进程内存 RSS (FastLIO ikd-tree 增长 / Nav2 内存泄漏)
N6=$((DUR/(INTERVAL*6) - 1)); [ "$N6" -ge 1 ] || N6=1
LC_ALL=C pidstat -r -p ALL $((INTERVAL*6)) $N6 > "$OUT/pidstat_mem.log" 2>&1 &
P2=$!
# 2.3 每核利用率 (验证隔离核空闲 + 非隔离核是否打满)
LC_ALL=C mpstat -P ALL $INTERVAL $N1 > "$OUT/mpstat.log" 2>&1 &
P3=$!
# 2.4 swap 活动 + 运行队列
LC_ALL=C vmstat $INTERVAL $N1 > "$OUT/vmstat.log" 2>&1 &
P4=$!
# 2.5 PSI 压力 (CPU/内存/io, "资源不够"的最直接内核证据)
( while :; do
    ts=$(date +%H:%M:%S)
    cpu=$(grep some /proc/pressure/cpu      2>/dev/null | tr -d '\n')
    mem=$(cat  /proc/pressure/memory        2>/dev/null | tr '\n' ' ')
    io=$( grep some /proc/pressure/io       2>/dev/null | tr -d '\n')
    echo "$ts | cpu_some: $cpu | mem: $mem | io_some: $io"
    sleep $INTERVAL
  done ) > "$OUT/psi.log" 2>&1 &
P5=$!
# 2.6 温度/降频/频率 (排除热降频造成的"假性资源不足")
( while :; do
    ts=$(date +%H:%M:%S)
    tz=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
    fr=$(cat /sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq 2>/dev/null)
    th=$(awk '{s+=$2} END{print s+0}' /sys/devices/system/cpu/cpu*/thermal_throttle/throttle_count 2>/dev/null)
    echo "$ts max_temp=${tz:-NA} cpu4_freq=${fr:-NA} total_throttle=${th:-NA}"
    sleep $INTERVAL
  done ) > "$OUT/thermal.log" 2>&1 &
P6=$!
# 2.7 内存水位
( while :; do
    awk -v t="$(date +%H:%M:%S)" \
      '/MemAvailable/{a=$2} /Slab/{s=$2} END{printf "%s MemAvailable=%.0fMB Slab=%.0fMB\n",t,a/1024,s/1024}' \
      /proc/meminfo
    sleep $INTERVAL
  done ) > "$OUT/mem.log" 2>&1 &
P7=$!

# ---------- 3. 可选: cyclictest ----------
if [ "${WITH_CYCLICTEST:-0}" = "1" ]; then
  # 挂到隔离核 CPU4 上测最坏调度延迟; 与"仅小脑"基线对比
  cyclictest -m -S -p 90 -h 400 -i 1000 -l 100000 -a 4 -t 1 -q > "$OUT/cyclictest.log" 2>&1 &
  P8=$!
  log "cyclictest 运行中 (绑定 CPU4, 优先级90)"
fi

# ---------- 4. 可选: ROS2 话题频率 (大脑功能达标证据) ----------
TOPIC_LIST="/livox/lidar /livox/imu /Odometry /cloud_registered_body /odom /cmd_vel_limiter"
if [ -n "${ROS_SETUP:-}" ] && [ -f "${ROS_SETUP:-}" ]; then
  source "$ROS_SETUP"
  for tp in $TOPIC_LIST; do
    ( timeout 120 ros2 topic hz "$tp" --window 600 > "$OUT/topic_hz_$(echo $tp | tr '/' '_').log" 2>&1 ) &
  done
  log "话题频率采样中 (每个 120s): $TOPIC_LIST"
fi

# ---------- 5. 周期性摘要 (stdout 每60s一行) ----------
START=$SECONDS
NEXT=$((SECONDS+60))
log "观测进行中... Ctrl+C 可提前结束并生成报告"
while [ $((SECONDS-START)) -lt "$DUR" ]; do
  if [ $SECONDS -ge $NEXT ]; then
    cpu_some=$(grep some /proc/pressure/cpu 2>/dev/null | grep -o 'avg60=[0-9.]*')
    memavail=$(awk '/MemAvailable/{printf "%.1f", $2/1048576}' /proc/meminfo)
    si_so=$(vmstat 1 2 | tail -1 | awk '{print "si="$7"/s so="$8"/s"}')
    log "摘要: ${cpu_some:-PSI_NA} MemAvail=${memavail}GB ${si_so}"
    NEXT=$((NEXT+60))
  fi
  sleep 2
done

# ---------- 6. 停止采集 ----------
log "采集结束, 生成判定报告 ..."
for p in ${P1:-} ${P2:-} ${P3:-} ${P4:-} ${P5:-} ${P6:-} ${P7:-} ${P8:-}; do
  kill "$p" 2>/dev/null; wait "$p" 2>/dev/null
done

# ---------- 7. 判定报告 ----------
REPORT="$OUT/report.txt"
{
echo "=============================================================="
echo " F1 同机部署资源判定报告   $(date)"
echo " 观测时长 ${DUR}s | 输出目录 $OUT"
echo "=============================================================="
echo ""
echo " [系统层判定]  (阈值依据 doc/同机部署配置指南.md 第8节)"
echo "----------------------------------------------------------------"
# a) Swap 活动: vmstat si/so 全程必须为 0
swapsio=$(awk 'NR>2 && ($7>0 || $8>0){c++} END{print c+0}' "$OUT/vmstat.log")
[ "$swapsio" = "0" ] && R1="PASS  swap 无换入换出" || R1="FAIL  swap 活动出现 ${swapsio} 次(si/so>0) — 会造成实时缺页抖动"
echo " 1. $R1"
# b) 内存余量 (mem.log 行格式 "HH:MM:SS MemAvailable=NNNNMB Slab=NNNNMB", 2026-09-02 修复:
#    原解析取 $2 整串 "MemAvailable=NNNNMB" 数值强转恒 0 → 永远 FAIL)
minmem=$(awk '{split($2,a,"="); v=a[2]+0; if(v>0 && (min==""||v<min)) min=v} END{printf "%.2f", min+0}' "$OUT/mem.log" 2>/dev/null)
awk -v m="$minmem" 'BEGIN{exit !(m+0>2048)}' && R2="PASS  MemAvailable 最低 ${minmem}MB (>2GiB)" \
                                        || R2="FAIL  MemAvailable 最低 ${minmem}MB (<2GiB) — 内存不足"
echo " 2. $R2"
# c) PSI CPU some avg60 峰值 (有线程排队等CPU)
#    2026-09-02 修复: 限定 cpu_some 段——原 grep 全文 avg60 会混入 mem/io 的值串到 cpu 名下
maxpsi=$(grep -o 'cpu_some: some avg10=[0-9.]* avg60=[0-9.]*' "$OUT/psi.log" 2>/dev/null | grep -o 'avg60=[0-9.]*' | cut -d= -f2 | sort -n | tail -1)
[ -z "$maxpsi" ] && maxpsi=-1
awk -v m="$maxpsi" 'BEGIN{exit !(m+0<=5.0)}' && R3="PASS  PSI cpu some avg60 峰值 ${maxpsi:-NA}% (<=5%)" \
                                          || R3="FAIL  PSI cpu some avg60 峰值 ${maxpsi}% (>5%) — CPU 明显排队, 大脑线程被饿"
echo " 3. $R3"
# d) PSI memory (2026-09-02 修复: 限定 mem 段的 some avg60, 原解析混入 cpu/io)
maxpsimem=$(grep -o 'mem: [^|]*' "$OUT/psi.log" 2>/dev/null | grep -o 'some avg10=[0-9.]* avg60=[0-9.]*' | grep -o 'avg60=[0-9.]*' | cut -d= -f2 | sort -n | tail -1)
[ -z "$maxpsimem" ] && maxpsimem=-1
awk -v m="$maxpsimem" 'BEGIN{exit !(m+0<=0.1)}' && R4="PASS  PSI memory 基本为 0" \
                                            || R4="FAIL  PSI memory avg60 峰值 ${maxpsimem}% — 存在内存回收压力"
echo " 4. $R4"
# e) 隔离核污染: mpstat Average 汇总行 ($1="Average:", $2=CPU号, $3=%usr, $5=%sys)
for cpun in 4 6; do
  u=$(awk -v c="$cpun" '$1=="Average:" && $2==c {printf "%.2f", $3+$5; found=1} END{if(!found) print "NA"}' "$OUT/mpstat.log" 2>/dev/null)
  awk -v v="$u" 'BEGIN{exit !(v=="NA" || v+0<=2.0)}' && echo " 5. PASS  隔离核 CPU$cpun 平均占用 ${u}% (<=2%, 大脑未污染实时核)" \
                                                 || echo " 5. FAIL  隔离核 CPU$cpun 平均占用 ${u}% (>2%) — 实时核被污染! 检查线程affinity"
done
# f) 非隔离核饱和度: 取最忙核的 %usr+%sys 峰值 (mpstat 逐间隔数据行: $2=CPU号)
busiest=$(awk '$2 ~ /^[0-9]+$/ && $2!="4" && $2!="6" {v=$3+$5; if(v>m) {m=v; c=$2}} END{if(c!="") printf "CPU%s %.2f%%", c, m; else print "NA 0"}' "$OUT/mpstat.log" 2>/dev/null)
bn=$(echo $busiest | awk '{print $1}'); bv=$(echo $busiest | awk '{print $2}' | tr -d '%')
awk -v v="$bv" 'BEGIN{exit !(v+0<=85)}' && R6="PASS  最忙非隔离核 ${bn} 峰值 ${bv}%" \
                                     || R6="WARN  最忙非隔离核 ${bn} 峰值 ${bv}% (>85%) — 峰值饱和, 看 PSI 是否排队定夺"
echo " 6. $R6"
# g) 热降频
maxtemp=$(grep -o 'max_temp=[0-9]*' "$OUT/thermal.log" 2>/dev/null | cut -d= -f2 | sort -n | tail -1)
thr0=$(head -1 "$OUT/thermal.log" | grep -o 'total_throttle=[0-9]*' | cut -d= -f2)
thr1=$(tail -1 "$OUT/thermal.log" | grep -o 'total_throttle=[0-9]*' | cut -d= -f2)
awk -v t="$maxtemp" 'BEGIN{exit !(t==""||t+0<90000)}' && R7="PASS  峰值温度 $(( ${maxtemp:-0}/1000 ))°C (<90°C)" \
                                                 || R7="FAIL  峰值温度 $(( maxtemp/1000 ))°C (>=90°C) — 热节流会伪装成资源不足"
echo " 7. $R7"
if [ -n "${thr0:-}" ] && [ -n "${thr1:-}" ] && [ "$thr1" != "$thr0" ]; then
  echo "    !! throttle_count 期间增长 ${thr0} -> ${thr1}, 发生了硬件降频"
fi
# h) OOM
oom=$(dmesg -T 2>/dev/null | grep -c 'Out of memory' || true)
[ "${oom:-0}" = "0" ] && echo " 8. PASS  无 OOM kill 记录" || echo " 8. FAIL  OOM 记录 ${oom} 条 — 内存不够, 有进程被杀"
echo ""
if [ "${WITH_CYCLICTEST:-0}" = "1" ]; then
  echo " [实时性判定] (小脑是否被大脑拖累, 最终否决指标)"
  echo "----------------------------------------------------------------"
  lat=$(grep 'Max Latencies' "$OUT/cyclictest.log" 2>/dev/null | awk '{print $NF}')
  [ -z "$lat" ] && lat=999999
  awk -v l="$lat" 'BEGIN{exit !(l+0<100)}' && echo " 9. PASS  cyclictest 最大延迟 ${lat}us (<100us)" \
                                       || echo " 9. FAIL  cyclictest 最大延迟 ${lat}us (>=100us) — 大脑负载破坏实时性, 不准入"
fi
echo ""
echo " [大脑功能层判定] (人工核对, 数据来源见各 log)"
echo "----------------------------------------------------------------"
echo " 10. 话题频率:  查看 topic_hz_*.log, 要求实测均值>=标称95%:"
echo "     /livox/lidar 与 /Odometry >=9.5Hz, /odom 持续更新"
echo " 11. FastLIO 帧耗时: runtime_pos_log 打开后, 终端 '[ mapping ] ave total'"
echo "     要求 <70ms 且随建图时间无持续增长趋势"
echo " 12. Nav2: 运行期日志无 'missed its requested rate'/'took too long' WARN"
echo " 13. Open3D 定位: 'time_this_loc' 稳定且 <100ms 周期预算"
echo ""
echo " 详细数据: pidstat_threads.log / mpstat.log / psi.log / vmstat.log / mem.log / thermal.log"
echo "=============================================================="
} | tee "$REPORT"

log "完成. 报告: $REPORT"
