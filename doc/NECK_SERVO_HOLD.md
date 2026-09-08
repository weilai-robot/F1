# 颈部双舵机"保持不动"集成说明

> 最终实现以 Humanoid_motion main（PR #2, merge `6d02369`）为准：
> `neck_hold.py` 简化为**仅扭矩使能**；`run.sh`/`run_with_recording.sh` 启动时使能、**退出时释放**。

## 背景与硬件链路

F1 颈部装有两个 **HTD-35H 串行总线舵机**，控制板（STM32F407 扩展板）通过 CH340 USB 串口接在**小脑 Intel 主机**上：

```
小脑 minipc USB ──CH340(/dev/ttyUSB0, 115200)── STM32F407 扩展板 ── 总线 ── 2× HTD35H
```

| 舵机 | 安装位置 | 功能 | 限位 | 中位 |
|---|---|---|---|---|
| ID 1 | 脖子内 | 头部左右转动 | 0–1000 | 500 |
| ID 2 | 脖子上方 | 点头（前后） | 0–**650** | 500 |

协议与 API 详见 vendor 仓库：https://gitee.com/zangmy04/servo_control_new
（`servo_analysis.md` 协议分析 + `hostcomputer/使用说明.md`）

**需求**：小脑 / 大脑+小脑运行期间，这两个舵机不参与控制，但必须保持不动。总线舵机上电默认扭矩释放（软），必须显式发送**扭矩锁定（SubCmd `0x0B`）**，舵机才会在当前位置主动保持、受扰自动回位。

## 实际实现（启动脚本钩子，已在真机验证）

真机两种运行场景（仅小脑 / 大脑+小脑）都必经小脑启动脚本：

```
run.sh / run_with_recording.sh
  ├─ 启动时: [ -e /dev/ttyUSB0 ] → timeout 10 neck_hold.py（仅使能, SubCmd 0x0B）
  │           ├─ 成功 → 打印"扭矩使能完成"
  │           └─ 失败 → WARN 放行（NECK_SERVO_REQUIRED=1 时中止启动）
  ├─ aimrt_main 运行（头部保持锁定）
  └─ 退出时: trap EXIT/INT/TERM/HUP → release_neck() 释放扭矩(0x0C)
```

### 文件（均在 motion_control `install/linux/bin/`，随 CMake install 进 build/，随 pack 上真机）

| 文件 | 说明 |
|---|---|
| `htd35h_controller.py` | vendor 控制库**原样拷贝**（处理 DTR/RTS、1.5s STM32 开机等待、接收线程） |
| `neck_hold.py` | **仅扭矩使能**：不回中、不读位置；退出码 0/2(串口失败)/3(缺依赖) |
| `run.sh` / `run_with_recording.sh` | 启动使能 + 退出释放 trap + 退出码透传 |

### 环境变量

| 变量 | 默认 | 说明 |
|---|---|---|
| `NECK_SERVO_PORT` | `/dev/ttyUSB0` | 串口设备（多 CH340 时建议 udev 别名） |
| `NECK_SERVO_REQUIRED` | `0` | `1` = 使能失败时中止小脑启动 |

## ⚠️ 雷达外参注意（Mid-360 装在头部）

当前语义为**原位使能 + 退出释放**：小脑退出后头部可自由转动，下次启动锁在"头部自然停靠位"。Mid-360 在头上 ⇒ 雷达外参 = 头部实际停靠位。**使用约定**：
- 建图与定位必须保持同一头部姿态（建议每次启动前把头部摆到标定位，或确认停靠位稳定）
- 若后续发现停靠位漂移影响定位，可恢复 `set_position` 回中逻辑（初始版 `4a18f25` 有 home 模式实现可参考），并同步校准雷达外参

## 真机部署（一次性配置）

1. **依赖**：`sudo apt install python3-serial`
2. **权限**：`sudo usermod -aG dialout robot` 后**重新登录**（否则 Permission denied）
3. **删 brltty**（Ubuntu 22.04 会抢占 CH340，症状：lsusb 有设备但无 ttyUSB 节点，dmesg 见 `interface 0 claimed by ch341 while 'brltty' sets config`）：
   ```bash
   sudo apt remove brltty   # 然后拔插 USB
   ```
4. **舵机供电**：HTD35H **不吃 USB 电**，需独立供电轨（6–8.4V）；只插 USB 时 STM32 正常回显调试日志但舵机全部 `timeout`——这是"串口通、总线不通"的典型症状

## 验证清单（真机已通过）

1. `python3 neck_hold.py` → 打印 `扭矩使能完成, 舵机: [1, 2]`；手推头部有回位力
2. `bash run.sh` → 启动钩子打印使能成功；Ctrl-C 退出后打印 `颈部舵机已下使能`（头变软）
3. 无串口机器（仿真）→ `跳过使能`，启动不受影响

## 已知注意事项

- **串口独占**：同一串口同时只能一个进程打开
- **每次打开串口重启 STM32**（CH340 DTR 边沿，库内等 1.5s）——释放钩子重开串口会使退出慢约 3s 且打印 STM32 启动横幅，属预期
- **未来大脑控头**：直接发 `set_position` 覆盖使能位；长时间释放用 `enable_torque(id, False)`
- 方案 B（C++ 集成进 `DcuDriverModule::Initialize()`）为后续把颈部纳入正式驱动框架时的迁移方向
