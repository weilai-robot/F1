# A 零位 → B stand → Guide 自动准备/播放（2026-10-17）

当前三键：A 用 2 秒进入零位并保持，B 用 2 秒进入 stand 并保持；站稳后一次 Guide 授权 4 秒 default_q 过渡、4 秒连续就绪等待和自动播放，不再按 X 或第二次 Guide。反馈中断、姿态/速度不合格会重计时；Back/Start 或 A 返回零位会取消自动播放。

Mac 上通过 YAML 状态图/按键映射/29 关节目标/AMP stand 组合/资产一致性检查、shell/Python 语法检查，以及 clang++ C++17 编译运行的实际就绪门控/按下沿独立检查。完整 ROS/Eigen/gtest 单测已更新，尚未在 Ubuntu 运行。Guide CLI 通过 mock 验证只发送一个等待订阅匹配的 volatile 事件，未启动 ROS 或电机。

物理探针使用训练 MJCF、v11 辨识 armature、饱和关节 PD、理想反馈和当前资产。从已站立状态开始、stand 驻留 3 秒；不模拟 A 零位或 B 到 stand 的物理过程，不运行 C++/ROS/DCU。

| 记录 | 就绪等待 | yaw | 自动启动时刻（准备开始后） | 播放结果 |
| --- | --- | --- | --- | --- |
| `auto_yaw_0.json` | 原配置 3 s | 0 | 7 s | 647/647 |
| `auto_yaw_0p8.json` | 原配置 3 s | 0.8 | 7 s | 647/647 |
| `auto_yaw_minus1p2.json` | 原配置 3 s | -1.2 | 7 s | 第 153 拍锚点保护 |
| `hold4_yaw_0.json` | 候选 4 s | 0 | 8 s | 647/647 |
| `hold4_yaw_0p8.json` | 候选 4 s | 0.8 | 8 s | 647/647 |
| `hold4_yaw_minus1p2.json` | 候选 4 s | -1.2 | 8 s | 647/647 |
| `configured_hold4_yaw_minus1p2.json` | 正式配置 4 s | -1.2 | 8 s | 647/647 |

4 秒候选临时仅覆盖 `dance_ready_hold_s`，JSON 标记 `experimental_ready_hold_override_s: 4.0`。通过后将 4 秒写入正式 YAML；`configured_*.json` 使用该正式配置、不覆盖参数。`wait_only.json` 是旧 3 秒配置、只允许观察 1 秒等待的结果；`configured_wait_only.json` 用当前 4 秒配置进行相同等待检查，确认策略保持 0 拍、没有自动启动。探针的 `ready_wait_timeout_s` 只是离线观察预算，生产代码会继续等待就绪。

锚点保护保持 0.6 m，未放宽倾角、反馈超时或关节限位。本轮结果不能证明任意初始状态都能接管，仍需 Ubuntu 完整构建、真实状态估计接入及吊架验收。
