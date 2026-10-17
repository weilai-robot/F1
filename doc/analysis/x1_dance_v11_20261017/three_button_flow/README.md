# 历史 A → X → Guide 流程复核（2026-10-17）

本目录保留旧流程的原始记录。当前已按用户要求改为 A 零位 → B stand → Guide（准备位 → 自动跳舞），不再需要 X 或准备后的第二次 Guide。当前复核见相邻的 `guide_auto_start/`。

正常操作为 A → X → Guide。A 自动执行 2 秒零位过渡，实测到位后执行 2 秒 stand 过渡；stand 到位才接受 X。X 执行 4 秒 default_q 过渡，准备条件连续满足 3 秒后提示 Guide。等待不自动启动，Guide 提前按住需要释放重按。Back/Start 可中断任何阶段。

Mac 检查结果：

- 使用 clang++、C++17、`-Wall -Wextra -Werror` 编译并运行实际 `dance_start_gate.h` 和 `button_trigger.h` 的独立检查：连续 3 秒、早按/按住 Guide、释放重按、失效/超过 80 ms 检查间隔及退出复位均通过。
- YAML 状态图、仅三种启动按钮、所有阶段 keep/idle 退出、内部零位完成事件、29 关节目标与限位、default_q 二进制一致性、资产 SHA256 检查通过。stand 目标与增益等于未修改 AMP 配置的 `pd_zero + pd_stand` 合成结果。
- Python AST、shell 语法检查通过。新增完整 Ubuntu C++ 回归测试，尚未执行。

以下数值探针使用训练 MJCF、辨识 armature、饱和关节 PD、理想反馈和当前部署资产。均从已站立状态开始，stand 驻留 3 秒；不涵盖冷启动零位到 stand 的物理过程，不运行 C++/ROS/DCU。

| 文件 | 准备过渡后保持 | yaw | 结果 | 连续就绪时长 |
| --- | --- | --- | --- | --- |
| `early_guide.json` | 1 s | 0 | Guide 拒绝，策略 0 拍 | 1 s |
| `ready_guide.json` | 4 s | 0 | 完成 647 拍 | 4 s |
| `ready_yaw_guide.json` | 4 s | 0.8 | 完成 647 拍 | 4 s |

尚需 Ubuntu 完整构建/C++ 单测、真实反馈与估计接入、零位 → stand 吊架联调及舞蹈动态接管验收。
