# RL Control Module

## 使用说明

该模块为强化学习的模型推理部分。

仿真软件包位于`src/assistant/robot_description`目录中。

### 参数配置

仿真模式下使用的独立配置参数，参见[仿真配置文件](/src/module/rl_control_module/cfg/rl_x1_sim.yaml)。

| 字段                  |  含义                        | 常用配置及详解                                                   |
| -------------------- | ------------------------- | ---------------------------------------------------------------------- |
| freq                 | 控制器控制频率            | 1000                                               |
| use_sim_handles      | 是否在使用仿真            | 仿真中与真机，控制器发布的topic有所不同                      |
| sub_joy_***_name     | 设置状态机切换topic名称   | sub_joy_start_name: /start_control<br />sub_joy_zero_name: /zero_mode<br />sub_joy_stand_name: /stand_mode<br />sub_joy_work_name: /walk_mode<br /> |
| sub_joy_vel_name     | 设置机器人速度指令topic名称| sub_joy_vel_name: /cmd_vel
| sub_imu_data_name    | 设置imu数据topic名称      |                                                              |
| sub_joint_state_name | 设置关节反馈数据topic名称 |                                                              |
| pub_joint_cmd_name   | 设置关节控制指令topic名称 |                                                              |
| joint_conf           | 配置腿部关节参数          | init_state  # RL Controller 的输出 offset，当action = 0时，机器人腿部关节角度 <br />stiffness   # 驱动器 PD 参数 <br />damping |
| walk_step_conf       | 控制器步态配置            | action_scale          # 推理的增量结果，缩放系数<br />decimation            # 采样间隔<br />cycle_time            # 步态周期<br />sw_mode             # 关闭原地踏步<br />cmd_threshold     |
| obs_scales           | 观测值/反馈值，缩放系数   | lin_vel        # 控制命令输入<br />ang_vel     # imu角速度<br />dof_pos     # 关节角误差<br />dof_vel      # 关节角速度<br />quat          #  imu 姿态角 |
| onnx_conf            | onnx配置                  | policy_file              # onnx模型文件位置 <br />actions_size          # 输出值维度 <br />observations_size # 观测值维度 <br />num_hist               # 观测值buffer尺寸 <br />observations_clip  # 观测值限制 <br />actions_clip           # 输出值限制 |

### Channel handle

根据Cfg yaml parse: 中设置的topic名称，生成相应的订阅和发布器。

### State Machine

![rl_control_img](rl_control_img.png)

启动 rl_control_module 时，会进入`IDLE`状态。仿真模式下直接进入 `ZERO` 状态。

### ROS2 Topic 相关

#### Subcribe

`/start_control`、`/zero_mode`、`/stand_mode`、`/walk_mode`：状态机切换命令

`/cmd_vel`: 机器人运动指令

`/imu/data`:imu 数据，频率1000hz

`/jonit_states`: 关节数据，频率1000hz

#### Publish

`/effort_controller/commands`:仿真环境下的控制指令，频率1000hz

`/joint_cmd`:真实环境下的控制指令，频率1000hz
