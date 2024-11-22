# RL Control Module

## Usage Instructions

This module serves as the model inference component for reinforcement learning.

The simulation software package is in the `src/assistant/robot_description` directory.

### Parameter Configuration

For independent configuration parameters used in simulation mode, refer to the [simulation configuration file](/src/module/rl_control_module/cfg/rl_x1_sim.yaml).

| Field                   |  Description                         | Common Configurations and Descriptions                                                   |
| -------------------- | ------------------------- | ---------------------------------------------------------------------- |
| freq                  | Controller's control frequency             | 1000                                               |
| use_sim_handles       | Indicates if simulation is in use             | Topics published by the controller differ in simulation and real robot                      |
| sub_joy_***_name      | Sets the topic name for state machine transitions    | sub_joy_start_name: /start_control<br />sub_joy_zero_name: /zero_mode<br />sub_joy_stand_name: /stand_mode<br />sub_joy_work_name: /walk_mode<br /> |
| sub_joy_vel_name      | Sets the topic name for robot velocity commands | sub_joy_vel_name: /cmd_vel
| sub_imu_data_name     | Sets the topic name for IMU data      |                                                              |
| sub_joint_state_name  | Sets the topic name for joint feedback data |                                                              |
| pub_joint_cmd_name    | Sets the topic name for joint control commands |                                                              |
| joint_conf            | Configures leg joint parameters           | init_state  # The output offset of the RL Controller. When action = 0, the robot's leg joint angle <br /> stiffness   # PD parameter of the actuator <br />damping |
| walk_step_conf        | Configures the gait for the controller             | action_scale          # Scaling factor for the incremental results of inference<br />decimation            # Sampling interval<br />cycle_time            # Gait cycle<br />sw_mode             # Disables in-place stepping<br />cmd_threshold     |
| obs_scales            | Scaling factors for observation values/feedback values    | lin_vel        # Control command input

<br />ang_vel     # IMU angular velocity<br />dof_pos     # Joint angle error<br />dof_vel      # Joint angular velocity<br />quat          #  IMU attitude angle |
| onnx_conf             | ONNX configuration                   | policy_file              # Location of the ONNX model file <br />actions_size          # Dimension of output values <br />observations_size # Dimension of observation values <br />num_hist               # Size of the observation value buffer <br />observations_clip  # Limit for observations <br />actions_clip           # Limit for output values |

### Channel handle

Based on the topic names set in the Cfg yaml parse: corresponding subscribers and publishers are generated.

### State Machine

![rl_control_img2](rl_control_img.png)

When starting rl_control_module, it enters the `IDLE` state. In simulation mode, it directly enters the `ZERO` state.

### ROS2 Topic Related

#### Subscribe

`/start_control`, `/zero_mode`, `/stand_mode`, `/walk_mode`: commands for state machine transitions

`/cmd_vel`: commands for robot movement

`/imu/data`: IMU data, with a frequency of 1000 Hz

`/jonit_states`: joint data, with a frequency of 1000 Hz

#### Publish

`/effort_controller/commands`: control commands in the simulation environment, frequency 1000 Hz

`/joint_cmd`: control commands in the real-world environment, frequency 1000 Hz
