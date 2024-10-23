# Joy Stick Module

## About the Joystick

[Logitech F710](https://www.logitechg.com/zh-cn/products/gamepads/f710-wireless-gamepad.940-000172.html?sp=1&searchclick=logi) joystick is used in the project.

## Joystick Settings

To view the joystick mapping settings, please refer to [jstest-gtk](https://github.com/Grumbel/jstest-gtk).

```bash
jstest-gtk
```

The default mapping is usually as shown below, with red indicating buttons and green indicating joysticks.

![joy_map](joy_map.jpg "joy_map")

| Button/Joystick      | topic/srv      | type                     | Function                                                                                           |
| -------------------- | -------------- | ------------------------ | -------------------------------------------------------------------------------------------------- |
| Button 7             | /start_control | std_msgs/msg/Float32     | Press to publish a topic for controlling the state machine                                         |
| Button 1             | /zero_mode     | std_msgs/msg/Float32     | Press to publish a topic for controlling the state machine                                         |
| Button 0             | /stand_mode    | std_msgs/msg/Float32     | Press to publish a topic for controlling the state machine                                         |
| Button 2             | /walk_mode     | std_msgs/msg/Float32     | Press to publish a topic for controlling the state machine                                         |
| Button 3             | /reset_world   | std_srvs/srv/Empty       | Press to send an srv request for resetting models in gazebo                                        |
| Button 4 + Joystick  | /cmd_vel       | geometry_msgs/msg/Twist  | Publishing robot movement commands, left joystick for translation and right joystick for rotation  |

![joy_teleop](joy_teleop.gif "joy_teleop")
