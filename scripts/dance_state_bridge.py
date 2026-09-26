#!/usr/bin/env python3
"""Bridge verified odometry and TF into X1 BeyondMimic dance observations.

Requires a live odom -> base_link -> lumbar_pitch_link TF chain. The odometry
twist must be expressed in base_link; no frame guessing or zero filling occurs.
This script runs only on the Linux ROS 2 host.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def age_seconds(now: Time, stamp) -> float:
    return (now - Time.from_msg(stamp)).nanoseconds * 1e-9


class DanceStateBridge(Node):
    def __init__(self):
        super().__init__("dance_state_bridge")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("anchor_frame", "lumbar_pitch_link")
        self.declare_parameter("max_age_s", 0.06)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.anchor_frame = self.get_parameter("anchor_frame").value
        self.max_age = float(self.get_parameter("max_age_s").value)
        if self.max_age <= 0 or self.max_age > 0.08:
            raise ValueError("max_age_s must be in (0, 0.08]")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.anchor_pub = self.create_publisher(TransformStamped, "/dance/anchor_pose", 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "/dance/base_velocity", 10)
        self.last_odom = None
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(0.02, self.publish_state)  # 50 Hz, matching the policy

    def on_odom(self, message: Odometry) -> None:
        if message.header.frame_id != self.odom_frame or message.child_frame_id != self.base_frame:
            self.last_odom = None
            return
        values = (message.twist.twist.linear.x, message.twist.twist.linear.y,
                  message.twist.twist.linear.z)
        if not all(math.isfinite(value) for value in values):
            self.last_odom = None
            return
        self.last_odom = message

    def publish_state(self) -> None:
        if self.last_odom is None:
            return
        now = self.get_clock().now()
        odom_age = age_seconds(now, self.last_odom.header.stamp)
        if odom_age < -0.02 or odom_age > self.max_age:
            return
        try:
            anchor = self.buffer.lookup_transform(
                self.odom_frame, self.anchor_frame, Time(), timeout=Duration(seconds=0.005)
            )
        except TransformException:
            return
        anchor_age = age_seconds(now, anchor.header.stamp)
        if anchor_age < -0.02 or anchor_age > self.max_age:
            return
        if abs(anchor_age - odom_age) > 0.05:
            return
        transform = anchor.transform
        numbers = (transform.translation.x, transform.translation.y, transform.translation.z,
                   transform.rotation.x, transform.rotation.y, transform.rotation.z,
                   transform.rotation.w)
        if not all(math.isfinite(value) for value in numbers):
            return
        norm = math.sqrt(sum(value * value for value in numbers[3:]))
        if norm < 0.9 or norm > 1.1:
            return
        velocity = TwistStamped()
        velocity.header.stamp = self.last_odom.header.stamp
        velocity.header.frame_id = self.base_frame
        velocity.twist.linear = self.last_odom.twist.twist.linear
        velocity.twist.angular = self.last_odom.twist.twist.angular
        self.anchor_pub.publish(anchor)
        self.velocity_pub.publish(velocity)


def main() -> None:
    rclpy.init()
    node = DanceStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
