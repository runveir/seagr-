#!/usr/bin/env python3
"""
action_node.py — SEAGR Phase 3

Subscribes to GreetingCmd, ramps joints smoothly over ~2s, holds, then relaxes.
Publishes std_msgs/Float64 to each Gazebo joint cmd_pos topic via ros_gz_bridge.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from seagr_ros2_msgs.msg import GreetingCmd

JOINT_TOPICS = {
    "right_shoulder": "/model/seagr_robot/joint/base_to_right_shoulder/cmd_pos",
    "right_elbow": "/model/seagr_robot/joint/right_shoulder_to_elbow/cmd_pos",
    "left_shoulder": "/model/seagr_robot/joint/base_to_left_shoulder/cmd_pos",
    "left_elbow": "/model/seagr_robot/joint/left_shoulder_to_elbow/cmd_pos",
    "right_shoulder_adduct": "/model/seagr_robot/joint/right_shoulder_adduct/cmd_pos",
    "left_shoulder_adduct": "/model/seagr_robot/joint/left_shoulder_adduct/cmd_pos",
    "head_nod": "/model/seagr_robot/joint/base_to_head/cmd_pos",
}

RAMP_STEPS = 20      # increments per ramp
RAMP_PERIOD = 0.1    # seconds between increments (~2s total ramp)


class ActionNode(Node):
    def __init__(self):
        super().__init__('action_node')
        self.publishers_map = {
            joint: self.create_publisher(Float64, topic, 10)
            for joint, topic in JOINT_TOPICS.items()
        }
        self.current_positions = {joint: 0.0 for joint in JOINT_TOPICS}
        self.subscription = self.create_subscription(
            GreetingCmd, 'greeting_cmd', self.on_greeting_cmd, 10)
        self.busy = False
        self.ramp_timer = None
        self.hold_timer = None
        self.ramp_down_timer = None

    def on_greeting_cmd(self, msg: GreetingCmd):
        if self.busy:
            self.get_logger().warn("Already executing a greeting, ignoring.")
            return

        targets = {
            "right_shoulder": msg.right_shoulder_target,
            "right_elbow": msg.right_elbow_target,
            "left_shoulder": msg.left_shoulder_target,
            "left_elbow": msg.left_elbow_target,
            "right_shoulder_adduct": msg.right_shoulder_adduct_target,
            "left_shoulder_adduct": msg.left_shoulder_adduct_target,
            "head_nod": msg.head_nod_target,
        }
        self.execute_gesture(targets, msg.hold_seconds)

    def execute_gesture(self, targets, hold_seconds):
        self.busy = True
        starts = dict(self.current_positions)
        step = {"n": 0}

        def ramp_up():
            step["n"] += 1
            frac = step["n"] / RAMP_STEPS
            for joint, target in targets.items():
                pos = starts[joint] + (target - starts[joint]) * frac
                self.publish_joint(joint, pos)
            if step["n"] >= RAMP_STEPS:
                self.ramp_timer.cancel()
                self.hold_timer = self.create_timer(
                    hold_seconds, lambda: self.start_ramp_down(targets, starts))

        self.ramp_timer = self.create_timer(RAMP_PERIOD, ramp_up)

    def start_ramp_down(self, targets, starts):
        self.hold_timer.cancel()
        step = {"n": 0}

        def ramp_down():
            step["n"] += 1
            frac = step["n"] / RAMP_STEPS
            for joint, target in targets.items():
                pos = target + (starts[joint] - target) * frac
                self.publish_joint(joint, pos)
            if step["n"] >= RAMP_STEPS:
                self.ramp_down_timer.cancel()
                self.busy = False

        self.ramp_down_timer = self.create_timer(RAMP_PERIOD, ramp_down)

    def publish_joint(self, joint, position):
        msg = Float64()
        msg.data = float(position)
        self.publishers_map[joint].publish(msg)
        self.current_positions[joint] = position


def main(args=None):
    rclpy.init(args=args)
    node = ActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()