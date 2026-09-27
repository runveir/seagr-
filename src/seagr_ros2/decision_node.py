#!/usr/bin/env python3
"""
decision_node.py — SEAGR Phase 2

Subscribes to UserDetection, calls greeting_logic, publishes GreetingCmd.
"""

import rclpy
from rclpy.node import Node
from seagr_ros2_msgs.msg import UserDetection, GreetingCmd
from .greeting_logic import greeting_logic

# Joint targets per greeting type — extend as needed.
# right/left_shoulder_adduct: SMALL swing of the whole arm toward center —
# in a real namaste the elbows stay apart, only the forearms angle in.
# Too much adduction drags the elbows together too (not what we want, per
# reference photo), so this is intentionally modest — the elbow bend does
# the actual work of bringing hands together.
GREETING_JOINTS = {
    "namaste": {
        "right_shoulder": 1.0,
        "right_elbow": -1.9,
        "left_shoulder": 1.0,
        "left_elbow": -1.9,
        "right_shoulder_adduct": 0.6,
        "left_shoulder_adduct": 0.6,
    },
    "wave": {
        "right_shoulder": 1.0,
        "right_elbow": 0.6,
        "left_shoulder": 0.0,
        "left_elbow": 0.0,
        "right_shoulder_adduct": 0.0,
        "left_shoulder_adduct": 0.0,
    },
}

# How long the robot holds the pose before releasing, keyed by detected
# emotion. Happy = brief and energetic, sad = lingering/gentle. Falls
# back to 1.0s for emotions not listed (surprised, angry).
HOLD_SECONDS_BY_EMOTION = {
    "happy": 0.5,
    "neutral": 1.0,
    "sad": 2.0,
}
DEFAULT_HOLD_SECONDS = 1.0

# Head nod angle used only when emotion is happy — small negative pitch,
# ramped/held/released the same way as every other joint (see action_node).
NOD_ANGLE = -0.3

# Scales greeting_logic's base amplitude up/down by detected emotion.
# happy/surprised -> bigger, more energetic gesture. sad/angry -> smaller,
# more subdued gesture, since a full-amplitude greeting reads as tone-deaf
# toward someone who looks upset.
EMOTION_AMPLITUDE_MODULATION = {
    "happy": 1.3,
    "surprised": 1.15,
    "neutral": 1.0,
    "sad": 0.6,
    "angry": 0.75,
}
AMPLITUDE_MIN = 0.3
AMPLITUDE_MAX = 1.5

# action_node's gesture takes ~6s (2s ramp up + 2s hold + 2s ramp down —
# see RAMP_STEPS/RAMP_PERIOD/HOLD_SECONDS in action_node.py). We add a
# further 10s pause after that before looking for facial changes again,
# so the robot isn't re-triggering mid-gesture or immediately after.
GESTURE_DURATION_ESTIMATE = 6.0
POST_GESTURE_COOLDOWN = 10.0
TOTAL_COOLDOWN = GESTURE_DURATION_ESTIMATE + POST_GESTURE_COOLDOWN


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.subscription = self.create_subscription(
            UserDetection, 'user_detection', self.on_detection, 10)
        self.publisher = self.create_publisher(GreetingCmd, 'greeting_cmd', 10)
        self.last_greeting_time = None  # seconds, from self.get_clock()

    def on_detection(self, msg: UserDetection):
        if msg.badge_id == "unknown":
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_greeting_time is not None:
            elapsed = now - self.last_greeting_time
            if elapsed < TOTAL_COOLDOWN:
                return  # still mid-gesture or in post-gesture cooldown
        self.last_greeting_time = now

        greeting_type, base_amplitude = greeting_logic(
            badge_id=msg.badge_id, emotion=msg.emotion)

        modulation = EMOTION_AMPLITUDE_MODULATION.get(msg.emotion, 1.0)
        amplitude = base_amplitude * modulation
        amplitude = max(AMPLITUDE_MIN, min(AMPLITUDE_MAX, amplitude))

        joints = GREETING_JOINTS.get(greeting_type, GREETING_JOINTS["namaste"])

        cmd = GreetingCmd()
        cmd.right_shoulder_target = joints["right_shoulder"] * amplitude
        cmd.right_elbow_target = joints["right_elbow"] * amplitude
        cmd.left_shoulder_target = joints["left_shoulder"] * amplitude
        cmd.left_elbow_target = joints["left_elbow"] * amplitude
        cmd.right_shoulder_adduct_target = joints["right_shoulder_adduct"] * amplitude
        cmd.left_shoulder_adduct_target = joints["left_shoulder_adduct"] * amplitude
        cmd.head_nod_target = NOD_ANGLE if msg.emotion == "happy" else 0.0
        cmd.hold_seconds = HOLD_SECONDS_BY_EMOTION.get(msg.emotion, DEFAULT_HOLD_SECONDS)
        cmd.greeting_type = greeting_type
        cmd.amplitude = float(amplitude)

        self.publisher.publish(cmd)
        self.get_logger().info(
            f"{msg.badge_id} ({msg.emotion}) -> {greeting_type} "
            f"(base={base_amplitude:.2f} x{modulation} = {amplitude:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()