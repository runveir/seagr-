# SEAGR — ROS2 Demo

A ROS2 + Gazebo implementation of the ideas from our paper:
**[S.E.A.G.R: A Socially and Emotionally Aware Greeting Robot Framework with Dual-Layer Cultural and Affective Modulation](https://arxiv.org/abs/2607.16341)**

## What it does

Identifies a person via webcam, reads their emotional state, and greets them with a culturally-appropriate gesture — modulating gesture amplitude, hold duration, and adding a happy-nod based on detected emotion.

**Pipeline:** `perception_node` (identity + emotion) → `decision_node` (emotion → amplitude/hold/nod) → `action_node` (joint control in Gazebo)

Identification: face_recognition (dlib ResNet face embeddings) matched against pre-enrolled encodings, via OpenCV webcam capture
Emotion recognition: geometric heuristic over facial landmarks (mouth aspect ratio, eyebrow displacement, normalized by face width) — no trained classifier, real-time on CPU
Middleware: ROS2, custom .msg interfaces (UserDetection, GreetingCmd) for perception → decision → action
Simulation/Actuation: Gazebo, URDF robot model, JointPositionController plugins per joint, bridged via ros_gz_bridge
Language: Python

| Emotion | Amplitude | Hold |
|---|---|---|
| Happy | 1.3x | 0.5s |
| Surprised | 1.15x | 1.0s |
| Neutral | 1.0x | 1.0s |
| Angry | 0.75x | 1.0s |
| Sad | 0.6x | 2.0s |

## Code

- `src/seagr_ros2/perception_node.py` — face recognition + emotion heuristic (facial landmark geometry)
- `src/seagr_ros2/decision_node.py` — emotion → greeting parameters
- `src/seagr_ros2/greeting_logic.py` — per-person greeting selection
- `src/seagr_ros2/action_node.py` — joint ramping in Gazebo
- `src/seagr_ros2_msgs/` — `UserDetection` and `GreetingCmd` message definitions
- `src/urdf/`, `src/launch/` — robot model and launch config

Demo video: `demo_video.mp4`
