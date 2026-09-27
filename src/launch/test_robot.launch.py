#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('seagr_ros2')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'seagr_robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', 'empty.sdf'],
        output='screen'
    )

    # Delay spawn to let Gazebo start
    spawn_entity = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-name', 'seagr_robot',
                    '-x', '0', '-y', '0', '-z', '0.5',
                    '-file', urdf_file
                ],
                output='screen'
            )
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': urdf_content},
            {'use_sim_time': True}
        ]
    )

    # Bridges ROS2 std_msgs/Float64 publishes (from action_node) onto the
    # Gazebo Transport gz.msgs.Double topics the JointPositionController
    # plugins listen on. ']' = ROS -> Gazebo direction only.
    joint_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_cmd_bridge',
        output='screen',
        arguments=[
            '/model/seagr_robot/joint/base_to_right_shoulder/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/right_shoulder_to_elbow/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/base_to_left_shoulder/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/left_shoulder_to_elbow/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/right_shoulder_adduct/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/left_shoulder_adduct/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/seagr_robot/joint/base_to_head/cmd_pos'
            '@std_msgs/msg/Float64]gz.msgs.Double',
        ]
    )

    # SEAGR pipeline nodes — delayed past the robot spawn (6s) so the
    # action_node isn't trying to publish to joint topics that don't
    # exist yet. A few extra seconds of buffer past the spawn delay.
    perception_node = Node(
        package='seagr_ros2',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    decision_node = Node(
        package='seagr_ros2',
        executable='decision_node',
        name='decision_node',
        output='screen'
    )

    action_node = Node(
        package='seagr_ros2',
        executable='action_node',
        name='action_node',
        output='screen'
    )

    pipeline_nodes = TimerAction(
        period=9.0,
        actions=[joint_bridge, perception_node, decision_node, action_node]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        pipeline_nodes,
    ])