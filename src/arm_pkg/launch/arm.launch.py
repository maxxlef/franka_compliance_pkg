#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.


import os

from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, Shutdown, TimerAction


def generate_launch_description():
    #==================== Launch Arguments ====================
    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    interactive_marker_parameter_name = 'interactive_marker'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name, default='true')
    interactive_marker = LaunchConfiguration(interactive_marker_parameter_name, default='true')

    #====================== Xacro Files ======================
    franka_xacro_file = PathJoinSubstitution([
        FindPackageShare('franka_description'),
        'robots',
        'panda_arm.urdf.xacro'
    ])
    #====================== Controllers ======================
    franka_controllers = PathJoinSubstitution([
            FindPackageShare('arm_pkg'),
            'config',
            'franka_controllers.yaml'
    ])
    
    #===================== Config Files ======================
    config_parameters_file = PathJoinSubstitution([
        FindPackageShare('arm_pkg'),
        'config',
        'params.yaml'
    ])

    #=================== Robot Descriptions ==================
    robot_description_arm = Command(
        [FindExecutable(name='xacro'),
         ' ', franka_xacro_file, 
         ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, 
         ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])
    
    #============== Declare Launch Arguments ==============
    robot_ip_arg = DeclareLaunchArgument(
        robot_ip_parameter_name,
        default_value='172.16.0.102',
        description='Hostname or IP address of the robot.')
    use_fake_hardware_arg = DeclareLaunchArgument(
        use_fake_hardware_parameter_name,
        default_value='false',
        description='Use fake hardware')
    fake_sensor_commands_arg = DeclareLaunchArgument(
        fake_sensor_commands_parameter_name,
        default_value='false',
        description="Fake sensor commands. Only valid when '{}' is true".format(
            use_fake_hardware_parameter_name))
    load_gripper_arg = DeclareLaunchArgument(
        load_gripper_parameter_name,
        default_value='false',
        description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                    'without an end-effector.')
    use_rviz_arg = DeclareLaunchArgument(
        use_rviz_parameter_name,
        default_value='true',
        description='Launch RViz2 with the arm_pkg configuration.')
    interactive_marker_arg = DeclareLaunchArgument('interactive_marker',
                         default_value='true',
                         description="Activation de l'interactive marker")
    #======================== Nodes ========================
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states'],
             'rate': 30}],
    )
    franka_control2 = Node(
        package='franka_control2',
        executable='franka_control2_node',
        parameters=[{'robot_description': robot_description_arm}, franka_controllers],
        remappings=[('joint_states', 'franka/joint_states')],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
#        prefix=['xterm -e gdb --args'],
        on_exit=Shutdown(),
    )
    #------------ Controller Spawner -------------
    controller_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    controller_franka_robot_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )
    controller_cartesian_compliance = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['cartesian_compliance_controller'],
        output='screen',)
    #------------------------------------------------
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_arm}],
        output='screen'
    )
    interactive_marker_node = Node(
        package='arm_pkg',
        executable='interactive_marker_node',
        output='screen',
        condition=IfCondition(interactive_marker),
    )
    force_bridge_node = Node(
        package='arm_pkg',
        executable='force_bridge_node',
        name='force_bridge_node',
        parameters=[config_parameters_file],
        output='screen'
    )
    wrench_service_node = Node(
        package='arm_pkg',
        executable='wrench_service_node',
        name='wrench_service_node',
        parameters=[config_parameters_file],
        output='screen'
    )
    motion_smoother_node = Node(
        package='arm_pkg',
        executable='motion_smoother_node',
        name='motion_smoother_node',
        parameters=[config_parameters_file],
        output='screen'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('arm_pkg'),
            'rviz',
            'display.rviz'
        ])],
        output='log',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        load_gripper_arg,
        use_rviz_arg,
        interactive_marker_arg,

        joint_state_publisher,
        franka_control2,
        # Timer 2seconds to ensure the controller is ready before spawning
        TimerAction(
            period=2.0,
            actions=[
                controller_joint_state_broadcaster,
                controller_franka_robot_state_broadcaster,
                controller_cartesian_compliance,
                    ]
        ),
        # Timer 3seconds to ensure the controller is ready before launching code
        TimerAction(
            period=3.0,
            actions=[        
                robot_state_publisher,
                interactive_marker_node,
                force_bridge_node,
                wrench_service_node,
                motion_smoother_node,
                rviz2,
                ]
        )
    ])
