from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package share directory
    pkg_path = get_package_share_directory('robot_simulation')
    world_path = os.path.join(pkg_path, 'worlds', 'simple_obstacles.world')
    
    # Gazebo process
    gazebo_process = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        shell=False
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': 
            '''<?xml version="1.0"?>
<robot name="differential_robot">
  <link name="base_link">
    <visual><geometry><cylinder length="0.1" radius="0.2"/></geometry><material name="blue"><color rgba="0 0 0.8 1"/></material></visual>
    <collision><geometry><cylinder length="0.1" radius="0.2"/></geometry></collision>
    <inertial><mass value="2.0"/><inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/></inertial>
  </link>
  <link name="left_wheel">
    <visual><geometry><cylinder length="0.05" radius="0.05"/></geometry><material name="black"><color rgba="0 0 0 1"/></material></visual>
    <collision><geometry><cylinder length="0.05" radius="0.05"/></geometry></collision>
    <inertial><mass value="0.5"/><inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/></inertial>
  </link>
  <link name="right_wheel">
    <visual><geometry><cylinder length="0.05" radius="0.05"/></geometry><material name="black"><color rgba="0 0 0 1"/></material></visual>
    <collision><geometry><cylinder length="0.05" radius="0.05"/></geometry></collision>
    <inertial><mass value="0.5"/><inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/></inertial>
  </link>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="1.5708 0 0"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="right_wheel"/>
    <origin xyz="0 -0.15 0" rpy="1.5708 0 0"/><axis xyz="0 1 0"/>
  </joint>
  <link name="laser_link">
    <collision><geometry><box size="0.01 0.01 0.01"/></geometry></collision>
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual>
    <inertial><mass value="0.1"/><inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/></inertial>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="laser_link"/>
    <origin xyz="0.2 0 0.05" rpy="0 0 0"/>
  </joint>
  <gazebo reference="laser_link">
    <sensor type="ray" name="laser_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.0</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="laser_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
      <max_wheel_torque>10</max_wheel_torque>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
</robot>'''
        }]
    )
    
    # Spawn robot (delayed start)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'differential_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.2'
        ],
        output='screen'
    )
    
    # Obstacle avoidance node (start after spawn)
    obstacle_avoidance = Node(
        package='robot_simulation',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen'
    )
    
    # Delay spawn until gazebo is ready
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    
    # Delay obstacle avoidance until after spawn
    delayed_obstacle_avoidance = TimerAction(
        period=7.0,
        actions=[obstacle_avoidance]
    )
    
    return LaunchDescription([
        gazebo_process,
        robot_state_publisher,
        delayed_spawn,
        delayed_obstacle_avoidance
    ])
