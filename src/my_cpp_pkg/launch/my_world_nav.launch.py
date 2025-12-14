import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_name = 'my_cpp_pkg'
    pkg_share = get_package_share_directory(pkg_name)

    # Paths
    world_path = os.path.join(pkg_share, 'worlds', 'map.world')
    map_path = os.path.join(pkg_share, 'maps', 'map.yaml')
    urdf_path = os.path.join(pkg_share, 'robot', 'robot.urdf')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    nav2_pkg = get_package_share_directory('nav2_bringup')
    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # --- 1. INFRASTRUCTURE (Gazebo + RSP + TF) ---
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': f'-r -s -v 4 {world_path}'}.items()
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Static TF for Lidar (adjust values if needed)
    lidar_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'security_robot/base_footprint/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # --- 2. SPAWN ROBOT (T=2s) ---
    
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'security_robot', '-z', '0.1'],
                output='screen'
            )
        ]
    )

    # --- 3. SENSOR BRIDGE & EKF (T=4s) ---
    
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'use_sim_time': True,
            'qos_overrides./scan.subscriber.reliability': 'reliable',
        }],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('odometry/filtered', 'odometry/filtered')]
    )

    delayed_sensors = TimerAction(period=4.0, actions=[bridge_node, ekf_node])

    # --- 4. NAV2 & RVIZ (T=7s) ---

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_path,
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            'autostart': 'true',
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_yaw': '0.0'
        }.items()
    )

    # Remap /odom to /odometry/filtered for Nav2
    nav2_group = GroupAction(
        actions=[
            SetRemap(src='/odom', dst='/odometry/filtered'),
            nav2_bringup
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_pkg, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_nav = TimerAction(period=7.0, actions=[nav2_group, rviz_node])

    # --- 5. FORCE INITIAL POSE (T=12s) ---
    force_initial_pose = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '-1', '/initialpose', 
                    'geometry_msgs/msg/PoseWithCovarianceStamped', 
                    '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'
                ],
                output='screen'
            )
        ]
    )

    # --- 6. PATROL MANAGER (T=15s) ---
    patrol_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='my_cpp_pkg',
                executable='patrol_manager.py',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # --- 7. CAMERA WINDOW (Add this) ---
    camera_monitor = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/camera'],  # Automatically selects the topic
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        lidar_tf_fix,
        spawn_entity,
        delayed_sensors,
        delayed_nav,
        force_initial_pose,
        patrol_node,
        camera_monitor  # <--- New Node Added here
    ])


WAYPOINTS_LIST = [
    [7.9, 11.1],
    [12.1, -11.1],
    [-10.8, 9.9],
    [-9.7, -11.2]
]