import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    # 1. 常量与路径获取
    robot_name_in_model = "fishbot"
    pkg_description = get_package_share_directory('fishbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    default_model_path = os.path.join(pkg_description, 'urdf/fishbot/fishbot.urdf.xacro')
    # 确保你的 world 文件路径正确
    default_world_path = os.path.join(pkg_description, 'world/colorful_apartment')

    # 2. 声明 Launch 参数
    action_declare_arg_mode_path = DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')

    # 3. 解析 Xacro 生成 robot_description
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)
      
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True 
        }]
    )

    # 4. 包含 Gazebo Sim (使用 -s 开启 Server 模式，减轻 CPU 负担)
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[('gz_args', [default_world_path, ' -r -v 4'])]
    )

    # 5. 在 Gazebo 中生成机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', robot_name_in_model, '-topic', 'robot_description'],
        output='screen',
    )

    # 6. ROS <-> Gazebo 桥接 (注意：删除了 /tf，避免与控制器冲突)
    ros_gz_bridge = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # /tf 由控制器发布，这里不再从 Gazebo 桥接
        ],
        output='screen'
    )

    # 7. RViz2 
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_description, 'config/rviz/fishbot_nav.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 8. 控制器加载器 (Spawner)
    # 使用 Node 形式，它们会自动等待 controller_manager 准备就绪
    load_joint_state_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_joint_state_broadcaster'],
        output='screen',
    )
    
    load_fishbot_diff_drive_controller = launch_ros.actions.Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fishbot_diff_drive_controller'], 
        output='screen',
    )

    # 9. SLAM Toolbox (延迟 5 秒启动，确保控制器已经完全跑起来)
    start_slam_toolbox_node = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
                ),
                launch_arguments={'use_sim_time': 'True'}.items()
            )
        ]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        rviz_node,
        ros_gz_bridge,
        load_joint_state_controller,
        load_fishbot_diff_drive_controller,
        start_slam_toolbox_node,
    ])