from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_dir = get_package_share_directory('slam_toolbox')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    imu_dir = get_package_share_directory('wit_ros2_imu')
    localization_dir = get_package_share_directory('robot_localization')

    # 自定义的nav2参数文件路径
    custom_nav2_params = '/home/astwea/nav_ws/src/run_saodi/params/nav.yaml'

    # 启动 nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'params_file': custom_nav2_params}.items()
    )

    # 启动 slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        )
    )

    # 启动雷达
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rplidar_dir, 'launch', 'rplidar_c1_launch.py')
        )
    )
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_dir,  'imu.launch.py')
        )
    )

    # 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_dir, 'rviz', 'nav2_default_view.rviz')]
    )


    return LaunchDescription([
        rplidar_launch,
        imu_launch,
        slam_launch,
        nav2_launch,
        rviz_node,
    ])

