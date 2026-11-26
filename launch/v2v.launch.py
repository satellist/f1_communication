"""ros2 launch car_v2v v2v.launch.py [centerline_csv:=/path/to/centerline.csv] [car_id:=N]"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def default_centerline():
    # install/car_v2v/share/car_v2v -> 往上四層是工作區根目錄
    ws = os.environ.get('CAR_WS') or os.path.abspath(
        os.path.join(get_package_share_directory('car_v2v'), '..', '..', '..', '..'))
    p = os.path.join(ws, 'src', 'car_control', 'car_control', 'waypoints', 'waypoints.csv')
    return p if os.path.exists(p) else ''


def generate_launch_description():
    share = get_package_share_directory('car_v2v')
    return LaunchDescription([
        DeclareLaunchArgument('centerline_csv', default_value=default_centerline(),
                              description='兩台車要用同一條中心線'),
        DeclareLaunchArgument('car_id', default_value='0', description='0 = 依 MAC 自動'),
        Node(
            package='car_v2v',
            executable='v2v_bridge',
            name='v2v_bridge',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'v2v.yaml'),
                {'centerline_csv': LaunchConfiguration('centerline_csv'),
                 'car_id': ParameterValue(LaunchConfiguration('car_id'), value_type=int)},
            ],
        ),
    ])
