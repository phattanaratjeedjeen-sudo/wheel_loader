import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def find_and_play_bag(context, *args, **kwargs):
    """Finds the latest rosbag in a directory and creates an ExecuteProcess action to play it."""
    bags_dir = LaunchConfiguration('bags_dir').perform(context)
    bag_folders = glob.glob(os.path.join(bags_dir, 'rosbag2_*'))

    if not bag_folders:
        return [LogInfo(msg=f"Warning: No rosbags found in directory: '{bags_dir}'. Skipping bag play.")]

    latest_bag_path = max(bag_folders)
    log_msg = LogInfo(msg=f"Found and playing latest bag: '{latest_bag_path}'")
    bag_play_action = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', latest_bag_path],
        output='screen'
    )
    return [log_msg, bag_play_action]


def generate_launch_description():
    pkg_share = get_package_share_directory('load_estimation')

    juggler_layout_arg = DeclareLaunchArgument(
        'juggler_layout',
        default_value=os.path.join(pkg_share, 'config', 'juggler_layout.xml')
    )

    bags_dir_arg = DeclareLaunchArgument(
        'bags_dir',
        default_value='empty',
    )

    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=['-l', LaunchConfiguration('juggler_layout')]
    )

    find_and_play_bag_action = OpaqueFunction(function=find_and_play_bag)

    load_estimation_node = Node(
        package='load_estimation',
        executable='load_estimate.py',
    )

    monitor_node = Node(
        package='load_estimation',
        executable='raw_data.py',
    )

    call_srv_node = Node(
        package='load_estimation',
        executable='call_srv.py',
    )
    
    return LaunchDescription([
        juggler_layout_arg,
        bags_dir_arg,
        plotjuggler_node,
        # call_srv_node,
        monitor_node,
        find_and_play_bag_action,
        load_estimation_node,
    ])