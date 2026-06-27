from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare the launch arguments
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.1',
        description='Smoothing factor for the low-pass filter.'
    )
    window_size_arg = DeclareLaunchArgument(
        'window_size',
        default_value='50',
        description='Window size for the median filter.'
    )
    
    # Get the path to the package share directory
    load_estimation_share_dir = get_package_share_directory('load_estimation')
    plotjuggler_layout_path = PathJoinSubstitution([load_estimation_share_dir, 'config', 'plottjuggler.xml'])

    sensor_filter_node = Node(
        package='load_estimation',
        executable='sensor_filter.py',
        name='sensor_filter',
        output='screen',
        parameters=[{
            'alpha': LaunchConfiguration('alpha'),
            'window_size': LaunchConfiguration('window_size')
        }]
    )

    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=['-l', plotjuggler_layout_path]
    )

    return LaunchDescription([
        alpha_arg,
        window_size_arg,
        sensor_filter_node,
        plotjuggler_node
    ])