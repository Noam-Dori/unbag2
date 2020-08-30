from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from os.path import join


def generate_launch_description():
    config = join(get_package_share_directory('unbag2'), 'cfg', 'unbag2.yml')
    files = DeclareLaunchArgument('files', default_value='',
                                  description='string containing a list of all files/directories to try to read as '
                                              'bag files')
    files_param = LaunchConfiguration('files')

    return LaunchDescription([
        files,
        LogInfo(msg=["Un-Bagging files: ", files_param]),
        Node(package='unbag2', executable='unbag2', output='screen',
             name='unbag2', parameters=[config, {'files': files_param}])
    ])
