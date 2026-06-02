import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('controlegeral_carrinho')  # <- altere para o nome do seu pacote

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type':     'serial',
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'laser',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Sensitivity',
        }],
        output='screen',
    )

    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[os.path.join(pkg_share, 'config', 'bno055_params.yaml')],
        output='screen',
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')],
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'base_link', 'bno055'],
        output='screen',
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'base_link', 'laser'],  
        output='screen',
    )



    return LaunchDescription([
        sllidar_node,
        bno055_node,
        robot_localization_node,
        slam_toolbox_node,
        imu_tf,
        laser_tf,
    ])
