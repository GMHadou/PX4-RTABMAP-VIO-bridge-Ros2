import launch
from launch_ros.actions import Node

def generate_launch_description():

    # VIO transformation node to convert VIO data to PX4 format
    vio_transform_node = Node(
        name='vio_transform',
        namespace='vio_transform',
        package='px4_vslam',
        executable='vio_transform'
    )

    return launch.LaunchDescription([
        vio_transform_node
    ])

