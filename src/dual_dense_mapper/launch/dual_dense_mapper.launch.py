from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_dense_mapper',
            executable='dual_dense_mapper_node',
            name='dual_dense_mapper',
            output='screen',
            parameters=[{
                'target_frame': 'map',

                'left_depth':  '/S1/stereo1_l/depth',
                'left_info':   '/S1/stereo1_l/camera_info',

                'right_depth': '/S1/stereo2_r/depth',
                'right_info':  '/S1/stereo2_r/camera_info',

                'sample_step': 4,
                'min_z': 0.2,
                'max_z': 30.0,
                'voxel': 0.05,

                'pub_cloud': '/dense_cloud_map',
                'publish_hz': 2.0,

                # TF 稳定关键
                'tf_timeout': 0.05,
                'tf_backoff_ms': 20,
                'tf_retries': 3,
            }]
        )
    ])
