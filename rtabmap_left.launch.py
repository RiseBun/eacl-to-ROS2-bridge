from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os

def generate_launch_description():

    delete_db_arg = DeclareLaunchArgument(
        'delete_db',
        default_value='true',
        description='Delete previous database on start'
    )

    # 1) base_link -> 左目相机 TF（用你标好的外参）
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.05639', '-0.00186', '0.00336',   # x y z
            '-0.16659', '0.68475', '-0.68136', '0.19776',  # qx qy qz qw
            'base_link', 'S1/stereo1_l'
        ],
        output='screen'
    )

    # 2) RTAB-Map SLAM
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        condition=IfCondition(LaunchConfiguration('delete_db')),

        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,   # 发布 map->odom

            'subscribe_odom': True,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'queue_size': 30,

            # ==== 检测频率 ====
            'Rtabmap/DetectionRate': '1.0',
            'RGBD/LinearUpdate': '0.0',
            'RGBD/AngularUpdate': '0.0',

            # ==== 视觉特征 / 回环 ====
            'Reg/Strategy': '0',
            'Vis/FeatureType': '6',        # ORB
            'Kp/DetectorStrategy': '6',
            'Kp/MaxFeatures': '1200',
            'Vis/MinInliers': '15',
            'Vis/InlierDistance': '0.03',

            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityByTime': 'false',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'RGBD/ProximityMaxPaths': '3',
            'RGBD/ProximityRadius': '2.0',

            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/OptimizeMaxError': '2.5',

            'odom_tf_linear_variance':  0.01,
            'odom_tf_angular_variance': 0.01,
            'Odom/Strategy': '0',
            'Odom/FilteringStrategy': '1',

            'Mem/IncrementalMemory': 'true',
            'Mem/InitWMWithAllNodes': 'true',

            'Grid/FromDepth': 'true',  # 使用深度建图
            'Grid/3D': 'true',
            'Grid/RangeMax': '8.0',
            'Grid/DepthDecimation': '1',  # 不降采样深度图

            'Cloud/Decimation': '1',
            'Cloud/VoxelSize': '0.01',  # 保持高密度
            'Cloud/OutputVoxelized': 'true',

            'Cloud/Assemble': 'true',          # 累积 map 上所有节点的云
            'Rtabmap/PublishRAMCloud': 'true', # 发布 /rtabmap/cloud_map

            'database_path': '/home/li/.ros/rtabmap_left.db',
        }],
        
        remappings=[
            ('rgb/image', '/S1/stereo1_l'),
            ('depth/image', '/S1/stereo1_l/depth'),  # 修改 depth 图像话题
            ('rgb/camera_info', '/S1/stereo1_l/camera_info'),
            ('odom', '/S1/vio_odom'),
        ]
    )

    # 3) RTAB-Map 可视化
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',

            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'approx_sync': True,
            'queue_size': 30,

            'Grid/3D': 'true',
            'Grid/RangeMax': '8.0',
            'Cloud/Decimation': '1',
            'Cloud/VoxelSize': '0.01',
        }],
        remappings=[
            ('rgb/image', '/S1/stereo1_l'),
            ('depth/image', '/S1/stereo1_l/depth'),
            ('rgb/camera_info', '/S1/stereo1_l/camera_info'),
            ('odom', '/S1/vio_odom'),
        ]
    )

    return LaunchDescription([
        delete_db_arg,
        tf_camera_link,
        rtabmap_slam,
        rtabmap_viz,
    ])
