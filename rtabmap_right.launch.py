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

    # 1) base_link -> 右目相机 TF
    # 请确保这里的外参是准确的：base_link 到 S1/stereo2_r 光心的变换
    tf_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.05639', '-0.00186', '0.00336',             # x y z
            '0.16659', '-0.68475', '0.68136', '0.19776',  # qx qy qz qw
            'base_link', 'S1/stereo2_r'
        ],
        output='screen'
    )

    # 2) RTAB-Map SLAM (Mapping 模式)
    rtabmap_slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap_right',
        output='screen',
        arguments=['--delete_db_on_start'],
        condition=IfCondition(LaunchConfiguration('delete_db')),

        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom', # 对应 VIO 输出的 frame_id
            'map_frame_id': 'map',
            'publish_tf': True,      # 由 RTAB-Map 发布 map -> odom 的修正

            'subscribe_odom': True,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'approx_sync': True,
            'queue_size': 20,
            
            # ==========================================
            # 【关键修改】QoS 设置 (2 = Best Effort)
            # 必须与驱动发布的 QoS 保持一致，否则收不到数据
            # ==========================================
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_depth': 2,        # 如果 rtabmap 版本较新支持此参数
            'qos_odom': 2,         # VIO 如果也是 Best Effort，这里也要改

            # ==== 性能 ====
            'Rtabmap/DetectionRate': '0.0', # 尽可能快
            'RGBD/LinearUpdate': '0.0',     # 只要有数据就更新
            'RGBD/AngularUpdate': '0.0',

            # ==== 特征匹配 ====
            'Vis/FeatureType': '6',         # ORB (通常比 GFTT 慢但旋转鲁棒性好，GFTT=0 更快)
            'Kp/MaxFeatures': '1000',       # 稍微降一点提速
            'Vis/MinInliers': '15',

            # ==== 优化稠密点云效果 ====
            'Grid/FromDepth': 'true',
            'Grid/3D': 'true',
            'Grid/RangeMax': '4.0',         # 太远的点通常噪声大，截断掉
            'Grid/RayTracing': 'false',     # 【建议】先关掉，防止VIO漂移导致墙面被擦除
            
            # 【重点】体素大小决定了"稠密"程度
            'Cloud/VoxelSize': '0.01',      # 1cm 精度 (之前是 0.05)
            'Cloud/Decimation': '1',        # 1=不降采样，保留所有细节
            'Grid/DepthDecimation': '1',    # 建图时也不降采样
            
            # 填补深度图空洞 (如果是立体视觉，通常有很多空洞，开启这个会让图更好看)
            'Grid/FillHolesSize': '1', 
            
            'Cloud/OutputVoxelized': 'true',
            'Rtabmap/PublishRAMCloud': 'true',

            # 数据库路径
            'database_path': '/home/li/.ros/rtabmap_right.db',
        }],
        
        remappings=[
            ('rgb/image', '/S1/stereo2_r'),
            ('depth/image', '/S1/stereo2_r/depth'),
            ('rgb/camera_info', '/S1/stereo2_r/camera_info'),
            ('odom', '/S1/vio_odom'),
        ]
    )

    # 3) RTAB-Map 可视化
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz_right',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_odom': True,
            'approx_sync': True,
            'queue_size': 30,
            
            # Viz 也需要 QoS 匹配
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_odom': 2,

            'Grid/3D': 'true',
            'Cloud/VoxelSize': '0.01', # 保持高精度显示
        }],
        remappings=[
            ('rgb/image', '/S1/stereo2_r'),
            ('depth/image', '/S1/stereo2_r/depth'),
            ('rgb/camera_info', '/S1/stereo2_r/camera_info'),
            ('odom', '/S1/vio_odom'),
        ]
    )

    return LaunchDescription([
        delete_db_arg,
        tf_camera_link,
        rtabmap_slam,
        rtabmap_viz,
    ])