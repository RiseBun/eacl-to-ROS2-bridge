from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    topic_left_rgb   = '/S1/stereo1_l'
    topic_left_depth = '/S1/stereo1_l/depth'
    topic_left_info  = '/S1/stereo1_l/camera_info'
    
    topic_right_rgb   = '/S1/stereo2_r'
    topic_right_depth = '/S1/stereo2_r/depth'
    topic_right_info  = '/S1/stereo2_r/camera_info'
    
    odom_topic = '/S1/vio_odom'
    
    sync_params = [{
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'queue_size': 100,      
        'topic_queue_size': 50, 
        'qos': 2,               
        'qos_camera_info': 2,
        'depth_scale': 1.0
    }]

    sync_left = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync_left',
        output='screen',
        parameters=sync_params,
        remappings=[
            ('rgb/image',       topic_left_rgb),
            ('depth/image',     topic_left_depth),
            ('rgb/camera_info', topic_left_info),
            ('rgbd_image',      'rgbd_image0')
        ]
    )

    sync_right = Node(
        package='rtabmap_sync', executable='rgbd_sync', name='rgbd_sync_right',
        output='screen',
        parameters=sync_params,
        remappings=[
            ('rgb/image',       topic_right_rgb),
            ('depth/image',     topic_right_depth),
            ('rgb/camera_info', topic_right_info),
            ('rgbd_image',      'rgbd_image1')
        ]
    )

    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'], 
        
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': True,
            'rgbd_cameras': 2,        
            'subscribe_odom': True,
            'qos_odom': 2,            
            'queue_size': 200,
            'approx_sync': True,
            'wait_for_transform': 0.5,

            # ==================================================
            #    🌟 可视化核心配置：重新开启点云生成 🌟
            # ==================================================
            
            # 1. 开启基础建图开关 (可视化需要它们来生成数据)
            'Grid/FromDepth': 'true',        # 允许从深度图生成数据
            'Grid/3D': 'true',               # 开启 3D 点云
            'Grid/RayTracing': 'false',      # 依然关闭射线追踪以省电/省 CPU
            
            # 2. 点云发布参数 (RViz 看到的内容)
            'Publish/Map': 'true',           # 关键：开启 /cloud_map 话题发布
            'Rtabmap/CreateIntermediateNodes': 'false', 
            
            # 3. 性能平衡：限制点云密度
            # 既然是可视化版本，不建议点云太密，否则 RViz 会卡
            'Grid/VoxelSize': '0.05',        # 5cm 体素下采样，平衡精细度与流畅度
            'Grid/MaxDistance': '10.0',      # 只显示 10 米内的点云
            'Grid/CellSize': '0.05',         # 栅格大小
            
            # 4. 其它保持定位性能的参数
            'Mem/SaveDepth': 'true',    
            'Mem/SaveRGB': 'true',      
            'Rtabmap/DetectionRate': '1',
            'RGBD/LinearUpdate': '0.1',
            'RGBD/AngularUpdate': '0.05',
            'Vis/MinInliers': '15',
            'RGBD/OptimizeMaxError': '4.0',
            'Publish/Tf': 'true',
        }],
        
        remappings=[
            ('odom', odom_topic),
            ('rgbd_image0', 'rgbd_image0'),
            ('rgbd_image1', 'rgbd_image1'),
        ]
    )

    return LaunchDescription([
        sync_left,
        sync_right,
        rtabmap_slam,
    ])