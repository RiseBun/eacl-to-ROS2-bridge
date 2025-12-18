from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ==========================================
    # 1. 话题配置
    # ==========================================
    topic_left_rgb   = '/S1/stereo1_l'
    topic_left_depth = '/S1/stereo1_l/depth'
    topic_left_info  = '/S1/stereo1_l/camera_info'
    
    topic_right_rgb   = '/S1/stereo2_r'
    topic_right_depth = '/S1/stereo2_r/depth'
    topic_right_info  = '/S1/stereo2_r/camera_info'
    
    odom_topic = '/S1/vio_odom'
    
    # ==========================================
    # 2. 同步节点参数
    # ==========================================
    sync_params = [{
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'queue_size': 100,      
        'topic_queue_size': 50, 
        'qos': 2,               
        'qos_camera_info': 2,
        'depth_scale': 1.0
    }]

    # ==========================================
    # 3. 同步节点
    # ==========================================
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

    # ==========================================
    # 4. RTAB-Map 主节点 (修正版)
    # ==========================================
    rtabmap_slam = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'], 
        
        parameters=[{
            # --- ROS 包装器参数 (可以使用 Python 类型) ---
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
            'wait_for_transform': 0.2, 

            # ==================================================
            #    核心优化配置 (必须使用字符串 String)
            # ==================================================
            
            # 1.【彻底关闭建图】
            'Grid/FromDepth': 'false',       # 必须是字符串
            'Grid/3D': 'false',              # 必须是字符串
            'Grid/RayTracing': 'false',      
            'Grid/GlobalMinSize': '0.0',     
            'Rtabmap/CreateIntermediateNodes': 'false', 
            
            # 2.【关闭 ROS 话题发布】
            'Publish/Map': 'false',          
            'Publish/Octomap': 'false',      
            
            # 3.【数据保存】
            'Mem/SaveDepth': 'true',    
            'Mem/SaveRGB': 'true',      
            'Mem/ImageCompressionFormat': '.jpg', 
            'Mem/STMSize': '30',         

            # 4.【定位精度与策略】
            'Rtabmap/DetectionRate': '10',   # 字符串 '10'
            'RGBD/LinearUpdate': '0.05',     # 字符串 '0.05'
            'RGBD/AngularUpdate': '0.02',    
            
            # 回环检测参数
            'Vis/MinInliers': '15',          
            'Vis/EstimationType': '0',       
            'RGBD/OptimizeMaxError': '4.0',  
            
            # 5.【TF 发布】
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