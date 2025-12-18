# vk_sdk ROS2 Example
ROS2 example package to bridge vk_sdk ecal messages to ROS2 messages

## Sample Usage

Assuming the `/opt/vilota` directory exists (vk_system is installed), you can build and run the example as follows:

```
colcon build --packages-select vk_ros2_driver
ros2 launch vk_ros2_driver example.launch.py
```

- The following DP180 topics are provided as examples: Images, Disparity, Odometry, Imu
- Odometry frame and camera base_link_frame are defined in config/example.yaml
- TFs from base_link to imu / cameras are obtained from imu and image msgs
