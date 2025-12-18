#ifndef VK_ROS2_DRIVER_HPP_
#define VK_ROS2_DRIVER_HPP_

#include <vk_sdk/Sdk.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <atomic> 

namespace vkc
{

/***************************************
 * Stereo Calibration Struct (不变)
 ***************************************/
struct StereoCalib
{
  int width  = 0;
  int height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double baseline = 0.0;
  std::string model;
  std::vector<double> dist;
  bool valid = false;
};

class CalibrationManager
{
public:
  bool load(const std::string &saved,
            const std::string &left,
            const std::string &right,
            rclcpp::Logger logger);

  inline bool get(const std::string &key, StereoCalib &out) const
  {
    auto it = map_.find(key);
    if (it == map_.end()) return false;
    out = it->second;
    return it->second.valid;
  }
private:
  std::map<std::string, StereoCalib> map_;
};

class VkRos2Driver;
class ImuReceiver;
class OdometryReceiver;
class ImageReceiver;
class DisparityReceiver;

/***************************************
 * VkRos2Driver Node
 ***************************************/
class VkRos2Driver : public rclcpp::Node
{
public:
  explicit VkRos2Driver(const rclcpp::NodeOptions &options);
  ~VkRos2Driver();

  inline const CalibrationManager &calib() const { return calib_; }

  // ========================================================================
  // 【终极修复】智能时间同步函数 (Industrial Grade)
  // 1. 强制使用 System Time (解决无/clock回放问题)
  // 2. 全状态重置 (解决重置后时间卡顿问题)
  // 3. 2.0s 积压容忍 (保护跨话题同步)
  // ========================================================================
  template <typename HeaderReader>
  rclcpp::Time stamp_from(const HeaderReader& h) const
  {
      // 1. 获取当前数据的原始设备时间
      const int64_t t_dev = static_cast<int64_t>(h.getStampMonotonic());
      
      // 2. 【关键改动】强制使用系统墙上时间，忽略 use_sim_time
      // static 保证只初始化一次，避免重复开销
      static rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
      const int64_t now_ns = sys_clock.now().nanoseconds();

      // 参数配置 (更加鲁棒的参数)
      constexpr int64_t kJumpBackNs   = 500000000LL;    // 0.5s：回放循环检测阈值
      constexpr int64_t kMaxLagNs     = 2000000000LL;   // 2.0s：【放宽】积压保护阈值，防止频繁重置破坏同步
      constexpr int64_t kFutureTolNs  =  50000000LL;    // 50ms：未来时间容忍度

      // 定义重置锚点的 Lambda (全状态重置)
      auto reset_anchor = [&](int64_t dev_t, int64_t ros_t) {
          t_dev0_ns_.store(dev_t, std::memory_order_release);
          t_ros0_ns_.store(ros_t, std::memory_order_release);
          last_dev_ns_.store(dev_t, std::memory_order_release);
          last_ros_ns_.store(ros_t, std::memory_order_release);
      };

      // A) 初始化：如果还没初始化，直接建立锚点
      if (!time_inited_.load(std::memory_order_acquire)) {
          reset_anchor(t_dev, now_ns);
          time_inited_.store(true, std::memory_order_release);
          return rclcpp::Time(now_ns);
      }

      // B) 铁律1：设备时间单调性检查 (检测循环回放)
      const int64_t last_dev = last_dev_ns_.load(std::memory_order_acquire);
      if (t_dev + kJumpBackNs < last_dev) {
          // RLOG(INFO) << "Time Loop detected! Resetting anchor.";
          reset_anchor(t_dev, now_ns);
      }

      // C) 计算映射后的 ROS 时间
      int64_t t_ros = t_ros0_ns_.load(std::memory_order_acquire)
                    + (t_dev - t_dev0_ns_.load(std::memory_order_acquire));

      // D) 铁律2：积压保护 (Lag Protection)
      // 如果算出来的时间比现在慢超过 2秒，说明严重积压或时间源漂移
      if (now_ns - t_ros > kMaxLagNs) {
          reset_anchor(t_dev, now_ns); // 重置一切
          t_ros = now_ns;              // 强制拉回现在
      }

      // E) 铁律3：未来时间钳制 (Future Clamp)
      if (t_ros > now_ns + kFutureTolNs) {
          t_ros = now_ns;
      }

      // F) 保证 ROS 时间输出单调 (不倒退)
      const int64_t last_ros = last_ros_ns_.load(std::memory_order_acquire);
      if (t_ros < last_ros) {
          t_ros = last_ros; 
      }

      // 更新历史记录
      last_dev_ns_.store(t_dev, std::memory_order_release);
      last_ros_ns_.store(t_ros, std::memory_order_release);
      
      return rclcpp::Time(t_ros);
  }

  friend class ImuReceiver;
  friend class OdometryReceiver;
  friend class ImageReceiver;
  friend class DisparityReceiver;

private:
  void log_cb(vkc::LogLevel level, std::string_view message);
  void publish_static_extrinsics_from_saved(const std::string& saved_path);

  std::unique_ptr<vkc::VisualKit> visualkit;
  CalibrationManager calib_;

  std::string odom_frame_;
  std::string base_link_frame_;
  bool publish_tf_ = true;

  /* Time Sync Variables */
  mutable std::atomic<bool> time_inited_{false};
  mutable std::atomic<int64_t> t_ros0_ns_{0}; 
  mutable std::atomic<int64_t> t_dev0_ns_{0}; 
  
  mutable std::atomic<int64_t> last_dev_ns_{0}; 
  mutable std::atomic<int64_t> last_ros_ns_{0};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;

  image_transport::ImageTransport it_;

  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imu_pub_;
  std::map<std::string, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pub_;
  std::map<std::string, image_transport::Publisher> img_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> ci_pub_;
  std::map<std::string, image_transport::Publisher> disp_pub_;
};

/***************************************
 * Receiver Definitions (不变)
 ***************************************/
class ImuReceiver : public vkc::Receiver<vkc::Imu>
{
public:
  ImuReceiver(const VkRos2Driver &driver,
              rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
              const std::string &frame_id)
    : driver_(driver), pub_(pub), frame_id_(frame_id) {}
  ReceiverStatus handle(const Message<Shared<Imu>> &msg) override;
private:
  const VkRos2Driver &driver_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  std::string frame_id_;
};

class OdometryReceiver : public vkc::Receiver<vkc::Odometry3d>
{
public:
  OdometryReceiver(const VkRos2Driver &driver,
                   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub)
    : driver_(driver), pub_(pub) {}
  ReceiverStatus handle(const Message<Shared<Odometry3d>> &msg) override;
private:
  const VkRos2Driver &driver_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

class ImageReceiver : public vkc::Receiver<vkc::Image>
{
public:
  ImageReceiver(const VkRos2Driver &driver,
                image_transport::Publisher &img_pub,
                rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ci_pub,
                const std::string &topic_name,
                const std::string &clean_key)
    : driver_(driver), img_pub_(img_pub), ci_pub_(ci_pub),
      topic_name_(topic_name), clean_key_(clean_key) {}
  ReceiverStatus handle(const Message<Shared<Image>> &msg) override;
private:
  const VkRos2Driver &driver_;
  image_transport::Publisher &img_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ci_pub_;
  std::string topic_name_;
  std::string clean_key_;
};

class DisparityReceiver : public vkc::Receiver<vkc::Disparity>
{
public:
  DisparityReceiver(const VkRos2Driver &driver,
                    image_transport::Publisher &pub,
                    const std::string &topic_name);
  ReceiverStatus handle(const Message<Shared<Disparity>> &msg) override;
private:
  const VkRos2Driver &driver_;
  image_transport::Publisher &pub_;
  std::string topic_name_;
  std::string clean_key_;
  std::string camera_frame_;
  double disparity_scale_;
};

} // namespace vkc

#endif // VK_ROS2_DRIVER_HPP_