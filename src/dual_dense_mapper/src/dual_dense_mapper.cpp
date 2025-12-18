#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>

#include <mutex>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::PointCloud2;

static inline bool isFinite(float v) { return std::isfinite(v); }

class DualDenseMapper : public rclcpp::Node
{
public:
  DualDenseMapper() : Node("dual_dense_mapper")
  {
    // ===============================
    // Params
    // ===============================
    target_frame_ = this->declare_parameter<std::string>("target_frame", "odom"); // 建议先 odom，map 稳了再换

    left_depth_topic_  = this->declare_parameter<std::string>("left_depth",  "/S1/stereo1_l/depth");
    left_info_topic_   = this->declare_parameter<std::string>("left_info",   "/S1/stereo1_l/camera_info");

    right_depth_topic_ = this->declare_parameter<std::string>("right_depth", "/S1/stereo2_r/depth");
    right_info_topic_  = this->declare_parameter<std::string>("right_info",  "/S1/stereo2_r/camera_info");

    // 采样/深度过滤
    sample_step_ = this->declare_parameter<int>("sample_step", 4); // 1 最密，CPU 最重
    min_z_       = this->declare_parameter<double>("min_z", 0.2);
    max_z_       = this->declare_parameter<double>("max_z", 30.0);

    // 体素
    voxel_       = this->declare_parameter<double>("voxel", 0.05);
    voxel_map_   = this->declare_parameter<double>("voxel_map", 0.08); // 地图层更粗一点，控点数

    // 发布
    pub_map_topic_  = this->declare_parameter<std::string>("pub_map", "/dense_cloud_map");
    pub_inc_topic_  = this->declare_parameter<std::string>("pub_incremental", "/dense_cloud_inc");
    publish_hz_     = this->declare_parameter<double>("publish_hz", 2.0);

    // 地图大小控制
    max_points_     = static_cast<size_t>(this->declare_parameter<int>("max_points", 2500000));
    downsample_every_n_ = this->declare_parameter<int>("downsample_every_n", 1); // 每 N 次发布做一次地图体素滤波
    if (downsample_every_n_ < 1) downsample_every_n_ = 1;

    // 彩色可视化（左蓝右红）
    publish_rgb_ = this->declare_parameter<bool>("publish_rgb", true);
    pub_map_rgb_topic_ = this->declare_parameter<std::string>("pub_map_rgb", "/dense_cloud_map_rgb");

    // TF “稳健参数”
    tf_timeout_    = this->declare_parameter<double>("tf_timeout", 0.05);
    tf_backoff_ms_ = this->declare_parameter<int>("tf_backoff_ms", 20);
    tf_retries_    = this->declare_parameter<int>("tf_retries", 3);

    if (sample_step_ < 1) sample_step_ = 1;

    // ===============================
    // TF Buffer & Listener
    // ===============================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
      this->get_clock(),
      tf2::durationFromSec(30.0)
    );
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface()
    );
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    // ===============================
    // Publishers
    // ===============================
    // RViz 常见场景：传感器流 -> best effort 更稳
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    pub_map_ = this->create_publisher<PointCloud2>(pub_map_topic_, qos);
    pub_inc_ = this->create_publisher<PointCloud2>(pub_inc_topic_, qos);
    pub_map_rgb_ = this->create_publisher<PointCloud2>(pub_map_rgb_topic_, qos);

    

    if (publish_rgb_) {
      pub_map_rgb_ = this->create_publisher<PointCloud2>(pub_map_rgb_topic_, qos);
    }

    pub_status_ = this->create_publisher<std_msgs::msg::String>("/mapper_status", rclcpp::QoS(1));

    // ===============================
    // Sync: left (depth + info)
    // ===============================
    sub_l_depth_.subscribe(this, left_depth_topic_, rmw_qos_profile_sensor_data);
    sub_l_info_.subscribe(this, left_info_topic_,  rmw_qos_profile_sensor_data);

    sync_l_ = std::make_shared<SyncL>(SyncL(50), sub_l_depth_, sub_l_info_);
    sync_l_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
    sync_l_->registerCallback(
      std::bind(&DualDenseMapper::cbLeft, this, std::placeholders::_1, std::placeholders::_2));

    // ===============================
    // Sync: right (depth + info)
    // ===============================
    sub_r_depth_.subscribe(this, right_depth_topic_, rmw_qos_profile_sensor_data);
    sub_r_info_.subscribe(this, right_info_topic_,  rmw_qos_profile_sensor_data);

    sync_r_ = std::make_shared<SyncR>(SyncR(50), sub_r_depth_, sub_r_info_);
    sync_r_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
    sync_r_->registerCallback(
      std::bind(&DualDenseMapper::cbRight, this, std::placeholders::_1, std::placeholders::_2));

    // ===============================
    // Timer publish
    // ===============================
    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_hz_));
    timer_pub_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&DualDenseMapper::publishClouds, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "DualDenseMapper started. target_frame=%s, map=%s, inc=%s, hz=%.2f, voxel=%.3f, voxel_map=%.3f, rgb=%d",
      target_frame_.c_str(), pub_map_topic_.c_str(), pub_inc_topic_.c_str(),
      publish_hz_, voxel_, voxel_map_, (int)publish_rgb_);
  }

private:
  using SubImg  = message_filters::Subscriber<Image>;
  using SubInfo = message_filters::Subscriber<CameraInfo>;
  using Policy  = message_filters::sync_policies::ApproximateTime<Image, CameraInfo>;
  using SyncL   = message_filters::Synchronizer<Policy>;
  using SyncR   = message_filters::Synchronizer<Policy>;

  enum class Side { LEFT, RIGHT };

  void cbLeft(const Image::ConstSharedPtr depth, const CameraInfo::ConstSharedPtr info)
  { processOne(depth, info, Side::LEFT); }

  void cbRight(const Image::ConstSharedPtr depth, const CameraInfo::ConstSharedPtr info)
  { processOne(depth, info, Side::RIGHT); }

  bool lookupTfTargetFromCam(
    const std::string &cam_frame,
    const builtin_interfaces::msg::Time &stamp_in,
    geometry_msgs::msg::TransformStamped &out_tf)
  {
    builtin_interfaces::msg::Time stamp = stamp_in;

    for (int i = 0; i < tf_retries_; ++i)
    {
      try
      {
        const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_);

        if (!tf_buffer_->canTransform(target_frame_, cam_frame, stamp, timeout))
        {
          continue;
        }

        out_tf = tf_buffer_->lookupTransform(target_frame_, cam_frame, stamp);
        return true;
      }
      catch (const tf2::ExtrapolationException &)
      {
        // 回退一点点时间，避免 “未来外推”
        rclcpp::Time t(stamp.sec, stamp.nanosec, RCL_ROS_TIME);
        t = t - rclcpp::Duration(0, tf_backoff_ms_ * 1000 * 1000);

        const int64_t ns = t.nanoseconds();
        stamp.sec     = static_cast<int32_t>(ns / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        continue;
      }
      catch (const tf2::LookupException &)
      {
        return false; // 树断了/目标 frame 不存在
      }
      catch (const std::exception &)
      {
        return false;
      }
    }
    return false;
  }

  void processOne(const Image::ConstSharedPtr depth, const CameraInfo::ConstSharedPtr info, Side side)
  {
    // Only 32FC1
    if (depth->encoding != "32FC1")
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Depth encoding is %s (expect 32FC1)", depth->encoding.c_str());
      return;
    }

    const int h = static_cast<int>(depth->height);
    const int w = static_cast<int>(depth->width);
    if (h <= 0 || w <= 0) return;

    const double fx = info->k[0];
    const double fy = info->k[4];
    const double cx = info->k[2];
    const double cy = info->k[5];
    if (fx <= 1e-9 || fy <= 1e-9) return;

    const std::string cam_frame = depth->header.frame_id;

    geometry_msgs::msg::TransformStamped tf;
    if (!lookupTfTargetFromCam(cam_frame, depth->header.stamp, tf))
    {
      std::lock_guard<std::mutex> lk(mtx_stats_);
      tf_miss_++;
      return;
    }

    // 统计：TF 命中
    {
      std::lock_guard<std::mutex> lk(mtx_stats_);
      tf_hit_++;
    }

    // Eigen transform
    Eigen::Quaterniond q(
      tf.transform.rotation.w,
      tf.transform.rotation.x,
      tf.transform.rotation.y,
      tf.transform.rotation.z
    );
    q.normalize();
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d t(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);

    // depth raw pointer
    const size_t step_bytes = depth->step;
    const uint8_t *base = depth->data.data();

    // 增量云：用于 /dense_cloud_inc（也用于并入地图）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inc_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    inc_rgb->reserve(static_cast<size_t>((h / sample_step_) * (w / sample_step_)));

    // 左蓝右红
    const uint8_t cr = (side == Side::LEFT)  ? 30  : 255;
    const uint8_t cg = (side == Side::LEFT)  ? 60  : 30;
    const uint8_t cb = (side == Side::LEFT)  ? 255 : 30;

    for (int v = 0; v < h; v += sample_step_)
    {
      const float *row = reinterpret_cast<const float*>(base + static_cast<size_t>(v) * step_bytes);
      for (int u = 0; u < w; u += sample_step_)
      {
        float z = row[u];
        if (!isFinite(z) || z < static_cast<float>(min_z_) || z > static_cast<float>(max_z_)) continue;

        const double x = (static_cast<double>(u) - cx) * static_cast<double>(z) / fx;
        const double y = (static_cast<double>(v) - cy) * static_cast<double>(z) / fy;

        Eigen::Vector3d pc(x, y, static_cast<double>(z));
        Eigen::Vector3d pw = R * pc + t;

        pcl::PointXYZRGB p;
        p.x = static_cast<float>(pw.x());
        p.y = static_cast<float>(pw.y());
        p.z = static_cast<float>(pw.z());
        p.r = cr; p.g = cg; p.b = cb;
        inc_rgb->push_back(p);
      }
    }

    if (inc_rgb->empty()) return;

    // 对增量做一次轻量 voxel（可选但强烈建议）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inc_rgb_ds(new pcl::PointCloud<pcl::PointXYZRGB>());
    {
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(inc_rgb);
      vg.setLeafSize(static_cast<float>(voxel_), static_cast<float>(voxel_), static_cast<float>(voxel_));
      vg.filter(*inc_rgb_ds);
    }
    if (inc_rgb_ds->empty()) return;

    // 并入地图 + 记录最后时间戳
    {
      std::lock_guard<std::mutex> lk(mtx_);
      *map_rgb_ += *inc_rgb_ds;

      last_stamp_ = depth->header.stamp;
      has_last_stamp_ = true;
    }

    // 保存一份增量给发布线程（避免占用回调线程做 toROSMsg）
    {
      std::lock_guard<std::mutex> lk(mtx_inc_);
      *inc_rgb_buf_ += *inc_rgb_ds;
    }
  }

  void publishClouds()
  {
    // 1) 取出增量
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inc(new pcl::PointCloud<pcl::PointXYZRGB>());
    {
      std::lock_guard<std::mutex> lk(mtx_inc_);
      if (!inc_rgb_buf_->empty()) {
        *inc = *inc_rgb_buf_;
        inc_rgb_buf_->clear();
      }
    }

    // 2) 取 stamp / 地图
    builtin_interfaces::msg::Time stamp;
    bool has_stamp = false;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_copy(new pcl::PointCloud<pcl::PointXYZRGB>());
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (has_last_stamp_) {
        stamp = last_stamp_;
        has_stamp = true;
      }
      if (!map_rgb_->empty()) {
        *map_copy = *map_rgb_;
      }
    }

    if (!has_stamp) {
      publishStatus("waiting for first frame...");
      return;
    }

    // 3) 发布增量（用于 RViz 看“实时流”）
    if (!inc->empty()) {
      PointCloud2 msg_inc;
      pcl::toROSMsg(*inc, msg_inc);
      msg_inc.header.frame_id = target_frame_;
      msg_inc.header.stamp = stamp;          // 关键：跟 TF 时间对齐
      pub_inc_->publish(msg_inc);
    }

    // 4) 地图控点：每 N 次发布做一次 voxel_map_（避免无限增长）
    publish_count_++;
    if ((publish_count_ % downsample_every_n_) == 0 && !map_copy->empty())
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_ds(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::VoxelGrid<pcl::PointXYZRGB> vg;
      vg.setInputCloud(map_copy);
      vg.setLeafSize(static_cast<float>(voxel_map_), static_cast<float>(voxel_map_), static_cast<float>(voxel_map_));
      vg.filter(*map_ds);

      // 进一步粗暴控点（超上限就再滤一次 / 或你可以做裁剪）
      if (map_ds->size() > max_points_) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_ds2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> vg2;
        vg2.setInputCloud(map_ds);
        const float v2 = static_cast<float>(voxel_map_ * 1.3);
        vg2.setLeafSize(v2, v2, v2);
        vg2.filter(*map_ds2);
        map_ds = map_ds2;
      }

      // 写回主地图
      {
        std::lock_guard<std::mutex> lk(mtx_);
        *map_rgb_ = *map_ds;
        *map_copy = *map_ds;
      }
    }

    // 5) 发布地图：RGB 或 XYZ
    if (!map_copy->empty())
    {
      // RGB 地图
      if (publish_rgb_ && pub_map_rgb_) {
        PointCloud2 msg_map_rgb;
        pcl::toROSMsg(*map_copy, msg_map_rgb);
        msg_map_rgb.header.frame_id = target_frame_;
        msg_map_rgb.header.stamp = stamp;
        pub_map_rgb_->publish(msg_map_rgb);
      }

      // 兼容：再发布一个 XYZ（有些工具只想要 XYZ）
      pcl::PointCloud<pcl::PointXYZ>::Ptr map_xyz(new pcl::PointCloud<pcl::PointXYZ>());
      map_xyz->reserve(map_copy->size());
      for (const auto &p : map_copy->points) {
        map_xyz->push_back(pcl::PointXYZ(p.x, p.y, p.z));
      }
      PointCloud2 msg_map;
      pcl::toROSMsg(*map_xyz, msg_map);
      msg_map.header.frame_id = target_frame_;
      msg_map.header.stamp = stamp;
      pub_map_->publish(msg_map);
    }

    // 6) 状态输出（不再盲飞）
    publishStatus();
  }

  void publishStatus(const std::string &extra = "")
  {
    size_t map_pts = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      map_pts = map_rgb_->size();
    }

    uint64_t hit = 0, miss = 0;
    {
      std::lock_guard<std::mutex> lk(mtx_stats_);
      hit = tf_hit_;
      miss = tf_miss_;
    }

    const double ratio = (hit + miss) ? (100.0 * double(hit) / double(hit + miss)) : 0.0;

    std::ostringstream ss;
    ss << "target_frame=" << target_frame_
       << " map_pts=" << map_pts
       << " tf_hit=" << hit
       << " tf_miss=" << miss
       << " hit_rate=" << ratio << "%"
       << " voxel=" << voxel_
       << " voxel_map=" << voxel_map_
       << " sample_step=" << sample_step_
       << " max_points=" << max_points_;
    if (!extra.empty()) ss << " | " << extra;

    std_msgs::msg::String msg;
    msg.data = ss.str();
    pub_status_->publish(msg);
  }

private:
  // Params
  std::string target_frame_;

  std::string left_depth_topic_, left_info_topic_;
  std::string right_depth_topic_, right_info_topic_;

  std::string pub_map_topic_;
  std::string pub_inc_topic_;
  std::string pub_map_rgb_topic_;

  int sample_step_;
  double min_z_, max_z_;
  double voxel_;
  double voxel_map_;
  double publish_hz_;

  double tf_timeout_;
  int tf_backoff_ms_;
  int tf_retries_;

  bool publish_rgb_{true};
  size_t max_points_{2500000};
  int downsample_every_n_{1};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub/Timer
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_inc_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_map_rgb_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  uint64_t publish_count_{0};

  // Sync
  SubImg sub_l_depth_, sub_r_depth_;
  SubInfo sub_l_info_, sub_r_info_;
  std::shared_ptr<SyncL> sync_l_;
  std::shared_ptr<SyncR> sync_r_;

  // Map accumulation (RGB)
  std::mutex mtx_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_rgb_{new pcl::PointCloud<pcl::PointXYZRGB>()};

  // Increment buffer for publishing
  std::mutex mtx_inc_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr inc_rgb_buf_{new pcl::PointCloud<pcl::PointXYZRGB>()};

  // Last stamp (critical for TF alignment)
  builtin_interfaces::msg::Time last_stamp_;
  bool has_last_stamp_{false};

  // Stats
  std::mutex mtx_stats_;
  uint64_t tf_hit_{0};
  uint64_t tf_miss_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DualDenseMapper>());
  rclcpp::shutdown();
  return 0;
}
