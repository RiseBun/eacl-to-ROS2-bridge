#include "vk_ros2_driver/vk_ros2_driver.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

// 必须包含 QoS 相关的头文件
#include <rclcpp/qos.hpp>
#include <rmw/qos_profiles.h>

namespace vkc
{
using nlohmann::json;

// =================================================================================
// 静态工具函数
// =================================================================================

static inline std::string strip_leading_slash(std::string s) {
  while (!s.empty() && s.front() == '/') s.erase(s.begin());
  return s;
}

static std::string clean_topic_name(std::string t) {
  const std::string suffix = "/hfflow";
  auto pos = t.rfind(suffix);
  if (pos != std::string::npos) t = t.substr(0, pos);
  if (t.rfind("S1/", 0) == 0) t = t.substr(3);
  if (!t.empty() && t[0] == '/') t = t.substr(1);
  return t;
}

static std::string clean_frame_name(std::string t) {
  const std::string suffix = "/hfflow";
  auto pos = t.rfind(suffix);
  if (pos != std::string::npos) t = t.substr(0, pos);
  return strip_leading_slash(t);
}

static Eigen::Isometry3d parse_T(const json& T) {
  Eigen::Quaterniond q(
    T.value("qw", 1.0), T.value("qx", 0.0), T.value("qy", 0.0), T.value("qz", 0.0)
  );
  q.normalize();

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = q.toRotationMatrix();
  iso.translation() = Eigen::Vector3d(
    T.value("px", 0.0), T.value("py", 0.0), T.value("pz", 0.0)
  );
  return iso;
}

// =================================================================================
// CalibrationManager
// =================================================================================

bool CalibrationManager::load(const std::string &saved,
                              const std::string &left,
                              const std::string &right,
                              rclcpp::Logger logger)
{
  map_.clear();
  json j;
  try {
    std::ifstream ifs(saved);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(logger, "Cannot open saved_calib: %s", saved.c_str());
      return false;
    }
    ifs >> j;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger, "Exception reading calib: %s", e.what());
    return false;
  }

  if (!j.contains("value0")) {
    RCLCPP_ERROR(logger, "saved_calib missing value0");
    return false;
  }
  auto &v0 = j["value0"];
  auto cam_names = v0["cam_names"];
  auto res = v0["resolution"];
  auto intrs = v0["intrinsics"];

  double base_l = 0.0, base_r = 0.0;
  try {
    json jl;
    std::ifstream fl(left);
    if (fl.is_open()) {
      fl >> jl;
      base_l = jl.value("baseline", 0.0);
    }
  } catch (...) {}
  try {
    json jr;
    std::ifstream fr(right);
    if (fr.is_open()) {
      fr >> jr; 
      base_r = jr.value("baseline", 0.0);
    }
  } catch (...) {}

  for (size_t i = 0; i < cam_names.size(); ++i) {
    std::string raw = cam_names[i].get<std::string>();
    std::string key = clean_topic_name(raw);

    StereoCalib C;
    C.width  = res[i][0].get<int>();
    C.height = res[i][1].get<int>();

    auto intr = intrs[i]["intrinsics"];
    C.fx = intr.value("fx", 0.0);
    C.fy = intr.value("fy", 0.0);
    C.cx = intr.value("cx", 0.0);
    C.cy = intr.value("cy", 0.0);
    C.model = intrs[i].value("camera_type", std::string("pinhole"));

    if (C.model == "kb4") {
      C.dist = {
        intr.value("k1", 0.0),
        intr.value("k2", 0.0),
        intr.value("k3", 0.0),
        intr.value("k4", 0.0)
      };
    } else if (C.model == "ds") {
      C.dist = {
        intr.value("xi", 0.0),
        intr.value("alpha", 0.0)
      };
    }

    if (key.find("stereo1_l") != std::string::npos) {
      C.baseline = base_l;
    } else if (key.find("stereo2_r") != std::string::npos) {
      C.baseline = base_r;
    }

    C.valid = true;
    map_[key] = C;

    RCLCPP_INFO(logger,
                "Calib loaded: key=%s, fx=%.3f, fy=%.3f, baseline=%.6f, size=%dx%d",
                key.c_str(), C.fx, C.fy, C.baseline, C.width, C.height);
  }

  return true;
}

// =================================================================================
// Receiver Implementations
// =================================================================================

// ---------------------- ImuReceiver ----------------------
vkc::ReceiverStatus ImuReceiver::handle(const vkc::Message<vkc::Shared<vkc::Imu>> &message) {
    sensor_msgs::msg::Imu imu_ros_message;
    auto imu = message.payload.reader();
    
    imu_ros_message.header.stamp = driver_.stamp_from(imu.getHeader());
    imu_ros_message.header.frame_id = frame_id_;

    imu_ros_message.linear_acceleration.x = imu.getLinearAcceleration().getX();
    imu_ros_message.linear_acceleration.y = imu.getLinearAcceleration().getY();
    imu_ros_message.linear_acceleration.z = imu.getLinearAcceleration().getZ();
    imu_ros_message.angular_velocity.x = imu.getAngularVelocity().getX();
    imu_ros_message.angular_velocity.y = imu.getAngularVelocity().getY();
    imu_ros_message.angular_velocity.z = imu.getAngularVelocity().getZ();

    auto body = imu.getExtrinsic().getBodyFrame(); 
    imu_ros_message.orientation.x = body.getOrientation().getX();
    imu_ros_message.orientation.y = body.getOrientation().getY();
    imu_ros_message.orientation.z = body.getOrientation().getZ();
    imu_ros_message.orientation.w = body.getOrientation().getW();

    pub_->publish(imu_ros_message);

    return vkc::ReceiverStatus::Open;
}

// ---------------------- OdometryReceiver ----------------------
vkc::ReceiverStatus OdometryReceiver::handle(const vkc::Message<vkc::Shared<vkc::Odometry3d>> &message) {
    nav_msgs::msg::Odometry odometry_ros_message;
    auto odometry = message.payload.reader();
    
    odometry_ros_message.header.stamp = driver_.stamp_from(odometry.getHeader());
    odometry_ros_message.header.frame_id = driver_.odom_frame_; 
    odometry_ros_message.child_frame_id = driver_.base_link_frame_;

    // Pose
    odometry_ros_message.pose.pose.position.x = odometry.getPose().getPosition().getX();
    odometry_ros_message.pose.pose.position.y = odometry.getPose().getPosition().getY();
    odometry_ros_message.pose.pose.position.z = odometry.getPose().getPosition().getZ();
    odometry_ros_message.pose.pose.orientation.x = odometry.getPose().getOrientation().getX();
    odometry_ros_message.pose.pose.orientation.y = odometry.getPose().getOrientation().getY();
    odometry_ros_message.pose.pose.orientation.z = odometry.getPose().getOrientation().getZ();
    odometry_ros_message.pose.pose.orientation.w = odometry.getPose().getOrientation().getW();
    
    // Twist
    odometry_ros_message.twist.twist.linear.x = odometry.getTwist().getLinear().getX();
    odometry_ros_message.twist.twist.linear.y = odometry.getTwist().getLinear().getY();
    odometry_ros_message.twist.twist.linear.z = odometry.getTwist().getLinear().getZ();
    odometry_ros_message.twist.twist.angular.x = odometry.getTwist().getAngular().getX();
    odometry_ros_message.twist.twist.angular.y = odometry.getTwist().getAngular().getY();
    odometry_ros_message.twist.twist.angular.z = odometry.getTwist().getAngular().getZ();

    // Covariance
    auto poseCov = odometry.getPoseCovariance();
    for (size_t i = 0; i < poseCov.size(); i++) {
        odometry_ros_message.pose.covariance[i]=poseCov[i];
    }
    auto twistCov = odometry.getTwistCovariance();
    for (size_t i = 0; i < twistCov.size(); i++) {
        odometry_ros_message.twist.covariance[i]=twistCov[i];
    }
    
    pub_->publish(odometry_ros_message);

    // TF
    if (driver_.publish_tf_) {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = odometry_ros_message.header.stamp;
        odom_tf.header.frame_id = odometry_ros_message.header.frame_id;
        odom_tf.child_frame_id = odometry_ros_message.child_frame_id;
        odom_tf.transform.translation.x = odometry_ros_message.pose.pose.position.x;
        odom_tf.transform.translation.y = odometry_ros_message.pose.pose.position.y;
        odom_tf.transform.translation.z = odometry_ros_message.pose.pose.position.z;
        odom_tf.transform.rotation = odometry_ros_message.pose.pose.orientation;
        
        driver_.tf_pub_->sendTransform(odom_tf);
    }

    return vkc::ReceiverStatus::Open;
}

// ---------------------- ImageReceiver ----------------------
vkc::ReceiverStatus ImageReceiver::handle(const vkc::Message<vkc::Shared<vkc::Image>> &message) {
    cv::Mat imageMat;
    auto image = message.payload.reader();
    std_msgs::msg::Header header;
    
    header.stamp = driver_.stamp_from(image.getHeader());
    header.frame_id = topic_name_; 
    
    uint32_t imageHeight = image.getHeight();
    uint32_t imageWidth = image.getWidth();
    long imageSize = image.getData().size();
    auto imageEncoding = image.getEncoding();

    switch (imageEncoding)
    {
    case vkc::Image::Encoding::MONO8:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_8UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_GRAY2RGB);
        break;
    case vkc::Image::Encoding::MONO16:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_16UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_GRAY2RGB);
        break;
    case vkc::Image::Encoding::YUV420:
        imageMat = cv::Mat(imageHeight * 3 / 2, imageWidth, CV_8UC1,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        cv::cvtColor(imageMat, imageMat, cv::COLOR_YUV2BGR_IYUV);
        break;
    case vkc::Image::Encoding::BGR8:
        imageMat = cv::Mat(imageHeight, imageWidth, CV_8UC3,
                            const_cast<unsigned char *>(image.getData().asBytes().begin()));
        imageMat = imageMat.reshape(1, imageHeight);
        // 【关键修复】不做转换，保留 BGR，与编码 "bgr8" 匹配
        break;
    case vkc::Image::Encoding::JPEG:
        imageMat = cv::imdecode(cv::Mat(1, imageSize, CV_8UC1,
                                        const_cast<unsigned char *>(image.getData().asBytes().begin())),
                               cv::IMREAD_COLOR);
        break;
    default:
        RCLCPP_WARN(driver_.get_logger(), "Unsupported image encoding");
        return vkc::ReceiverStatus::Closed;
    }

    StereoCalib C;
    if (driver_.calib().get(clean_key_, C) && C.valid) {
      sensor_msgs::msg::CameraInfo ci;
      ci.header = header; 
      ci.width = imageWidth;
      ci.height = imageHeight;
      
       double sx = static_cast<double>(imageWidth) / C.width;
       double sy = static_cast<double>(imageHeight) / C.height;
       double fx = C.fx * sx;
       double fy = C.fy * sy;
       double cx = C.cx * sx;
       double cy = C.cy * sy;

       ci.k[0] = fx; ci.k[1] = 0;  ci.k[2] = cx;
       ci.k[3] = 0;  ci.k[4] = fy; ci.k[5] = cy;
       ci.k[6] = 0;  ci.k[7] = 0;  ci.k[8] = 1;

       if (C.model == "kb4") {
         ci.distortion_model = "equidistant";
         ci.d = C.dist;
       } else if (C.model == "pinhole") {
         ci.distortion_model = "plumb_bob";
         ci.d = {0, 0, 0, 0, 0};
       } else {
         ci.distortion_model = "plumb_bob";
         ci.d = {0, 0, 0, 0, 0};
       }
       
       // 【关键建议】显式初始化 R 为单位阵
       ci.r = {1,0,0, 0,1,0, 0,0,1};

       ci.p[0] = fx; ci.p[1] = 0;  ci.p[2] = cx; ci.p[3]  = 0;
       ci.p[4] = 0;  ci.p[5] = fy; ci.p[6] = cy; ci.p[7]  = 0;
       ci.p[8] = 0;  ci.p[9] = 0;  ci.p[10]= 1;  ci.p[11] = 0;

       bool is_right = (clean_key_.find("_r") != std::string::npos || clean_key_.find("right") != std::string::npos);
       if (is_right && C.baseline > 1e-6) {
         ci.p[3] = -fx * C.baseline;
       }

      ci_pub_->publish(ci);
    }
    
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "bgr8", imageMat).toImageMsg();
    img_pub_.publish(*msg);
    
    return vkc::ReceiverStatus::Open;
}

// ---------------------- DisparityReceiver ----------------------
// 【优化】构造函数传值，初始化成员对象
DisparityReceiver::DisparityReceiver(const VkRos2Driver &driver,
                                     image_transport::Publisher pub,
                                     image_transport::Publisher pub_mm,
                                     const std::string &topic_name)
  : driver_(driver),
    pub_(pub),
    pub_mm_(pub_mm),
    topic_name_(topic_name),
    disparity_scale_(32.0)
{
  clean_key_ = clean_topic_name(topic_name_);
  const std::string disp_suffix = "/disparity";
  auto pos_ck = clean_key_.rfind(disp_suffix);
  if (pos_ck != std::string::npos) {
    clean_key_ = clean_key_.substr(0, pos_ck);
  }

  camera_frame_ = strip_leading_slash(topic_name_);
  auto pos_cf = camera_frame_.rfind(disp_suffix);
  if (pos_cf != std::string::npos) {
    camera_frame_ = camera_frame_.substr(0, pos_cf);
  }
}

vkc::ReceiverStatus DisparityReceiver::handle(const vkc::Message<vkc::Shared<vkc::Disparity>> &message) {
    auto d = message.payload.reader();
    const int src_H = static_cast<int>(d.getHeight());
    const int src_W = static_cast<int>(d.getWidth());

    StereoCalib C;
    if (!driver_.calib().get(clean_key_, C) || !C.valid || C.baseline <= 1e-6) {
        return vkc::ReceiverStatus::Open;
    }

    // 1. 获取 Disparity 数据
    int src_type = (d.getEncoding() == Disparity::Encoding::DISPARITY8) ? CV_8UC1 : CV_16UC1;
    cv::Mat disp_raw(src_H, src_W, src_type, const_cast<unsigned char*>(d.getData().asBytes().begin()));

    const float fx_B = static_cast<float>(C.fx * C.baseline);
    const float scale = disparity_scale_;

    cv::Mat disp_f;
    cv::Mat depth_meters;

    if (src_type == CV_8UC1) {
        disp_raw.convertTo(disp_f, CV_32FC1, 1.0 / scale);
    } else {
        disp_raw.convertTo(disp_f, CV_32FC1, 1.0 / scale);
    }

    // =================================================================================
    // 【Bug Fix 1】Invalid Mask 计算顺序修复
    // 先计算 Mask (小于阈值认为无效)，然后再做 max 截断
    // =================================================================================
    cv::Mat invalid_mask = (disp_f < 1e-6f); 
    
    // 防止除零
    cv::max(disp_f, 1e-6f, disp_f);
    
    // 计算深度 (Meters)
    cv::divide(fx_B, disp_f, depth_meters);

    // 将无效区域置 0
    depth_meters.setTo(0.0f, invalid_mask);

    // =================================================================================
    // 2. Resize to Color Resolution
    // =================================================================================
    cv::Mat depth_final_meters;
    
    if (C.width > 0 && C.height > 0 && (src_W != C.width || src_H != C.height)) {
        cv::resize(depth_meters, depth_final_meters, cv::Size(C.width, C.height), 0, 0, cv::INTER_NEAREST);
    } else {
        depth_final_meters = depth_meters;
    }

    // 准备 Header
    std_msgs::msg::Header header;
    header.stamp = driver_.stamp_from(d.getHeader());
    header.frame_id = camera_frame_;

    // =================================================================================
    // 3. 发布 32FC1 (Meters)
    // =================================================================================
    auto msg_float = cv_bridge::CvImage(header, "32FC1", depth_final_meters).toImageMsg();
    pub_.publish(*msg_float);

    // =================================================================================
    // 4. 发布 16UC1 (Millimeters)
    // =================================================================================
    cv::Mat depth_mm;
    {
        // =============================================================================
        // 【Bug Fix 2】废弃 unsafe 的 copyTo，改用 clone + setTo(0) 处理 NaN
        // =============================================================================
        cv::Mat depth_safe = depth_final_meters.clone();
        
        // 检查 NaN/Inf
        cv::Mat finite_mask = (depth_safe == depth_safe); // NaN != NaN -> False
        
        // 将 NaN 区域置 0
        depth_safe.setTo(0.0f, ~finite_mask);

        // 截断过远距离 (防止溢出 uint16)
        cv::min(depth_safe, 65.535f, depth_safe);

        // 转换
        depth_safe.convertTo(depth_mm, CV_16UC1, 1000.0);
    }

    auto msg_u16 = cv_bridge::CvImage(header, "16UC1", depth_mm).toImageMsg();
    pub_mm_.publish(*msg_u16);

    return vkc::ReceiverStatus::Open;
}

// =================================================================================
// Driver Node
// =================================================================================

// publish_static_extrinsics_from_saved 函数保持不变...
void VkRos2Driver::publish_static_extrinsics_from_saved(const std::string& saved_path)
{
    std::ifstream f(saved_path);
    if (!f.good()) {
        RCLCPP_ERROR(get_logger(), "Could not open saved_calib file: %s", saved_path.c_str());
        return;
    }
    json j;
    try {
        f >> j;
    } catch (const json::parse_error& e) {
        RCLCPP_ERROR(get_logger(), "Failed to parse saved_calib JSON: %s", e.what());
        return;
    }
    if (!j.contains("value0")) return;
    auto& v = j["value0"];
    Eigen::Isometry3d T_imu_body = parse_T(v["T_imu_body"]);
    Eigen::Isometry3d T_body_imu = T_imu_body.inverse();
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
    auto cam_names = v["cam_names"];
    auto T_imu_cams = v["T_imu_cam"];

    for (size_t i = 0; i < cam_names.size() && i < T_imu_cams.size(); ++i) {
        std::string raw_name = cam_names[i].get<std::string>();
        std::string child_frame = clean_frame_name(raw_name); 
        Eigen::Isometry3d T_imu_cam = parse_T(T_imu_cams[i]);
        Eigen::Isometry3d T_body_cam = T_body_imu * T_imu_cam;
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now(); 
        t.header.frame_id = base_link_frame_; 
        t.child_frame_id = child_frame;       
        Eigen::Vector3d trans = T_body_cam.translation();
        Eigen::Quaterniond rot(T_body_cam.rotation());
        t.transform.translation.x = trans.x();
        t.transform.translation.y = trans.y();
        t.transform.translation.z = trans.z();
        t.transform.rotation.x = rot.x();
        t.transform.rotation.y = rot.y();
        t.transform.rotation.z = rot.z();
        t.transform.rotation.w = rot.w();
        static_transforms.push_back(t);
        RCLCPP_INFO(get_logger(), "Static TF: %s -> %s", base_link_frame_.c_str(), child_frame.c_str());
    }
    if (!static_transforms.empty()) {
        static_tf_pub_->sendTransform(static_transforms);
    }
}

VkRos2Driver::VkRos2Driver(const rclcpp::NodeOptions &options)
    : Node("vk_ros2_driver", options),
      visualkit(std::move(vkc::VisualKit::create(std::nullopt))),
      it_(std::shared_ptr<VkRos2Driver>(this, [](auto *) {})) 
{
    this->declare_parameter("imu_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("odometry_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("image_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter("disparity_topics", rclcpp::PARAMETER_STRING_ARRAY);
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<std::string>("odometry_frame", "odom");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter<std::string>("saved_calib", "/home/li/pc_debug/saved_calib.json");
    this->declare_parameter<std::string>("disp_left",  "/home/li/pc_debug/1765404945733167282_disparity_calib_left.json");
    this->declare_parameter<std::string>("disp_right", "/home/li/pc_debug/1765404945733167282_disparity_calib_right.json");
    
    auto imu_topics = this->get_parameter_or<std::vector<std::string>>("imu_topics", {});
    auto odometry_topics = this->get_parameter_or<std::vector<std::string>>("odometry_topics", {});
    auto image_topics = this->get_parameter_or<std::vector<std::string>>("image_topics", {});
    auto disparity_topics = this->get_parameter_or<std::vector<std::string>>("disparity_topics", {});

    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    odom_frame_ = this->get_parameter("odometry_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();

    std::string saved = get_parameter("saved_calib").as_string();
    std::string left  = get_parameter("disp_left").as_string();
    std::string right = get_parameter("disp_right").as_string();
    calib_.load(saved, left, right, get_logger());

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    publish_static_extrinsics_from_saved(saved);

    vkc::installLoggingCallback(std::bind(
        &vkc::VkRos2Driver::log_cb, this, std::placeholders::_1, std::placeholders::_2));

    // =================================================================================
    // 【QoS】Reliable
    // =================================================================================
    
    // ImageTransport Profile
    rmw_qos_profile_t qos_image_reliable = rmw_qos_profile_sensor_data;
    qos_image_reliable.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_image_reliable.history     = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos_image_reliable.depth       = 10;
    qos_image_reliable.durability  = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    // Standard Publisher Profile
    rclcpp::QoS qos_reliable(rclcpp::KeepLast(10));
    qos_reliable.reliable();
    qos_reliable.durability_volatile();

    for (const auto& topic : imu_topics) {
        imu_pub_[topic] = this->create_publisher<sensor_msgs::msg::Imu>(topic, qos_reliable);
        visualkit->source().install(topic, std::make_unique<vkc::ImuReceiver>(
            *this, imu_pub_[topic], topic));
    }
    for (const auto& topic : odometry_topics) {
        odom_pub_[topic] = this->create_publisher<nav_msgs::msg::Odometry>(topic, qos_reliable);
        visualkit->source().install(topic, std::make_unique<vkc::OdometryReceiver>(
            *this, odom_pub_[topic]));
    }
    for (const auto& topic : image_topics) {
        img_pub_[topic] = it_.advertise(topic, qos_image_reliable);
        
        ci_pub_[topic] = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            topic + "/camera_info", qos_reliable);
        
        std::string clean_key = clean_topic_name(topic);
        visualkit->source().install(topic, std::make_unique<vkc::ImageReceiver>(
            *this, img_pub_[topic], ci_pub_[topic], topic, clean_key));
    }
    for (const auto& topic : disparity_topics) {
        std::string depth_topic = topic;
        const std::string suffix = "/disparity";
        auto pos = depth_topic.rfind(suffix);
        if (pos != std::string::npos) {
            depth_topic.replace(pos, suffix.size(), "/depth");
        } else {
            depth_topic += "/depth";
        }

        // 1. Meters (32FC1)
        disp_pub_[topic] = it_.advertise(depth_topic, qos_image_reliable);
        
        // 2. Millimeters (16UC1)
        std::string depth_topic_mm = depth_topic + "_mm";
        disp_pub_mm_[topic] = it_.advertise(depth_topic_mm, qos_image_reliable); // 这里会用到头文件里的 map 定义

        visualkit->source().install(topic, std::make_unique<vkc::DisparityReceiver>(
            *this, 
            disp_pub_[topic], 
            disp_pub_mm_[topic], 
            topic));
            
        RCLCPP_INFO(get_logger(), "[Depth] Publishing:\n  Float(m): %s\n  Uint16(mm): %s", 
            depth_topic.c_str(), depth_topic_mm.c_str());
    }
    visualkit->source().start();
}

VkRos2Driver::~VkRos2Driver() {
    visualkit->source().stop();
}

void VkRos2Driver::log_cb(vkc::LogLevel level, std::string_view message) {
    switch (level) {
        case vkc::LogLevel::TRACE:
            RCLCPP_DEBUG(get_logger(), "%.*s", (int)message.size(), message.data());
            break;
        case vkc::LogLevel::DEBUG:
            RCLCPP_DEBUG(get_logger(), "%.*s", (int)message.size(), message.data());
            break;
        case vkc::LogLevel::INFO:
            RCLCPP_INFO(get_logger(), "%.*s", (int)message.size(), message.data());
            break;
        case vkc::LogLevel::WARN:
            RCLCPP_WARN(get_logger(), "%.*s", (int)message.size(), message.data());
            break;
        case vkc::LogLevel::ERROR:
            RCLCPP_ERROR(get_logger(), "%.*s", (int)message.size(), message.data());
            break;
        default:
            RCLCPP_ERROR(get_logger(), "Invalid log level");
    }
}

} // namespace vkc

RCLCPP_COMPONENTS_REGISTER_NODE(vkc::VkRos2Driver)