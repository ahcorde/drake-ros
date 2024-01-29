#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/camera_common.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace drake_ros_examples
{
class SyncCamerasPointCloud : public rclcpp::Node
{
  using ExactPolicy = message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::PointCloud2>;
  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::PointCloud2>;

  using ExactSync = message_filters::Synchronizer<ExactPolicy>;
  using ApproximateSync = message_filters::Synchronizer<ApproximatePolicy>;

public:
  explicit SyncCamerasPointCloud(rclcpp::NodeOptions options)
  : Node("sync_cameras_point_cloud_node", options)
  {
    using namespace std::placeholders;

    this->declare_parameter("use_system_default_qos", false);
    bool exact_sync = this->declare_parameter("exact_sync", true);

    if(exact_sync)
    {
      this->exact_sync_.reset(
            new ExactSync(
              ExactPolicy(40),
              sub_color_image_, sub_depth_image_,
              sub_color_info_, sub_depth_info_,
              sub_points_));
      this->exact_sync_->registerCallback(
        std::bind(&SyncCamerasPointCloud::callback, this, _1, _2, _3, _4, _5));
    } else {
      approximate_sync_.reset(
        new ApproximateSync(
          ApproximatePolicy(40),
          sub_color_image_, sub_depth_image_,
          sub_color_info_, sub_depth_info_,
          sub_points_));
      approximate_sync_->registerCallback(
        std::bind(&SyncCamerasPointCloud::callback, this, _1, _2, _3, _4, _5));
    }

    const bool use_system_default_qos =
      this->get_parameter("use_system_default_qos").as_bool();
    rclcpp::QoS image_sub_qos = rclcpp::SensorDataQoS();
    if (use_system_default_qos) {
      image_sub_qos = rclcpp::SystemDefaultsQoS();
    }
    const auto image_sub_rmw_qos = image_sub_qos.get_rmw_qos_profile();

    // For compressed topics to remap appropriately, we need to pass a
    // fully expanded and remapped topic name to image_transport
    auto node_base = this->get_node_base_interface();
    std::string color_topic =
      node_base->resolve_topic_or_service_name("/color/image_raw", false);
    std::string depth_topic =
      node_base->resolve_topic_or_service_name("/depth/image_raw", false);
    std::string camera_info_color_topic =
      node_base->resolve_topic_or_service_name("/color/camera_info", false);
    std::string camera_info_depth_topic =
      node_base->resolve_topic_or_service_name("/depth/camera_info", false);
    std::string points_topic =
      node_base->resolve_topic_or_service_name("/points", false);

    // Setup hints and QoS overrides
    image_transport::TransportHints hints(this);
    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();

    sub_color_image_.subscribe(
      this, color_topic, hints.getTransport(), image_sub_rmw_qos, sub_opts);
    sub_depth_image_.subscribe(
      this, depth_topic, hints.getTransport(), image_sub_rmw_qos, sub_opts);
    sub_color_info_.subscribe(this, camera_info_color_topic, image_sub_rmw_qos, sub_opts);
    sub_depth_info_.subscribe(this, camera_info_depth_topic, image_sub_rmw_qos, sub_opts);
    sub_points_.subscribe(this, points_topic, image_sub_rmw_qos, sub_opts);

    publisher_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points_sync", rclcpp::SensorDataQoS());
    publisher_color_ = create_publisher<sensor_msgs::msg::Image>("/color_sync/image_raw", rclcpp::SensorDataQoS());
    publisher_depth_ = create_publisher<sensor_msgs::msg::Image>("/depth_sync/image_raw", rclcpp::SensorDataQoS());
    publisher_color_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/color_sync/camera_info", rclcpp::SensorDataQoS());
    publisher_depth_camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("/depth_sync/camera_info", rclcpp::SensorDataQoS());

    auto timer_callback =
      [this]() -> void {
        RCLCPP_INFO(this->get_logger(), "Hz synchronizer: '%d'", count_hz);
        count_hz = 0;
      };
    timer_ = this->create_wall_timer(1000ms, timer_callback);
  }

void callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & color_image_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_image_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & color_info_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & depth_info_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & point_cloud_msgs)
{
  // std::cout << "callback" << std::endl;
  publisher_points_->publish(*point_cloud_msgs.get());
  publisher_depth_->publish(*depth_image_msg.get());
  publisher_color_->publish(*color_image_msg.get());
  publisher_color_camera_info_->publish(*color_info_msg.get());
  publisher_depth_camera_info_->publish(*depth_info_msg.get());
  count_hz++;
}

private:

  image_transport::SubscriberFilter sub_color_image_, sub_depth_image_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_color_info_, sub_depth_info_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_points_;

  std::shared_ptr<ExactSync> exact_sync_;
  std::shared_ptr<ApproximateSync> approximate_sync_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_color_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_color_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_depth_camera_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_points_;

  int count_hz{0};
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace drake_ros_examples

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(drake_ros_examples::SyncCamerasPointCloud)
