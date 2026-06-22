#include <memory>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h> // Humble: .h (newer distros: cv_bridge.hpp)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

class ArucoDetector : public rclcpp::Node {
public:
  ArucoDetector() : Node("aruco_detector") {
    // NOTE: 0.192 m, not 0.25 m — the black ArUco square fills only
    // 400/520 of the texture mapped onto the 0.25 m plate.
    marker_length_ = declare_parameter("marker_length", 0.192);
    target_id_ = declare_parameter("marker_id", 0);
    const auto image_topic = declare_parameter<std::string>(
        "image_topic", "/camera/color/image_raw");
    const auto info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/camera/color/camera_info");

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    params_ = cv::aruco::DetectorParameters::create();

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS(),
        std::bind(&ArucoDetector::onCameraInfo, this, _1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::SensorDataQoS(),
        std::bind(&ArucoDetector::onImage, this, _1));

    pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("marker_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    RCLCPP_INFO(get_logger(),
                "aruco_detector ready, waiting for camera_info...");
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    RCLCPP_INFO_ONCE(get_logger(), "camera_info received: %ux%u", msg->width,
                     msg->height);
    K_ = (cv::Mat_<double>(3, 3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3],
          msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);
    D_ = cv::Mat(static_cast<int>(msg->d.size()), 1, CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i) {
      D_.at<double>(static_cast<int>(i)) = msg->d[i];
    }
    have_info_ = true;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!have_info_)
      return;

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids, params_);
    if (ids.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no ArUco markers detected in view");
      return;
    }

    std::string id_list;
    for (int id : ids)
      id_list += std::to_string(id) + " ";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "detected %zu marker(s): [%s]", ids.size(),
                         id_list.c_str());

    int idx = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == target_id_) {
        idx = static_cast<int>(i);
        break;
      }
    }
    if (idx < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "target id %d not among detected markers [%s]",
                           target_id_, id_list.c_str());
      return;
    }

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, K_, D_, rvecs,
                                         tvecs);
    const cv::Vec3d &rvec = rvecs[idx];
    const cv::Vec3d &tvec = tvecs[idx];

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    tf2::Matrix3x3 tf_R(
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    tf2::Quaternion q;
    tf_R.getRotation(q);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                         "marker %d  pos=[%.3f %.3f %.3f] m  dist=%.3f m  "
                         "quat=[%.3f %.3f %.3f %.3f]",
                         target_id_, tvec[0], tvec[1], tvec[2], cv::norm(tvec),
                         q.x(), q.y(), q.z(), q.w());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = msg->header.frame_id; // camera_color_optical_frame
    pose.pose.position.x = tvec[0];
    pose.pose.position.y = tvec[1];
    pose.pose.position.z = tvec[2];
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = pose.header;
    tf.child_frame_id = "aruco_marker_detected";
    tf.transform.translation.x = tvec[0];
    tf.transform.translation.y = tvec[1];
    tf.transform.translation.z = tvec[2];
    tf.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  double marker_length_;
  int target_id_;
  bool have_info_{false};
  cv::Mat K_, D_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoDetector>());
  rclcpp::shutdown();
  return 0;
}
