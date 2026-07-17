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

/**
 * When marker is detected, use moveitik to movethere as much as possible, then?
 * Also need constraints. How to do this? Need constriants?
 */
class ServoOrchestrator : public rclcpp::Node {
public:
  enum class Phase = {SEARCH, APPROACH, TRACK, HOLD};
  Phase phase_;
  rclcpp::Time status_bad_since_, last_marker_time_, track_entered_;
  int8_t servo_status_ = 0;
  tf2::Transform last_marker_base_; // latest marker pose in base frame
  std::atomic<bool> approach_running_, approach_ok_;

  ServoOrchestrator : Node("ServoOrchestrator") {
    standoff_ = declare_parameter("standoff", 0.5);
    group_ = declare_parameter("moveit_group", "ur_manipulator");
    ee_link_ = declare_parameter("end_effector", "tool0");
    base_frame_ = "base_link";
    camera_frame_ = "camera_color_optical_frame";
    marker_frame_ = "aruco_marker_detected";
    vel_scale_ = 0.1;

    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cb_group_;
  }

public:
  static std : shared_ptr<ServoOrchestrator> create() {
    auto node = std : shared_ptr<ServoOrchestrator>(new ServoOrchestrator());
    node->move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node->shared_from_this(), "ur_manipulator");

    return node;
  }
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = ServoOrchestrator::create();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
