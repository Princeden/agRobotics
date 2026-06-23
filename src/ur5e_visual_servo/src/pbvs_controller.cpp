#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using std::placeholders::_1;

static tf2::Vector3 clampVec(const tf2::Vector3 &v, double limit) {
  const double n = v.length();
  return (n > limit && n > 1e-9) ? v * (limit / n) : v;
}

class PBVSController : public rclcpp::Node {
public:
  PBVSController() : Node("pbvs_controller") {
    // Goal: marker centered (x=y=0) at `standoff` m along the optical axis
    // (+z).
    standoff_ = declare_parameter("standoff", 0.5);
    kt_ = declare_parameter("k_trans", 0.8);
    kr_ = declare_parameter("k_rot", 0.5);
    db_t_ = declare_parameter("deadband_trans", 0.01); // m
    db_r_ = declare_parameter("deadband_rot", 0.02);   // rad
    max_lin_ = declare_parameter("max_lin", 0.10);     // m/s
    max_ang_ = declare_parameter("max_ang", 0.50);     // rad/s
    enable_rot_ = declare_parameter("enable_rotation", true);
    // Servo halts if no command arrives within incoming_command_timeout (~0.1
    // s), but the camera only yields poses at ~8-10 Hz. Republish the last
    // twist on a fast timer so motion stays smooth; zero it if the pose goes
    // stale.
    const double publish_rate = declare_parameter("publish_rate", 100.0); // Hz
    cmd_timeout_ = declare_parameter("cmd_timeout", 0.5);                 // s
    frame_id_ = declare_parameter<std::string>("command_frame",
                                               "camera_color_optical_frame");
    const auto cmd_topic = declare_parameter<std::string>(
        "cmd_topic", "/servo_node/delta_twist_cmds");

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "marker_pose", 10, std::bind(&PBVSController::onPose, this, _1));
    twist_pub_ =
        create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);
    timer_ =
        create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate),
                          std::bind(&PBVSController::publishCmd, this));
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const tf2::Vector3 t(msg->pose.position.x, msg->pose.position.y,
                         msg->pose.position.z);

    // Translation error in the camera frame; desired = (0, 0, standoff).
    // Moving the camera +x shifts the marker -x in-frame, so the camera
    // velocity that drives the marker to target is +k*(measured - desired).
    const tf2::Vector3 e_t(t.x(), t.y(), t.z() - standoff_);
    tf2::Vector3 v = e_t * kt_;

    // Orientation error -> camera angular velocity (axis-angle).
    tf2::Vector3 w(0, 0, 0);
    if (enable_rot_) {
      const tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
                              msg->pose.orientation.z, msg->pose.orientation.w);
      const tf2::Matrix3x3 R(q);
      // Marker optical-frame convention: marker +Z faces the camera, so a
      // square-on marker reads as R = diag(1,-1,-1) (180 deg about X), NOT
      // identity. Using identity made the controller command a constant ~180 deg
      // flip (angular.x pinned at max_ang) that drove the wrist into a
      // singularity. Desired orientation is therefore diag(1,-1,-1).
      const tf2::Matrix3x3 R_des(1, 0, 0, 0, -1, 0, 0, 0, -1);
      tf2::Matrix3x3 R_err = R_des * R.transpose();
      tf2::Quaternion qe;
      R_err.getRotation(qe);
      double angle = qe.getAngle();
      if (angle > M_PI)
        angle -= 2.0 * M_PI; // take shortest rotation
      const tf2::Vector3 axis = qe.getAxis();
      w = axis * (kr_ * angle);
    }

    // Deadband: stop when basically converged.
    if (e_t.length() < db_t_)
      v = tf2::Vector3(0, 0, 0);
    if (w.length() < kr_ * db_r_)
      w = tf2::Vector3(0, 0, 0);

    v = clampVec(v, max_lin_);
    w = clampVec(w, max_ang_);

    // Stash for the publish timer; it stamps with the current clock so Servo
    // sees fresh commands even between camera frames.
    last_lin_ = v;
    last_ang_ = w;
    if (!msg->header.frame_id.empty())
      frame_id_ = msg->header.frame_id; // camera_color_optical_frame
    last_pose_time_ = now();
    have_pose_ = true;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "err_t=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] w=[%.3f %.3f %.3f]", e_t.x(),
        e_t.y(), e_t.z(), v.x(), v.y(), v.z(), w.x(), w.y(), w.z());
  }

  void publishCmd() {
    tf2::Vector3 v = last_lin_, w = last_ang_;
    // Safety stop: if the marker pose is stale (or never seen), command zero.
    if (!have_pose_ || (now() - last_pose_time_).seconds() > cmd_timeout_) {
      v = tf2::Vector3(0, 0, 0);
      w = tf2::Vector3(0, 0, 0);
    }

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = frame_id_;
    cmd.twist.linear.x = v.x();
    cmd.twist.linear.y = v.y();
    cmd.twist.linear.z = v.z();
    cmd.twist.angular.x = w.x();
    cmd.twist.angular.y = w.y();
    cmd.twist.angular.z = w.z();
    twist_pub_->publish(cmd);
  }

  double standoff_, kt_, kr_, db_t_, db_r_, max_lin_, max_ang_;
  double cmd_timeout_;
  bool enable_rot_;
  std::string frame_id_;
  bool have_pose_ = false;
  rclcpp::Time last_pose_time_;
  tf2::Vector3 last_lin_{0, 0, 0};
  tf2::Vector3 last_ang_{0, 0, 0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PBVSController>());
  rclcpp::shutdown();
  return 0;
}
