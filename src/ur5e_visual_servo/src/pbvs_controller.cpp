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
    standoff_ = declare_parameter("standoff", 0.30);
    kt_ = declare_parameter("k_trans", 0.8);
    kr_ = declare_parameter("k_rot", 0.5);
    db_t_ = declare_parameter("deadband_trans", 0.01); // m
    db_r_ = declare_parameter("deadband_rot", 0.02);   // rad
    max_lin_ = declare_parameter("max_lin", 0.10);     // m/s
    max_ang_ = declare_parameter("max_ang", 0.50);     // rad/s
    enable_rot_ = declare_parameter("enable_rotation", true);
    const auto cmd_topic = declare_parameter<std::string>(
        "cmd_topic", "/servo_node/delta_twist_cmds");

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "marker_pose", 10, std::bind(&PBVSController::onPose, this, _1));
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(cmd_topic, 10);
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
      // R_desired = identity (square-on)  =>  R_err = R_desired * R^T = R^T.
      // Identity is the STARTING assumption; M2 is where you confirm/flip i
      tf2::Matrix3x3 R_err = R.transpose();
      tf2::Quaternion qe;
      R_err.getRotation(qe);
      double angle = qe.getAngle();
      if (angle > M_PI)
        angle -= 2.0 * M_PI; // take shortest rotation
      const tf2::Vector3 axis = qe.getAxis();
      w = axis * (kr_ * angle);
    }

    // Deadband: stop when basically converged.
    if (e_t.length() < db_t_) v = tf2::Vector3(0, 0, 0);
    if (w.length() < kr_ * db_r_) w = tf2::Vector3(0, 0, 0);

    v = clampVec(v, max_lin_);
    w = clampVec(w, max_ang_);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = msg->header.stamp;
    cmd.header.frame_id = msg->header.frame_id;    // camera_color_optical_frame
    cmd.twist.linear.x = v.x();  cmd.twist.linear.y = v.y();  cmd.twist.linear.z = v.z();
    cmd.twist.angular.x = w.x(); cmd.twist.angular.y = w.y(); cmd.twist.angular.z = w.z();
    twist_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "err_t=[%.3f %.3f %.3f] v=[%.3f %.3f %.3f] w=[%.3f %.3f %.3f]",
      e_t.x(), e_t.y(), e_t.z(), v.x(), v.y(), v.z(), w.x(), w.y(), w.z());
  }

  double standoff_, kt_, kr_, db_t_, db_r_, max_lin_, max_ang_;
  bool enable_rot_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PBVSController>());
  rclcpp::shutdown();
  return 0;
}
