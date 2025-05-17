#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <array>
#include <chrono>
#include <cmath>
#include <iostream> 

class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode(const rclcpp::NodeOptions & options)
  : Node("control_loop_node_250517", options)
  {
    // 声明并获取话题参数
    target_angle_topic_ = this->declare_parameter<std::string>(
      "TARGET_ANGLE_TOPIC", "NoYamlRead/TargetAngle");
    angle_error_topic_ = this->declare_parameter<std::string>(
      "ANGLE_ERROR_TOPIC", "NoYamlRead/TargetImageAngle");
    vehicle_angle_topic_ = this->declare_parameter<std::string>(
      "VEHICLE_ANGLE_TOPIC", "NoYamlRead/Odom");
    gimbal_angle_topic_ = this->declare_parameter<std::string>(
      "GIMBAL_ANGLE_TOPIC", "NoYamlRead/GimbalState");
    vehicle_cmd_topic_ = this->declare_parameter<std::string>(
      "VEHICLE_CMD_TOPIC", "NoYamlRead/SportCmd");
    gimbal_cmd_topic_ = this->declare_parameter<std::string>(
      "GIMBAL_CMD_TOPIC", "NoYamlRead/JoyFloatCmd");

    // ===== 订阅 =====
    target_angle_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      target_angle_topic_, 10,
      std::bind(&ControlLoopNode::target_angle_callback, this, std::placeholders::_1));

    angle_error_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      angle_error_topic_, 10,
      std::bind(&ControlLoopNode::angle_error_callback, this, std::placeholders::_1));

    vehicle_angle_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      vehicle_angle_topic_, 10,
      std::bind(&ControlLoopNode::vehicle_angle_callback, this, std::placeholders::_1));

    gimbal_angle_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      gimbal_angle_topic_, 10,
      std::bind(&ControlLoopNode::gimbal_angle_callback, this, std::placeholders::_1));

    // ===== 发布 =====
    vehicle_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      vehicle_cmd_topic_, 10);

    gimbal_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      gimbal_cmd_topic_, 10);

    RCLCPP_INFO(get_logger(), "control_loop_node_250517 started.\n"
      "  angle_error_topic: %s\n"
      "  vehicle_angle_topic: %s\n"
      "  gimbal_angle_topic: %s\n"
      "  sport_cmd_topic: %s",
      angle_error_topic_.c_str(),
      vehicle_angle_topic_.c_str(),
      gimbal_angle_topic_.c_str(),
      vehicle_cmd_topic_.c_str());
  }

private:
  void target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3) {

      roll_target  = msg->data[0] * M_PI / 180.0;
      pitch_target = msg->data[1] * M_PI / 180.0;
      yaw_target   = msg->data[2] * M_PI / 180.0;

      roll_optic  = roll_gimbal + roll_vehicle;
      pitch_optic = pitch_gimbal + pitch_vehicle;
      yaw_optic   = yaw_gimbal + yaw_vehicle;

      roll_error  = roll_target - roll_optic;
      pitch_error = pitch_target - pitch_optic;
      yaw_error   = yaw_target - yaw_optic;

      control_loop_function();
    }
  }

  void angle_error_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      roll_error  = 0;
      pitch_error = msg->data[1] * M_PI / 180.0;
      yaw_error   = -msg->data[0] * M_PI / 180.0;

      roll_optic  = roll_gimbal + roll_vehicle;
      pitch_optic = pitch_gimbal + pitch_vehicle;
      yaw_optic   = yaw_gimbal + yaw_vehicle;

      roll_target  = roll_optic + roll_error;
      pitch_target = pitch_optic + pitch_error;
      yaw_target   = yaw_optic + yaw_error;

      control_loop_function();
    }
  }

  void vehicle_angle_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion quat(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll_vehicle, pitch_vehicle, yaw_vehicle);
    pitch_vehicle = -pitch_vehicle;
  }

  void gimbal_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 6) {
      roll_gimbal  = msg->data[0] * M_PI / 180.0;
      pitch_gimbal = msg->data[1] * M_PI / 180.0;
      yaw_gimbal   = msg->data[2] * M_PI / 180.0;
    }
  }

  void control_loop_function()
  {
    gimbal_controller_function();

    vehicle_controller_function();
  }

  void gimbal_controller_function()
  {
    double P_gimbal_pitch = -50;
    double P_gimbal_yaw   = -50;

    gimbal_action_pub_fun(30000001, P_gimbal_yaw*yaw_error, P_gimbal_pitch*pitch_error);
  }

  void vehicle_controller_function()
  {
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt = now - last_time;
    last_time = now;

    if(dt.count() > 0.5)
    {
      robot_posture_yaw = 0;
      robot_posture_pitch = 0;
    }

    // 姿态I控制器
    if (std::fabs(yaw_target - yaw_vehicle) > 0.3 || std::fabs(pitch_target - pitch_vehicle) > 0.3)
      robot_posture_enable = 1;
    if (robot_posture_enable) {
      robot_posture_yaw   =std::clamp(robot_posture_yaw+ (yaw_target-yaw_vehicle)*0.05, -0.5,0.5);
      robot_posture_pitch =std::clamp(robot_posture_pitch+(pitch_target-pitch_vehicle)*0.05,-0.5,0.5);
    }
    vehicle_action_pub_fun(22232400, robot_posture_yaw, robot_posture_pitch,0,0);

    // 运动P控制器
    if (std::fabs(yaw_target - yaw_vehicle) > 1.0 && dt.count() < 0.5) {
      robot_motion_enable = 1;
      robot_motion_yaw = std::clamp((yaw_target - yaw_vehicle)*1.0, -1.0, 1.0);
      vehicle_action_pub_fun(25202123, 0, 0, robot_motion_yaw, 0);
    } else if (robot_motion_enable > 0) {
      vehicle_action_pub_fun(16170000, 0, 0, 0, 0);
      robot_motion_enable = 0;
    }
  }

  void vehicle_action_pub_fun(double code, double p1, double p2, double p3, double p4)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {code, p1, p2, p3, p4};
    vehicle_action_pub_->publish(out);
  }


  void gimbal_action_pub_fun(double code, double p1, double p2)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {code, p1, p2};
    gimbal_action_pub_->publish(out);
  }

  // Subscribers & Publishers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_error_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          vehicle_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    vehicle_action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    gimbal_action_pub_;

  // Parameters
  std::string target_angle_topic_, angle_error_topic_, vehicle_angle_topic_, gimbal_angle_topic_, vehicle_cmd_topic_, gimbal_cmd_topic_;

  // State variables
  double roll_target=0, pitch_target=0, yaw_target=0;
  double roll_optic=0, pitch_optic=0, yaw_optic=0;
  double roll_error=0, pitch_error=0, yaw_error=0;
  double roll_gimbal=0, pitch_gimbal=0, yaw_gimbal=0;
  double roll_vehicle=0, pitch_vehicle=0, yaw_vehicle=0;
  int robot_motion_enable=0, robot_posture_enable=0;
  double robot_posture_roll=0, robot_posture_pitch=0, robot_posture_yaw=0;
  double robot_motion_roll=0, robot_motion_pitch=0, robot_motion_yaw=0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "--params-file",
    "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Tools/config.yaml"
  });
  auto node = std::make_shared<ControlLoopNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}