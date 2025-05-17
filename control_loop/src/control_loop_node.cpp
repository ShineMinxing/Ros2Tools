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
  : Node("control_loop_node", options)
  {
    // 声明并获取话题参数
    target_angle_topic_ = this->declare_parameter<std::string>(
      "TARGET_ANGLE_TOPIC", "TEST/TargetImageAngle");
    odom_topic_ = this->declare_parameter<std::string>(
      "ODOM_TOPIC", "TEST/Odom");
    gimbal_state_topic_ = this->declare_parameter<std::string>(
      "GIMBAL_STATE_TOPIC", "TEST/GimbalState");
    sport_cmd_topic_ = this->declare_parameter<std::string>(
      "SPORT_CMD_TOPIC", "TEST/SportCmd");
    joy_cmd_topic_ = this->declare_parameter<std::string>(
      "JOY_CMD_TOPIC", "TEST/JoyFloatCmd");

    // ===== 订阅 =====
    target_angle_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      target_angle_topic_, 10,
      std::bind(&ControlLoopNode::target_angle_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&ControlLoopNode::odom_callback, this, std::placeholders::_1));

    gimbal_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      gimbal_state_topic_, 10,
      std::bind(&ControlLoopNode::gimbal_state_callback, this, std::placeholders::_1));

    // ===== 发布 =====
    go2_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      sport_cmd_topic_, 10);

    gimbal_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      joy_cmd_topic_, 10);

    RCLCPP_INFO(get_logger(), "control_loop_node started.\n"
      "  target_angle_topic: %s\n"
      "  odom_topic: %s\n"
      "  gimbal_state_topic: %s\n"
      "  sport_cmd_topic: %s",
      target_angle_topic_.c_str(),
      odom_topic_.c_str(),
      gimbal_state_topic_.c_str(),
      sport_cmd_topic_.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion quat(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(quat);
    m.getRPY(roll_robot, pitch_robot, yaw_robot);
    pitch_robot = -pitch_robot;
  }

  void gimbal_state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 6) {
      yaw_gimbal   = msg->data[2] * M_PI / 180.0;
      pitch_gimbal = msg->data[1] * M_PI / 180.0;
    }
  }

  void target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      yaw_image   = -msg->data[0] * M_PI / 180.0;
      pitch_image =  msg->data[1] * M_PI / 180.0;
      update_target_and_publish();
    }
  }

  void go2_action_pub_fun(double code, double p1, double p2, double p3, double p4)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {code, p1, p2, p3, p4};
    go2_action_pub_->publish(out);
  }


  void gimbal_action_pub_fun(double code, double p1, double p2)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = {code, p1, p2};
    gimbal_action_pub_->publish(out);
  }

  void update_target_and_publish()
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

    // 发布初步命令
    gimbal_action_pub_fun(30000001, -50*yaw_image, -50*pitch_image);

    double yaw_target   = yaw_robot + yaw_gimbal + yaw_image;
    double pitch_target = pitch_robot + pitch_gimbal + pitch_image;

    // 运动控制示例
    if (std::fabs(yaw_target - yaw_robot) > 1.0 && dt.count() < 0.5) {
      robot_motion_enable = 1;
      robot_motion_yaw = std::clamp((yaw_target - yaw_robot)*1.0, -1.0, 1.0);
      go2_action_pub_fun(25202123, 0, 0, robot_motion_yaw, 0);
    } else if (robot_motion_enable > 0) {
      go2_action_pub_fun(16170000, 0, 0, 0, 0);
      robot_motion_enable = 0;
    }

    // 姿态控制示例
    if (std::fabs(yaw_target - yaw_robot) > 0.3 || std::fabs(pitch_target - pitch_robot) > 0.3)
      robot_posture_enable = 1;
    if (robot_posture_enable) {
      robot_posture_yaw   =std::clamp(robot_posture_yaw+ (yaw_target-yaw_robot)*0.05, -0.5,0.5);
      robot_posture_pitch =std::clamp(robot_posture_pitch+(pitch_target-pitch_robot)*0.05,-0.5,0.5);
    }
    go2_action_pub_fun(22232400, robot_posture_yaw, robot_posture_pitch,0,0);

    // // 输出目标角度
    // std::cout << "yaw_robot-" << yaw_robot << " pitch_robot-" << pitch_robot << std::endl;
    // std::cout << "yaw_gimbal-" << yaw_gimbal << " pitch_gimbal-" << pitch_gimbal << std::endl;
    // std::cout << "yaw_image-" << yaw_image << " pitch_image-" << pitch_image << std::endl;
    // std::cout << "yaw_target - yaw_robot: " << yaw_target - yaw_robot << std::endl;
    // std::cout << "robot_motion_yaw: " << robot_motion_yaw << std::endl;
    // std::cout << "________________"  << std::endl;
  }

  // Subscribers & Publishers
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    go2_action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    gimbal_action_pub_;

  // Parameters
  std::string target_angle_topic_, odom_topic_, gimbal_state_topic_, sport_cmd_topic_, joy_cmd_topic_;

  // State variables
  double yaw_robot=0, pitch_robot=0, roll_robot=0;
  double yaw_gimbal=0, pitch_gimbal=0;
  double yaw_image=0, pitch_image=0;
  int robot_motion_enable=0, robot_posture_enable=0;
  double robot_motion_yaw=0;
  double robot_posture_yaw=0, robot_posture_pitch=0;
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