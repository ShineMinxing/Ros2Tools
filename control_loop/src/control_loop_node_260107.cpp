#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

/**
 * @brief Control loop node:
 *  - Subscribe target/track angles + vehicle odom + gimbal state.
 *  - Publish gimbal commands (22202100) to align target in image.
 *  - Publish navigation-goal commands (ACTION_NAV_GOAL) to SportControlNode::Actions():
 *      Value1 = goal_x (map), Value2 = goal_y (map), Value3 = yaw(rad), Value4 = 0
 *  - Optional posture control for body twisting:
 *      (1) big yaw -> turning command 25202123
 *      (2) small residual -> I posture command 22232400
 *
 * NOTE:
 *  - angle_error_callback() assumes radians.
 *  - angle_error_topic payload can be:
 *      [yaw_error, pitch_error] (rad)
 *      [yaw_error, pitch_error, range] (rad, rad, m)
 *
 * TERMINATE NAV:
 *  - When pitch_target enters [0.8, 1.2] rad, publish:
 *      [ACTION_NAV_GOAL, -999.999, -999.999, -999.999, 0]
 *    SportControlNode::sendGoal() interprets it as cancel navigation.
 */
class ControlLoopNode : public rclcpp::Node
{
public:
  ControlLoopNode(const rclcpp::NodeOptions & options)
  : Node("control_loop_node_260107", options)
  {
    target_angle_topic_ = this->declare_parameter<std::string>(
      "TARGET_ANGLE_TOPIC", "NoYamlRead/TargetAngle");
    angle_error_topic_ = this->declare_parameter<std::string>(
      "ANGLE_ERROR_TOPIC", "NoYamlRead/TargetImageAngle");
    vehicle_angle_topic_ = this->declare_parameter<std::string>(
      "VEHICLE_ANGLE_TOPIC", "NoYamlRead/Odom");
    gimbal_angle_topic_ = this->declare_parameter<std::string>(
      "GIMBAL_ANGLE_TOPIC", "NoYamlRead/GimbalState");
    angle_reset_topic_ = this->declare_parameter<std::string>(
      "ANGLE_RESET_TOPIC", "NoYamlRead/SportCmd");
    vehicle_cmd_topic_ = this->declare_parameter<std::string>(
      "VEHICLE_CMD_TOPIC", "NoYamlRead/SportCmd");
    gimbal_cmd_topic_ = this->declare_parameter<std::string>(
      "GIMBAL_CMD_TOPIC", "NoYamlRead/SportCmd");

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

    angle_reset_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      angle_reset_topic_, 10,
      std::bind(&ControlLoopNode::angle_reset_callback, this, std::placeholders::_1));

    vehicle_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      vehicle_cmd_topic_, 10);

    gimbal_action_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      gimbal_cmd_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "control_loop_node_260107 started.\n"
      "  target_angle_topic : %s\n"
      "  angle_error_topic  : %s\n"
      "  vehicle_odom_topic : %s\n"
      "  gimbal_state_topic : %s\n"
      "  sport_cmd_topic    : %s\n",
      target_angle_topic_.c_str(),
      angle_error_topic_.c_str(),
      vehicle_angle_topic_.c_str(),
      gimbal_angle_topic_.c_str(),
      vehicle_cmd_topic_.c_str());
  }

private:
  // ============================================================
  // Action Codes
  // ============================================================
  static constexpr int ACTION_NAV_GOAL = 22262700;
  static constexpr double NAV_TERM = -999.999;

  // ============================================================
  // Navigation / timing state
  // ============================================================
  double vehicle_x = 0.0, vehicle_y = 0.0;
  bool   odom_ready = false;

  bool   last_goal_valid = false;
  double last_goal_x = 0.0, last_goal_y = 0.0, last_goal_yaw = 0.0;
  std::chrono::steady_clock::time_point last_goal_pub_time = std::chrono::steady_clock::now();

  bool nav_active_ = false;
  std::chrono::steady_clock::time_point last_cancel_pub_time_ = std::chrono::steady_clock::now();

  // 原版“Max_Intervel”的意图：间隔太久就别控制（认为数据陈旧）
  std::chrono::steady_clock::time_point last_ctrl_time_ = std::chrono::steady_clock::now();

  // Range from angle_error_topic (optional)
  bool   range_ready_ = false;
  double target_range_m_ = std::numeric_limits<double>::quiet_NaN();

  // ============================================================
  // Utility
  // ============================================================
  static double wrapToPi(double a)
  {
    return std::remainder(a, 2.0 * M_PI);
  }

  bool should_publish_goal(double x, double y, double yaw)
  {
    if (!last_goal_valid) return true;

    const double dx = x - last_goal_x;
    const double dy = y - last_goal_y;
    const double dist = std::hypot(dx, dy);
    const double dyaw = std::fabs(wrapToPi(yaw - last_goal_yaw));

    return (dist > 2.0) || (dyaw > (30.0 * M_PI / 180.0));
  }

  static bool valid_range(double d)
  {
    return std::isfinite(d) && (d > 0.05) && (d < 500.0);
  }

  // ============================================================
  // Callbacks
  // ============================================================
  void target_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;

    roll_optic  = roll_gimbal + roll_vehicle;
    pitch_optic = pitch_gimbal + pitch_vehicle;
    yaw_optic   = yaw_gimbal + yaw_vehicle;

    roll_target  =  msg->data[0] * M_PI / 180.0;
    pitch_target = -msg->data[1] * M_PI / 180.0;
    yaw_target   =  msg->data[2] * M_PI / 180.0;

    if (IMU_follow_enable)
    {
      roll_error  = roll_target  - roll_target_init  - roll_optic;
      pitch_error = pitch_target - pitch_target_init - pitch_optic;
      yaw_error   = yaw_target   - yaw_target_init   - yaw_optic;
      yaw_error = wrapToPi(yaw_error);

      control_loop_function();
    }
  }

  void angle_error_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (auto_track_enable != 1) return;
    if (msg->data.size() < 2) return;

    const auto & d = msg->data;

    yaw_error   = -static_cast<double>(d[0]);
    pitch_error =  static_cast<double>(d[1]);
    roll_error  = 0.0;

    range_ready_ = false;
    target_range_m_ = std::numeric_limits<double>::quiet_NaN();
    if (d.size() >= 3 && d[2] != 0.0)
    {
      const double r = static_cast<double>(d[2]);
      if (valid_range(r))
      {
        range_ready_ = true;
        target_range_m_ = r;
      }
    }

    roll_optic  = roll_gimbal + roll_vehicle;
    pitch_optic = pitch_gimbal + pitch_vehicle;
    yaw_optic   = yaw_gimbal + yaw_vehicle;

    roll_target  = roll_optic  + roll_error;
    pitch_target = pitch_optic + pitch_error;
    yaw_target   = wrapToPi(yaw_optic + yaw_error);

    control_loop_function();
  }

  void vehicle_angle_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    vehicle_x = msg->pose.pose.position.x;
    vehicle_y = msg->pose.pose.position.y;
    odom_ready = true;

    tf2::Quaternion quat(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(quat);
    m.getRPY(roll_vehicle, pitch_vehicle, yaw_vehicle);

    pitch_vehicle = -pitch_vehicle;

    static double last_roll_vehicle = 0, last_pitch_vehicle = 0, last_yaw_vehicle = 0;
    if (roll_vehicle - last_roll_vehicle >  0.8 * M_PI) roll_vehicle_correct  -= 1;
    if (roll_vehicle - last_roll_vehicle < -0.8 * M_PI) roll_vehicle_correct  += 1;
    if (pitch_vehicle - last_pitch_vehicle >  0.8 * M_PI) pitch_vehicle_correct -= 1;
    if (pitch_vehicle - last_pitch_vehicle < -0.8 * M_PI) pitch_vehicle_correct += 1;
    if (yaw_vehicle - last_yaw_vehicle >  0.8 * M_PI) yaw_vehicle_correct   -= 1;
    if (yaw_vehicle - last_yaw_vehicle < -0.8 * M_PI) yaw_vehicle_correct   += 1;

    last_roll_vehicle  = roll_vehicle;
    last_pitch_vehicle = pitch_vehicle;
    last_yaw_vehicle   = yaw_vehicle;
  }

  void gimbal_angle_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) return;
    roll_gimbal  = msg->data[0] * M_PI / 180.0;
    pitch_gimbal = msg->data[1] * M_PI / 180.0;
    yaw_gimbal   = msg->data[2] * M_PI / 180.0;
  }

  void angle_reset_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) return;
    const double code = msg->data[0];

    if (code == 22130000)
    {
      IMU_follow_enable = 0;
      auto_track_enable = 0;
      robot_motion_enable = 0;

      roll_target_init  = roll_target  - roll_optic;
      pitch_target_init = pitch_target - pitch_optic;
      yaw_target_init   = yaw_target   - yaw_optic;

      // reset posture/turning
      robot_posture_start = 0;
      robot_turning_start = 0;
      robot_posture_yaw = 0.0;
      robot_posture_pitch = 0.0;

      nav_active_ = false;
      last_goal_valid = false;

      RCLCPP_INFO(get_logger(), "All Stop and Reset.");
    }

    if (code == 22100000)
    {
      IMU_follow_enable = 1;
      auto_track_enable = 0;
      robot_motion_enable = 0;

      roll_target_init  = roll_target  - roll_optic;
      pitch_target_init = pitch_target - pitch_optic;
      yaw_target_init   = yaw_target   - yaw_optic;

      robot_posture_start = 0;
      robot_turning_start = 0;
      robot_posture_yaw = 0.0;
      robot_posture_pitch = 0.0;

      nav_active_ = false;
      last_goal_valid = false;

      RCLCPP_INFO(get_logger(), "IMU Follow Mode Enable.");
    }

    if (code == 22110000)
    {
      IMU_follow_enable = 0;
      auto_track_enable = 1;
      robot_motion_enable = 0;
      RCLCPP_INFO(get_logger(), "Auto Track Mode Enable.");
    }

    if (code == 22120000)
    {
      IMU_follow_enable = 0;
      auto_track_enable = 1;
      robot_motion_enable = 1;
      RCLCPP_INFO(get_logger(), "Auto Motion Mode Enable.");
    }
  }

  // ============================================================
  // Control loop
  // ============================================================
  void control_loop_function()
  {
    gimbal_controller_function();
    vehicle_controller_function();
  }

  void gimbal_controller_function()
  {
    const double P_gimbal_pitch = 1.0;
    const double P_gimbal_yaw   = 1.0;
    gimbal_action_pub_fun(22202100, P_gimbal_yaw * yaw_error, P_gimbal_pitch * pitch_error);
  }

  /**
   * @brief Vehicle controller:
   *  - nav goal (range-aware) + terminate nav when pitch in band
   *  - when pitch in band: do "turning first" then "posture I" (same as your original style)
   */
  void vehicle_controller_function()
{
  // ====== 完全照你原版的“dt < Max_Intervel 才输出控制”结构 ======
  static auto last_time = std::chrono::steady_clock::now();
  const double Max_Intervel = 0.3;

  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt = now - last_time;
  last_time = now;

  // 这些 start 标志在你新文档里如果没有，就用 static 放函数里（不改类定义）
  static int robot_motion_start  = 0;
  static int robot_turning_start = 0;

  if (!robot_motion_enable) return;
  if (!odom_ready) return;

  // ====== 关键：用 fabs(pitch_target) 做 band 判断（与你原版一致） ======
  const double pitch_mag = std::fabs(pitch_target);

  // ====== 一个小工具：在要手动控底盘前，先 cancel nav（避免被 Nav2 覆盖） ======
  auto cancel_nav_if_active = [&]()
  {
    if (nav_active_)
    {
      vehicle_action_pub_fun(ACTION_NAV_GOAL, NAV_TERM, NAV_TERM, NAV_TERM, 0.0);
      nav_active_ = false;
      last_goal_valid = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
  };

  // ==========================================================
  // 1) 运动P控制器（pitch 出界就走这个）—— 参考你原版
  // ==========================================================
  if ((pitch_mag < 0.8 || pitch_mag > 1.2) && dt.count() < Max_Intervel)
  {
    cancel_nav_if_active();   // <-- 新增：避免 Nav2 打架

    robot_motion_start = 1;

    // 参考你原版：用 1.2 做目标点（保持原调参习惯）
    // 用 pitch_mag 来算，避免 pitch_target 为负导致“永远往前走”的怪相
    double forward_cmd = std::clamp((1.2 - pitch_mag) * 1.0, -0.25, 0.25);

    double yaw_sum = (yaw_error + yaw_gimbal);
    double yaw_cmd = std::clamp(yaw_sum * 1.0, -1.0, 1.0);

    vehicle_action_pub_fun(25202123, 0.0, forward_cmd, yaw_cmd, 0.0);
    return;
  }
  else if (robot_motion_start)
  {
    robot_motion_start = 0;
    vehicle_action_pub_fun(16170000, 0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    // 不 return 也可以，但按你原版这里 return 更干净
    return;
  }

  // ==========================================================
  // 2) 转身控制（大 yaw 误差）—— 参考你原版
  // ==========================================================
  const double yaw_sum = (yaw_error + yaw_gimbal);

  if (std::fabs(yaw_sum) > M_PI / 4.0 && dt.count() < Max_Intervel)
  {
    cancel_nav_if_active();   // <-- 新增：避免 Nav2 打架

    robot_turning_start = 1;
    double yaw_cmd = std::clamp(yaw_sum * 1.0, -1.0, 1.0);
    vehicle_action_pub_fun(25202123, 0.0, 0.0, yaw_cmd, 0.0);
    // 这里你原版没 return，让后面 posture 也有机会跑；保持一致就不 return
  }
  else if (robot_turning_start)
  {
    robot_turning_start = 0;

    // 参考你原版：退出转身时清掉 posture 积分，避免残留偏置
    robot_posture_yaw = 0.0;
    robot_posture_pitch = 0.0;

    vehicle_action_pub_fun(16170000, 0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  // ==========================================================
  // 3) 姿态I控制器（扭身）—— 参考你原版
  // ==========================================================
  const double pitch_sum = (pitch_error + pitch_gimbal);

  if ((std::fabs(yaw_sum) > 0.3 || std::fabs(pitch_sum) > 0.3) && dt.count() < Max_Intervel)
  {
    cancel_nav_if_active();   // <-- 新增：避免 Nav2 打架

    robot_posture_start = 1;

    // 完全按你原版：固定步长积分（不乘 dt）
    robot_posture_yaw =
      std::clamp(robot_posture_yaw + yaw_sum * 0.02, -0.1, 0.1);

    robot_posture_pitch =
      std::clamp(robot_posture_pitch + pitch_sum * 0.02, -0.5, 0.5);

    vehicle_action_pub_fun(22232400, robot_posture_yaw, robot_posture_pitch, 0.0, 0.0);
  }
  else if (robot_posture_start)
  {
    robot_posture_start = 0;
    robot_posture_yaw = 0.0;
    robot_posture_pitch = 0.0;

    vehicle_action_pub_fun(22232400, 0.0, 0.0, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
}

  // ============================================================
  // Publisher helpers
  // ============================================================
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

  // ============================================================
  // ROS interfaces
  // ============================================================
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_error_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          vehicle_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gimbal_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr angle_reset_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    vehicle_action_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    gimbal_action_pub_;

  // Parameters
  std::string target_angle_topic_;
  std::string angle_error_topic_;
  std::string vehicle_angle_topic_;
  std::string gimbal_angle_topic_;
  std::string vehicle_cmd_topic_;
  std::string gimbal_cmd_topic_;
  std::string angle_reset_topic_;

  // ============================================================
  // State variables
  // ============================================================
  int IMU_follow_enable = 0;
  int auto_track_enable = 0;
  int robot_motion_enable = 0;

  // 参考原版：turning / posture 两个 start 标志
  int robot_turning_start = 0;
  int robot_posture_start = 0;

  double roll_target = 0, pitch_target = 0, yaw_target = 0;
  double roll_target_init = 0, pitch_target_init = 0, yaw_target_init = 0;

  double roll_optic = 0, pitch_optic = 0, yaw_optic = 0;
  double roll_error = 0, pitch_error = 0, yaw_error = 0;

  double roll_gimbal = 0, pitch_gimbal = 0, yaw_gimbal = 0;
  double roll_vehicle = 0, pitch_vehicle = 0, yaw_vehicle = 0;

  double roll_vehicle_correct = 0, pitch_vehicle_correct = 0, yaw_vehicle_correct = 0;

  // posture integrator states (body twist)
  double robot_posture_pitch = 0.0;
  double robot_posture_yaw   = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "--params-file",
    "/home/smx/WorkSpace/GDS_LeggedRobot/src/Ros2Tools/config.yaml"
  });

  auto node = std::make_shared<ControlLoopNode>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
