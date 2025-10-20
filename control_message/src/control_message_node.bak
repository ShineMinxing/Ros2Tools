#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>

using std::placeholders::_1;

class ControlMessageNode : public rclcpp::Node
{
public:
  explicit ControlMessageNode(const rclcpp::NodeOptions & opt)
  : Node("control_message_node", opt)
  {
    // 从 config.yaml 读取参数
    this->get_parameter_or("joy_topic",       joy_topic_, std::string("/joy"));
    this->get_parameter_or("sport_cmd_topic", cmd_topic_, std::string("NoYamlRead/SportCmd"));

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&ControlMessageNode::joy_cb, this, _1));

    last_op_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
      "ControlMessageNode listening '%s' → publishing '%s'",
      joy_topic_.c_str(), cmd_topic_.c_str());
  }

private:
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr m)
  {
    // 1) 读取按键 / 摇杆
    for (int i = 0; i < 8 && i < (int)m->buttons.size(); ++i) Buttons[i] = m->buttons[i];
    for (int i = 0; i < 8 && i < (int)m->axes.size();    ++i) Axes[i]    = m->axes[i];

    // 2) 计算距上次发布的间隔
    now_time_  = this->now();
    Last_Operation_Duration_Time = (now_time_ - last_op_time_).seconds();

    /* ============ 映射逻辑 ============ */
    if (Buttons[6]) { publish_cmd(16000000,0,0,0,0); return; }

    if (Buttons[4] && Buttons[5] && Last_Operation_Duration_Time > 0.3) {
      publish_cmd(14150000,0,0,0,0); return;
    }

    // ---- 只按 RT ----
    if (Axes[5] < -0.5 && Axes[2] > 0.9) {
      if (Axes[0]||Axes[1]||Axes[3]||Axes[4])
        publish_cmd(25202123, Axes[0], Axes[1], Axes[3], Axes[4]);
    }

    // ---- 只按 LT ----
    if (Axes[2] < -0.5 && Axes[5] > 0.9) {
      if (Axes[0]||Axes[1]) publish_cmd(22202100, Axes[0], Axes[1], 0, 0);
      if (Axes[3]||Axes[4]) publish_cmd(22232400, Axes[3], Axes[4], 0, 0);
    }

    if (Axes[5] < -0.5 && Axes[2] > 0.9 && Last_Operation_Duration_Time > 0.3) {
      if      (Buttons[0]) publish_cmd(25100000,0,0,0,0); //A
      else if (Buttons[1]) publish_cmd(25110000,0,0,0,0); //B
      else if (Buttons[2]) publish_cmd(25120000,0,0,0,0); //X
      else if (Buttons[3]) publish_cmd(25130000,0,0,0,0); //Y
      else if (Buttons[4]) publish_cmd(25140000,0,0,0,0); //LB
      else if (Buttons[5]) publish_cmd(25150000,0,0,0,0); //RB
      else if (Buttons[6]) publish_cmd(25160000,0,0,0,0); //SELECT
      else if (Buttons[7]) publish_cmd(25170000,0,0,0,0); //START
      else if (Axes[6]||Axes[7]) publish_cmd(25262700, Axes[6], Axes[7], 0, 0);
    }
    else if (Axes[2] < -0.5 && Axes[5] > 0.9 && Last_Operation_Duration_Time > 0.3) {
      if      (Buttons[0]) publish_cmd(22100000,0,0,0,0);
      else if (Buttons[1]) publish_cmd(22110000,0,0,0,0);
      else if (Buttons[2]) publish_cmd(22120000,0,0,0,0);
      else if (Buttons[3]) publish_cmd(22130000,0,0,0,0);
      else if (Buttons[4]) publish_cmd(22140000,0,0,0,0);
      else if (Buttons[5]) publish_cmd(22150000,0,0,0,0);
      else if (Buttons[6]) publish_cmd(22160000,0,0,0,0);
      else if (Buttons[7]) publish_cmd(22170000,0,0,0,0);
      else if (Axes[6]||Axes[7]) publish_cmd(22262700, Axes[6], Axes[7], 0, 0);
    }
  }

  /* 只有在真正发布时才更新时间戳 */
  void publish_cmd(int code, double v1, double v2, double v3, double v4)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = { double(code), v1, v2, v3, v4 };
    cmd_pub_->publish(out);
    last_op_time_ = now_time_;
  }

  std::string joy_topic_, cmd_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  int   Buttons[8]{};
  float Axes[8]{};

  double Last_Operation_Duration_Time = 0.0;
  rclcpp::Time last_op_time_;
  rclcpp::Time now_time_;

  // 可按需保留
  bool MotionFlag = false;
  double guide_x_vel=0, guide_y_vel=0, guide_yaw_vel=0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opt;
  opt.allow_undeclared_parameters(true)
     .automatically_declare_parameters_from_overrides(true)
     .arguments({
       "--ros-args",
       "--params-file",
       "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Tools/config.yaml"
     });

  rclcpp::spin(std::make_shared<ControlMessageNode>(opt));
  rclcpp::shutdown();
  return 0;
}
