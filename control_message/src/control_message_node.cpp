#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <string>

// ====== 仅最小必要的新增头文件 ======
#include <vector>
#include <array>
#include <thread>
#include <atomic>
#include <mutex>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

using std::placeholders::_1;

class ControlMessageNode : public rclcpp::Node
{
public:
  explicit ControlMessageNode(const rclcpp::NodeOptions & opt)
  : Node("control_message_node", opt)
  {
    // 从 config.yaml 读取参数（保持原有风格，不 declare，避免与 auto-declare 冲突）
    this->get_parameter_or("joy_topic",       joy_topic_, std::string("/joy"));
    this->get_parameter_or("sport_cmd_topic", cmd_topic_, std::string("NoYamlRead/SportCmd"));

    // 新增：是否启用键盘->/joy
    this->get_parameter_or("Keyboard2Joystick_Enable", keyboard_enabled_, false);

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(cmd_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&ControlMessageNode::joy_cb, this, _1));

    last_op_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
      "ControlMessageNode listening '%s' → publishing '%s' (Keyboard2Joystick_Enable=%s)",
      joy_topic_.c_str(), cmd_topic_.c_str(), keyboard_enabled_ ? "true" : "false");

    if (keyboard_enabled_) {
      joy_kb_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);
      init_keyboard_state_();
      if (open_all_keyboard_events_()) {
        kb_running_.store(true);
        kb_thread_ = std::thread([this]{ keyboard_reader_loop_(); });
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "No keyboard event device opened. Add user to 'input' group or run with sudo.");
      }
    }
  }

  ~ControlMessageNode() override
  {
    kb_running_.store(false);
    if (kb_thread_.joinable()) kb_thread_.join();
    for (int fd : kb_fds_) if (fd >= 0) ::close(fd);
  }

private:
  /* ===================== 原始逻辑：订阅 /joy 并发布命令（未改） ===================== */
  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr m)
  {
    // 1) 读取按键 / 摇杆
    for (int i = 0; i < 8 && i < (int)m->buttons.size(); ++i) Buttons[i] = m->buttons[i];
    for (int i = 0; i < 8 && i < (int)m->axes.size();    ++i) Axes[i]    = m->axes[i];

    // 2) 计算距上次发布的间隔
    now_time_  = this->now();
    Last_Operation_Duration_Time = (now_time_ - last_op_time_).seconds();

    /* ============ 映射逻辑：保持不变 ============ */
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

  /* 只有在真正发布时才更新时间戳（未改） */
  void publish_cmd(int code, double v1, double v2, double v3, double v4)
  {
    std_msgs::msg::Float64MultiArray out;
    out.data = { double(code), v1, v2, v3, v4 };
    cmd_pub_->publish(out);
    last_op_time_ = now_time_;
  }

  /* ===================== 键盘 → Joy（监听全部键盘，事件驱动发布） ===================== */
  void init_keyboard_state_()
  {
    kb_buttons_.fill(0);
    kb_axes_.fill(0.0f);
    // 触发器默认未按：+1；按下：-1
    kb_axes_[2] = +1.0f; // LT (Alt)
    kb_axes_[5] = +1.0f; // RT (Space)
    l_left_ = l_right_ = l_up_ = l_down_ = false;
    r_left_ = r_right_ = r_up_ = r_down_ = false;
    alt_down_ = space_down_ = false;
  }

  // 解析 /proc/bus/input/devices，找出所有带 "kbd" 的 eventX
  std::vector<std::string> enumerate_keyboard_event_paths_()
  {
    std::vector<std::string> paths;
    std::ifstream fin("/proc/bus/input/devices");
    if (!fin.good()) return paths;

    std::string line, handlers;
    while (std::getline(fin, line)) {
      if (line.rfind("H:", 0) == 0) {
        handlers = line;
        if (handlers.find("kbd") != std::string::npos) {
          std::istringstream ss(handlers);
          std::string tok;
          while (ss >> tok) {
            auto pos = tok.find("event");
            if (pos != std::string::npos) {
              std::string ev = tok.substr(pos);
              // 确保 event 后都是数字
              if (ev.size() >= 6 && std::all_of(ev.begin()+5, ev.end(), ::isdigit)) {
                paths.emplace_back("/dev/input/" + ev);
              }
            }
          }
        }
      }
    }
    std::sort(paths.begin(), paths.end());
    paths.erase(std::unique(paths.begin(), paths.end()), paths.end());
    return paths;
  }

  bool open_all_keyboard_events_()
  {
    auto paths = enumerate_keyboard_event_paths_();
    if (paths.empty()) {
      RCLCPP_WARN(this->get_logger(), "No keyboard handlers with 'kbd' found in /proc/bus/input/devices.");
      return false;
    }
    for (auto &p : paths) {
      int fd = ::open(p.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0) {
        RCLCPP_WARN(this->get_logger(), "open(%s) failed: %s", p.c_str(), strerror(errno));
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "Keyboard device opened: %s", p.c_str());
      kb_fds_.push_back(fd);
    }
    return !kb_fds_.empty();
  }

  // 事件驱动：有按键“按下/抬起”才发布（忽略 repeat=2）
  void keyboard_reader_loop_()
  {
    std::vector<pollfd> pfds;
    pfds.reserve(kb_fds_.size());
    for (int fd : kb_fds_) pfds.push_back(pollfd{fd, POLLIN, 0});

    while (rclcpp::ok() && kb_running_.load()) {
      int ret = ::poll(pfds.data(), pfds.size(), 50);
      if (ret > 0) {
        bool changed = false;
        for (auto &p : pfds) {
          if (p.revents & POLLIN) {
            struct input_event ev;
            ssize_t n = ::read(p.fd, &ev, sizeof(ev));
            while (n == (ssize_t)sizeof(ev)) {
              if (ev.type == EV_KEY && (ev.value == 0 || ev.value == 1)) {
                changed = handle_key_event_(ev.code, ev.value) || changed;
              }
              n = ::read(p.fd, &ev, sizeof(ev));
            }
          }
        }
        if (changed) publish_current_joy_();  // 仅当有变化才发布
      }
      // 小睡降低CPU
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // 返回：此次事件是否导致状态变化
  bool set_button_(int idx, bool pressed)
  {
    std::lock_guard<std::mutex> lk(kb_mtx_);
    int old = kb_buttons_[idx];
    int now = pressed ? 1 : 0;
    if (old != now) { kb_buttons_[idx] = now; return true; }
    return false;
  }

  bool handle_key_event_(unsigned short code, int value)
  {
    const bool pressed = (value != 0);
    bool changed = false;

    // 1..8 → Buttons[0..7]
    if (code >= KEY_1 && code <= KEY_8) {
      int idx = code - KEY_1;
      return set_button_(idx, pressed);
    }

    // WASD → 左摇杆
    if (code == KEY_A || code == KEY_D || code == KEY_W || code == KEY_S) {
      std::lock_guard<std::mutex> lk(kb_mtx_);
      bool old0 = kb_axes_[0], old1 = kb_axes_[1];
      if (code == KEY_A) l_left_  = pressed;
      if (code == KEY_D) l_right_ = pressed;
      if (code == KEY_W) l_up_    = pressed;
      if (code == KEY_S) l_down_  = pressed;
      compose_left_stick_();
      return (old0 != kb_axes_[0]) || (old1 != kb_axes_[1]);
    }

    // 方向键 → 右摇杆
    if (code == KEY_LEFT || code == KEY_RIGHT || code == KEY_UP || code == KEY_DOWN) {
      std::lock_guard<std::mutex> lk(kb_mtx_);
      bool old3 = kb_axes_[3], old4 = kb_axes_[4];
      if (code == KEY_LEFT)  r_left_  = pressed;
      if (code == KEY_RIGHT) r_right_ = pressed;
      if (code == KEY_UP)    r_up_    = pressed;
      if (code == KEY_DOWN)  r_down_  = pressed;
      compose_right_stick_();
      return (old3 != kb_axes_[3]) || (old4 != kb_axes_[4]);
    }

    // Alt → LB + LT（按钮 + 轴）
    if (code == KEY_LEFTALT || code == KEY_RIGHTALT) {
      std::lock_guard<std::mutex> lk(kb_mtx_);
      bool old_b = kb_buttons_[4];
      float old_a = kb_axes_[2];
      alt_down_   = pressed;
      kb_buttons_[4] = alt_down_ ? 1 : 0;         // LB
      kb_axes_[2]    = alt_down_ ? -1.0f : +1.0f; // LT
      return (old_b != kb_buttons_[4]) || (old_a != kb_axes_[2]);
    }

    // Space → RB + RT（按钮 + 轴）
    if (code == KEY_SPACE) {
      std::lock_guard<std::mutex> lk(kb_mtx_);
      bool  old_b = kb_buttons_[5];
      float old_a = kb_axes_[5];
      space_down_ = pressed;
      kb_buttons_[5] = space_down_ ? 1 : 0;       // RB
      kb_axes_[5]    = space_down_ ? -1.0f : +1.0f; // RT
      return (old_b != kb_buttons_[5]) || (old_a != kb_axes_[5]);
    }

    return changed;
  }

  void compose_left_stick_()
  {
    kb_axes_[0] = (l_right_ ? 1.0f : 0.0f) + (l_left_ ? -1.0f : 0.0f);
    kb_axes_[1] = (l_up_    ? 1.0f : 0.0f) + (l_down_ ? -1.0f : 0.0f);
  }

  void compose_right_stick_()
  {
    kb_axes_[3] = (r_right_ ? 1.0f : 0.0f) + (r_left_ ? -1.0f : 0.0f);
    kb_axes_[4] = (r_up_    ? 1.0f : 0.0f) + (r_down_ ? -1.0f : 0.0f);
  }

  void publish_current_joy_()
  {
    if (!joy_kb_pub_) return;
    sensor_msgs::msg::Joy msg;
    {
      std::lock_guard<std::mutex> lk(kb_mtx_);
      msg.axes.assign(kb_axes_.begin(), kb_axes_.end());
      msg.buttons.assign(kb_buttons_.begin(), kb_buttons_.end());
    }
    msg.header.stamp = this->now();
    joy_kb_pub_->publish(msg);
  }

private:
  // ===== 原有成员 =====
  std::string joy_topic_, cmd_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

  int   Buttons[8]{};
  float Axes[8]{};

  double       Last_Operation_Duration_Time = 0.0;
  rclcpp::Time last_op_time_;
  rclcpp::Time now_time_;

  bool  MotionFlag = false;
  double guide_x_vel=0, guide_y_vel=0, guide_yaw_vel=0;

  // ===== 键盘相关（新增）=====
  bool   keyboard_enabled_{false};

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_kb_pub_;
  std::thread kb_thread_;
  std::atomic<bool> kb_running_{false};
  std::mutex kb_mtx_;

  std::vector<int> kb_fds_;                 // 监听的所有键盘 event fd
  std::array<int,   8> kb_buttons_{};       // A,B,X,Y,LB,RB,SELECT,START
  std::array<float, 8> kb_axes_{};          // 0/1 左摇杆, 2 LT, 3/4 右摇杆, 5 RT

  bool l_left_{false},  l_right_{false}, l_up_{false},  l_down_{false};
  bool r_left_{false},  r_right_{false}, r_up_{false},  r_down_{false};
  bool alt_down_{false}, space_down_{false};
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
