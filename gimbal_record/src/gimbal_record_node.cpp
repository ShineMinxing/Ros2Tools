#include <chrono>
#include <fstream>
#include <vector>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;
using std::placeholders::_1;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
using std_msgs::msg::Float64MultiArray;

class GimbalRecorder : public rclcpp::Node
{
public:
  GimbalRecorder() : Node("gimbal_recorder"), frame_count_(0)
  {
    /* ---------- 1. 话题订阅 ---------- */
    sub_img_ = create_subscription<Image>(
      "/SMX/Camera_Raw", 10,
      std::bind(&GimbalRecorder::on_image, this, _1));
    sub_bt_ = create_subscription<Float64MultiArray>(
      "/SMX/BTAngle", 10, std::bind(&GimbalRecorder::on_bt, this, _1));
    sub_gs_ = create_subscription<Float64MultiArray>(
      "/SMX/GimbalState", 10, std::bind(&GimbalRecorder::on_gs, this, _1));
    sub_imu_ = create_subscription<Imu>(
      "/SMX/BTIMU", 10, std::bind(&GimbalRecorder::on_imu, this, _1));

    /* ---------- 2. 生成时间后缀 ---------- */
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
    localtime_r(&tt, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "_%Y%m%d_%H%M%S");
    suffix_ = oss.str();

    /* ---------- 3. 输出目录 ---------- */
    auto share = ament_index_cpp::get_package_share_directory("gimbal_record");
    fs::path ws_root = fs::path(share).parent_path().parent_path().parent_path().parent_path();
    out_dir_ = ws_root / "src" / "Ros2Tools" / "local_file";
    fs::create_directories(out_dir_);

    /* ---------- 4. 打开 CSV ---------- */
    csv_path_ = out_dir_ / ("Msg" + suffix_ + ".csv");
    csv_.open(csv_path_);
    csv_ << "time_s,"
            "angle_x,angle_y,tilt,"
            "g0,g1,g2,g3,g4,g5,"
            "ori_x,ori_y,ori_z,ori_w,"
            "av_x,av_y,av_z,"
            "la_x,la_y,la_z\n";

    RCLCPP_INFO(get_logger(), "Recorder ready. CSV: %s", csv_path_.c_str());
  }

  ~GimbalRecorder() override
  {
    if (video_writer_.isOpened()) video_writer_.release();
    if (csv_.is_open()) csv_.close();

    RCLCPP_INFO(get_logger(), "Total frames recorded: %zu", frame_count_);
    fs::path vp = out_dir_ / ("Camera" + suffix_ + ".mp4");
    if (fs::exists(vp))
      RCLCPP_INFO(get_logger(), "MP4 size: %zu bytes", fs::file_size(vp));
  }

private:
  /* ---------- 回调缓存 ---------- */
  void on_bt (const Float64MultiArray::SharedPtr m){ latest_bt_  = *m; }
  void on_gs (const Float64MultiArray::SharedPtr m){ latest_gs_  = *m; }
  void on_imu(const Imu::SharedPtr              m){ latest_imu_ = *m; }

  /* ---------- 图像回调 ---------- */
  void on_image(const Image::SharedPtr msg)
  {
    cv_bridge::CvImageConstPtr cvptr;
    try { cvptr = cv_bridge::toCvShare(msg, "bgr8"); }
    catch (const cv_bridge::Exception &e){
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what()); return;
    }
    const cv::Mat &frame = cvptr->image;

    /* ---- 延迟创建 H.264 VideoWriter ---- */
    if (!video_writer_.isOpened()) {
      video_path_ = out_dir_ / ("Camera" + suffix_ + ".mp4");
      video_writer_.open(
        video_path_.string(),
        cv::VideoWriter::fourcc('a','v','c','1'),  // H.264/AVC fourcc
        30.0, frame.size());
      if (!video_writer_.isOpened()){
        RCLCPP_ERROR(get_logger(),
          "Failed to open VideoWriter with H.264 (avc1). "
          "Ensure OpenCV was built with FFmpeg + x264.");          // :contentReference[oaicite:1]{index=1}
        return;
      }
      RCLCPP_INFO(get_logger(), "Writing H.264 video: %s", video_path_.c_str());
    }

    /* ---- 写帧 + 计数 ---- */
    video_writer_.write(frame);
    ++frame_count_;

    /* ---- 写 CSV ---- */
    double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    csv_ << std::fixed << std::setprecision(9) << t << ',';
    write_vec(latest_bt_.data, 3);
    write_vec(latest_gs_.data, 6);
    csv_ << latest_imu_.orientation.x << ',' << latest_imu_.orientation.y << ','
         << latest_imu_.orientation.z << ',' << latest_imu_.orientation.w << ','
         << latest_imu_.angular_velocity.x << ',' << latest_imu_.angular_velocity.y << ','
         << latest_imu_.angular_velocity.z << ','
         << latest_imu_.linear_acceleration.x << ',' << latest_imu_.linear_acceleration.y << ','
         << latest_imu_.linear_acceleration.z << '\n';
  }

  /* ---------- 工具函数 ---------- */
  void write_vec(const std::vector<double>& v, size_t n){
    for (size_t i=0;i<n;++i) csv_ << (i<v.size()? v[i] : 0.0) << ',';
  }

  /* ---------- 成员 ---------- */
  rclcpp::Subscription<Image>::SharedPtr             sub_img_;
  rclcpp::Subscription<Float64MultiArray>::SharedPtr sub_bt_, sub_gs_;
  rclcpp::Subscription<Imu>::SharedPtr               sub_imu_;

  Float64MultiArray latest_bt_, latest_gs_;
  Imu               latest_imu_;

  std::string       suffix_;
  fs::path          out_dir_, csv_path_, video_path_;
  std::ofstream     csv_;
  cv::VideoWriter   video_writer_;
  size_t            frame_count_;
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalRecorder>());
  rclcpp::shutdown();
  return 0;
}
