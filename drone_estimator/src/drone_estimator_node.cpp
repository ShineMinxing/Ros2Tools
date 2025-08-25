#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

extern "C" {
  #include "EstimatorPortN.h"
}

using std::placeholders::_1;

class DroneEstimatorNode : public rclcpp::Node {
public:
    explicit DroneEstimatorNode(const rclcpp::NodeOptions &options)
    : Node("drone_estimator_node", options)
    {
        // 参数声明
        declare_parameter<std::string>("input_obs_topic", "/SMX/YOLO_Targets");
        declare_parameter<std::string>("output_state_topic", "/SMX/DroneStateEstimate");
        declare_parameter<bool>("pick_nearest_axis", false);
        declare_parameter<double>("angle_axis_weight", 1.0);
        declare_parameter<double>("x_apt", 0.0);
        declare_parameter<double>("y_apt", 0.0);
        declare_parameter<double>("z_apt", 0.0);
        declare_parameter<double>("g", 9.8);
        declare_parameter<double>("drag_k", 0.0123);
        declare_parameter<double>("c_int", 81.3);

        // TF 相关参数
        declare_parameter<std::string>("parent_frame_id", "map");
        declare_parameter<std::string>("child_frame_id",  "uav");

        // 取参
        input_obs_topic_     = get_parameter("input_obs_topic").as_string();
        output_state_topic_  = get_parameter("output_state_topic").as_string();
        par_x_apt            = get_parameter("x_apt").as_double();
        par_y_apt            = get_parameter("y_apt").as_double();
        par_z_apt            = get_parameter("z_apt").as_double();
        par_g                = get_parameter("g").as_double();
        par_k                = get_parameter("drag_k").as_double();
        par_c_int            = get_parameter("c_int").as_double();
        parent_frame_id_     = get_parameter("parent_frame_id").as_string();
        child_frame_id_      = get_parameter("child_frame_id").as_string();

        // 初始化 C 估计端口
        StateSpaceModel3_Initialization(&StateSpaceModel3_);
        nx_ = StateSpaceModel3_.Nx;
        nz_ = StateSpaceModel3_.Nz;

        // 覆盖 Double_Par
        StateSpaceModel3_.Double_Par[0]  = par_x_apt; // x_apt
        StateSpaceModel3_.Double_Par[1]  = par_y_apt; // y_apt
        StateSpaceModel3_.Double_Par[2]  = par_z_apt; // z_apt
        StateSpaceModel3_.Double_Par[10] = par_g;     // g
        StateSpaceModel3_.Double_Par[11] = par_k;     // drag_k
        StateSpaceModel3_.Double_Par[12] = par_c_int; // c_int

        // 订阅 YOLO 观测（Float64MultiArray：N×5）
        sub_obs_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            input_obs_topic_, rclcpp::QoS(10),
            std::bind(&DroneEstimatorNode::obsCallback, this, _1));

        // 发布估计状态
        pub_state_ = create_publisher<std_msgs::msg::Float64MultiArray>(output_state_topic_, 10);

        // TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    ~DroneEstimatorNode() override {
        StateSpaceModel3_EstimatorPortTermination(&StateSpaceModel3_);
    }

private:
    void obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // data: [t_k, t_E, d, roll, pitch] * N
        const auto &data = msg->data;
        if (data.empty()) return;

        const int stride = 5;
        if (data.size() % stride != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "接收到的数据长度 %zu 不是 5 的整数倍，忽略本次。", data.size());
            return;
        }
        const int n = static_cast<int>(data.size() / stride);
        if (n <= 0) return;

        int pick = 0;

        // 组装 z（5 维）
        double z5[5];
        for (int j = 0; j < 5; ++j) z5[j] = data[pick*stride + j];

        // 时间戳
        const double ts = this->now().seconds();

        // 调用 C 接口估计
        StateSpaceModel3_EstimatorPort(z5, ts, &StateSpaceModel3_);

        // 发布 EstimatedState（长度 nx_）
        std_msgs::msg::Float64MultiArray out;
        out.layout.dim.clear();
        std_msgs::msg::MultiArrayDimension dim;
        dim.label  = "state";
        dim.size   = nx_;
        dim.stride = nx_;
        out.layout.dim.push_back(dim);
        out.layout.data_offset = 0;

        out.data.resize(nx_);
        for (int i = 0; i < nx_; ++i) {
            out.data[i] = StateSpaceModel3_.EstimatedState[i];
        }
        pub_state_->publish(out);

        // === 发送 TF: map -> uav ===
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = parent_frame_id_;
        tf_msg.child_frame_id  = child_frame_id_;
        tf_msg.transform.translation.x = StateSpaceModel3_.EstimatedState[0];
        tf_msg.transform.translation.y = StateSpaceModel3_.EstimatedState[3];
        tf_msg.transform.translation.z = StateSpaceModel3_.EstimatedState[6];
        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_msg);

        geometry_msgs::msg::TransformStamped tf_pre;
        tf_pre.header.stamp = this->now();
        tf_pre.header.frame_id = parent_frame_id_;
        tf_pre.child_frame_id  = "uav_pre";   // 你要的名字
        tf_pre.transform.translation.x = StateSpaceModel3_.PredictedState[0];
        tf_pre.transform.translation.y = StateSpaceModel3_.PredictedState[3];
        tf_pre.transform.translation.z = StateSpaceModel3_.PredictedState[6];
        tf_pre.transform.rotation.x = 0.0;
        tf_pre.transform.rotation.y = 0.0;
        tf_pre.transform.rotation.z = 0.0;
        tf_pre.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_pre);
    }

    // 参数 & 句柄
    std::string input_obs_topic_;
    std::string output_state_topic_;
    double par_x_apt{0.0}, par_y_apt{0.0}, par_z_apt{0.0}, par_g{9.8}, par_k{0.0123}, par_c_int{81.3};
    std::string parent_frame_id_{"map"};
    std::string child_frame_id_{"uav"};

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_state_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // C 估计器实例
    int nx_{9};
    int nz_{5};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.arguments({
        "--ros-args",
        "--params-file",
        "/home/unitree/ros2_ws/LeggedRobot/src/Ros2Tools/config.yaml"
    });

    auto node = std::make_shared<DroneEstimatorNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
