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
        declare_parameter<double>("score_threshold", 7);
        declare_parameter<double>("g", 9.8);
        declare_parameter<double>("drag_k", 0.0123);
        declare_parameter<double>("c_int", 81.3);
        declare_parameter<std::string>("parent_frame_id", "map");
        declare_parameter<std::string>("child_frame_id",  "uav");

        // 取参
        input_obs_topic_     = get_parameter("input_obs_topic").as_string();
        output_state_topic_  = get_parameter("output_state_topic").as_string();
        score_threshold      = get_parameter("score_threshold").as_double();
        par_g                = get_parameter("g").as_double();
        par_k                = get_parameter("drag_k").as_double();
        par_c_int            = get_parameter("c_int").as_double();
        parent_frame_id_     = get_parameter("parent_frame_id").as_string();
        child_frame_id_      = get_parameter("child_frame_id").as_string();

        // 打印参数
        RCLCPP_INFO(this->get_logger(), "===== DroneEstimator 参数 =====");
        RCLCPP_INFO(this->get_logger(), "input_obs_topic    : %s", input_obs_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "output_state_topic : %s", output_state_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "score_threshold    : %.3f", score_threshold);
        RCLCPP_INFO(this->get_logger(), "g                  : %.3f", par_g);
        RCLCPP_INFO(this->get_logger(), "drag_k             : %.5f", par_k);
        RCLCPP_INFO(this->get_logger(), "c_int              : %.3f", par_c_int);
        RCLCPP_INFO(this->get_logger(), "parent_frame_id    : %s", parent_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "child_frame_id     : %s", child_frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "================================");

        // 初始化 C 估计端口
        StateSpaceModel3_Initialization(&StateSpaceModel3_);

        lastTime = this->now().seconds();
        StateSpaceModel3_.StateUpdateTimestamp = lastTime;
        nx_ = StateSpaceModel3_.Nx;
        nz_ = StateSpaceModel3_.Nz;

        // 覆盖 Double_Par
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
        // 1) 基本校验
        if (msg->data.empty() || msg->layout.dim.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "数据或布局为空。");
            return;
        }

        const auto &data   = msg->data;
        const auto &label0 = msg->layout.dim[0];

        // 2) 用 layout 推断 N 与每条观测维度 F (= stride/size)
        if (label0.size == 0 || label0.stride == 0 || label0.stride % label0.size != 0 || (label0.stride / label0.size) != (nz_+1) || static_cast<size_t>(label0.stride) != msg->data.size() || label0.size >= 20) 
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "布局或数据不合法: size=%u stride=%u nz_=%d data=%zu", label0.size, label0.stride, nz_, msg->data.size());
            return;
        }

        // 3) 解析 gimbal_location（来自 dim[0].label = "location:x,y,z"）
        try {
            const std::string &label = label0.label;
            const std::string tag = "location:";
            auto p = label.find(tag);
            if (p != std::string::npos) {
                std::string s = label.substr(p + tag.size()); // "x,y,z"
                size_t p1 = s.find(',');
                size_t p2 = (p1 == std::string::npos) ? std::string::npos : s.find(',', p1 + 1);
                if (p1 != std::string::npos && p2 != std::string::npos) {
                    StateSpaceModel3_.Double_Par[0] = std::stod(s.substr(0, p1));                  // x_apt
                    StateSpaceModel3_.Double_Par[1] = std::stod(s.substr(p1 + 1, p2 - p1 - 1));    // y_apt
                    StateSpaceModel3_.Double_Par[2] = std::stod(s.substr(p2 + 1));                 // z_apt
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "解析 gimbal_location 失败: %s", e.what());
            return;
        }

        // 4) 取第最佳观测
        double currentTime = this->now().seconds();
        double obser_temp[5], obser_error[8], score[20], pick[5], best_score = 0.0, dT = currentTime - lastTime;

        if(!InitiateFlag)
        {
            for (int i = 0; i < label0.size; ++i)
            {
                for (int j = 0; j < nz_; ++j) 
                    obser_temp[j] = data[i * (nz_+1) + j];
                score[i] = data[i * (nz_+1) + 5];
                if(score[i] > best_score)
                {
                    best_score = score[i];
                    for (int j = 0; j < nz_; ++j)
                        pick[j] = obser_temp[j];
                }
            }
            if(best_score<0.7) return;
            InitiateFlag = true;

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100, "pick1=%.3lf,pick2=%.3lf,pick3=%.3lf,pick4=%.3lf,pick5=%.3lf", pick[0],pick[1],pick[2],pick[3],pick[4]);
        }
        else
        {
            double dx = StateSpaceModel3_.EstimatedState[0] - StateSpaceModel3_.Double_Par[0];
            double dy = StateSpaceModel3_.EstimatedState[3] - StateSpaceModel3_.Double_Par[1];
            double dz = StateSpaceModel3_.EstimatedState[6] - StateSpaceModel3_.Double_Par[2];

            double z1_est = atan2(dy, dx);
            double z2_est = atan2(dz, sqrt(dx*dx + dy*dy));
            double z3_est = sqrt(dx*dx + dy*dy + dz*dz);

            double z1_scp = abs(StateSpaceModel3_.EstimatedState[1] * dT) + abs(StateSpaceModel3_.EstimatedState[4] * dT)  + abs(StateSpaceModel3_.EstimatedState[2] * StateSpaceModel3_.EstimatedState[2] * dT * dT / 2 /2) + abs(StateSpaceModel3_.EstimatedState[5] * StateSpaceModel3_.EstimatedState[5] * dT * dT / 2 /2);
            double z2_scp = abs(StateSpaceModel3_.EstimatedState[7] * dT) + abs(StateSpaceModel3_.EstimatedState[8] * StateSpaceModel3_.EstimatedState[8] * dT * dT / 2 /2);
            double z3_scp = z1_scp + z2_scp;
            z1_scp = atan2(z1_scp, z3_est);
            z2_scp = atan2(z2_scp, z3_est);

            for (int i = 0; i < label0.size; ++i)
            {
                for (int j = 0; j < nz_; ++j)
                    obser_temp[j] = data[i * (nz_+1) + j];

                obser_error[0] = abs(obser_temp[0] - z1_est + 0.01);
                obser_error[1] = abs(obser_temp[1] - z2_est + 0.01);
                obser_error[2] = abs(obser_temp[2] - z3_est + 0.01);

                score[i] = z1_scp/obser_error[0] + z2_scp/obser_error[1] + z3_scp/obser_error[2] + data[i * (nz_+1) + 5] + dT;

                if(score[i] > best_score)
                {
                    best_score = score[i];
                    obser_error[3] = z1_scp/obser_error[0];
                    obser_error[4] = z2_scp/obser_error[1];
                    obser_error[5] = z3_scp/obser_error[2];
                    obser_error[6] = z1_scp/data[i * (nz_+1) + 5];
                    obser_error[7] = dT;
                    for (int j = 0; j < nz_; ++j)
                        pick[j] = obser_temp[j];              
                }
            }

            if(best_score < score_threshold)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 33, "没有可信的目标, best_score=%.3lf, part_1=%.3lf, part_2=%.3lf, part_3=%.3lf, part_4=%.3lf, part_5=%.3lf,", best_score,obser_error[3],obser_error[4],obser_error[5],obser_error[6],obser_error[7]);
                return;
            }
        }

        lastTime = currentTime;

        // 调用 C 接口估计
        StateSpaceModel3_EstimatorPort(pick, currentTime, &StateSpaceModel3_);

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
    double par_x_apt{0.0}, par_y_apt{0.0}, par_z_apt{0.0}, score_threshold{7}, par_g{9.8}, par_k{0.0123}, par_c_int{81.3};
    std::string parent_frame_id_{"map"};
    std::string child_frame_id_{"uav"};

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_state_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // C 估计器实例
    int nx_{9};
    int nz_{5};
    double lastTime;
    bool InitiateFlag = false;
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
