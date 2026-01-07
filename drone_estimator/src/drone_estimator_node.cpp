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
#include <visualization_msgs/msg/marker.hpp>   // ★ 新增：RViz Marker

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
        declare_parameter<std::string>("rviz_mesh_resource",
            "/home/smx/WorkSpace/GDS_LeggedRobot/src/Ros2Tools/drone_estimator/src/Drone.STL");
        declare_parameter<std::vector<double>>("rviz_scale", {1.0, 1.0, 1.0});
        declare_parameter<std::vector<double>>("rviz_color", {0.6, 0.65, 0.7, 1.0});
        declare_parameter<std::string>("rviz_ns", "drone_estimator");

        // 取参
        input_obs_topic_     = get_parameter("input_obs_topic").as_string();
        output_state_topic_  = get_parameter("output_state_topic").as_string();
        score_threshold      = get_parameter("score_threshold").as_double();
        par_g                = get_parameter("g").as_double();
        par_k                = get_parameter("drag_k").as_double();
        par_c_int            = get_parameter("c_int").as_double();
        parent_frame_id_     = get_parameter("parent_frame_id").as_string();
        child_frame_id_      = get_parameter("child_frame_id").as_string();
        rviz_mesh_resource_ = get_parameter("rviz_mesh_resource").as_string();
        rviz_scale_         = get_parameter("rviz_scale").as_double_array();
        rviz_color_         = get_parameter("rviz_color").as_double_array();
        rviz_ns_            = get_parameter("rviz_ns").as_string();

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
        RCLCPP_INFO(this->get_logger(), "rviz_mesh_resource : %s", rviz_mesh_resource_.c_str());
        RCLCPP_INFO(this->get_logger(), "rviz_scale         : [%.2f, %.2f, %.2f]", rviz_scale_[0], rviz_scale_[1], rviz_scale_[2]);
        RCLCPP_INFO(this->get_logger(), "rviz_color         : [%.2f, %.2f, %.2f, %.2f]", rviz_color_[0], rviz_color_[1], rviz_color_[2], rviz_color_[3]);
        RCLCPP_INFO(this->get_logger(), "rviz_ns            : %s", rviz_ns_.c_str());
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

        // RViz Marker 发布器
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
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
        double obser_temp[6], obser_error[3], scope[4] = {0}, record[10], score, best_score = 0.0, pick[5], dT = currentTime - lastTime;

        if(!InitiateFlag) //每个消息中可能检测到多个目标，初始化时使用可能性最大的目标作为无人机
        {
            for (int i = 0; i < label0.size; ++i)
            {
                for (int j = 0; j < nz_; ++j) 
                    obser_temp[j] = data[i * (nz_+1) + j];
                score = data[i * (nz_+1) + 5];
                if(score > best_score)
                {
                    best_score = score;
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
            // 获取估计的无人机位置
            double x_est = StateSpaceModel3_.EstimatedState[0];
            double y_est = StateSpaceModel3_.EstimatedState[3];
            double z_est = StateSpaceModel3_.EstimatedState[6];

            double x_obser = 0.0, y_obser = 0.0, z_obser = 0.0;

            // 
            for (int i = 0; i < label0.size; ++i)
            {
                for (int j = 0; j <= nz_; ++j)
                    obser_temp[j] = data[i * (nz_+1) + j];

                x_obser = StateSpaceModel3_.Double_Par[0] + obser_temp[2] * cos(obser_temp[1]) * cos(obser_temp[0]);
                y_obser = StateSpaceModel3_.Double_Par[1] + obser_temp[2] * cos(obser_temp[1]) * sin(-obser_temp[0]);
                z_obser = StateSpaceModel3_.Double_Par[2] + obser_temp[2] * sin(obser_temp[1]);
                
                scope[0] = obser_temp[2] / 20;
                scope[1] = abs(StateSpaceModel3_.EstimatedState[1]*dT + StateSpaceModel3_.EstimatedState[2]/2*dT*dT + scope[0]);
                scope[2] = abs(StateSpaceModel3_.EstimatedState[4]*dT + StateSpaceModel3_.EstimatedState[5]/2*dT*dT + scope[0]);
                scope[3] = abs(StateSpaceModel3_.EstimatedState[7]*dT + StateSpaceModel3_.EstimatedState[8]/2*dT*dT + scope[0]);

                obser_error[0] = abs(x_obser - x_est + 0.0001);
                obser_error[1] = abs(y_obser - y_est + 0.0001);
                obser_error[2] = abs(z_obser - z_est + 0.0001);

                score = (std::clamp(scope[1]/obser_error[0],0.0,0.5) + std::clamp(scope[2]/obser_error[1],0.0,0.5) + std::clamp(scope[3]/obser_error[2],0.0,0.5)) * obser_temp[5] + dT*10;

                if(score > best_score && StateSpaceModel3_.Double_Par[1] != -2)
                {
                    best_score = score;
                    record[0] = obser_temp[2]*5/obser_temp[5];
                    // record[1] = std::clamp(scope[1]/obser_error[0],0.0,0.5);
                    // record[2] = std::clamp(scope[2]/obser_error[1],0.0,0.5);
                    // record[3] = std::clamp(scope[3]/obser_error[2],0.0,0.5);
                    record[1] = StateSpaceModel3_.Double_Par[0];
                    record[2] = StateSpaceModel3_.Double_Par[1];
                    record[3] = StateSpaceModel3_.Double_Par[2];
                    record[4] = x_obser;
                    record[5] = y_obser;
                    record[6] = z_obser;
                    for (int j = 0; j < nz_; ++j)
                        pick[j] = obser_temp[j];              
                }
            }

            if(best_score < score_threshold)
            {
                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 33, "没有可信的目标, best_score=%.3lf, R=%.3lf,\n part_1=%.3lf, part_2=%.3lf, part_3=%.3lf, part_4=%.3lf, part_5=%.3lf, part_6=%.3lf", best_score,record[0],record[1],record[2],record[3],record[4],record[5],record[6]);

                // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 33, "PASS: best_score=%.3lf,X=%.3lf,Y=%.3lf,Z=%.3lf,o1=%.3lf,o2=%.3lf,o3=%.3lf,o4=%.3lf,o5=%.3lf", best_score,StateSpaceModel3_.Double_Par[0],StateSpaceModel3_.Double_Par[1],StateSpaceModel3_.Double_Par[2],pick[0]/3.1415*180,pick[1]/3.1415*180,pick[2],pick[3]/3.1415*180,pick[4]/3.1415*180);
                return;
            }else
            {
                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 33, "选中的目标, best_score=%.3lf, R=%.3lf,\n part_1=%.3lf, part_2=%.3lf, part_3=%.3lf, part_4=%.3lf, part_5=%.3lf, part_6=%.3lf", best_score,record[0],record[1],record[2],record[3],record[4],record[5],record[6]);

                std::cout << std::showpos; // 显示正号
                std::cout << std::fixed << std::setprecision(3); // 固定小数点后3位
                
                std::cout
                    << " X:" << std::setw(6) << std::internal << record[1]
                    << " Y:" << std::setw(6) << std::internal << record[2]
                    << " Z:" << std::setw(6) << std::internal << record[3]
                    << " x:" << std::setw(6) << std::internal << record[4]
                    << " y:" << std::setw(6) << std::internal << record[5]
                    << " z:" << std::setw(6) << std::internal << record[6]
                    << std::endl;

                // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 33, "X=%.3lf,Y=%.3lf,Z=%.3lf,o1=%.3lf,o2=%.3lf,o3=%.3lf,o4=%.3lf,o5=%.3lf", StateSpaceModel3_.Double_Par[0],StateSpaceModel3_.Double_Par[1],StateSpaceModel3_.Double_Par[2],pick[0]/3.1415*180,pick[1]/3.1415*180,pick[2],pick[3]/3.1415*180,pick[4]/3.1415*180);
            }
        }

        lastTime = currentTime;

        StateSpaceModel3_.Matrix_R[nz_ * 2 + 2] = record[0];

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
        tf_pre.child_frame_id  = child_frame_id_ + "_pre";
        tf_pre.transform.translation.x = StateSpaceModel3_.PredictedState[0];
        tf_pre.transform.translation.y = StateSpaceModel3_.PredictedState[3];
        tf_pre.transform.translation.z = StateSpaceModel3_.PredictedState[6];
        tf_pre.transform.rotation.x = 0.0;
        tf_pre.transform.rotation.y = 0.0;
        tf_pre.transform.rotation.z = 0.0;
        tf_pre.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(tf_pre);

        // RViz 网格模型 Marker（位姿与 tf_msg 一致，直接在 map 坐标系下显示）===
        if (marker_pub_) {
            visualization_msgs::msg::Marker drone_now;
            drone_now.header.stamp = this->now();
            drone_now.header.frame_id = parent_frame_id_;
            drone_now.ns = rviz_ns_ + "_now";
            drone_now.id = 1;
            drone_now.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            drone_now.action = visualization_msgs::msg::Marker::ADD;

            // 位置：采用当前估计位置
            drone_now.pose.position.x = StateSpaceModel3_.EstimatedState[0];
            drone_now.pose.position.y = StateSpaceModel3_.EstimatedState[3];
            drone_now.pose.position.z = StateSpaceModel3_.EstimatedState[6];
            // 姿态：无旋转（如有需要可改为估计朝向）
            drone_now.pose.orientation.x = 0.0;
            drone_now.pose.orientation.y = 0.0;
            drone_now.pose.orientation.z = 0.0;
            drone_now.pose.orientation.w = 1.0;

            // 网格资源：file:// 绝对路径（RViz 需要 URI）
            drone_now.mesh_resource = std::string("file://") + rviz_mesh_resource_;
            drone_now.mesh_use_embedded_materials = false;

            drone_now.color.r = rviz_color_[0];
            drone_now.color.g = rviz_color_[1];
            drone_now.color.b = rviz_color_[2];
            drone_now.color.a = rviz_color_[3];

            drone_now.scale.x = rviz_scale_[0];
            drone_now.scale.y = rviz_scale_[1];
            drone_now.scale.z = rviz_scale_[2];

            drone_now.lifetime = rclcpp::Duration(0,0);
            marker_pub_->publish(drone_now);


            visualization_msgs::msg::Marker drone_pre;
            drone_pre.header.stamp = this->now();
            drone_pre.header.frame_id = parent_frame_id_;
            drone_pre.ns = rviz_ns_ + "_pre";
            drone_pre.id = 1;
            drone_pre.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            drone_pre.action = visualization_msgs::msg::Marker::ADD;

            // 位置：采用当前估计位置
            drone_pre.pose.position.x = StateSpaceModel3_.PredictedState[0];
            drone_pre.pose.position.y = StateSpaceModel3_.PredictedState[3];
            drone_pre.pose.position.z = StateSpaceModel3_.PredictedState[6];
            // 姿态：无旋转（如有需要可改为估计朝向）
            drone_pre.pose.orientation.x = 0.0;
            drone_pre.pose.orientation.y = 0.0;
            drone_pre.pose.orientation.z = 0.0;
            drone_pre.pose.orientation.w = 1.0;

            // 网格资源：file:// 绝对路径（RViz 需要 URI）
            drone_pre.mesh_resource = std::string("file://") + rviz_mesh_resource_;
            drone_pre.mesh_use_embedded_materials = false;

            drone_pre.color.r = 0;
            drone_pre.color.g = 0;
            drone_pre.color.b = 0.5;
            drone_pre.color.a = 0.2;

            drone_pre.scale.x = rviz_scale_[0];
            drone_pre.scale.y = rviz_scale_[1];
            drone_pre.scale.z = rviz_scale_[2];

            drone_pre.lifetime = rclcpp::Duration(0,0);
            marker_pub_->publish(drone_pre);
        }
    }

    // 参数 & 句柄
    std::string input_obs_topic_;
    std::string output_state_topic_;
    double par_x_apt{0.0}, par_y_apt{0.0}, par_z_apt{0.0}, score_threshold{7}, par_g{9.8}, par_k{0.0123}, par_c_int{81.3};
    std::string parent_frame_id_{"map"};
    std::string child_frame_id_{"uav"};
    std::string rviz_mesh_resource_;
    std::vector<double> rviz_scale_;
    std::vector<double> rviz_color_;
    std::string rviz_ns_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_state_;
    std::unique_ptr<tf2_ros::TransformBroadcaster>  tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

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
        "/home/smx/WorkSpace/GDS_LeggedRobot/src/Ros2Tools/config.yaml"
    });

    auto node = std::make_shared<DroneEstimatorNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}