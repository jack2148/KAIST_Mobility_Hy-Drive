#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/bool.hpp> 
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

struct Point { double x; double y; };

class StanleyP2 : public rclcpp::Node
{
public:
    StanleyP2() : Node("stanley_p2_node")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // 1. Subscribers
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyP2::pose_callback, this, _1));
        
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", qos_profile);

        // [통신 수신] MainController의 명령을 받아 변수 업데이트
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "/CAV_01/cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->stop_signal_ = msg->data; });

        sub_lane_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/CAV_01/cmd_lane", qos_profile,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                current_waypoints_ = (msg->data == 1) ? &waypoints_lane1_ : &waypoints_lane2_;
            });

        // [핵심] Main에서 계산한 Dynamic Speed를 받음
        sub_speed_cmd_ = this->create_subscription<std_msgs::msg::Float64>(
            "/CAV_01/cmd_speed", qos_profile,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->target_speed_ = msg->data; 
            });

        // 2. 파라미터
        this->declare_parameter("csv_path_1", "tool/JSp2Lane1.csv"); 
        this->declare_parameter("csv_path_2", "tool/JSp2Lane2.csv"); 
        this->declare_parameter("k_gain", 1.7);
        this->declare_parameter("max_steer", 0.7);
        this->declare_parameter("center_to_front", 0.17);
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("steer_gain", 1.0);
        this->declare_parameter("forward_step", 15);
        
        // 초기값은 0.0으로 둬서 Main의 명령을 기다리게 함 (안전)
        this->target_speed_ = 0.3; 

        std::string path1 = this->get_parameter("csv_path_1").as_string();
        std::string path2 = this->get_parameter("csv_path_2").as_string();
        
        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double(); 
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 

        load_waypoints(path1, waypoints_lane1_);
        load_waypoints(path2, waypoints_lane2_);

        current_waypoints_ = &waypoints_lane2_;
        RCLCPP_INFO(this->get_logger(), "Stanley P2 Ready. Waiting for Main Controller commands...");
    }

private:
    // ... (load_waypoints, normalize_angle 동일) ...
    void load_waypoints(const std::string& path, std::vector<Point>& target_vec) {
        std::ifstream file(path);
        if (!file.is_open()) return;
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) try { target_vec.push_back({std::stod(row[0]), std::stod(row[1])}); } catch (...) {}
        }
    }
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (stop_signal_) {
            pub_accel_->publish(geometry_msgs::msg::Accel());
            return; 
        }
        if (!current_waypoints_ || current_waypoints_->empty()) return;

        // ... (Stanley 계산 로직 동일, 생략 없이 필요하면 복붙해드림) ...
        const auto& path_points = *current_waypoints_;
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_points.size(); ++i) {
            double dist = std::hypot(front_x - path_points[i].x, front_y - path_points[i].y);
            if (dist < min_dist) { min_dist = dist; nearest_idx = i; }
        }

        if (nearest_idx == -1 || min_dist > 3.0) {
            pub_accel_->publish(geometry_msgs::msg::Accel());
            return;
        }

        int next_idx = (nearest_idx + 1) % path_points.size();
        double map_x = path_points[nearest_idx].x;
        double map_y = path_points[nearest_idx].y;
        double path_dx = path_points[next_idx].x - map_x;
        double path_dy = path_points[next_idx].y - map_y;
        double path_len = std::hypot(path_dx, path_dy);
        if (path_len < 1e-6) path_len = 1e-6;
        double cross_product = (front_x - map_x) * path_dy - (front_y - map_y) * path_dx; 
        double cte = cross_product / path_len;

        int target_idx = (nearest_idx + forward_step_) % path_points.size();
        int next_target_idx = (target_idx + 1) % path_points.size();
        double target_dx = path_points[next_target_idx].x - path_points[target_idx].x;
        double target_dy = path_points[next_target_idx].y - path_points[target_idx].y;
        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // [중요] Main에서 받은 target_speed_ 사용
        double v_clamped = std::max(target_speed_, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);
        double steer_angle = normalize_angle(heading_error + cte_correction);
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle * steer_gain_));

        auto msg_out = geometry_msgs::msg::Accel();
        msg_out.linear.x = target_speed_;
        msg_out.angular.z = (target_speed_ / wheelbase_) * std::tan(steer_angle);
        pub_accel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_lane_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_speed_cmd_;

    bool stop_signal_ = false;
    std::vector<Point> waypoints_lane1_, waypoints_lane2_; 
    std::vector<Point>* current_waypoints_; 
    double k_gain_, max_steer_, target_speed_, center_to_front_, wheelbase_, steer_gain_;
    int forward_step_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyP2>());
    rclcpp::shutdown();
    return 0;
}