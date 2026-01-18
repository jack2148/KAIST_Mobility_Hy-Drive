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

struct Point {
    double x;
    double y;
};

class StanleyP2 : public rclcpp::Node
{
public:
    StanleyP2()
    : Node("stanley_p2_node")
    {
        // QoS 설정
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // Subscriber: /Ego_pose
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyP2::pose_callback, this, _1));
        
        // Publisher: /Accel
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        // 비상 정지 명령 수신
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; 
            });

        // [P2] 차선 변경 명령 수신 (1, 2, 3)
        sub_lane_cmd_ = this->create_subscription<std_msgs::msg::Int32>(
            "/cmd_lane", qos_profile,
            std::bind(&StanleyP2::callback_lane_cmd, this, _1));

        // [P2] 속도 제어 명령 수신
        sub_speed_cmd_ = this->create_subscription<std_msgs::msg::Float64>(
            "/cmd_speed", qos_profile,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                this->target_speed_ = msg->data;
            });

        // === 파라미터 설정 ===
        this->declare_parameter("csv_path_1", "tool/fastp2.csv");
        this->declare_parameter("csv_path_2", "tool/slowp2.csv");
        this->declare_parameter("csv_path_3", "tool/staticp2.csv");
        
        this->declare_parameter("k_gain", 1.7);
        this->declare_parameter("max_steer", 0.9);
        this->declare_parameter("target_speed", 2.0);
        this->declare_parameter("center_to_front", 0.17);
        this->declare_parameter("wheelbase", 0.33);       
        this->declare_parameter("steer_gain", 1.0);
        this->declare_parameter("forward_step", 15);

        // 파라미터 로드
        std::string path1 = this->get_parameter("csv_path_1").as_string();
        std::string path2 = this->get_parameter("csv_path_2").as_string();
        std::string path3 = this->get_parameter("csv_path_3").as_string();

        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 
    
        // 3개의 경로 파일 로딩
        load_waypoints(path1, waypoints_lane1_);
        load_waypoints(path2, waypoints_lane2_);
        load_waypoints(path3, waypoints_lane3_);

        // 초기 경로 설정
        if (!waypoints_lane1_.empty()) current_waypoints_ = &waypoints_lane1_;
        else current_waypoints_ = &waypoints_lane2_;
        
        RCLCPP_INFO(this->get_logger(), "Stanley P2 Initialized. Skip first 5 poses.");
    }

private:
    void load_waypoints(const std::string& path, std::vector<Point>& target_vector) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", path.c_str());
            return;
        }
        std::string line;
        std::getline(file, line); // 헤더 스킵
        target_vector.clear();
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) { row.push_back(cell); }
            if (row.size() >= 2) {
                try { target_vector.push_back({std::stod(row[0]), std::stod(row[1])}); } 
                catch (...) { continue; }
            }
        }
    }

    void callback_lane_cmd(const std_msgs::msg::Int32::SharedPtr msg) {
        int lane_num = msg->data;
        std::vector<Point>* next_waypoints = nullptr;
        if (lane_num == 1) next_waypoints = &waypoints_lane1_;
        else if (lane_num == 2) next_waypoints = &waypoints_lane2_;
        else if (lane_num == 3) next_waypoints = &waypoints_lane3_;

        if (next_waypoints && !next_waypoints->empty() && current_waypoints_ != next_waypoints) {
            current_waypoints_ = next_waypoints;
        }
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ★ 상태 변수 추가
    int pose_init_count_ = 0; // 초기 데이터 무시 카운터
    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    bool is_reset_check_active_ = false;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 1. [핵심] 처음 5개 데이터 무시
        if (pose_init_count_ < 5) {
            pose_init_count_++;
            return; // 그냥 리턴
        }

        if (stop_signal_) {
            auto stop_msg = geometry_msgs::msg::Accel();
            stop_msg.linear.x = 0.0; stop_msg.angular.z = 0.0; 
            pub_accel_->publish(stop_msg);
            return; 
        }
        if (current_waypoints_->empty()) return;

        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 리셋 감지 (갑자기 위치가 3m 이상 튀면 다시 5개 무시)
        if (is_reset_check_active_) {
            double jump = std::hypot(center_x - prev_x_, center_y - prev_y_);
            if (jump > 3.0) {
                pose_init_count_ = 0; // 카운터 초기화 -> 다시 5개 무시
                is_reset_check_active_ = false;
                return;
            }
        }
        prev_x_ = center_x;
        prev_y_ = center_y;
        is_reset_check_active_ = true;

        // 3. 전륜 차축 계산
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 4. 가장 가까운 Waypoint 찾기 (전역 검색)
        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_waypoints_->size(); ++i) {
            double dx = front_x - (*current_waypoints_)[i].x;
            double dy = front_y - (*current_waypoints_)[i].y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        if (nearest_idx == -1) return;

        // 5. 제어 계산
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        double map_x = (*current_waypoints_)[nearest_idx].x;
        double map_y = (*current_waypoints_)[nearest_idx].y;
        double next_map_x = (*current_waypoints_)[next_nearest_idx].x;
        double next_map_y = (*current_waypoints_)[next_nearest_idx].y;

        double path_dx = next_map_x - map_x;
        double path_dy = next_map_y - map_y;
        double path_len = std::hypot(path_dx, path_dy);
        if (path_len < 1e-6) path_len = 1e-6;

        double dx = front_x - map_x;
        double dy = front_y - map_y;
        double cross_product = dx * path_dy - dy * path_dx; 
        double cte = cross_product / path_len;

        // Lookahead
        int target_idx = (nearest_idx + forward_step_) % current_waypoints_->size();
        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // Stanley Law
        double v_clamped = std::max(target_speed_, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);
        double steer_angle = heading_error + cte_correction; 
        
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // Publish
        auto msg_out = geometry_msgs::msg::Accel();
        double final_speed = target_speed_;
        
        if (final_speed < 0.1) {
            msg_out.angular.z = 0.0;
        } else {
            msg_out.angular.z = (final_speed / wheelbase_) * std::tan(steer_angle);
        }
        msg_out.linear.x = final_speed;
        pub_accel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_lane_cmd_; 
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_speed_cmd_; 

    bool stop_signal_ = false;
    std::vector<Point> waypoints_lane1_, waypoints_lane2_, waypoints_lane3_;
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