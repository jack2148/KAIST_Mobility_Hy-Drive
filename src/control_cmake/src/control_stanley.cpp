#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/bool.hpp> 
#include <std_msgs/msg/float32.hpp> 

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

// ANSI 색상 코드 정의
const std::string ANSI_RESET   = "\033[0m";
const std::string ANSI_GREEN   = "\033[32m";
const std::string ANSI_YELLOW  = "\033[33m";
const std::string ANSI_RED     = "\033[31m";
const std::string ANSI_CYAN    = "\033[36m";

struct Point {
    double x;
    double y;
};

class StanleyTrackerNode : public rclcpp::Node
{
public:
    StanleyTrackerNode()
    : Node("stanley_tracker_node")
    {
        // QoS 설정: SensorData (Best Effort, Volatile)
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // 파라미터 설정
        this->declare_parameter("original_way_path", "tool/cav1p3.csv");
        this->declare_parameter("inside_way_path", "tool/cav1p3_inside.csv");
        this->declare_parameter("k_gain", 2.0);
        this->declare_parameter("max_steer", 0.7);       // 최대 조향각 (rad)
        this->declare_parameter("target_speed", 2.0);    // 기본 속도 (m/s)
        this->declare_parameter("center_to_front", 0.17);// 중심~전륜 거리 (m)
        this->declare_parameter("wheelbase", 0.33);      // 축거 (m)
        this->declare_parameter("steer_gain", 1.0);
        this->declare_parameter("forward_step", 15);     // Lookahead Step

        // 파라미터 로드
        original_csv_path_ = this->get_parameter("original_way_path").as_string();
        inside_csv_path_ = this->get_parameter("inside_way_path").as_string();
        
        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 

        // 경로 파일 로딩
        load_waypoints(original_csv_path_, waypoints_original_);
        load_waypoints(inside_csv_path_, waypoints_inside_);
        
        // 초기 경로는 Original로 설정
        current_waypoints_ = &waypoints_original_;
        is_inside_path_active_ = false;

        // 1. Pose 구독
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
        
        // 2. Accel 발행
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        // 3. [관제탑] 정지 명령
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; 
                if (this->stop_signal_) {
                    auto stop_msg = geometry_msgs::msg::Accel();
                    stop_msg.linear.x = 0.0;  
                    stop_msg.angular.z = 0.0; 
                    this->pub_accel_->publish(stop_msg);
                }
            });

        // 4. [관제탑] 경로 변경 요청 (Inside <-> Original)
        sub_change_way_ = this->create_subscription<std_msgs::msg::Bool>(
            "change_waypoint", qos_profile,
            std::bind(&StanleyTrackerNode::callback_change_waypoint, this, _1));

        // 5. [관제탑] HV 속도 (회전교차로용)
        sub_hv_vel_ = this->create_subscription<std_msgs::msg::Float32>(
            "hv_vel", qos_profile,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->hv_vel_ = msg->data;
            });

        // 6. [관제탑] 회전교차로 상태
        sub_is_roundabout_ = this->create_subscription<std_msgs::msg::Bool>(
            "is_roundabout", qos_profile,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->is_roundabout_ = msg->data;
            });
    }

private:
    void load_waypoints(const std::string& path, std::vector<Point>& target_vector) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", path.c_str());
            return;
        }

        std::string line;
        std::getline(file, line); // Header skip
        target_vector.clear();

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }
            if (row.size() >= 2) {
                Point p;
                try {
                    p.x = std::stod(row[0]);
                    p.y = std::stod(row[1]);
                    target_vector.push_back(p);
                } catch (...) {
                    continue; 
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", target_vector.size(), path.c_str());
    }

    // 경로 변경 콜백
    void callback_change_waypoint(const std_msgs::msg::Bool::SharedPtr msg) {
        bool request_inside = msg->data;

        // Inside 경로 요청이 왔고, 현재 Inside가 아니며, 데이터가 있을 때만 전환
        if (request_inside) {
            if (!is_inside_path_active_ && !waypoints_inside_.empty()) {
                RCLCPP_INFO(this->get_logger(), "%s[SWITCH] Switching to INSIDE PATH%s", ANSI_CYAN.c_str(), ANSI_RESET.c_str());
                is_inside_path_active_ = true;
                current_waypoints_ = &waypoints_inside_;
                // 전체 검색 방식을 사용하므로 인덱스 초기화 불필요
            }
        } 
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 0. 비상 정지 체크
        if (stop_signal_) {
            auto stop_msg = geometry_msgs::msg::Accel();
            stop_msg.linear.x = 0.0;  
            stop_msg.angular.z = 0.0; 
            pub_accel_->publish(stop_msg);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PAUSED by Main Controller");
            return; 
        }

        if (current_waypoints_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current waypoint list is empty!");
            return;
        }

        // 1. 차량 상태 추출
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 전륜(Front Axle) 위치 계산 (Stanley 핵심)
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. Nearest Waypoint Search (전체 검색 - 값이 튀는 현상 방지)
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

        // 4. 경로 복귀 로직 (Inside -> Original)
        // Inside 경로의 끝부분(마지막 5개)에 도달하면 Original로 자동 복귀
        if (is_inside_path_active_) {
            if (nearest_idx >= (int)current_waypoints_->size() - 5) { 
                RCLCPP_INFO(this->get_logger(), "%s[RETURN] End of Inside Path. Returning to ORIGINAL.%s", ANSI_YELLOW.c_str(), ANSI_RESET.c_str());
                is_inside_path_active_ = false;
                current_waypoints_ = &waypoints_original_;
                return; // 이번 턴 종료하고 다음 턴에 Original 경로로 다시 계산
            }
        }

        // 5. CTE 계산
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        
        // Inside 경로 끝부분 랩어라운드 방지
        if (is_inside_path_active_ && next_nearest_idx == 0) {
            next_nearest_idx = nearest_idx;
        }

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

        // 6. Heading Error 계산
        int target_idx = (nearest_idx + forward_step_) % current_waypoints_->size();
        
        // Inside 경로 오버플로우 처리
        if (is_inside_path_active_ && (nearest_idx + forward_step_) >= (int)current_waypoints_->size()) {
            target_idx = current_waypoints_->size() - 1;
        }

        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_target_idx == 0) {
             next_target_idx = target_idx; 
        }
        
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        // 점이 겹쳐있을 경우 대비
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // 7. 속도 결정
        double final_speed = target_speed_;
        if (is_roundabout_) {
            final_speed = std::max((double)hv_vel_, 0.0);
        }

        // 8. Stanley Control Law 적용
        double v_clamped = std::max(final_speed, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 9. 최종 메시지 발행 (Accel: linear.x=Speed, angular.z=YawRate)
        auto msg_out = geometry_msgs::msg::Accel();
        // [중요] 조향각 -> YawRate 변환
        double yaw_rate = (final_speed / wheelbase_) * std::tan(steer_angle);

        msg_out.linear.x = final_speed;
        msg_out.angular.z = yaw_rate;

        pub_accel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_change_way_;
    
    // 관제탑 추가 구독
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hv_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_roundabout_;

    bool stop_signal_ = false;
    
    std::vector<Point> waypoints_original_;
    std::vector<Point> waypoints_inside_;
    std::vector<Point>* current_waypoints_; // 현재 활성화된 경로를 가리키는 포인터

    std::string original_csv_path_;
    std::string inside_csv_path_;
    
    double k_gain_;
    double max_steer_;
    double target_speed_;
    double center_to_front_;
    double wheelbase_; 
    double steer_gain_;
    int forward_step_;

    bool is_inside_path_active_ = false; 

    float hv_vel_ = 0.0f;
    bool is_roundabout_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
