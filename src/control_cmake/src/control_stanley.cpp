#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp> 
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
        auto qos_profile = rclcpp::SensorDataQoS(); 

        // 파라미터 설정
        this->declare_parameter("original_way_path", "tool/cav1p3.csv");
        this->declare_parameter("inside_way_path", "tool/cav1p3_inside.csv");
        this->declare_parameter("k_gain", 1.7);
        this->declare_parameter("max_steer", 0.7);
        this->declare_parameter("target_speed", 2.0);
        this->declare_parameter("center_to_front", 0.17); 
        this->declare_parameter("wheelbase", 0.33);       
        this->declare_parameter("steer_gain", 1.0);
        this->declare_parameter("forward_step", 15);

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

        load_waypoints(original_csv_path_, waypoints_original_);
        load_waypoints(inside_csv_path_, waypoints_inside_);
        current_waypoints_ = &waypoints_original_;
        is_inside_path_active_ = false;

        // 1. Pose 구독
        // 시뮬레이터가 /Ego_pose를 주는지 /CAV_xx/Ego_pose를 주는지에 따라 다르지만
        // 보통 상대경로 "Ego_pose"를 쓰면 네임스페이스가 붙습니다.
        // 만약 시뮬레이터가 전역 이름("/Ego_pose")을 쓴다면 슬래시를 붙여야 합니다.
        // 기존에 잘 되던 코드 그대로 "/Ego_pose" 유지합니다.
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
    
        // 2. Accel 발행 (상대 경로 X -> 기존 코드 유지)
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        // 3. [관제탑] 정지 명령 (상대 경로)
        // 네임스페이스가 CAV_01이면 -> /CAV_01/cmd_stop 구독
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

        // 4. [관제탑] 경로 변경 (상대 경로)
        // 네임스페이스가 CAV_01이면 -> /CAV_01/change_waypoint 구독
        sub_change_way_ = this->create_subscription<std_msgs::msg::Bool>(
            "change_waypoint", qos_profile,
            std::bind(&StanleyTrackerNode::callback_change_waypoint, this, _1));

        // 5. [관제탑] HV 속도 (상대 경로 - 수정됨)
        // 네임스페이스가 CAV_01이면 -> /CAV_01/hv_vel 구독
        // MainTrafficController가 이제 /CAV_01/hv_vel을 쏴주므로 매칭 성공!
        sub_hv_vel_ = this->create_subscription<std_msgs::msg::Float32>(
            "hv_vel", qos_profile,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                this->hv_vel_ = msg->data;
            });

        // 6. [관제탑] 회전교차로 상태 (상대 경로)
        // 네임스페이스가 CAV_01이면 -> /CAV_01/is_roundabout 구독
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
        std::getline(file, line);
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

    void callback_change_waypoint(const std_msgs::msg::Bool::SharedPtr msg) {
        bool request_inside = msg->data;

        if (request_inside) {
            if (!is_inside_path_active_ && !waypoints_inside_.empty()) {
                RCLCPP_INFO(this->get_logger(), "%s[SWITCH] Switching to INSIDE PATH%s", ANSI_CYAN.c_str(), ANSI_RESET.c_str());
                is_inside_path_active_ = true;
                current_waypoints_ = &waypoints_inside_;
                last_nearest_idx_ = -1; 
            }
        } 
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // Nearest Waypoint
        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        int search_start = 0;
        int search_end = current_waypoints_->size();

        if (last_nearest_idx_ != -1) {
            search_start = 0;
            search_end = current_waypoints_->size();
        }
            
        for (int i = search_start; i < search_end; ++i) {
            double dx = front_x - (*current_waypoints_)[i].x;
            double dy = front_y - (*current_waypoints_)[i].y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // Return to Original Path Logic
        if (is_inside_path_active_) {
            if (nearest_idx >= (int)current_waypoints_->size() - 5) { 
                RCLCPP_INFO(this->get_logger(), "%s[RETURN] End of Inside Path. Returning to ORIGINAL.%s", ANSI_YELLOW.c_str(), ANSI_RESET.c_str());
                is_inside_path_active_ = false;
                current_waypoints_ = &waypoints_original_;
                last_nearest_idx_ = -1; 
                return; 
            }
        }
        else {
             if (last_nearest_idx_ != -1) {
                size_t total = current_waypoints_->size();
                bool wrap_around = (last_nearest_idx_ > total * 0.9) && (nearest_idx < total * 0.1); 
                
                if (wrap_around) {
                    lap_count_++;
                    if (lap_count_ == 5) {
                        RCLCPP_INFO(this->get_logger(), "%s[SUCCESS] 5 LAPS COMPLETED!%s", ANSI_GREEN.c_str(), ANSI_RESET.c_str());
                    } else {
                        RCLCPP_INFO(this->get_logger(), "Lap %d Completed.", lap_count_);
                    }
                }
            }
        }
        
        last_nearest_idx_ = nearest_idx;

        // CTE Calculation
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        
        if (is_inside_path_active_ && next_nearest_idx == 0 && nearest_idx == (int)current_waypoints_->size() - 1) {
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

        // Heading Error Calculation
        int target_idx = (nearest_idx + forward_step_) % current_waypoints_->size();
        
        if (is_inside_path_active_ && (nearest_idx + forward_step_) >= (int)current_waypoints_->size()) {
            target_idx = current_waypoints_->size() - 1;
        }

        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_target_idx == 0) {
             next_target_idx = target_idx; 
        }
        
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // [속도 제어]
        double final_speed = target_speed_;
        
        // 회전교차로에서는 HV 속도 추종
        if (is_roundabout_) {
            final_speed = std::max((double)hv_vel_, 0.0);
        }

        // Stanley Control Law
        double v_clamped = std::max(final_speed, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // Publish Message
        auto msg_out = geometry_msgs::msg::Accel();
        double yaw_rate = (final_speed / wheelbase_) * std::tan(steer_angle);

        msg_out.linear.x = final_speed;
        msg_out.angular.z = yaw_rate;

        pub_accel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_change_way_;
    
    // [관제탑 추가 구독]
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hv_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_roundabout_;

    bool stop_signal_ = false;
    
    std::vector<Point> waypoints_original_;
    std::vector<Point> waypoints_inside_;
    std::vector<Point>* current_waypoints_;

    std::string original_csv_path_;
    std::string inside_csv_path_;
    
    double k_gain_;
    double max_steer_;
    double target_speed_;
    double center_to_front_;
    double wheelbase_; 
    double steer_gain_;
    int forward_step_;

    int lap_count_ = 0;
    int last_nearest_idx_ = -1;
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
