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
        this->declare_parameter("max_steer", 0.7);       
        this->declare_parameter("target_speed", 2.0);    
        this->declare_parameter("center_to_front", 0.17);
        this->declare_parameter("wheelbase", 0.33);      
        this->declare_parameter("steer_gain", 1.0);
        this->declare_parameter("forward_step", 15);     
        
        // [설정] 초기 웜업 횟수 (약 0.2~0.3초 대기)
        this->declare_parameter("warmup_steps", 10); 

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
        warmup_steps_target_ = this->get_parameter("warmup_steps").as_int();

        // 경로 파일 로딩
        load_waypoints(original_csv_path_, waypoints_original_);
        load_waypoints(inside_csv_path_, waypoints_inside_);
        
        current_waypoints_ = &waypoints_original_;
        is_inside_path_active_ = false;

        // Subscriber & Publisher
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
        
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", qos_profile);

        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; 
                if (this->stop_signal_) this->publish_stop_command();
            });

        sub_change_way_ = this->create_subscription<std_msgs::msg::Bool>(
            "change_waypoint", qos_profile,
            std::bind(&StanleyTrackerNode::callback_change_waypoint, this, _1));

        sub_hv_vel_ = this->create_subscription<std_msgs::msg::Float32>(
            "hv_vel", qos_profile,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->hv_vel_ = msg->data; });

        sub_is_roundabout_ = this->create_subscription<std_msgs::msg::Bool>(
            "is_roundabout", qos_profile,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->is_roundabout_ = msg->data; });
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
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) {
                try {
                    Point p = {std::stod(row[0]), std::stod(row[1])};
                    target_vector.push_back(p);
                } catch (...) { continue; }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", target_vector.size(), path.c_str());
    }

    void callback_change_waypoint(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_inside_path_active_ && !waypoints_inside_.empty()) {
            is_inside_path_active_ = true;
            current_waypoints_ = &waypoints_inside_;
        } 
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void publish_stop_command() {
        auto stop_msg = geometry_msgs::msg::Accel();
        stop_msg.linear.x = 0.0;  
        stop_msg.angular.z = 0.0; 
        pub_accel_->publish(stop_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // [수정] 데이터 지연 체크 로직 완전 삭제됨

        // 1. [초기 웜업] 노드 실행 직후만 작동 (위치 튐 방지)
        if (current_warmup_count_ < warmup_steps_target_) {
            current_warmup_count_++;
            publish_stop_command();
            if (current_warmup_count_ % 10 == 0 || current_warmup_count_ == 1) {
                RCLCPP_INFO(this->get_logger(), "Initial Warming up... (%d/%d)", current_warmup_count_, warmup_steps_target_);
            }
            return; // 웜업 중에는 계산 안 함
        }

        // 2. 비상 정지 체크
        if (stop_signal_) {
            publish_stop_command();
            return; 
        }

        if (current_waypoints_->empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Current waypoint list is empty!");
            return;
        }

        // --- Stanley Logic ---

        // 1. 차량 상태 추출
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 전륜 위치 계산
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. Nearest Waypoint Search
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

        // 4. 경로 복귀 로직
        if (is_inside_path_active_) {
            if (nearest_idx >= (int)current_waypoints_->size() - 5) { 
                is_inside_path_active_ = false;
                current_waypoints_ = &waypoints_original_;
                return; 
            }
        }

        // 5. CTE 계산
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_nearest_idx == 0) next_nearest_idx = nearest_idx;

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
        
        if (is_inside_path_active_ && (nearest_idx + forward_step_) >= (int)current_waypoints_->size()) {
            target_idx = current_waypoints_->size() - 1;
        }

        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_target_idx == 0) next_target_idx = target_idx; 
        
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // 7. 속도 결정
        double final_speed = target_speed_;
        if (is_roundabout_) final_speed = std::max((double)hv_vel_, 0.0);

        // 8. Stanley Control Law
        double v_clamped = std::max(final_speed, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 9. 최종 발행
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

    int warmup_steps_target_;
    int current_warmup_count_ = 0;

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