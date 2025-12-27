#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/bool.hpp> 

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

        // Subscriber: /Ego_pose (차량 중심 좌표, 비표준 Yaw 포함)
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
        
        // Publisher: /Accel (linear.x=속도, angular.z=조향각)
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; // True면 정지
            });

        // === 파라미터 설정 ===
        this->declare_parameter("csv_path", "tool/p1_1.csv");
        this->declare_parameter("k_gain", 2.5);          // Stanley Gain (반응성 조절) 기존 2.5
        this->declare_parameter("max_steer", 0.9);       // 최대 조향각 (rad) 
        this->declare_parameter("target_speed", 0.5);    // 기본 주행 속도 (m/s) 기존 0.5
        this->declare_parameter("center_to_front", 0.15);// 차량 중심에서 전륜까지 거리 (m)
        this->declare_parameter("steer_gain", 1.5);      // 조향각 증폭 계수 (Understeer 보정용) 기존 1.5
        this->declare_parameter("forward_step", 15);      // Lookahead Step 클수록 늦게 반응 기존 15

        // 파라미터 로드
        csv_path_ = this->get_parameter("csv_path").as_string();
        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 

        // 경로 파일 로딩
        load_waypoints(csv_path_);
    }

private:
    // CSV 파일에서 Waypoint (x, y) 로드
    void load_waypoints(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", path.c_str());
            return;
        }

        std::string line;
        // 첫 줄(헤더) 처리
        std::getline(file, line); 

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
                    waypoints_.push_back(p);
                } catch (...) {
                    continue; // 변환 실패 시 스킵
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints", waypoints_.size());
    }

    // 각도 정규화 (-PI ~ PI)
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 차량 충돌 위험 시 일시정지
        if (stop_signal_) {
            auto stop_msg = geometry_msgs::msg::Accel();
            stop_msg.linear.x = 0.0;  // 속도 0 (정지)
            stop_msg.angular.z = 0.0; // 조향 0 (직진 정지) or 기존 조향 유지
            
            pub_accel_->publish(stop_msg);
            
            // 로그 출력 (너무 자주 찍히지 않도록 Throttle 사용)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "PAUSED by Main Controller");
            return; 
        }

        // 충돌위험 X일 때 기본 주행
        if (waypoints_.empty()) return;

        // 1. 차량 현재 상태 추출 (Center Pose)
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 전륜 차축(Front Axle) 위치 계산
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. 가장 가까운 Waypoint 찾기 (Nearest Neighbor)
        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < waypoints_.size(); ++i) {
            double dx = front_x - waypoints_[i].x;
            double dy = front_y - waypoints_[i].y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // 4. 경로 타겟 설정 (Lookahead 적용) [수정된 부분]
        // 가장 가까운 점(Nearest) 대신, forward_step_ 만큼 앞선 점을 타겟으로 잡음
        // 이렇게 하면 코너가 다가올 때 미리 핸들을 꺾게 됨
        int target_idx = (nearest_idx + forward_step_) % waypoints_.size();
        int next_target_idx = (target_idx + 1) % waypoints_.size();
        
        double map_x = waypoints_[target_idx].x;
        double map_y = waypoints_[target_idx].y;
        double next_map_x = waypoints_[next_target_idx].x;
        double next_map_y = waypoints_[next_target_idx].y;

        double path_dx = next_map_x - map_x;
        double path_dy = next_map_y - map_y;
        
        // 경로의 헤딩 (Path Yaw)
        double path_yaw = std::atan2(path_dy, path_dx);

        // 5. Cross Track Error (CTE) 계산
        // (주의: CTE도 '미리 보기 점'을 기준으로 계산됨)
        double dx = front_x - map_x;
        double dy = front_y - map_y;
        
        double path_len = std::hypot(path_dx, path_dy);
        if (path_len < 1e-6) path_len = 1e-6;

        // 외적 값
        double cross_product = dy * path_dx - dx * path_dy; 
        double cte = cross_product / path_len;

        // 6. Stanley Control Law
        
        // (1) Heading Error
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // (2) CTE Correction
        double v_clamped = std::max(target_speed_, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        // 최종 조향각 (부호: -)
        double steer_angle = heading_error - cte_correction;
        steer_angle = normalize_angle(steer_angle);

        // [추가] 조향각 증폭 (Steer Gain 적용)
        steer_angle *= steer_gain_;

        // 조향각 제한 (Saturation)
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 7. 제어 명령 발행
        auto msg_out = geometry_msgs::msg::Accel();
        
        // [수정] 감속 로직 제거: 무조건 목표 속도 유지
        double final_speed = target_speed_;
        
        /*
        // 등속 주행을 위해 감속 로직은 주석 처리 유지
        if (std::abs(steer_angle) > 0.4) { 
             final_speed *= 0.6; 
        }
        */
        

        msg_out.linear.x = final_speed;
        msg_out.angular.z = steer_angle;

        pub_accel_->publish(msg_out);

        // 디버깅 로그
        RCLCPP_INFO(this->get_logger(), "CTE: %.3f | HeadErr: %.3f | Steer: %.3f | Velocity: %.3f", cte, heading_error, steer_angle, final_speed);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    bool stop_signal_ = false;
    
    std::vector<Point> waypoints_;
    std::string csv_path_;
    
    double k_gain_;
    double max_steer_;
    double target_speed_;
    double center_to_front_;
    double steer_gain_;
    int forward_step_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
