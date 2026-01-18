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
        // QoS 설정: SensorData (Best Effort, Volatile) - 매뉴얼 요구사항 준수
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // === 파라미터 설정 ===
        this->declare_parameter("csv_path", "tool/cav1p1.csv");
        this->declare_parameter("k_gain", 2.0);          // 2.0 ~ 2.5사이 어딘가로 맞추면 될 듯?
        this->declare_parameter("max_steer", 0.7);       // 최대 조향각 (rad) - 약 50도
        this->declare_parameter("target_speed", 2.0);    // 기본 주행 속도 (m/s)
        this->declare_parameter("center_to_front", 0.17);// 차량 중심에서 전륜까지 거리 (m)
        this->declare_parameter("wheelbase", 0.33);      // [추가] 축거 (Wheelbase) (m)
        this->declare_parameter("steer_gain", 1.0);      // [변경] 각속도 변환을 하므로 gain은 1.0 추천
        this->declare_parameter("forward_step", 15);     // Lookahead Step
        
        // [추가] 초기 안정화를 위한 Warmup 횟수 (요청하신 대로 기본값 5)
        this->declare_parameter("warmup_steps", 10);

        // 파라미터 로드
        csv_path_ = this->get_parameter("csv_path").as_string();
        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double(); 
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 
        warmup_steps_target_ = this->get_parameter("warmup_steps").as_int();

        // 경로 파일 로딩
        load_waypoints(csv_path_);

        // Subscriber: /Ego_pose (차량 중심 좌표)
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
        
        // Publisher: /Accel (linear.x=속도, angular.z=각속도[rad/s])
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        // 비상 정지 명령 수신
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; // True면 정지 
                if (this->stop_signal_) {
                    publish_stop_command();
                }
            });
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

    // [추가] 정지 명령 발행 함수화 (코드 중복 제거)
    void publish_stop_command() {
        auto stop_msg = geometry_msgs::msg::Accel();
        stop_msg.linear.x = 0.0;  
        stop_msg.angular.z = 0.0; 
        pub_accel_->publish(stop_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // [추가] 0-1. 데이터 신선도 체크 (Timestamp Check)
        // 현재 시간과 메시지 시간의 차이가 1.0초 이상이면 오래된 데이터로 간주
        rclcpp::Time msg_time = msg->header.stamp;
        rclcpp::Time now = this->now();
        double time_diff = (now - msg_time).seconds();

        if (std::abs(time_diff) > 1.0) {
            current_warmup_count_ = 0; // 카운트 리셋 (다시 안정화 대기)
            publish_stop_command();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Old pose detected (Diff: %.2fs). Waiting...", time_diff);
            return;
        }

        // [추가] 0-2. 초기 안정화 (Warm-up)
        // 설정된 횟수(warmup_steps_target_=5)만큼 데이터를 흘려보냄
        if (current_warmup_count_ < warmup_steps_target_) {
            current_warmup_count_++;
            publish_stop_command();
            // 첫 프레임과 완료 시점에 로그 출력
            if (current_warmup_count_ == 1 || current_warmup_count_ == warmup_steps_target_) {
                RCLCPP_INFO(this->get_logger(), "Warming up... (%d/%d)", 
                    current_warmup_count_, warmup_steps_target_);
            }
            return; 
        }

        // 0-3. 차량 충돌 위험 시 일시정지 (비상 정지)
        if (stop_signal_) {
            publish_stop_command();
            return; 
        }

        if (waypoints_.empty()) return;

        // 1. 차량 현재 상태 추출 (Center Pose)
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z; 

        // 2. 전륜 차축(Front Axle) 위치 계산 (Stanley는 전륜 기준이 안정적)
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. 가장 가까운 Waypoint 찾기
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

        // 4. CTE & Heading Error 계산
        int next_nearest_idx = (nearest_idx + 1) % waypoints_.size();
        
        double map_x = waypoints_[nearest_idx].x;
        double map_y = waypoints_[nearest_idx].y;
        double next_map_x = waypoints_[next_nearest_idx].x;
        double next_map_y = waypoints_[next_nearest_idx].y;

        double path_dx = next_map_x - map_x;
        double path_dy = next_map_y - map_y;
        double path_len = std::hypot(path_dx, path_dy);
        if (path_len < 1e-6) path_len = 1e-6;

        // CTE 계산 (Cross Track Error)
        double dx = front_x - map_x;
        double dy = front_y - map_y;
        double cross_product = dx * path_dy - dy * path_dx; 
        double cte = cross_product / path_len;

        // Heading Error 계산 (Lookahead 적용)
        int target_idx = (nearest_idx + forward_step_) % waypoints_.size();
        int next_target_idx = (target_idx + 1) % waypoints_.size();
        
        double target_dx = waypoints_[next_target_idx].x - waypoints_[target_idx].x;
        double target_dy = waypoints_[next_target_idx].y - waypoints_[target_idx].y;
        double path_yaw = std::atan2(target_dy, target_dx);

        double heading_error = normalize_angle(path_yaw - current_yaw);

        // 5. Stanley Control Law
        double v_clamped = std::max(target_speed_, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        // 최종 조향각 (Steer Angle)
        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        
        // 조향 이득 적용 (필요 시)
        steer_angle *= steer_gain_;

        // 조향각 제한 (Saturation)
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 6. 제어 명령 생성 및 발행
        auto msg_out = geometry_msgs::msg::Accel();
        
        double final_speed = target_speed_;

        // [중요 수정] 조향각(delta) -> 각속도(omega) 변환
        // Kinematic Bicycle Model: omega = (v / L) * tan(delta)
        double yaw_rate = (final_speed / wheelbase_) * std::tan(steer_angle);

        msg_out.linear.x = final_speed;
        msg_out.angular.z = yaw_rate; 

        pub_accel_->publish(msg_out);
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
    double wheelbase_;
    double steer_gain_;
    int forward_step_;

    // [추가] Warmup 관련 변수
    int warmup_steps_target_;
    int current_warmup_count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyTrackerNode>());
    rclcpp::shutdown();
    return 0;
}