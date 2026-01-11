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

        // Subscriber: /Ego_pose
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&StanleyTrackerNode::pose_callback, this, _1));
        
        // Publisher: /Accel
        pub_accel_ = this->create_publisher<geometry_msgs::msg::Accel>(
            "/Accel", qos_profile);

        // 비상 정지 명령 수신
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "/cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; 
            });

        // [NEW] 경로 변경 명령 수신
        sub_change_way_ = this->create_subscription<std_msgs::msg::Bool>(
            "/change_waypoint", qos_profile,
            std::bind(&StanleyTrackerNode::callback_change_waypoint, this, _1));

        // === 파라미터 설정 ===
        // 기존 csv_path를 두 개로 분리
        this->declare_parameter("original_way_path", "tool/cav1p3.csv");
        this->declare_parameter("inside_way_path", "tool/cav1p3_inside.csv");
        
        this->declare_parameter("k_gain", 1.3);
        this->declare_parameter("max_steer", 0.7);
        this->declare_parameter("target_speed", 2.0);
        this->declare_parameter("center_to_front", 0.17); // 차량 길이 관련
        this->declare_parameter("wheelbase", 0.33);       // 카이스트 규정 0.33m
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
    
        // 두 개의 경로 파일 로딩
        load_waypoints(original_csv_path_, waypoints_original_);
        load_waypoints(inside_csv_path_, waypoints_inside_);

        // 초기 경로 설정: Original
        current_waypoints_ = &waypoints_original_;
        is_inside_path_active_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Stanley Tracker Initialized. Mode: ORIGINAL PATH");
    }

private:
    // [MODIFIED] 경로 파일을 읽어 특정 벡터에 저장하도록 수정
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

    // [NEW] 경로 변경 콜백
    void callback_change_waypoint(const std_msgs::msg::Bool::SharedPtr msg) {
        bool request_inside = msg->data;

        if (request_inside) {
            // 변경 요청(true)이 들어왔고, 아직 Inside 모드가 아니라면 전환
            if (!is_inside_path_active_ && !waypoints_inside_.empty()) {
                RCLCPP_INFO(this->get_logger(), "%s[SWITCH] Switching to INSIDE PATH%s", ANSI_CYAN.c_str(), ANSI_RESET.c_str());
                is_inside_path_active_ = true;
                current_waypoints_ = &waypoints_inside_;
                last_nearest_idx_ = -1; // 경로가 바뀌었으므로 인덱스 초기화 (전역 검색 유도)
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

        // 1. 차량 상태 추출 (카이스트 규정 좌표계 준수: position.x, position.y, orientation.z)
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 전륜 차축 위치 계산 (center_to_front: 0.17m 고려)
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. 가장 가까운 Waypoint 찾기
        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        // 경로 전환 직후 등 last_nearest_idx_가 유효하지 않을 때는 전체 검색
        // 그 외에는 이전 인덱스 주변 부분 검색으로 효율화 가능하나, 안전을 위해 전체 검색 유지 또는 윈도우 검색
        // 여기서는 안전하게 전체 검색을 하되, start_idx 최적화 가능
        int search_start = 0;
        int search_end = current_waypoints_->size();

        // (옵션) 윈도우 서치: 경로가 바뀌지 않았다면 주변만 탐색
        if (last_nearest_idx_ != -1) {
            search_start = std::max(0, last_nearest_idx_ - 10);
            search_end = std::min((int)current_waypoints_->size(), last_nearest_idx_ + 50);
            // 만약 루프를 도는 경로라면 윈도우 처리가 더 복잡하므로 여기서는 단순화하여 전체 검색 사용
            // 요구사항의 'inside_way' 전환 로직 안정성을 위해 전체 검색 권장
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

        // ==========================================
        // [NEW] 경로 복귀 및 랩 카운트 로직
        // ==========================================
        
        // CASE A: Inside Way 주행 중
        if (is_inside_path_active_) {
            // Inside 경로의 끝부분(마지막 인덱스 근처)에 도달했는지 확인
            // 예: 전체 길이의 95% 지점 통과 시
            if (nearest_idx >= (int)current_waypoints_->size() - 5) {
                RCLCPP_INFO(this->get_logger(), "%s[RETURN] End of Inside Path. Returning to ORIGINAL.%s", ANSI_YELLOW.c_str(), ANSI_RESET.c_str());
                
                is_inside_path_active_ = false;
                current_waypoints_ = &waypoints_original_;
                
                // [중요] 경로가 바뀌었으므로 다음 루프에서 가장 가까운 점을 다시 찾아야 함 (전역 검색)
                last_nearest_idx_ = -1; 
                
                // 이번 틱 제어는 건너뛰거나, Original 경로 기준으로 다시 계산해야 함.
                // 안전하게 함수 리턴하여 다음 틱에 Original 경로 제어 수행
                return;
            }
        }
        // CASE B: Original Way 주행 중 (기존 랩 카운트 로직 유지)
        else {
             if (last_nearest_idx_ != -1 && current_waypoints_->size() > 50) {
                size_t total = current_waypoints_->size();
                // 끝 -> 시작 점프 감지
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
        // ==========================================


        // 4. CTE & Heading Error 계산
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        
        // Inside 경로의 끝에서 인덱스 오버플로우 방지 (Inside 경로는 루프가 아닐 수 있음)
        if (is_inside_path_active_ && next_nearest_idx == 0 && nearest_idx == (int)current_waypoints_->size() - 1) {
            next_nearest_idx = nearest_idx; // 마지막 점 유지
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

        // Lookahead Point 계산
        int target_idx = (nearest_idx + forward_step_) % current_waypoints_->size();
        
        // Inside 경로 끝부분 처리: 타겟 인덱스가 범위를 넘어가면 마지막 점으로 고정
        if (is_inside_path_active_ && (nearest_idx + forward_step_) >= (int)current_waypoints_->size()) {
            target_idx = current_waypoints_->size() - 1;
        }

        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_target_idx == 0) {
             next_target_idx = target_idx; // 마지막 점 유지
        }
        
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        // 마지막 점이라 방향 계산이 안되면 이전 방향 유지
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // 5. Stanley Control Law
        double v_clamped = std::max(target_speed_, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 6. 제어 명령 생성
        auto msg_out = geometry_msgs::msg::Accel();
        double final_speed = target_speed_;
        double yaw_rate = (final_speed / wheelbase_) * std::tan(steer_angle);

        msg_out.linear.x = final_speed;
        msg_out.angular.z = yaw_rate;

        pub_accel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr pub_accel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_change_way_; // [NEW]

    bool stop_signal_ = false;
    
    // [MODIFIED] 경로 저장용 벡터 분리
    std::vector<Point> waypoints_original_;
    std::vector<Point> waypoints_inside_;
    std::vector<Point>* current_waypoints_; // 현재 사용 중인 경로를 가리키는 포인터

    // [MODIFIED] 파라미터 변수명 변경
    std::string original_csv_path_;
    std::string inside_csv_path_;
    
    double k_gain_;
    double max_steer_;
    double target_speed_;
    double center_to_front_;
    double wheelbase_; 
    double steer_gain_;
    int forward_step_;

    // 랩 카운팅 및 경로 상태 변수
    int lap_count_ = 0;
    int last_nearest_idx_ = -1;
    bool is_inside_path_active_ = false; // [NEW] 현재 Inside 경로 주행 중인지 여부
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyTrackerNode>());
    rclcpp::shutdown();
    return 0;
}
