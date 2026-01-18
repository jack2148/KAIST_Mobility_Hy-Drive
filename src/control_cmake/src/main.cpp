// main_traffic_controller.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <map>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <regex>

using std::placeholders::_1;

// 차량 타입 열거형
enum class VehicleType {
    CAV, // 제어 가능
    HV   // 제어 불가 (장애물/위협)
};

<<<<<<< chan
struct VehicleState {
    double x, y;
    double speed;
    double yaw; 
    double yaw_rate; // 회전량 (rad/s)
    rclcpp::Time last_update;
    bool is_active;
    int id_num; 
    bool is_stopped;
    VehicleType type; // 차량 타입 구분
};
=======
class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        car_dims_ = {0.17, 0.16, 0.075, 0.075};
        front_padding_ = 0.8;
        side_padding_ = 0.6;
        approach_range_sq_ = 2.0 * 2.0;

        fourway_center_ = {-2.333, 0.0};
        fourway_len_ = 2.0;
        fourway_app_r_sq_ = 1.5 * 1.5;
        fourway_box_half_len_ = fourway_len_ / 2.0;

        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4;

        t3_box_x_min_ = -3.5; t3_box_x_max_ = -1.3;
        t3_1_y_min_ = 1.6;    t3_1_y_max_ = 2.7;
        t3_2_y_min_ = -2.7;   t3_2_y_max_ = -1.9;

        tmr_discovery_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this)
        );
        tmr_control_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MainTrafficController::control_loop, this)
        );

        RCLCPP_INFO(get_logger(), "Controller Started: Deadlock Resolution Logic Fixed.");
    }
>>>>>>> local

class MainTrafficController : public rclcpp::Node
{
public:
    MainTrafficController() 
    : Node("main_traffic_controller"), qos_(10)
    {
        qos_.best_effort();
        qos_.durability_volatile();

        // 감지 범위 15m (넉넉하게)
        this->declare_parameter("filter_dist", 15.0); 
        filter_dist_ = this->get_parameter("filter_dist").as_double();

        // -------------------------------------------------------------
        // [1] 교차로 정보 초기화
        // -------------------------------------------------------------
        // { x좌표, y좌표, 반경(m), 교차로ID }
        intersections_ = {
            {-2.333, 0.0, 0.8, 63} // 회전 교차로 중점
           
        };

        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&MainTrafficController::discover_vehicles, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&MainTrafficController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Main Controller Started. Approach & Turn Logic Active.");
    }

private:
    // [2] 교차로 구조체 정의
    struct Intersection {
        double x, y, radius;
        int id;
    };
    std::vector<Intersection> intersections_;

    // [추가] 진입 전 감지용 반경 및 회전 판단 기준
    const double APPROACH_RADIUS = 2.5;     // 진입로 감지 반경 (m)
    const double YAW_RATE_THRESHOLD = 0.15;  // 회전 판단 기준 (rad/s)

    // [3] 헬퍼 함수: 현재 위치가 어떤 교차로 안인지 확인 (반경 조절 가능)
    // check_radius가 음수면 기본 inter.radius(0.5) 사용, 양수면 그 값 사용
    int get_intersection_id(double x, double y, double check_radius = -1.0) {
        for (const auto& inter : intersections_) {
            double r = (check_radius < 0.0) ? inter.radius : check_radius;
            
            double dist = std::hypot(x - inter.x, y - inter.y);
            if (dist <= r) {
                return inter.id; // 해당 교차로 ID 반환
            }
        }
        return -1; // 교차로 아님
    }

    void discover_vehicles() {
        auto topic_names_and_types = this->get_topic_names_and_types();

        std::regex cav_regex(R"((.*)/?(CAV_?(\d+))/?.*$)"); 
        std::regex hv_regex(R"((.*)/?(HV_?(\d+))/?.*$)");    

        for (const auto& [name, types] : topic_names_and_types) {
            bool is_pose_topic = false;
            for (const auto& type : types) {
                if (type.find("geometry_msgs") != std::string::npos && 
                    type.find("PoseStamped") != std::string::npos) {
                    is_pose_topic = true;
                    break;
                }
            }
            if (!is_pose_topic) continue;

            std::smatch match;
            if (std::regex_search(name, match, cav_regex)) {
                std::string id = match[2].str();
                int num = std::stoi(match[3].str());
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, num, VehicleType::CAV);
            } 
            else if (std::regex_search(name, match, hv_regex)) {
                std::string id = match[2].str();
                int num = std::stoi(match[3].str());
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, num, VehicleType::HV);
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic_name, int num, VehicleType type) {
        std::string type_str = (type == VehicleType::CAV) ? "CAV" : "HV";
        RCLCPP_INFO(this->get_logger(), "New %s detected: %s (ID: %d)", type_str.c_str(), id.c_str(), num);
        
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                update_state(id, msg);
            });

        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
        } else {
            hv_subs_[id] = sub;
        }
        
        vehicle_db_[id] = {0,0,0,0,0, this->now(), false, num, false, type};
    }

    void update_state(const std::string& id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto& v = vehicle_db_[id];
        
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;
        double current_yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));

        double dt = (this->now() - v.last_update).seconds();
        if (dt > 0.0 && v.is_active) {
            double dist = std::hypot(msg->pose.position.x - v.x, msg->pose.position.y - v.y);
            double new_speed = dist / dt; 
            if (new_speed < 30.0) v.speed = v.speed * 0.7 + new_speed * 0.3;

            double diff = current_yaw - v.yaw;
            while(diff > M_PI) diff -= 2.0 * M_PI;
            while(diff < -M_PI) diff += 2.0 * M_PI;
            
            double instant_rate = std::abs(diff) / dt;
            v.yaw_rate = v.yaw_rate * 0.6 + instant_rate * 0.4;
        }

        v.x = msg->pose.position.x;
        v.y = msg->pose.position.y;
        v.yaw = current_yaw;
        v.last_update = this->now();
        v.is_active = true;
    }

    // ==================================================================================
    // [4] 메인 제어 루프 (진입 전 대기 및 회전 차량 양보 로직 적용)
    // ==================================================================================
    void control_loop() {
        if (vehicle_db_.empty()) return;

        std::map<std::string, bool> next_stop_state;
        
        // 초기화: 모든 CAV 출발(false)
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV) next_stop_state[id] = false; 
        }

        // 활성화된 CAV 리스트 추출
        std::vector<std::string> active_cavs;
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);
        }

        // CAV들 각각에 대해 판단 수행
        for(const auto& id_a : active_cavs) {
            auto& state_a = vehicle_db_[id_a];
            bool stop_command = false;
            
            // -------------------------------------------------------------
            // [나의 위치 판단]
            // -------------------------------------------------------------
            // (A) 내가 교차로 '안(Critical Zone)'에 있는가? (기본반경 0.5m)
            int my_critical_id = get_intersection_id(state_a.x, state_a.y, -1.0);
            
            // (B) 내가 교차로 '진입로(Approach Zone)'에 있는가? (확장반경 10.0m)
            int my_approach_id = get_intersection_id(state_a.x, state_a.y, APPROACH_RADIUS);

            // "나는 아직 교차로 안은 아닌데, 진입로에는 들어왔음" (진입 직전)
            bool am_i_approaching = (my_critical_id == -1) && (my_approach_id != -1);


            // 다른 차량들과의 관계 확인
            for(auto const& [id_b, state_b] : vehicle_db_) {
                if(id_a == id_b || !state_b.is_active) continue;

                // [로직 A] 기본 안전 거리 유지
                double dist = std::hypot(state_a.x - state_b.x, state_a.y - state_b.y);
                if (dist < 0.02) { // 0.2m 이내 초근접 시 정지 (Safety Bubble)
                     // 단, HV가 뒤에서 오면 도망가야 하므로 실제론 더 복잡하지만 여기선 단순화
                    if (state_b.type != VehicleType::HV) { 
                        stop_command = true; 
                    }
                }

                // -------------------------------------------------------------
                // [로직 B] 진입 전 대기 (Approach & Wait)
                // -------------------------------------------------------------
                if (am_i_approaching) {
                    // 상대방이 내가 들어가려는 그 교차로(my_approach_id) '안(Critical)'에 있는가?
                    int other_critical_id = get_intersection_id(state_b.x, state_b.y, -1.0);

                    if (other_critical_id == my_approach_id) {
                        // 그 차가 회전 중인가? (Yaw Rate 확인)
                        bool is_turning = std::abs(state_b.yaw_rate) > YAW_RATE_THRESHOLD;

                        // [조건] 상대가 내 목표 교차로 안에서 회전 중이면 -> 진입 금지(STOP)
                        if (is_turning) {
                            stop_command = true;
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "[%s] WAIT outside Node %d for TURNING vehicle %s", 
                                id_a.c_str(), my_approach_id, id_b.c_str());
                        }
                    }
                }

                // -------------------------------------------------------------
                // [로직 C] 교차로 내부 경합 (이미 둘 다 들어와 버렸을 때)
                // -------------------------------------------------------------
                // 상대방도 교차로 영역(Critical)에 있는지 확인
                int inter_id_b = get_intersection_id(state_b.x, state_b.y, -1.0);

                // 두 차량이 "같은 교차로" 영역 안에서 경합 중일 때
                if (my_critical_id != -1 && (my_critical_id == inter_id_b)) {
                    
                    // Case 1: 상대가 HV (통제 불능) -> 무조건 내가 양보
                    if (state_b.type == VehicleType::HV) {
                        stop_command = true;
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "[%s] Yielding to HV at Node %d", id_a.c_str(), my_critical_id);
                    }
                    // Case 2: 상대가 CAV -> ID 기반 우선순위
                    else {
                        // 규칙: ID 번호가 큰 쪽이 양보
                        if (state_a.id_num > state_b.id_num) {
                            stop_command = true;
                            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "[%s] Yielding to ID Priority (%s) at Node %d", id_a.c_str(), id_b.c_str(), my_critical_id);
                        }
                    }
                }
            }
            
            if (stop_command) next_stop_state[id_a] = true;
        }

        // 명령 발행
        for(auto& [id, should_stop] : next_stop_state) {
            if (vehicle_db_[id].is_stopped != should_stop) {
                RCLCPP_INFO(this->get_logger(), "[%s] State: %s -> %s", 
                    id.c_str(), vehicle_db_[id].is_stopped ? "STOP" : "GO", should_stop ? "STOP" : "GO");
            }
            
            vehicle_db_[id].is_stopped = should_stop;

            if (stop_pubs_.find(id) != stop_pubs_.end()) {
                std_msgs::msg::Bool msg;
                msg.data = should_stop;
                stop_pubs_[id]->publish(msg);
            }
        }
    }

    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::QoS qos_;

    std::map<std::string, VehicleState> vehicle_db_;
    
    // 구독자 분리
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs_;
    
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;

    double filter_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}