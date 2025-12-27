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

struct VehicleState {
    double x, y;
    double speed;
    double yaw; 
    double yaw_rate; // 회전량 (rad/s)
    rclcpp::Time last_update;
    bool is_active;
    int id_num; 
    bool is_stopped;
    VehicleType type; // [추가] 차량 타입 구분
};

class MainTrafficController : public rclcpp::Node
{
public:
    MainTrafficController() 
    : Node("main_traffic_controller"), qos_(10)
    {
        qos_.best_effort();
        qos_.durability_volatile();

        // 감지 범위 15m (넉넉하게)
        this->declare_parameter("filter_dist", 1.0); 
        filter_dist_ = this->get_parameter("filter_dist").as_double();

        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&MainTrafficController::discover_vehicles, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&MainTrafficController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Main Controller Started. HV Escape Logic Active.");
    }

private:
    void discover_vehicles() {
        auto topic_names_and_types = this->get_topic_names_and_types();

        // 정규표현식: HV_19, CAV_01 등 다양하게 처리
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
        
        // 구독 생성
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                update_state(id, msg);
            });

        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            // CAV만 제어 명령 발행 가능
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
            double new_speed = dist / dt; // m/s
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

    void control_loop() {
        if (vehicle_db_.empty()) return;

        std::map<std::string, bool> next_stop_state;
        
        // 초기화: 모든 CAV 출발(false)
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV) next_stop_state[id] = false; 
        }

        std::vector<std::string> active_cavs;
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);
        }
        
        double stop_dist_th = 4.0;    // 판단 거리
        double safety_bubble = 1.5;   // 절대 위험 거리

        for(size_t i=0; i<active_cavs.size(); ++i) {
            std::string id_a = active_cavs[i]; // 나 (CAV)
            auto& state_a = vehicle_db_[id_a];
            
            bool stop_command = false;
            std::string log_msg = "";

            for(auto const& [id_b, state_b] : vehicle_db_) {
                if(id_a == id_b || !state_b.is_active) continue;

                double dx = state_b.x - state_a.x;
                double dy = state_b.y - state_a.y;
                double dist = std::hypot(dx, dy);

                if(dist > filter_dist_) continue; 

                // 거리 체크 (Hysteresis 없음 - 생존 우선)
                if (dist > stop_dist_th) continue;

                // =========================================================
                // [논리] 생존 판단 (VS HV) 및 질서 판단 (VS CAV)
                // =========================================================
                
                double dot_prod = dx * std::cos(state_a.yaw) + dy * std::sin(state_a.yaw);
                bool target_in_front = dot_prod > 0;
                
                // ---------------------------------------------------------
                // 시나리오 1: 상대가 HV (통제 불능)
                // ---------------------------------------------------------
                if (state_b.type == VehicleType::HV) {
                    if (target_in_front) {
                        // HV가 내 앞에 있음 -> 들이받으면 내 손해 -> 멈춰야 함
                        if (dist < stop_dist_th) {
                            stop_command = true;
                            log_msg = "HV AHEAD (" + id_b + ")";
                        }
                    } else {
                        // HV가 내 뒤에 있음 (Dot < 0) -> 멈추면 죽음 -> 무조건 주행 (Escape)
                        // Safety Bubble(1.5m) 안에 들어와도 절대 멈추면 안 됨!
                        stop_command = false;
                        // 로그: 도망치는 중임을 알림
                        if (dist < 5.0) {
                            // 디버깅용 로그 (너무 자주 뜨면 주석 처리)
                            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                            //    "ESCAPE: HV %s is chasing %s! RUN!", id_b.c_str(), id_a.c_str());
                        }
                        
                        // 뒤에 HV가 있으면 다른 차와의 관계보다 생존이 우선이므로
                        // 즉시 루프를 탈출하여 'stop_command = false'를 확정짓고 싶지만,
                        // 내 앞에 또 다른 장애물이 있을 수 있으니 계속 검사는 해야 함.
                        // 하지만 'HV로부터의 도주'는 최우선 순위여야 하므로,
                        // 앞차 때문에 멈춰야 하는 상황(샌드위치)이 아니라면 가속해야 함.
                    }
                }
                // ---------------------------------------------------------
                // 시나리오 2: 상대가 CAV (통제 가능, 질서 준수)
                // ---------------------------------------------------------
                else {
                    // 기하학적 정보
                    double yaw_diff = std::abs(state_a.yaw - state_b.yaw);
                    while(yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
                    yaw_diff = std::abs(yaw_diff); 

                    // 같은 방향 (병주/추종)
                    if (yaw_diff < (45.0 * M_PI / 180.0)) {
                        if (target_in_front) {
                            stop_command = true; // 앞차 보호
                            log_msg = "CAV BEHIND " + id_b;
                        } else {
                            // 뒷차임 -> 통과
                        }
                    }
                    // 다른 방향 (교차)
                    else {
                        bool am_i_turning = state_a.yaw_rate > 0.15;
                        bool target_is_turning = state_b.yaw_rate > 0.15;

                        if (!am_i_turning && target_is_turning) {
                            // 나 직진 vs 쟤 회전 -> 통과
                        } else if (am_i_turning && !target_is_turning) {
                            stop_command = true;
                            log_msg = "YIELD TURN " + id_b;
                        } else {
                            // Tie -> ID 우선
                            if (state_a.id_num > state_b.id_num) {
                                stop_command = true;
                                log_msg = "YIELD ID " + id_b;
                            }
                        }
                    }

                    // [CAV끼리 최후의 보루] 초근접 시 정지 (단, 뒤에 HV가 없을 때만 유효하지만 복잡도 줄임)
                    if (dist < safety_bubble) {
                        stop_command = true;
                        log_msg = "CRITICAL " + id_b;
                    }
                }

                // ---------------------------------------------------------
                // [결정적 판단]
                // 만약 루프를 돌면서 "멈춰야 한다"는 신호가 떴더라도,
                // "내 바로 뒤에 HV가 붙어있다"면 멈추면 안 됨 (샌드위치 사고 방지).
                // 이를 확인하기 위해 루프 내에서 가장 위험한 HV(뒷차)가 있는지 체크해야 함.
                // ---------------------------------------------------------
                if (state_b.type == VehicleType::HV && !target_in_front && dist < 5.0) {
                     // 내 뒤 5m 이내에 HV가 있음 -> 비상 모드
                     // 앞에 뭐가 있든 들이받고라도 도망가야 살 수 있다면 false로 강제 변경
                     // (다만, 현실적으로는 앞차도 피해야 하므로 딜레마임. 여기서는 '도주 우선'으로 설정)
                     stop_command = false;
                     log_msg = "EMERGENCY ESCAPE from " + id_b;
                     break; // 더 볼 것도 없이 도망
                }

                if (stop_command) {
                    // 정지 결정되면 로그 찍고 탈출
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                        "[%s] STOP | Reason: %s | Dist: %.2fm", id_a.c_str(), log_msg.c_str(), dist);
                    break;
                }
            }
            
            if (stop_command) next_stop_state[id_a] = true;
        }

        // 명령 발행
        for(auto& [id, should_stop] : next_stop_state) {
            // 상태 변경 시 로그
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
    
    // 구독자 분리 (관리 편의)
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
