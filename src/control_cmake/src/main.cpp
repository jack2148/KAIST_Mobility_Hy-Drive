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
#include <iomanip>
#include <sstream>


using std::placeholders::_1;


// ==========================================
// [설정] 로그 색상 정의 (ANSI Escape Codes)
// ==========================================
const std::string ANSI_RESET  = "\033[0m";
const std::string ANSI_GREEN  = "\033[32m";  // 교차로 로직에서 멈춤 : 초록색
const std::string ANSI_YELLOW = "\033[33m";  // 단순 위험거리 계산에서 멈춤 : 노란색
const std::string ANSI_BLUE   = "\033[34m";  // 출발 : 파란색


// 차량 타입 열거형
enum class VehicleType {
    CAV, // 제어 가능 (Connected Automated Vehicle)
    HV   // 제어 불가 (Human Driven / Obstacle)
};


struct VehicleState {
    double x, y;
    double speed;
    double yaw; 
    double yaw_rate; 
    rclcpp::Time last_update;
    bool is_active;
    int id_num; 
    bool is_stopped;
    VehicleType type; 
};


class MainTrafficController : public rclcpp::Node
{
private:
    struct Intersection {
        double x, y, radius;
        int id;
    };


public:
    MainTrafficController() 
    : Node("main_traffic_controller"), qos_(10)
    {
        qos_.best_effort();
        qos_.durability_volatile();


        this->declare_parameter("filter_dist", 2.0); 
        filter_dist_ = this->get_parameter("filter_dist").as_double();


        // 교차로 정보 초기화
        intersections_ = {
            {-2.333, 0.0, 0.8, 63} // {x, y, radius, id}
        };


        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            std::bind(&MainTrafficController::discover_vehicles, this));


        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), 
            std::bind(&MainTrafficController::control_loop, this));
            
        RCLCPP_INFO(this->get_logger(), "Main Controller Started. Optimized Logic Active.");
    }


private:
    // [파라미터 설정]
    // 1. 차량 제원 및 정지 기준
    const double DIST_CENTER_TO_BUMPER = 0.17; // 차량 길이(0.33)의 절반 약 0.17
    const double SAFE_GAP_FRONT = 0.3;         // 최소 안전 거리
    const double CRITICAL_STOP_DIST = DIST_CENTER_TO_BUMPER + SAFE_GAP_FRONT; // 최종 정지 거리 (0.17 + 0.2 = 0.37m)


    // 2. [최적화용] 계산 트리거 범위
    const double CALCULATION_RANGE_SQ = 2.0 * 2.0; // 차량의 유클리드 거리가 2^2m 이내면 정밀 계산 시작
    
    // 횡방향 감지 폭 (0.15m = 차량 가로 폭)
    // 내 차 중심선에서 좌우 15cm 이내에 있어야 "내 앞차"로 인식
    const double DETECTION_WIDTH_SIDE = 0.15;
    
    // [수정] 시야각(FOV) 설정: 좌우 60도 (총 120도)
    const double VIEW_HALF_ANGLE = 60.0 * (M_PI / 180.0);


    // 3. 교차로 로직 파라미터
    const double APPROACH_RADIUS = 2.5;     
    const double YAW_RATE_THRESHOLD = 0.15; 
    // 3m 이상 멀어지면 교차로 로직도 무시 (연산 절감)
    const double MAX_INTERACTION_DIST_SQ = 1.0 * 1.0;


    // 멤버 변수
    std::vector<Intersection> intersections_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::QoS qos_;


    std::map<std::string, VehicleState> vehicle_db_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;


    double filter_dist_;


    // 교차로 ID 판별
    int get_intersection_id(double x, double y, double check_radius = -1.0) {
        for (const auto& inter : intersections_) {
            double r = (check_radius < 0.0) ? inter.radius : check_radius;
            if (std::hypot(x - inter.x, y - inter.y) <= r) {
                return inter.id; 
            }
        }
        return -1; 
    }


    // 차량 발견 및 등록
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
        double current_yaw = msg->pose.orientation.z; 
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


    void control_loop() {
        if (vehicle_db_.empty()) return;


        std::map<std::string, bool> next_stop_state;
        std::map<std::string, std::string> stop_reasons; 
        
        // 초기화
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV) {
                next_stop_state[id] = false; 
                stop_reasons[id] = "NONE";
            }
        }


        std::vector<std::string> active_cavs;
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);
        }


        // 제어 루프
        // 1순위 : 앞 차량과의 간격이 충돌 기준 거리보다 짧으면 정지 & 뒷 차량(HV일때만)과의 간격이 충돌기준 거리보다 짧으면 피함
        // 2순위 : 정지 상태 아닌데 교차로면 교차로 로직 따름
    
        for(const auto& id_a : active_cavs) {
            auto& state_a = vehicle_db_[id_a];
            bool stop_command = false;
            
            int my_critical_id = get_intersection_id(state_a.x, state_a.y, -1.0);
            int my_approach_id = get_intersection_id(state_a.x, state_a.y, APPROACH_RADIUS);
            bool am_i_approaching = (my_critical_id == -1) && (my_approach_id != -1);


            for(auto const& [id_b, state_b] : vehicle_db_) { // id_a :현재 차량
                if(id_a == id_b || !state_b.is_active) continue; // id_b : 현재 차량 제외 모든 차량에 대해 연산 수행


                double dx = state_b.x - state_a.x;
                double dy = state_b.y - state_a.y;
                double dist_sq = dx*dx + dy*dy; // 차량 사이의 유클리드 거리 계산


                // 너무 멀면 아예 무시
                if (dist_sq > MAX_INTERACTION_DIST_SQ) continue; 


                // [거리 기반 충돌 방지 로직]
                // 차량 간 거리가 1m 이내면 정밀 계산 수행
                if (dist_sq < CALCULATION_RANGE_SQ) {
                    
                    // 상대 좌표 변환 z축 기준 변환 행렬 계산
                    double local_x = dx * std::cos(-state_a.yaw) - dy * std::sin(-state_a.yaw); // 차량의 종방향 축 (|)
                    double local_y = dx * std::sin(-state_a.yaw) + dy * std::cos(-state_a.yaw); // 차량의 횡방향 축 (ㅡ)

                    // [수정] FOV 적용을 위한 각도 및 실제 거리 계산
                    double angle_to_obj = std::atan2(local_y, local_x);
                    double dist = std::sqrt(dist_sq);

                    if (local_x > 0.0 && // 상대 차량이 내 앞에 있는가
                        std::abs(angle_to_obj) < VIEW_HALF_ANGLE && // [수정] 시야각 내에 있는가 (사선 감지 가능)
                        dist < CRITICAL_STOP_DIST // [수정] 실제 유클리드 거리가 정지 기준 거리보다 짧은가
                        ) 
                    {
                        stop_command = true; // 정지 신호 발행
                        std::stringstream ss;
                        ss << "SAFETY (Obs: " << id_b << ", Dist: " << std::fixed << std::setprecision(2) << dist << "m)";
                        stop_reasons[id_a] = ss.str();
                    }
                }


                // [교차로 충돌 방지 로직]
                if (!stop_command) { // 정지 상태가 아닌 경우만 연산
                    
                    // (A) 진입 전 대기: 안에서 회전 중인 차가 있으면 대기
                    if (am_i_approaching) {
                        int other_critical_id = get_intersection_id(state_b.x, state_b.y, -1.0);
                        if (other_critical_id == my_approach_id) {
                            bool is_turning = std::abs(state_b.yaw_rate) > YAW_RATE_THRESHOLD;
                            if (is_turning) {
                                stop_command = true;
                                stop_reasons[id_a] = "INTERSECTION (Waiting for turning " + id_b + ")";
                            }
                        }
                    }


                    // (B) 진입 후 경합: 이미 둘 다 안에 있을 때
                    if (!stop_command) {
                        int inter_id_b = get_intersection_id(state_b.x, state_b.y, -1.0);
                        if (my_critical_id != -1 && (my_critical_id == inter_id_b)) {
                            // HV는 무조건 피함
                            if (state_b.type == VehicleType::HV) {
                                stop_command = true;
                                stop_reasons[id_a] = "INTERSECTION (Yield to HV " + id_b + ")";
                            }
                            // CAV 끼리는 ID 낮은 쪽이 우선 (높은 쪽이 양보)
                            else {
                                if (state_a.id_num > state_b.id_num) {
                                    stop_command = true;
                                    stop_reasons[id_a] = "INTERSECTION (Yield to Lower ID " + id_b + ")";
                                }
                            }
                        }
                    }
                }

                
                // 정지 확정되면 다른 차량 볼 필요 없음 (Immediate Break)
                if(stop_command) break; 
            }
            
            if (stop_command) next_stop_state[id_a] = true;
        }


        // -----------------------------------------------------------------
        // [Log & Publish]
        // -----------------------------------------------------------------
        for(auto& [id, should_stop] : next_stop_state) {
            // 상태 변화가 있을 때만 로그 출력
            if (vehicle_db_[id].is_stopped != should_stop) {
                std::string reason = stop_reasons[id];
                
                if (should_stop) {
                    if (reason.find("SAFETY") != std::string::npos) {
                        RCLCPP_INFO(this->get_logger(), "%s[%s] STOP: %s%s", 
                            ANSI_YELLOW.c_str(), id.c_str(), reason.c_str(), ANSI_RESET.c_str());
                    } 
                    else if (reason.find("INTERSECTION") != std::string::npos) {
                        RCLCPP_INFO(this->get_logger(), "%s[%s] STOP: %s%s", 
                            ANSI_GREEN.c_str(), id.c_str(), reason.c_str(), ANSI_RESET.c_str());
                    }
                    else {
                        RCLCPP_INFO(this->get_logger(), "[%s] STOP: %s", id.c_str(), reason.c_str());
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s[%s] GO: Path Clear%s", 
                        ANSI_BLUE.c_str(), id.c_str(), ANSI_RESET.c_str());
                }
            }
            
            vehicle_db_[id].is_stopped = should_stop;


            if (stop_pubs_.find(id) != stop_pubs_.end()) {
                std_msgs::msg::Bool msg;
                msg.data = should_stop;
                stop_pubs_[id]->publish(msg);
            }
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
