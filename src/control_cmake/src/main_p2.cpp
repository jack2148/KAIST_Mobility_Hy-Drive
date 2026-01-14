// main_p2.cpp
// Problem 2: 추월 및 고속 주행 (CSV 경로 기반 차선 인식 포함)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <regex>
#include <limits>

using std::placeholders::_1;

// 데이터 구조체 정의
struct Point {
    double x;
    double y;
};

enum class VehicleType { CAV, HV };

struct VehicleState {
    double x, y;
    double speed;
    double yaw; 
    rclcpp::Time last_update;
    bool is_active;
    int id_num; 
    bool is_stopped;
    VehicleType type;
};

class MainP2Controller : public rclcpp::Node
{
public:
    MainP2Controller() 
    : Node("main_p2_controller"), qos_(10)
    {
        qos_.best_effort();
        qos_.durability_volatile();

        // [설정] 맵이 작으므로 감지 범위 5m로 제한
        this->declare_parameter("filter_dist", 3.0); 
        filter_dist_ = this->get_parameter("filter_dist").as_double();

        // -------------------------------------------------------------
        // [핵심] CSV 경로 파일 로딩 (이게 없으면 차선 인식이 안됨!)
        // -------------------------------------------------------------
        this->declare_parameter("csv_path_1", "tool/fastp2.csv");
        this->declare_parameter("csv_path_2", "tool/slowp2.csv");
        
        std::string path1 = this->get_parameter("csv_path_1").as_string();
        std::string path2 = this->get_parameter("csv_path_2").as_string();

        load_waypoints(path1, lane1_points_);
        load_waypoints(path2, lane2_points_);
        
        RCLCPP_INFO(this->get_logger(), "Problem 2 Controller Started. Lane1: %zu, Lane2: %zu", 
            lane1_points_.size(), lane2_points_.size());

        // 타이머 설정
        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MainP2Controller::discover_vehicles, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MainP2Controller::control_loop, this));
    }

private:
    // 멤버 변수
    std::vector<Point> lane1_points_;
    std::vector<Point> lane2_points_;

    // CSV 파일 읽기 함수
    void load_waypoints(const std::string& path, std::vector<Point>& target_vec) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV: %s", path.c_str());
            return;
        }
        std::string line;
        std::getline(file, line); // 헤더 스킵
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) {
                try { target_vec.push_back({std::stod(row[0]), std::stod(row[1])}); } 
                catch (...) {}
            }
        }
    }

    // 경로까지의 최단 거리 계산
    double get_distance_to_path(double x, double y, const std::vector<Point>& points) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p : points) {
            double dist = std::hypot(x - p.x, y - p.y);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }

    // [핵심] 차선 인식 함수 (이게 있어야 control_loop가 돌아감)
    int identify_lane(double x, double y) {
        if (lane1_points_.empty() || lane2_points_.empty()) return 2;
        double d1 = get_distance_to_path(x, y, lane1_points_);
        double d2 = get_distance_to_path(x, y, lane2_points_);
        return (d1 < d2) ? 1 : 2;
    }

    // 유령 차량 제거
    void clean_stale_vehicles() {
        rclcpp::Time now = this->now();
        for (auto it = vehicle_db_.begin(); it != vehicle_db_.end(); ) {
            if ((now - it->second.last_update).seconds() > 0.5) {
                it = vehicle_db_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // 차량 발견 및 등록
    void discover_vehicles() {
        auto topic_names = this->get_topic_names_and_types();
        std::regex cav_re(R"((.*)/?(CAV_?(\d+))/?.*$)"); 
        std::regex hv_re(R"((.*)/?(HV_?(\d+))/?.*$)");    

        for (const auto& [name, types] : topic_names) {
            bool is_pose = false;
            for (const auto& t : types) if (t.find("PoseStamped") != std::string::npos) is_pose = true;
            if (!is_pose) continue;

            std::smatch m;
            if (std::regex_search(name, m, cav_re)) {
                std::string id = m[2].str();
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, std::stoi(m[3]), VehicleType::CAV);
            } else if (std::regex_search(name, m, hv_re)) {
                std::string id = m[2].str();
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, std::stoi(m[3]), VehicleType::HV);
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, int num, VehicleType type) {
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ update_state(id, msg); });
        
        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            // [중요] 3종 세트 퍼블리셔 생성
            stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
            speed_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(id + "/cmd_speed", qos_);
            lane_pubs_[id] = this->create_publisher<std_msgs::msg::Int32>(id + "/cmd_lane", qos_);
        } else {
            hv_subs_[id] = sub;
        }
        vehicle_db_[id] = {0,0,0,0, this->now(), false, num, false, type};
        RCLCPP_INFO(this->get_logger(), "Registered: %s", id.c_str());
    }

    void update_state(const std::string& id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto& v = vehicle_db_[id];
        double qz = msg->pose.orientation.z, qw = msg->pose.orientation.w;
        v.yaw = std::atan2(2.0*(qw*qz), 1.0-2.0*(qz*qz));
        v.x = msg->pose.position.x; v.y = msg->pose.position.y;
        v.last_update = this->now();
        v.is_active = true;
    }

    // ==================================================================================
    // [제어 로직] 요청하신 파라미터 적용 (Side 0.2 / Rear 0.3 / Front 0.5)
    // ==================================================================================
// ==================================================================================
    // [최종 수정] 회전 시 장애물 놓침 방지 + 반응 거리 확대
    // ==================================================================================
void control_loop() {
        if (vehicle_db_.empty()) return;
        clean_stale_vehicles(); 

        std::vector<std::string> active_cavs;
        for(auto const& [id, v] : vehicle_db_) 
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);

        for(const auto& my_id : active_cavs) {
            auto& my_car = vehicle_db_[my_id];

            int current_lane = identify_lane(my_car.x, my_car.y);
            int target_lane = current_lane;
            double MAX_SPEED = 1.7; 
            double target_speed = MAX_SPEED; 
            bool should_stop = false;
            std::string status_msg = "Cruising"; 

            double front_dist = 999.0;
            double my_cos = std::cos(my_car.yaw);
            double my_sin = std::sin(my_car.yaw);

            for(const auto& [other_id, other_car] : vehicle_db_) {
                if(my_id == other_id) continue; 
                double raw_dist = std::hypot(other_car.x - my_car.x, other_car.y - my_car.y);
                if (raw_dist > 6.0) continue; 

                // ----------------------------------------------------------------
                // ★ [패닉 스탑 수정] 고정 장애물은 옆 차선이면 무시!
                // ----------------------------------------------------------------
                if (raw_dist < 0.6) {
                    bool is_panic = true;

                    // (1) 고정형 장애물(속도 0)인지 확인
                    bool is_static = (std::abs(other_car.speed) < 0.1);
                    
                    if (is_static) {
                        // (2) 다른 차선에 있는지 확인
                        int other_lane = identify_lane(other_car.x, other_car.y);
                        if (other_lane != current_lane) {
                            // "안 움직이는 벽이 옆 차선에 있는 것 뿐이다." -> 패닉 해제
                            is_panic = false; 
                        }
                    }

                    if (is_panic) {
                        front_dist = 0.0;
                        should_stop = true;
                        status_msg = "PANIC STOP (" + other_id + ")";
                        break; // 진짜 위험하니까 루프 종료
                    }
                }

                // -------------------------------------------------------------
                // [장애물 인식 로직] (기존 동일)
                // -------------------------------------------------------------
                int other_lane = identify_lane(other_car.x, other_car.y);
                if (other_lane != current_lane) continue;

                double rx = other_car.x - my_car.x;
                double ry = other_car.y - my_car.y;
                double longitudinal = rx * my_cos + ry * my_sin; 
                
                if (longitudinal < 0) continue;
                if (longitudinal < front_dist) front_dist = longitudinal;
            }

            // --- 행동 결정 (기존 동일) ---
            if (!should_stop) {
                if (front_dist > 4.0) {
                    target_speed = MAX_SPEED; 
                    status_msg = "Clear";
                } 
                else {
                    bool can_overtake = false;
                    if (current_lane == 2) {
                        if (is_lane_safe(1, my_car)) {
                            target_lane = 1;
                            target_speed = MAX_SPEED; 
                            can_overtake = true;
                            status_msg = "Overtaking!";
                        } else {
                            status_msg = "Blocked (Lane 1 Unsafe)"; 
                        }
                    } else {
                        status_msg = "Blocked in Lane 1"; 
                    }

                    if (!can_overtake) {
                        target_lane = current_lane;
                        double SAFE_GAP = 0.6; 
                        double error_dist = front_dist - SAFE_GAP;
                        double acc_speed = error_dist * 2.0; 
                        
                        if (acc_speed < 0.1 && error_dist > 0.05) acc_speed = 0.1;
                        target_speed = std::max(0.0, std::min(MAX_SPEED, acc_speed));
                    }
                }
                
                if (front_dist < 0.4) {
                    target_speed = 0.0;
                    should_stop = true;
                    status_msg = "Safety Stop";
                }
            }

            // [로그 출력]
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "[%s] L:%d | Front:%.2fm | %s", 
                my_id.c_str(), current_lane, front_dist, status_msg.c_str());

            // 명령 발행
            if (lane_pubs_.count(my_id)) lane_pubs_[my_id]->publish(std_msgs::msg::Int32().set__data(target_lane));
            if (speed_pubs_.count(my_id)) speed_pubs_[my_id]->publish(std_msgs::msg::Float64().set__data(target_speed));
            if (stop_pubs_.count(my_id)) stop_pubs_[my_id]->publish(std_msgs::msg::Bool().set__data(should_stop));
        }
    }

// ==================================================================================
    // [공격적 튜닝] 추월 기준 대폭 완화 (Aggressive Overtaking)
    // ==================================================================================
    bool is_lane_safe(int target_lane_num, const VehicleState& my_car) {
        double my_cos = std::cos(my_car.yaw);
        double my_sin = std::sin(my_car.yaw);

        for(const auto& [id, other] : vehicle_db_) {
            if (!other.is_active) continue;
            
            // [완화 1] 감지 거리 축소 (3m -> 2.5m)
            // 너무 멀리 있는 차 때문에 쫄지 않도록 함
            double raw_dist = std::hypot(other.x - my_car.x, other.y - my_car.y);
            if (raw_dist > 2.5) continue; 

            // 목표 차선에 있는 물체만 검사
            if (identify_lane(other.x, other.y) == target_lane_num) {
                
                double rx = other.x - my_car.x;
                double ry = other.y - my_car.y;
                double dot = (rx * my_cos) + (ry * my_sin); // 내적

                // [완화 2] Side Check (좌우): 0.2m -> 0.15m
                // 내 차 중심에서 15cm만 떨어져 있어도(거의 닿을락 말락) 진입 허용
                if (std::abs(raw_dist) < 0.15) return false; 

                // [완화 3] 타입별 후방/전방 기준 대폭 축소
                bool is_static = (std::abs(other.speed) < 0.1); // 멈춰있는가?

                if (is_static) {
                    // 고정 장애물 (벽)
                    // Rear: 0.3m -> 0.1m (내 뒷바퀴만 빠져나가면 바로 꺾음)
                    if (dot < 0 && raw_dist < 0.1) return false;
                    
                    // Front: 0.5m -> 0.3m (아주 좁은 틈새라도 비집고 들어감)
                    if (dot > 0 && raw_dist < 0.3) return false;
                } else {
                    // 이동 차량 (CAV)
                    // Rear: 1.0m -> 0.6m (뒤차가 꽤 가까워도 속도 믿고 들어감)
                    if (dot < 0 && raw_dist < 0.6) return false;

                    // Front: 0.8m -> 0.4m (앞차에 바짝 붙어서 진입)
                    if (dot > 0 && raw_dist < 0.4) return false;
                }
            }
        }
        return true; 
    }

    rclcpp::TimerBase::SharedPtr discovery_timer_, control_timer_;
    rclcpp::QoS qos_;
    std::map<std::string, VehicleState> vehicle_db_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_, hv_subs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> lane_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> speed_pubs_;
    double filter_dist_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainP2Controller>());
    rclcpp::shutdown();
    return 0;
}