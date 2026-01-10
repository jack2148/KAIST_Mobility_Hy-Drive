#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>


// 로그 색상 정의
const std::string ANSI_RESET   = "\033[0m";
const std::string ANSI_BLUE    = "\033[34m";
const std::string ANSI_RED     = "\033[31m";
const std::string ANSI_CYAN    = "\033[36m";

namespace Geo {
    struct Vec2 {
        double x, y;
        Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
        Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
        double dot(const Vec2& o) const { return x * o.x + y * o.y; }
        double dist_Sq() const { return x*x + y*y; }
    };

    bool check_obb_intersection(const std::vector<Vec2>& box1, const std::vector<Vec2>& box2) {
        auto get_axes = [](const std::vector<Vec2>& b) {
            return std::vector<Vec2>{
                {b[1].x - b[0].x, b[1].y - b[0].y},
                {b[1].x - b[2].x, b[1].y - b[2].y}
            };
        };

        std::vector<Vec2> axes = get_axes(box1);
        auto axes2 = get_axes(box2);
        axes.insert(axes.end(), axes2.begin(), axes2.end());

        for (const auto& axis : axes) {
            double min1 = 1e9, max1 = -1e9;
            double min2 = 1e9, max2 = -1e9;

            for (const auto& p : box1) {
                double proj = p.dot(axis);
                min1 = std::min(min1, proj); max1 = std::max(max1, proj);
            }
            for (const auto& p : box2) {
                double proj = p.dot(axis);
                min2 = std::min(min2, proj); max2 = std::max(max2, proj);
            }
            if (max1 < min2 || max2 < min1) return false;
        }
        return true;
    }
}

struct Vehicle {
    std::string id;
    bool is_cav;
    Geo::Vec2 pos{0.0, 0.0};
    double z = 0.0;
    bool active = false;

    bool is_stopped = false;
    bool forced_stop = false; 
    std::string stop_cause = "";

    bool has_entered_roundabout = false; 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_way; 
};

class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        car_dims_ = {0.17, 0.16, 0.075, 0.075}; // 차량 축 기준 앞/뒤/좌/우 길이

        // 정면, 측면 충돌 방지 파라미터
        front_padding_ = 0.4;
        side_padding_ = 0.5;
        approach_range_sq_ = 2.0 * 2.0; 

        // 사지교차로 정보
        fourway_center_ = {-2.333, 0.0};
        fourway_len_ = 2.0; // 사지교차로 한 변의 길이. (정사각형임)
        fourway_app_r_sq_ = 1.5 * 1.5; 
        fourway_box_half_len_ = fourway_len_ / 2.0; // 2x2 크기의 사지교차로를 위아래로 반띵해서 직사각형 2개로 나눌거임 
    

        // 회전교차로 정보
        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4; 

        // 삼거리 정보
        t3_box_x_min_ = -3.5; t3_box_x_max_ = -1.3;
        t3_1_y_min_ = 1.6;  t3_1_y_max_ = 2.7; // 위쪽 삼거리가 t3_1
        t3_2_y_min_ = -2.7; t3_2_y_max_ = -1.9; // 아래쪽 삼거리가 t3_2

        // 타이머 생성
        tmr_discovery_ = create_wall_timer(std::chrono::seconds(1), // 1s마다 새로 생성된 차량 확인 및 갱신
            std::bind(&MainTrafficController::discover_vehicles, this));

        tmr_control_ = create_wall_timer(std::chrono::milliseconds(20), // 20ms마다 충돌 등 계산
            std::bind(&MainTrafficController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Controller Started: FCFS Priority Logic (Obey First, Then Act).");
    }

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;

    std::vector<double> car_dims_;
    double front_padding_, side_padding_, approach_range_sq_;
    Geo::Vec2 fourway_center_;
    double fourway_len_, fourway_app_r_sq_, fourway_box_half_len_; 
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;
    double t3_box_x_min_, t3_box_x_max_, t3_1_y_min_, t3_1_y_max_, t3_2_y_min_, t3_2_y_max_;


// ============ 제어에 필요한 함수 모음 ===============

    // target차량이 spot을 향하고 있는지 벡터 내적으로 확인 
    bool is_approaching(const Vehicle& spot, const Geo::Vec2& target_pos) {
        Geo::Vec2 vec = target_pos - spot.pos;
        return vec.dot({std::cos(spot.z), std::sin(spot.z)}) > 0;
    }

    // 패딩 포함 차량의 충돌 범위 도형(직사각형)을 생성
    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double front_ext, double side_pad) {
        double f = car_dims_[0] + front_ext;
        double r = car_dims_[1];
        double l = car_dims_[2] + side_pad;
        double rt = car_dims_[3] + side_pad;
        std::vector<Geo::Vec2> locals = {{f, l}, {f, -rt}, {-r, -rt}, {-r, l}};
        std::vector<Geo::Vec2> world_corners;
        double c = std::cos(v.z), s = std::sin(v.z);
        for (const auto& p : locals) {
            world_corners.push_back({v.pos.x + (p.x * c - p.y * s), v.pos.y + (p.x * s + p.y * c)});
        }
        return world_corners;
    }

    // 현재 맵에 있는 차량 종류 및 id 확인
    void discover_vehicles() {
        auto topic_map = this->get_topic_names_and_types();

        for (const auto& [name, types] : topic_map) {
            if (name.empty() || name[0] != '/') continue;

            std::string id = "";
            bool is_cav = false;
            
            // 이름이 /CAV_XX 또는 /HV_XX 형태인지 확인 (최소 길이: / + 3글자 접두사 + 2글자 숫자 = 6글자 이상)
            if (name.size() > 7) continue; 

            
            // CAV 확인
            if (name.substr(1, 4) == "CAV_") { // substr(1, 4)는 인덱스 1부터 4글자 -> "CAV_"
                std::string num_str = name.substr(5, 2); // "01"
                try {
                    id = "CAV_" + num_str;
                    is_cav = true;
                } catch (...) { continue; } // 예외 발생해도 무시
            }
            
            // HV 확인
            else if (name.substr(1, 3) == "HV_") {  // substr(1, 3)는 인덱스 1부터 3글자 -> "HV_"
                std::string num_str = name.substr(4, 2); // "19"
                try {
                    id = "HV_" + num_str;
                    is_cav = false;
                } catch (...) { continue; }
            }

            // ID가 존재하고 미등록 차량일 때만 등록 함수 호출
            if (!id.empty() && !vehicles_.count(id)){
                register_vehicle(id, name, is_cav);
            }
                
        }
    }


    // 차량 등록 함수
    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v; 
        v.id = id; 
        v.is_cav = is_cav;

        // 찾아낸 차량들은 현재 위치 토픽 구독
        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                // 맵의 재할당(Rehashing) 상황에서도 안전하게 접근하기 위해 ID로 검색
                if (vehicles_.count(id)) {
                    vehicles_[id].pos = {msg->pose.position.x, msg->pose.position.y};
                    vehicles_[id].z = msg->pose.orientation.z; 
                    vehicles_[id].active = true;
                }
            });

        // CAV 한정 정지 및 경로 변경 토픽 발행
        if (is_cav) {
            // 제어 토픽 생성 규칙: /ID/명령어 (예: /CAV_01/cmd_stop)
            v.pub_stop = create_publisher<std_msgs::msg::Bool>("/" + id + "/cmd_stop", rclcpp::SensorDataQoS());
            v.pub_change_way = create_publisher<std_msgs::msg::Bool>("/" + id + "/change_waypoint", rclcpp::SensorDataQoS());
        }

        // 차량 객체를 맵에 등록 (Move Semantics 사용)
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s (Topic: %s)", id.c_str(), topic.c_str());
    }

    // 삼거리 안에 특정 차량이 존재하는가 
    bool is_in_t3_zone(const Geo::Vec2& pos) {
        return (pos.x >= t3_box_x_min_ && pos.x <= t3_box_x_max_) && 
               ((pos.y >= t3_1_y_min_ && pos.y <= t3_1_y_max_) || (pos.y >= t3_2_y_min_ && pos.y <= t3_2_y_max_));
    }

    // 사지 교차로, 회전 교차로, 3거리 중 한 곳에라도 차량이 있으면 true
    bool is_in_intersection_zone(const Vehicle& v) {
        if ((v.pos - round_center_).dist_Sq() <= round_app_r_sq_) return true;
        if ((v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_) return true;
        if (is_in_t3_zone(v.pos)) return true; 
        return false;
    }

    // 정면, 측면 충돌 체크 로직
    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, std::string& log_msg) {

        // 회전교차로 내부 차량은 충돌 검사 안함 -> 만약 hv가 느려지면 이 조건 빼야함
        double dist_to_round = (my_cav.pos - round_center_).dist_Sq();
        if (dist_to_round <= round_app_r_sq_ && !is_approaching(my_cav, round_center_)) return false; 
        
        

        const double PARALLEL_ANGLE_THRESHOLD = 0.40; 
        double round_protect_sq = (round_radius_) * (round_radius_);
        double vehicle_width = car_dims_[2] + car_dims_[3]; 
        bool am_i_in_t3 = is_in_t3_zone(my_cav.pos);

        for (const auto& [tid, target] : vehicles_) {
            
            // my_cav ~ target간의 거리가 approach거리보다 크거나 target == my_cav인 경우 무시
            if (tid == my_cav.id || (my_cav.pos - target.pos).dist_Sq() > approach_range_sq_) continue;
          

            bool target_in_t3 = is_in_t3_zone(target.pos);
            bool t3_context = am_i_in_t3 && target_in_t3; 

            // my_cav 기준 target의 상대좌표 계산
            double dz = std::abs(std::abs(my_cav.z - target.z));
            while (dz > M_PI) dz -= 2.0 * M_PI;
            dz = std::abs(dz);
            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            
            // 나란히 주행하는 차량이면 측면 충돌은 무시해도 됨
            double side_dist = std::abs(-std::sin(my_cav.z) * dx + std::cos(my_cav.z) * dy);
            bool is_parallel = (dz < PARALLEL_ANGLE_THRESHOLD) || (dz > (M_PI - PARALLEL_ANGLE_THRESHOLD));
            bool is_next_lane = is_parallel && (side_dist > vehicle_width * 1.2); 
            
            if (!t3_context && is_next_lane) continue; // my_cav, target이 삼거리에 없고 옆라인이면 충돌 연산 무시

            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto front_box  = get_vehicle_corners(my_cav, front_padding_, 0.0);
            auto full_box   = get_vehicle_corners(my_cav, front_padding_, side_padding_);

            bool is_front = Geo::check_obb_intersection(front_box, target_box); // my_cav의 정면과 target이 닿았는가 => 정면 충돌
            bool is_side = false;
            if (!is_front) is_side = Geo::check_obb_intersection(full_box, target_box);

            if (is_front || is_side) {
                // T3 우선순위 (기존 유지)
                if (t3_context) {
                    if (my_cav.id > target.id) {
                        conflict_id = tid; log_msg = "T3 SIDE(ID) YIELD -> " + tid; return true; 
                    } else continue; 
                }

                // 앞뒤 관계 체크 (Blocking)
                double rel_x_target = (target.pos.x - my_cav.pos.x) * std::cos(my_cav.z) + (target.pos.y - my_cav.pos.y) * std::sin(my_cav.z);
                double rel_x_me = (my_cav.pos.x - target.pos.x) * std::cos(target.z) + (my_cav.pos.y - target.pos.y) * std::sin(target.z);

                if (rel_x_target > 0.0 && rel_x_me <= 0.0) {
                    conflict_id = tid; log_msg = "BLOCKED BY " + tid; return true;
                }
                if (rel_x_target <= 0.0 && rel_x_me > 0.0) continue; 
            }

            // ==========================================
            // [수정] 측면 충돌 처리 (공격적 로직: I GO, YOU STOP)
            // ==========================================
            if (is_side && !t3_context) {
                double t_dist_round = (target.pos - round_center_).dist_Sq();
                
                // 회전교차로 내부가 아니면 공격적으로 대응
                if (t_dist_round > round_protect_sq) {
                    // 상대가 CAV라면 강제로 세움
                    if (target.is_cav) {
                        vehicles_[tid].forced_stop = true;
                        vehicles_[tid].stop_cause = "FORCED_BY_" + my_cav.id + "_SIDE";
                    }
                    // 나는 멈추지 않음 (GO)
                    log_msg = "SIDE RISK -> I GO, TARGET STOPS";
                    continue; 
                }
            }
        }
        return false;
    }

    // ==========================================
    // [수정] 제어 루프 (우선순위 로직 핵심 변경)
    // ==========================================
    void control_loop() {
        if (vehicles_.empty()) return;
        
        // 매 틱마다 플래그 초기화 (필수)
        for (auto& [id, v] : vehicles_) v.forced_stop = false;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            bool should_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";

            // [핵심 변경] 1. 남이 나에게 내린 정지 명령을 최우선으로 확인
            // 내가 정지 명령을 받았다면, is_conflict(내가 남을 멈추게 하는 로직)를 실행하지 않음!
            if (my_cav.forced_stop) {
                should_stop = true;
                cause_id = my_cav.stop_cause;
                log_msg = "YIELDING (FORCED) -> " + cause_id;
            }
            // 2. 정지 명령이 없을 때만 상황 판단 수행
            else if (is_in_intersection_zone(my_cav)) {
                // 여기서 is_conflict가 호출되면, 나는 자유로운 상태이므로
                // 위험 감지 시 상대를 멈추게 하고(target.forced_stop=true) 나는 감(return false).
                if (is_conflict(my_cav, cause_id, log_msg)) {
                    should_stop = true;
                }
                
                // ... (교차로 진입 로직)
                if (!should_stop) {
                    if (check_fourway_rect_split(my_cav)) {
                        should_stop = true; cause_id = "4WAY_BLOCK"; log_msg = "4WAY_RECT_FULL";
                    } 
                    else {
                        bool round_yield = check_roundabout(my_cav);
                        if (round_yield) {
                            should_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                            my_cav.has_entered_roundabout = false; 
                        } else {
                            // 회전교차로 진입/경로 변경 로직
                            double dist_sq = (my_cav.pos - round_center_).dist_Sq();
                            double enter_zone_sq = round_app_r_sq_ * 1.1; 
                            if (dist_sq <= enter_zone_sq && dist_sq > (round_radius_*round_radius_) && !my_cav.has_entered_roundabout) {
                                int inside_cavs = count_cavs_in_roundabout();
                                bool go_inside = (inside_cavs >= 1);
                                std_msgs::msg::Bool way_msg; way_msg.data = go_inside;
                                my_cav.pub_change_way->publish(way_msg);
                                std::string path_str = go_inside ? "INSIDE" : "ORIGINAL";
                                RCLCPP_INFO(get_logger(), "%s[%s] Roundabout Entry -> Path: %s (Inside: %d)%s", ANSI_CYAN.c_str(), my_id.c_str(), path_str.c_str(), inside_cavs, ANSI_RESET.c_str());
                                my_cav.has_entered_roundabout = true; 
                            }
                        }
                    }
                }
            } else {
                 my_cav.has_entered_roundabout = false;
            }

            // [상태 반영]
            if (my_cav.is_stopped != should_stop) {
                if (should_stop) RCLCPP_INFO(get_logger(), "%s[%s] STOP: %s%s", ANSI_RED.c_str(), my_id.c_str(), log_msg.c_str(), ANSI_RESET.c_str());
                else RCLCPP_INFO(get_logger(), "%s[%s] GO: %s%s", ANSI_BLUE.c_str(), my_id.c_str(), (log_msg.empty() ? "Path Clear" : log_msg.c_str()), ANSI_RESET.c_str());
            }

            my_cav.is_stopped = should_stop;
            if (should_stop && my_cav.stop_cause.empty()) my_cav.stop_cause = cause_id;
            else if (!should_stop) my_cav.stop_cause.clear();

            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg; msg.data = should_stop; my_cav.pub_stop->publish(msg);
            }
        }
    }

    bool check_fourway_rect_split(const Vehicle& my_v) {
        double x_min = fourway_center_.x - fourway_box_half_len_;
        double x_max = fourway_center_.x + fourway_box_half_len_;
        double y_min = fourway_center_.y - fourway_box_half_len_;
        double y_max = fourway_center_.y + fourway_box_half_len_;

        bool i_am_inside = (my_v.pos.x >= x_min && my_v.pos.x <= x_max && my_v.pos.y >= y_min && my_v.pos.y <= y_max);
        if (i_am_inside) return false;
        if ((my_v.pos - fourway_center_).dist_Sq() > fourway_app_r_sq_) return false;
        if (!is_approaching(my_v, fourway_center_)) return false;

        bool am_i_top = (my_v.pos.y > fourway_center_.y);
        int count_top = 0, count_bottom = 0;
        for (const auto& [id, v] : vehicles_) {
            if (!v.active) continue; 
            if (v.pos.x >= x_min && v.pos.x <= x_max && v.pos.y >= y_min && v.pos.y <= y_max) {
                if (v.pos.y > fourway_center_.y) count_top++; else count_bottom++;
            }
        }
        return am_i_top ? (count_top >= 1) : (count_bottom >= 1);
    }

    int count_cavs_in_roundabout() {
        int count = 0;
        double r_sq = round_radius_ * round_radius_;
        for (const auto& [id, v] : vehicles_) {
            if (v.is_cav && v.active && (v.pos - round_center_).dist_Sq() <= r_sq) count++;
        }
        return count;
    }

    bool check_roundabout(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        double r_sq = round_radius_ * round_radius_;
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_)) return false;

        int cavs_inside = count_cavs_in_roundabout();
        if (cavs_inside >= 2) return true; 

        auto is_blocked_by_hv_check = [&](const Vehicle& v) -> bool {
            double default_check_left = 2.1 / round_radius_;
            double default_check_right = 0.2 / round_radius_;
            if (v.is_stopped && v.stop_cause == "ROUND_YIELD") default_check_left += 0.5; 
            double v_angle = std::atan2(v.pos.y - round_center_.y, v.pos.x - round_center_.x); 

            for (const auto& [tid, target] : vehicles_) {
                if (tid == v.id || !target.active || target.is_cav) continue;
                if ((target.pos - round_center_).dist_Sq() <= r_sq){
                    double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
                    double diff = t_angle - v_angle;
                    while (diff > M_PI) diff -= 2.0 * M_PI;
                    while (diff < -M_PI) diff += 2.0 * M_PI;
                    if ((diff > 0 && diff < default_check_right) || (diff <= 0 && diff > -default_check_left)) return true;
                }
            }
            return false;
        };

        if (is_blocked_by_hv_check(my_cav)) return true;

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.is_cav || !target.active) continue;
            double t_dist_sq = (target.pos - round_center_).dist_Sq();
            if (t_dist_sq > round_app_r_sq_ || t_dist_sq <= r_sq || !is_approaching(target, round_center_)) continue;
            if (is_blocked_by_hv_check(target)) continue;
            if (t_dist_sq < dist_sq - 0.01) return true;
            if (std::abs(t_dist_sq - dist_sq) <= 0.01 && tid < my_cav.id) return true; 
        }
        return false;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
