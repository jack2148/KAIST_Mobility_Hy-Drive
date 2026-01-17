#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp> 
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <deque> 


// 로그 색상 정의
const std::string ANSI_RESET   = "\033[0m";
const std::string ANSI_BLUE    = "\033[34m";
const std::string ANSI_RED     = "\033[31m";
const std::string ANSI_CYAN    = "\033[36m";
const std::string ANSI_YELLOW  = "\033[33m";
const std::string ANSI_GREEN   = "\033[32m";


// 2D 벡터 연산 및 OBB(Oriented Bounding Box) 충돌 감지를 위한 네임스페이스
namespace Geo {
    // 기본적인 벡터 연산을 위한 구조체
    struct Vec2 {
        double x, y;
        Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; } // 벡터 합 
        Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; } // 벡터 차
        double dot(const Vec2& o) const { return x * o.x + y * o.y; } // 벡터 내적
        double dist_Sq() const { return x*x + y*y; } // 두 점 사이의 거리의 제곱
    };

    // 두 개의 회전된 직사각형(OBB) 간의 충돌 여부를 분리 축 이론(SAT)을 사용하여 판별하는 함수
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

// 차량의 상태 정보(ID, 위치, 속도, 활성 여부 등)와 ROS 통신 객체(Publisher/Subscriber)를 관리하는 구조체
struct Vehicle {
    std::string id;
    bool is_cav;
    Geo::Vec2 pos{0.0, 0.0};
    double z = 0.0;
    bool active = false;
    bool is_stopped = false; // 현재 정지 상태인가
    bool forced_stop = false; // 다른 cav에 의해 강제로 정지되었는가
    std::string stop_cause = ""; // 내 차량을 멈추게 한 다른 차량은 누구인가
    bool has_entered_roundabout = false; // 회전교차로 내부에 들어갔는가


    // hv 속도 계산용 변수
    Geo::Vec2 last_pos{0.0, 0.0};
    rclcpp::Time last_time;

    // subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    
    // publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_way;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_roundabout;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hv_vel;
};

// 전체 교통 흐름을 제어하며 차량 간 충돌 방지 및 교차로 통행 우선순위를 관리하는 메인 ROS2 노드 클래스
class MainTrafficController : public rclcpp::Node {
public:
    // 노드를 초기화하고 차량 크기, 도로 구역 설정, 타이머(차량 발견 및 제어 루프)를 생성하는 생성자
    MainTrafficController() : Node("main_traffic_controller") {
        // 차량 정보
        car_info = {0.17, 0.16, 0.075, 0.075};
        front_padding_ = 1.2;
        side_padding_ = 0.4;

        // 충돌 감지 범위 : 이 범위 안에 my_cav외의 다른 차량이 있을 경우 충돌 감지 로직 실행
        approach_range_sq_ = 2.5 * 2.5;

        // 사지교차로 관련 변수
        fourway_center_ = {-2.333, 0.0};
        fourway_len_ = 2.0;
        fourway_app_r_sq_ = 1.7 * 1.7;
        fourway_box_half_len_ = fourway_len_ / 2.0;

        // 회전교차로 관련 변수
        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4;

        // 삼지교차로 관련 변수
        threeway_box_x_min_ = -3.5; threeway_box_x_max_ = -1.3;
        threeway_1_y_min_ = 1.5;    threeway_1_y_max_ = 2.7;
        threeway_2_y_min_ = -2.8;   threeway_2_y_max_ = -1.9;


        // 타이머
        tmr_discovery_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this)
        );
        tmr_control_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MainTrafficController::control_loop, this)
        );


        RCLCPP_INFO(get_logger(), "Controller Started: Publishing individual /hv_vel topics.");
    }


private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::map<std::string, std::string> conflict_info_;


    // HV 속도 이동 평균 필터용 변수
    std::deque<double> hv_vel_buffer_;
    const int vel_buffer_size_ = 10;

    // 각 교차로 및 거리 기반 충돌 감지용 변수
    std::vector<double> car_info;
    double front_padding_, side_padding_, approach_range_sq_;
    Geo::Vec2 fourway_center_;
    double fourway_len_, fourway_app_r_sq_, fourway_box_half_len_;
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;
    double threeway_box_x_min_, threeway_box_x_max_, threeway_1_y_min_, threeway_1_y_max_, threeway_2_y_min_, threeway_2_y_max_;

    // 차량 정지상황 발생 시 conflict_info에 멈춘 차량 - 원인 차량 추가
    void update_conflict_status(const std::string& stopped_cav, const std::string& cause_vehicle) {
        if (conflict_info_.count(stopped_cav) && conflict_info_[stopped_cav] == cause_vehicle) return; // 이미 같은 상황이 저장되어있으면 무시
        conflict_info_[stopped_cav] = cause_vehicle;
        RCLCPP_INFO(get_logger(), "%s[CONFLICT START] %s stopped by %s%s",
                    ANSI_YELLOW.c_str(), stopped_cav.c_str(), cause_vehicle.c_str(), ANSI_RESET.c_str());
    }

    // 차량 정지상황 해소 시 conflict_info에서 제거
    void clear_conflict_status(const std::string& stopped_cav) {
        if (conflict_info_.count(stopped_cav)) {
            RCLCPP_INFO(get_logger(), "%s[CONFLICT END] %s resumed (was stopped by %s)%s",
                        ANSI_CYAN.c_str(), stopped_cav.c_str(), conflict_info_[stopped_cav].c_str(), ANSI_RESET.c_str());
            conflict_info_.erase(stopped_cav);
        }
    }

    // 차량이 특정 목표 지점을 향해 이동하고 있는지 벡터 내적을 통해 판단하는 함수
    bool is_approaching(const Vehicle& spot, const Geo::Vec2& target_pos) {
        Geo::Vec2 vec = target_pos - spot.pos;
        return vec.dot({std::cos(spot.z), std::sin(spot.z)}) > 0;
    }

    // 차량의 4개의 꼭짓점을 월드좌표계로 반환하는 함수
    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double front_ext, double side_pad) {
        double front = car_info[0] + front_ext;
        double rear = car_info[1];
        double left = car_info[2] + side_pad;
        double right = car_info[3] + side_pad;
        std::vector<Geo::Vec2> locals = {{front, left}, {front, -right}, {-rear, -right}, {-rear, left}};
        std::vector<Geo::Vec2> world_corners;
        double c = std::cos(v.z), s = std::sin(v.z);
        for (const auto& p : locals) {
            world_corners.push_back({
                v.pos.x + (p.x * c - p.y * s),
                v.pos.y + (p.x * s + p.y * c)
            });
        }
        return world_corners;
    }

    // 현재 활성화된 ROS 토픽을 주기적으로 스캔하여 새로운 CAV 또는 HV 차량을 감지하고 등록하는 함수
    void discover_vehicles() {
        auto topic_map = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_map) {
            if (name.empty() || name[0] != '/') continue;
            std::string id = "";
            bool is_cav = false;
            if (name.size() > 7) continue;
            if (name.substr(1, 4) == "CAV_") {
                std::string num_str = name.substr(5, 2);
                try { id = "CAV_" + num_str; is_cav = true; } catch (...) { continue; }
            } else if (name.substr(1, 3) == "HV_") {
                std::string num_str = name.substr(4, 2);
                try { id = "HV_" + num_str; is_cav = false; } catch (...) { continue; }
            }
            if (!id.empty() && !vehicles_.count(id)) {
                register_vehicle(id, name, is_cav);
            }
        }
    }

    // 감지된 차량에 대한 구독(Sub) 및 발행(Pub) 객체를 생성 / HV 차량의 t linear 속도를 계산
    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v; v.id = id; v.is_cav = is_cav;
        v.last_time = this->now();


        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (vehicles_.count(id)) {
                    Vehicle& veh = vehicles_[id];
                    
                    double prev_x = veh.last_pos.x;
                    double prev_y = veh.last_pos.y;
                    rclcpp::Time prev_time = veh.last_time;


                    veh.pos = {msg->pose.position.x, msg->pose.position.y};
                    veh.z = msg->pose.orientation.z;
                    veh.active = true;
                    
                    veh.last_pos = veh.pos;
                    veh.last_time = msg->header.stamp;


                    // [HV_19 속도 계산 및 도메인별 pub 로직]
                    if (id == "HV_19") {
                        double dt = (veh.last_time - prev_time).seconds();
                        if (dt > 0.001) { 
                            double dx = veh.pos.x - prev_x;
                            double dy = veh.pos.y - prev_y;
                            double dist = std::hypot(dx, dy);
                            double vel = dist / dt;

                            // 이동 평균 필터
                            hv_vel_buffer_.push_back(vel);
                            if (hv_vel_buffer_.size() > vel_buffer_size_) {
                                hv_vel_buffer_.pop_front();
                            }

                            double sum = 0.0;
                            for (double v : hv_vel_buffer_) sum += v;
                            double avg_vel = sum / hv_vel_buffer_.size();


                            // /hv_vel 메시지 생성
                            std_msgs::msg::Float32 vel_msg;
                            vel_msg.data = static_cast<float>(avg_vel - 0.05); // 회전 교차로 내부 cav는 충돌 감지로직 사용 안함. -> 충돌 예방을 위해 실제 hv 속도값보다 더 작게 줌
                            
                            for (auto& [target_id, target_v] : vehicles_) {
                                if (target_v.is_cav && target_v.active && target_v.pub_hv_vel) {
                                    target_v.pub_hv_vel->publish(vel_msg);
                                }
                            }
                        }
                    }
                }
            }
        );

        // 각 CAV에 발행할 토픽 생성
        if (is_cav) {
            v.pub_stop = create_publisher<std_msgs::msg::Bool>("/" + id + "/cmd_stop", rclcpp::SensorDataQoS());
            v.pub_change_way = create_publisher<std_msgs::msg::Bool>("/" + id + "/change_waypoint", rclcpp::SensorDataQoS());
            v.pub_is_roundabout = create_publisher<std_msgs::msg::Bool>("/" + id + "/is_roundabout", rclcpp::SensorDataQoS());
            v.pub_hv_vel = create_publisher<std_msgs::msg::Float32>("/" + id + "/hv_vel", rclcpp::SensorDataQoS());
        }
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s", id.c_str());
    }


    // 차량이 threeway(삼지교차로) 구역 내에 위치하는지 판별하는 함수
    bool is_in_threeway_zone(const Geo::Vec2& pos) {
        return (pos.x >= threeway_box_x_min_ && pos.x <= threeway_box_x_max_) &&
               ((pos.y >= threeway_1_y_min_ && pos.y <= threeway_1_y_max_) ||
                (pos.y >= threeway_2_y_min_ && pos.y <= threeway_2_y_max_));
    }

    // 차량이 회전교차로 접근 구역 내에 위치하는지 판별하는 함수
    bool is_in_round_zone(const Vehicle& v){ return (v.pos - round_center_).dist_Sq() <= round_app_r_sq_; }

    // 차량이 사지교차로 접근 구역 내에 위치하는지 판별하는 함수
    bool is_in_fourway_zone(const Vehicle& v){ return (v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_; }

    // 차량이 정의된 3가지 주요 위험 구역(threeway, 회전교차로, 4거리) 중 하나에 있는지 확인하는 함수
    bool is_in_danger_zone(const Vehicle& v) {
        if ((v.pos - round_center_).dist_Sq() <= round_app_r_sq_) return true;
        if ((v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_) return true;
        if (is_in_threeway_zone(v.pos)) return true;
        return false;
    }

    // 두 차량이 평행하게 주행중인지 확인하는 함수
    bool is_parallel(const Vehicle& my_cav, const Vehicle& target){
        double dz = std::abs(std::abs(my_cav.z - target.z));
        while (dz > M_PI) dz -= 2.0 * M_PI;
        dz = std::abs(dz);
        return (dz < 0.40) || (dz > (M_PI - 0.40));
    }

    // 두 차량 간의 상황 별 충돌 위험 판단 함수
    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, std::string& log_msg, bool& deadlock_override) {

        // my_cav가 회전교차로 내부에서 회전중이면 충돌 감지 안함 => 진입만 잘한다면 멈추지 않고 회전하는게 더 안전하기 때문
        double dist_to_round = (my_cav.pos - round_center_).dist_Sq(); 
        if (dist_to_round <= (round_radius_* round_radius_) && !is_approaching(my_cav, round_center_)) return false;

        // 모든 차량(target)을 순회하며 충돌 위험이 있는지 판단
        for (const auto& [tid, target] : vehicles_) {
            
            // my_cav와 같은 차량이거나 충돌 감지 범위에 들어오지 않은 target은 넘김
            if (tid == my_cav.id || !target.active) continue;
            if ((my_cav.pos - target.pos).dist_Sq() > approach_range_sq_) continue;

            // 차량 실제 크기 + padding 기반 collision box 생성 
            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto full_box = get_vehicle_corners(my_cav, front_padding_, side_padding_);
            auto front_box = get_vehicle_corners(my_cav, front_padding_, 0.1);

            bool is_full = Geo::check_obb_intersection(full_box, target_box); // my_cav 정면+측면 충돌 감지
            bool is_front = Geo::check_obb_intersection(front_box, target_box); // my_cav 정면 충돌 감지

            // 차량 너비, 좌표, 차량 각도 차를 이용하여 target이 my_cav의 옆차선에서 주행중인지 판단 -> 옆차선이면 충돌위험 없으므로 무시 가능해짐
            double vehicle_width = car_info[2] + car_info[3];
            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            double side_dist = std::abs(-std::sin(my_cav.z) * dx + std::cos(my_cav.z) * dy);
            bool is_next_lane = is_parallel(my_cav, target) && (side_dist > vehicle_width * 1.2);

            // 실제 충돌 위험 플래그
            bool collision_detected = false;
            std::string temp_log = "";

            // 삼지교차로 충돌 감지 로직
            if (is_in_threeway_zone(my_cav.pos)) {
                if (tid == "CAV_03" || tid == "CAV_04") {
                    // cav3 or cav4와 cav1이 충돌하는 경우 : 나란히 주행하다가 cav1이 안쪽으로 꺾으며 충돌 가능 -> cav1이 정지 후 cav3 or cav4 먼저 보냄
                    if (my_cav.id == "CAV_01" && is_next_lane) {
                        collision_detected = true; temp_log = "threeway_NEXT_LANE";
                    // cav3 or cav4와 cav2가 충돌하는 경우 : 사지교차로에서 빠져나가는 cav2와 수직으로 충돌 가능 -> cav2가 정지 후 cav3 or cav4 먼저 보냄
                    } else if (my_cav.id == "CAV_02" && is_full && !is_next_lane) {
                        collision_detected = true; temp_log = "threeway_VERTICAL";
                    }
                }
            }  

            // 사지교차로 충돌 감지 로직
            else if (is_in_fourway_zone(my_cav)) {
                // 옆차선이 아니며 정면 충돌 위험이 있는 경우 my_cav가 정지
                if (is_front && !is_next_lane) {
                    collision_detected = true;
                    temp_log = "4WAY_FRONT";
                }
            }

            // 충돌 감지 시 conflict_info에 업데이트 및 데드락 방지
            if (collision_detected) {
                if (conflict_info_.count(tid) && conflict_info_[tid] == my_cav.id) { // 이미 target이 my_cav로 인해 정지한 경우 my_cav는 정지하지 않음 (데드락 방지)
                    deadlock_override = true;
                    continue;
                } else {
                    conflict_id = tid;
                    log_msg = temp_log;
                    return true;
                }
            }
        }
        return false;
    }

    // 각 구간 별 진입, 우선순위, 충돌 등에 대해 구체적인 제어를 하는 함수
    void control_loop() {
        if (vehicles_.empty()) return;


        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            // 회전 교차로 내부는 hv 속도에 맞춰 cav속도를 조절해야함 -> cav가 회전교차로 내부에 있다면 cav제어노드로 토픽 발행
            double dist_from_center = std::sqrt((my_cav.pos - round_center_).dist_Sq());
            bool inside_roundabout = (dist_from_center <= round_radius_);
            if (my_cav.pub_is_roundabout) {
                std_msgs::msg::Bool round_msg;
                round_msg.data = inside_roundabout;
                my_cav.pub_is_roundabout->publish(round_msg);
            }

            // 충돌 위험 요소가 사라졌다면 정지상태 -> 다시 주행 시작로 전환
            if (!is_in_danger_zone(my_cav)) {
                if (my_cav.is_stopped) {
                    RCLCPP_INFO(get_logger(), "%s[SAFE ZONE] %s -> [GO]%s",
                                ANSI_BLUE.c_str(), my_id.c_str(), ANSI_RESET.c_str());
                    std_msgs::msg::Bool msg; msg.data = false; my_cav.pub_stop->publish(msg);
                    my_cav.is_stopped = false;
                    my_cav.stop_cause.clear();
                    clear_conflict_status(my_id);
                } else {
                    std_msgs::msg::Bool msg; msg.data = false; my_cav.pub_stop->publish(msg);
                }
                continue;
            }


            bool my_cav_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";
            bool deadlock_active = false;
            std::string zone = is_in_threeway_zone(my_cav.pos) ? "threeway" :
                               (is_in_fourway_zone(my_cav) ? "4WAY" : "ROUND");


            // 다른 cav가 my_cav를 정지시켰다면 무조건 정지
            if (my_cav.forced_stop) {
                my_cav_stop = true; cause_id = my_cav.stop_cause; log_msg = "FORCED";
            }

            else {
                // is_conflict에서 정의한 충돌 방지 로직 실행 (my_cav-target 관계 기반임)
                if (is_conflict(my_cav, cause_id, log_msg, deadlock_active)) {
                    my_cav_stop = true;
                }
                // 각 교차로 별 세부 충돌 방지 로직 (my_cav-교차로 관계 기반임)
                else {
                    // 사지교차로 
                    if (zone == "4WAY") {
                        if (check_fourway_rect_split(my_cav)) {
                            my_cav_stop = true; cause_id = "4WAY_BLOCK"; log_msg = "4WAY_FULL";
                        }
                    }
                    // 회전교차로
                    else if (zone == "ROUND") {
                        if (check_roundabout(my_cav)) {
                            my_cav_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                            my_cav.has_entered_roundabout = false;
                        } 
                        // 진입 직전 회전할 차선 결정 (original or inside)
                        else {
                            double dist_sq = (my_cav.pos - round_center_).dist_Sq();
                            if (dist_sq <= round_app_r_sq_ * 1.1 &&
                                dist_sq > (round_radius_ * round_radius_) &&
                                !my_cav.has_entered_roundabout) {


                                bool go_inside = false;
                                if (my_cav.id == "CAV_01" &&
                                    vehicles_.count("CAV_04") &&
                                    vehicles_["CAV_04"].active &&
                                    (vehicles_["CAV_04"].pos - round_center_).dist_Sq() <= 2.5*2.5) {
                                    go_inside = true;
                                }
                                else if (my_cav.id == "CAV_02" &&
                                         vehicles_.count("CAV_03") &&
                                         vehicles_["CAV_03"].active &&
                                         (vehicles_["CAV_03"].pos - round_center_).dist_Sq() <= 2.5*2.5) {
                                    go_inside = true;
                                }
                                
                                std_msgs::msg::Bool way_msg;
                                way_msg.data = go_inside;
                                my_cav.pub_change_way->publish(way_msg);
                                RCLCPP_INFO(get_logger(), "[%s] [%s] -> PATH: %s",
                                            zone.c_str(), my_id.c_str(),
                                            go_inside ? "INSIDE" : "ORIGINAL");
                                my_cav.has_entered_roundabout = true;
                            }
                        }
                    } else {
                        my_cav.has_entered_roundabout = false;
                    }
                }
            }


            if (!my_cav_stop) {
                clear_conflict_status(my_id);
            }


            if (my_cav.is_stopped != my_cav_stop) {
                if (my_cav_stop) {
                    RCLCPP_INFO(get_logger(), "%s[%s] [%s] -> [STOP] %s%s",
                                ANSI_RED.c_str(), zone.c_str(), my_id.c_str(), log_msg.c_str(), ANSI_RESET.c_str());
                    update_conflict_status(my_id, cause_id);
                } else {
                    if (deadlock_active) {
                        RCLCPP_INFO(get_logger(), "%s[%s] [DEADLOCK RESOLVED] %s -> [GO]%s",
                                    ANSI_GREEN.c_str(), zone.c_str(), my_id.c_str(), ANSI_RESET.c_str());
                    } else {
                        RCLCPP_INFO(get_logger(), "%s[%s] [%s] -> [GO]%s",
                                    ANSI_BLUE.c_str(), zone.c_str(), my_id.c_str(), ANSI_RESET.c_str());
                    }
                }
            } else if (my_cav_stop) {
                update_conflict_status(my_id, cause_id);
            }


            my_cav.is_stopped = my_cav_stop;
            if (my_cav_stop && my_cav.stop_cause.empty()) {
                my_cav.stop_cause = cause_id;
            } else if (!my_cav_stop) {
                my_cav.stop_cause.clear();
            }


            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg; msg.data = my_cav_stop; my_cav.pub_stop->publish(msg);
            }
        }
    }

    // 사지교차로 진입 전, 내부 상황 파악 및 정지 판단하는 로직
    bool check_fourway_rect_split(const Vehicle& my_v) {
        double x_min = fourway_center_.x - fourway_box_half_len_;
        double x_max = fourway_center_.x + fourway_box_half_len_;
        double y_min = fourway_center_.y - fourway_box_half_len_;
        double y_max = fourway_center_.y + fourway_box_half_len_;


        bool i_am_inside = (my_v.pos.x >= x_min && my_v.pos.x <= x_max &&
                            my_v.pos.y >= y_min && my_v.pos.y <= y_max);
        if (i_am_inside) return false;
        if ((my_v.pos - fourway_center_).dist_Sq() > fourway_app_r_sq_) return false;
        if (!is_approaching(my_v, fourway_center_)) return false;


        bool am_i_top = (my_v.pos.y > fourway_center_.y);


        if (my_v.id == "CAV_01") {
            auto is_target_in_box = [&](const std::string& tid, bool check_top) {
                if (vehicles_.count(tid) && vehicles_.at(tid).active) {
                    const auto& tv = vehicles_.at(tid);
                    bool in_rect = (tv.pos.x >= x_min && tv.pos.x <= x_max &&
                                    tv.pos.y >= y_min && tv.pos.y <= y_max);
                    if (in_rect) {
                        bool tv_is_top = (tv.pos.y > fourway_center_.y);
                        return (tv_is_top == check_top);
                    }
                }
                return false;
            };


            if (am_i_top && is_target_in_box("CAV_03", true)) return true;
            if (!am_i_top && is_target_in_box("CAV_04", false)) return true;
        }


        int count_top = 0, count_bottom = 0;
        for (const auto& [id, v] : vehicles_) {
            if (!v.active) continue;
            if (v.pos.x >= x_min && v.pos.x <= x_max &&
                v.pos.y >= y_min && v.pos.y <= y_max) {
                if (v.pos.y > fourway_center_.y) count_top++;
                else count_bottom++;
            }
        }
        return am_i_top ? (count_top >= 2) : (count_bottom >= 2);
    }

    // 현재 회전교차로 내부에 진입해 있는 자율주행 차량(CAV)의 개수를 반환하는 함수
    int count_cavs_in_roundabout() {
        int count = 0;
        double r_sq = round_radius_ * round_radius_;
        for (const auto& [id, v] : vehicles_) {
            if (v.is_cav && v.active &&
                (v.pos - round_center_).dist_Sq() <= r_sq) count++;
        }
        return count;
    }

    // 회전교차로 진입 시 내부 차량 유무, 타 차량의 접근, 양보(Yield) 규칙 등을 종합하여 진입 가능 여부를 판단하는 함수
    bool check_roundabout(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        double r_sq = round_radius_ * round_radius_;
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_))
            return false;


        int cavs_inside = count_cavs_in_roundabout();
        if (cavs_inside >= 2) return true;


        auto is_blocked_by_hv_check = [&](const Vehicle& v) -> bool {
            double default_check_left = 2.1 / round_radius_;
            double default_check_right = 0.4 / round_radius_;
            if (v.is_stopped && v.stop_cause == "ROUND_YIELD") default_check_left += 0.5;
            double v_angle = std::atan2(v.pos.y - round_center_.y, v.pos.x - round_center_.x);


            for (const auto& [tid, target] : vehicles_) {
                if (tid == v.id || !target.active || target.is_cav) continue;
                if ((target.pos - round_center_).dist_Sq() <= r_sq){
                    double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
                    double diff = t_angle - v_angle;
                    while (diff > M_PI) diff -= 2.0 * M_PI;
                    while (diff < -M_PI) diff += 2.0 * M_PI;
                    if ((diff > 0 && diff < default_check_right) ||
                        (diff <= 0 && diff > -default_check_left)) return true;
                }
            }
            return false;
        };


        if (is_blocked_by_hv_check(my_cav)) return true;


        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.is_cav || !target.active) continue;
            bool is_pair = ((my_cav.id == "CAV_01" && tid == "CAV_04") ||
                            (my_cav.id == "CAV_04" && tid == "CAV_01") ||
                            (my_cav.id == "CAV_02" && tid == "CAV_03") ||
                            (my_cav.id == "CAV_03" && tid == "CAV_02"));
            double t_dist_sq = (target.pos - round_center_).dist_Sq();
            if (t_dist_sq > round_app_r_sq_ || t_dist_sq <= r_sq || !is_approaching(target, round_center_)) continue;
            if (is_blocked_by_hv_check(target)) continue;
            if (is_pair) continue;
            if (t_dist_sq < dist_sq - 0.01) return true;
            if (std::abs(t_dist_sq - dist_sq) <= 0.01 && tid < my_cav.id) return true;
        }
        return false;
    }
};

// ROS2 노드를 초기화하고 MainTrafficController를 실행하는 프로그램 진입점
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
