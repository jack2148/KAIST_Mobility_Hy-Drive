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
#include <iomanip>
#include <sstream>

using std::placeholders::_1;

// 색상 정의
const std::string ANSI_RESET  = "\033[0m";
const std::string ANSI_GREEN  = "\033[32m";
const std::string ANSI_YELLOW = "\033[33m";
const std::string ANSI_BLUE   = "\033[34m";
const std::string ANSI_RED    = "\033[31m";

namespace Geo {
    struct Vec2 {
        double x, y;
        Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
        Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
        double dot(const Vec2& o) const { return x * o.x + y * o.y; }
        double magSq() const { return x*x + y*y; }
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
    double yaw = 0.0;
    bool active = false;
    
    bool is_stopped = false;
    std::string stop_cause = ""; 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
};

class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        qos_.best_effort();
        qos_.durability_volatile();
        qos_.keep_last(1);

        car_dims_ = {0.17, 0.16, 0.075, 0.075}; 
        // 정지 기준 거리 1.0m
        conflict_range_ = 2.0; 

        fourway_center_ = {-2.333, 0.0};
        fourway_r_sq_ = 0.8 * 0.8;       
        fourway_app_r_sq_ = 1.5 * 1.5;

        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 2.0 * 2.0;
        round_radius_ = 1.025;

        tmr_discovery_ = create_wall_timer(std::chrono::seconds(1), 
            std::bind(&MainTrafficController::discover_vehicles, this));
        tmr_control_ = create_wall_timer(std::chrono::milliseconds(50), 
            std::bind(&MainTrafficController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Controller Started. Safety Margin: %.2fm", conflict_range_);
    }

private:
    rclcpp::QoS qos_{1};
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::unordered_set<std::string> fourway_inside_;

    std::vector<double> car_dims_; 
    double conflict_range_;
    
    Geo::Vec2 fourway_center_;
    double fourway_r_sq_, fourway_app_r_sq_;
    
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;

    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double margin = 0.0) {
        double f = car_dims_[0] + margin;
        double r = car_dims_[1]; 
        double l = car_dims_[2];
        double rt = car_dims_[3];

        std::vector<Geo::Vec2> locals = {{f, l}, {f, -rt}, {-r, -rt}, {-r, l}};
        std::vector<Geo::Vec2> world_corners;
        world_corners.reserve(4);

        double c = std::cos(v.yaw), s = std::sin(v.yaw);
        for(const auto& p : locals) {
            world_corners.push_back({
                v.pos.x + (p.x * c - p.y * s),
                v.pos.y + (p.x * s + p.y * c)
            });
        }
        return world_corners;
    }

    void discover_vehicles() {
        auto topic_map = this->get_topic_names_and_types();
        
        for (const auto& [name, types] : topic_map) {
            bool is_pose = false;
            for(const auto& t : types) if(t.find("PoseStamped") != std::string::npos) is_pose = true;
            if(!is_pose) continue;

            if (vehicles_.count(name)) continue; 
            
            std::string id = "";
            bool is_cav = false;
            
            size_t pos_cav = name.find("CAV_");
            size_t pos_hv = name.find("HV_");

            if (pos_cav != std::string::npos) {
                if (pos_cav + 6 <= name.size()) {
                    std::string num_str = name.substr(pos_cav + 4, 2);
                    try {
                        int num = std::stoi(num_str);
                        if (num >= 1 && num <= 4) {
                            id = "CAV_" + num_str; 
                            is_cav = true;
                        }
                    } catch (...) {}
                }
            } else if (pos_hv != std::string::npos) {
                if (pos_hv + 5 <= name.size()) {
                    std::string num_str = name.substr(pos_hv + 3, 2);
                    try {
                        int num = std::stoi(num_str);
                        if (num >= 19 && num <= 36) {
                            id = "HV_" + num_str;
                            is_cav = false;
                        }
                    } catch (...) {}
                }
            }

            if (id.empty()) continue;
            if (vehicles_.count(id)) continue; 

            register_vehicle(id, name, is_cav);
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v;
        v.id = id;
        v.is_cav = is_cav;

        auto cb = [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            if (vehicles_.count(id)) {
                auto& veh = vehicles_[id];
                veh.pos = {msg->pose.position.x, msg->pose.position.y};
                veh.yaw = msg->pose.orientation.z;
                veh.active = true;
            }
        };
        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(topic, qos_, cb);

        if (is_cav) {
            std::string stop_topic = "/" + id + "/cmd_stop";
            v.pub_stop = create_publisher<std_msgs::msg::Bool>(stop_topic, qos_);
        }
        
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s", id.c_str());
    }

    void control_loop() {
        if (vehicles_.empty()) return;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav) continue;
            if (!my_cav.active) continue;

            bool should_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";

            auto my_box = get_vehicle_corners(my_cav, conflict_range_);

            // 1. 차량 간 충돌 검사
            for (const auto& [tid, target] : vehicles_) {
                if (tid == my_id || !target.active) continue;
                
                double dist_sq = (my_cav.pos - target.pos).magSq();
                if (dist_sq > 25.0) continue;

                auto target_box = get_vehicle_corners(target);
                
                if (Geo::check_obb_intersection(my_box, target_box)) {
                    if (target.is_cav && !target.stop_cause.empty()) {
                        if (target.stop_cause == my_id) continue;
                    }
                    should_stop = true;
                    cause_id = tid;
                    
                    // 거리 로그 추가
                    std::stringstream ss;
                    ss << "CONFLICT with " << tid << " (Dist: " 
                       << std::fixed << std::setprecision(2) << std::sqrt(dist_sq) << "m)";
                    log_msg = ss.str();
                    break; 
                }
            }

            // 2. 교차로 로직
            if (!should_stop) {
                if (check_fourway(my_cav)) {
                    should_stop = true;
                    cause_id = "4WAY_FULL";
                    log_msg = "4WAY_FULL";
                } else if (check_roundabout(my_cav)) {
                    should_stop = true;
                    cause_id = "ROUND_YIELD";
                    log_msg = "ROUND_YIELD";
                }
            }

            // 3. 상태 업데이트 및 로깅
            if (my_cav.is_stopped != should_stop) {
                if (should_stop) {
                    if (log_msg.find("CONFLICT") != std::string::npos)
                        RCLCPP_INFO(get_logger(), "%s[%s] STOP: %s%s", ANSI_RED.c_str(), my_id.c_str(), log_msg.c_str(), ANSI_RESET.c_str());
                    else
                        RCLCPP_INFO(get_logger(), "%s[%s] STOP: %s%s", ANSI_GREEN.c_str(), my_id.c_str(), log_msg.c_str(), ANSI_RESET.c_str());
                } else {
                    RCLCPP_INFO(get_logger(), "%s[%s] GO: Path Clear%s", ANSI_BLUE.c_str(), my_id.c_str(), ANSI_RESET.c_str());
                }
            }

            my_cav.is_stopped = should_stop;
            if (should_stop) my_cav.stop_cause = cause_id;
            else my_cav.stop_cause.clear();

            // 4. 토픽 발행
            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg;
                msg.data = should_stop;
                my_cav.pub_stop->publish(msg);
            }
        }
    }

    bool check_fourway(const Vehicle& v) {
        Geo::Vec2 diff = v.pos - fourway_center_;
        double dist_sq = diff.magSq();

        if (dist_sq <= fourway_r_sq_) fourway_inside_.insert(v.id);
        else fourway_inside_.erase(v.id);

        if (dist_sq <= fourway_app_r_sq_) {
            double rel_x = (-diff.x) * std::cos(v.yaw) + (-diff.y) * std::sin(v.yaw);
            if (rel_x > 0) {
                if (fourway_inside_.count(v.id)) return false; 
                if (fourway_inside_.size() >= 2) return true;  
            }
        }
        return false;
    }

    bool check_roundabout(const Vehicle& my_cav) {
        // 1. 접근 구역(Approach Zone) 밖이면 검사 불필요
        if ((my_cav.pos - round_center_).magSq() > round_app_r_sq_) return false;

        int vehicles_inside_count = 0;
        bool hv_in_sector = false;

        // 부채꼴(Sector) 계산용 변수
        double half_angle = (1.0 / round_radius_) / 2.0;
        double angle_to_me = std::atan2(my_cav.pos.y - round_center_.y, my_cav.pos.x - round_center_.x);

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue;

            // Target의 회전교차로 중심으로부터의 거리
            double dist_sq = (target.pos - round_center_).magSq();
            double radius_sq = round_radius_ * round_radius_;

            // A. 회전교차로 내부(반경 이내) 차량 카운트
            if (dist_sq <= radius_sq) {
                vehicles_inside_count++;
            }

            // B. 진입 부채꼴 구역(Sector) 검사 (이미 내부 카운트된 차일 수도 있음)
            // Target이 부채꼴 반지름(교차로 반지름) 안에 있어야 함
            if (dist_sq <= radius_sq) {
                double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
                double angle_diff = std::abs(t_angle - angle_to_me);
                while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;

                // 부채꼴 각도 내에 있는 경우
                if (std::abs(angle_diff) <= half_angle) {
                    // HV일 때만 정지 플래그 설정
                    if (!target.is_cav) {
                        hv_in_sector = true;
                    }
                }
            }
        }

        // 조건 1: 교차로 내부에 3대 이상 있으면 무조건 정지 (혼잡 제어)
        if (vehicles_inside_count >= 3) return true;

        // 조건 2: 진입 부채꼴 구역에 HV가 있으면 정지 (HV 양보)
        if (hv_in_sector) return true;

        return false; // 그 외(CAV만 있거나 비어있음) 진입 허용
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
