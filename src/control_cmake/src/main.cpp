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

using std::placeholders::_1;

// 색상 정의
const std::string ANSI_RESET   = "\033[0m";
const std::string ANSI_GREEN   = "\033[32m";
const std::string ANSI_YELLOW  = "\033[33m";
const std::string ANSI_BLUE    = "\033[34m";
const std::string ANSI_RED     = "\033[31m";
const std::string ANSI_MAGENTA = "\033[35m";

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
    bool forced_stop = false; 
    std::string stop_cause = "";

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
};

class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        car_dims_ = {0.17, 0.16, 0.075, 0.075}; 

        front_padding_ = 0.4;
        side_padding_ = 0.4;
        approach_range_sq_ = 2.0 * 2.0; 

        // 1. 사지 교차로 설정
        fourway_center_ = {-2.333, 0.0};
        fourway_app_r_sq_ = 1.5 * 1.5; 
        fourway_box_half_len_ = 1.0; 

        // 2. 회전 교차로 설정
        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4; 

        // 3 & 4. 삼지 교차로 영역 설정
        t3_box_x_min_ = -3.5; t3_box_x_max_ = -1.3;
        t3_1_y_min_ = 1.6;  t3_1_y_max_ = 2.7;
        t3_2_y_min_ = -2.7; t3_2_y_max_ = -1.9;

        tmr_discovery_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this));

        tmr_control_ = create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&MainTrafficController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Controller Started: Geometric Priority + T3 ID Priority.");
    }

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;

    std::vector<double> car_dims_;
    double front_padding_;
    double side_padding_;
    double approach_range_sq_;

    Geo::Vec2 fourway_center_;
    double fourway_app_r_sq_;
    double fourway_box_half_len_; 

    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;

    double t3_box_x_min_, t3_box_x_max_;
    double t3_1_y_min_, t3_1_y_max_;
    double t3_2_y_min_, t3_2_y_max_;

    bool is_approaching(const Vehicle& v, const Geo::Vec2& target_pos) {
        Geo::Vec2 to = target_pos - v.pos;
        return to.dot({std::cos(v.yaw), std::sin(v.yaw)}) > 0;
    }

    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double front_ext, double side_pad) {
        double f = car_dims_[0] + front_ext;
        double r = car_dims_[1];
        double l = car_dims_[2] + side_pad;
        double rt = car_dims_[3] + side_pad;

        std::vector<Geo::Vec2> locals = {{f, l}, {f, -rt}, {-r, -rt}, {-r, l}};
        std::vector<Geo::Vec2> world_corners;
        world_corners.reserve(4);

        double c = std::cos(v.yaw), s = std::sin(v.yaw);
        for (const auto& p : locals) {
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
            for (const auto& t : types) if (t.find("PoseStamped") != std::string::npos) is_pose = true;
            if (!is_pose || vehicles_.count(name)) continue;

            std::string id = "";
            bool is_cav = false;

            size_t pos_cav = name.find("CAV_");
            size_t pos_hv = name.find("HV_");

            if (pos_cav != std::string::npos && pos_cav + 6 <= name.size()) {
                std::string num_str = name.substr(pos_cav + 4, 2);
                try {
                    int num = std::stoi(num_str);
                    if (num >= 1 && num <= 4) { id = "CAV_" + num_str; is_cav = true; }
                } catch (...) {}
            } else if (pos_hv != std::string::npos && pos_hv + 5 <= name.size()) {
                std::string num_str = name.substr(pos_hv + 3, 2);
                try {
                    int num = std::stoi(num_str);
                    if (num >= 19 && num <= 36) { id = "HV_" + num_str; is_cav = false; }
                } catch (...) {}
            }

            if (!id.empty() && !vehicles_.count(id)) register_vehicle(id, name, is_cav);
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v;
        v.id = id; v.is_cav = is_cav;
        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (vehicles_.count(id)) {
                    vehicles_[id].pos = {msg->pose.position.x, msg->pose.position.y};
                    vehicles_[id].yaw = msg->pose.orientation.z;
                    vehicles_[id].active = true;
                }
            });
        if (is_cav) v.pub_stop = create_publisher<std_msgs::msg::Bool>("/" + id + "/cmd_stop", rclcpp::SensorDataQoS());
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s", id.c_str());
    }

    // [T3 판단 헬퍼]
    bool is_in_t3_zone(const Geo::Vec2& pos) {
        bool in_x = (pos.x >= t3_box_x_min_ && pos.x <= t3_box_x_max_);
        bool in_y1 = (pos.y >= t3_1_y_min_ && pos.y <= t3_1_y_max_);
        bool in_y2 = (pos.y >= t3_2_y_min_ && pos.y <= t3_2_y_max_);
        return in_x && (in_y1 || in_y2);
    }

    bool is_in_intersection_zone(const Vehicle& v) {
        if ((v.pos - round_center_).magSq() <= round_app_r_sq_) return true;
        if ((v.pos - fourway_center_).magSq() <= fourway_app_r_sq_) return true;
        if (is_in_t3_zone(v.pos)) return true; // T3 체크 재활용
        return false;
    }

    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, std::string& log_msg) {
        // [라운드어바웃 진입 예외 처리]
        double dist_to_round = (my_cav.pos - round_center_).magSq();
        if (dist_to_round <= round_app_r_sq_) {
            if (!is_approaching(my_cav, round_center_)) {
                return false; 
            }
        }
        
        const double PARALLEL_ANGLE_THRESHOLD = 0.40; 
        double round_protect_sq = (round_radius_) * (round_radius_);
        double vehicle_width = car_dims_[2] + car_dims_[3]; 

        // 내가 T3 구역에 있는지 확인
        bool am_i_in_t3 = is_in_t3_zone(my_cav.pos);

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue;
            
            double dist_sq = (my_cav.pos - target.pos).magSq();
            if (dist_sq > approach_range_sq_) continue;

            // 상대도 T3에 있는지?
            bool target_in_t3 = is_in_t3_zone(target.pos);
            bool t3_context = am_i_in_t3 && target_in_t3; // 둘 다 T3 구역 내

            // 나란히 달리는 차량 체크
            double yaw_diff = std::abs(my_cav.yaw - target.yaw);
            while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
            yaw_diff = std::abs(yaw_diff);
            bool is_angle_parallel = (yaw_diff < PARALLEL_ANGLE_THRESHOLD) || 
                                     (yaw_diff > (M_PI - PARALLEL_ANGLE_THRESHOLD));

            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            double lateral_dist = std::abs(-std::sin(my_cav.yaw) * dx + std::cos(my_cav.yaw) * dy);

            bool is_next_lane = is_angle_parallel && (lateral_dist > vehicle_width * 1.2); 
            
            // [수정 사항] T3 구역이 아닐 때만 '옆 차선 무시' 로직 동작
            // T3 구역에서는 옆 차선이라도 side padding이 닿으면 충돌 로직으로 넘겨야 함
            if (!t3_context && is_next_lane) continue; 

            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto front_box  = get_vehicle_corners(my_cav, front_padding_, 0.0);
            auto full_box   = get_vehicle_corners(my_cav, front_padding_, side_padding_);

            bool is_front = Geo::check_obb_intersection(front_box, target_box);
            bool is_side = false;
            if (!is_front) is_side = Geo::check_obb_intersection(full_box, target_box);

            if (is_front || is_side) {
                // [수정 사항] T3 구역 전용 로직: ID 기반 양보 (큰 ID가 양보)
                if (t3_context) {
                    if (my_cav.id > target.id) {
                        conflict_id = tid;
                        log_msg = "T3 SIDE(ID) YIELD -> " + tid;
                        return true; // STOP (내가 ID가 더 크므로)
                    } else {
                        // 내 ID가 더 작으므로 나는 우선권 가짐 -> 무시하고 진행
                        continue; 
                    }
                }

                // --- 아래는 T3가 아닌 다른 구역(일반, 사지교차로 등)의 기존 로직 ---

                // 1. 내 좌표계에서 타겟의 X (내 앞인가?)
                double rel_x_target = (target.pos.x - my_cav.pos.x) * std::cos(my_cav.yaw) + 
                                      (target.pos.y - my_cav.pos.y) * std::sin(my_cav.yaw);
                
                // 2. 타겟 좌표계에서 나의 X (내가 타겟의 앞인가?)
                double rel_x_me = (my_cav.pos.x - target.pos.x) * std::cos(target.yaw) + 
                                  (my_cav.pos.y - target.pos.y) * std::sin(target.yaw);

                // 판단 A: 상대방이 내 앞에 있고, 나는 상대방의 뒤/옆에 있다 -> 양보 (STOP)
                if (rel_x_target > 0.0 && rel_x_me <= 0.0) {
                    conflict_id = tid;
                    log_msg = "BLOCKED BY " + tid;
                    return true;
                }

                // 판단 B: 나는 상대방의 앞에 있고, 상대방은 내 뒤/옆에 있다 -> 무시 (GO)
                if (rel_x_target <= 0.0 && rel_x_me > 0.0) {
                    continue; 
                }
            }

            // [기존 Side 로직] (T3가 아닌 경우에만 도달하거나, 위에서 걸러지지 않은 경우)
            // T3인 경우는 위에서 continue하거나 return true로 처리됨
            if (is_side && !t3_context) {
                // 나란히 달리는 상황(차선 변경 등)이 아니라면, 교차로 내에서는 무조건 위험 상황임!
                if (!is_angle_parallel) { 
                    conflict_id = tid;
                    log_msg = "SIDE COLLISION RISK -> STOP";
                    return true; // [중요] 여기서 멈춰야 함!
                }
                double t_dist_round = (target.pos - round_center_).magSq();
                if ((t_dist_round <= round_protect_sq)) continue;
                
                if (target.is_cav) {
                    vehicles_[tid].forced_stop = true;
                    vehicles_[tid].stop_cause = "FORCED_BY_" + my_cav.id;
                }
                log_msg = "SIDE CONTACT -> I GO";
            }
        }
        return false;
    }

    void control_loop() {
        if (vehicles_.empty()) return;
        for (auto& [id, v] : vehicles_) v.forced_stop = false;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            bool should_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";

            if (is_in_intersection_zone(my_cav)) {
                if (is_conflict(my_cav, cause_id, log_msg)) {
                    should_stop = true;
                    if (vehicles_.count(cause_id) && vehicles_[cause_id].is_cav && vehicles_[cause_id].stop_cause == my_id) {
                        // Deadlock 상태에서의 처리 (둘 다 멈춤)
                    }
                }

                if (!should_stop && my_cav.forced_stop) {
                    should_stop = true; cause_id = my_cav.stop_cause; log_msg = "YIELDING -> " + cause_id;
                }

                if (!should_stop) {
                    if (check_fourway_rect_split(my_cav)) {
                        should_stop = true; cause_id = "4WAY_BLOCK"; log_msg = "4WAY_RECT_FULL";
                    } else if (check_roundabout(my_cav)) {
                        should_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                    }
                }
            }

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

        bool i_am_inside = (my_v.pos.x >= x_min && my_v.pos.x <= x_max &&
                            my_v.pos.y >= y_min && my_v.pos.y <= y_max);
        if (i_am_inside) return false;

        if ((my_v.pos - fourway_center_).magSq() > fourway_app_r_sq_) return false;
        if (!is_approaching(my_v, fourway_center_)) return false;

        bool am_i_top = (my_v.pos.y > fourway_center_.y);
        int count_top = 0;
        int count_bottom = 0;

        for (const auto& [id, v] : vehicles_) {
            if (!v.active) continue; 
            if (v.pos.x >= x_min && v.pos.x <= x_max &&
                v.pos.y >= y_min && v.pos.y <= y_max) {
                if (v.pos.y > fourway_center_.y) count_top++;
                else count_bottom++;
            }
        }

        if (am_i_top) {
            if (count_top >= 1) return true; 
        } else {
            if (count_bottom >= 1) return true; 
        }
        return false; 
    }

    bool check_roundabout(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).magSq();
        double r_sq = round_radius_ * round_radius_;
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_)) return false;

        for (const auto& [tid, target] : vehicles_) {
            if (target.is_cav && target.active && (target.pos - round_center_).magSq() <= r_sq) return true;
        }

        auto is_blocked_by_hv_check = [&](const Vehicle& v) -> bool {
            double default_check_left = 2.1 / round_radius_;
            double default_check_right = 0.2 / round_radius_;

            if (v.is_stopped && v.stop_cause == "ROUND_YIELD") {
                default_check_left += 0.5; 
            }

            double v_angle = std::atan2(v.pos.y - round_center_.y, v.pos.x - round_center_.x); 

            for (const auto& [tid, target] : vehicles_) {
                if (tid == v.id || !target.active || target.is_cav) continue;
                
                if ((target.pos - round_center_).magSq() <= round_radius_ * round_radius_){
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
            double t_dist_sq = (target.pos - round_center_).magSq();
            if (t_dist_sq > round_app_r_sq_ || t_dist_sq <= r_sq || !is_approaching(target, round_center_)) continue;
            if (is_blocked_by_hv_check(target)) continue;
            if (t_dist_sq < dist_sq - 0.01) return true;
            if (std::abs(t_dist_sq - dist_sq) <= 0.01) {
                if (tid < my_cav.id) return true; 
            }
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