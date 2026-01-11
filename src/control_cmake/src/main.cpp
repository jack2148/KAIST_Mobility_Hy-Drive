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
#include <map>

// 로그 색상 정의
const std::string ANSI_RESET   = "\033[0m";
const std::string ANSI_BLUE    = "\033[34m";
const std::string ANSI_RED     = "\033[31m";
const std::string ANSI_CYAN    = "\033[36m";
const std::string ANSI_YELLOW  = "\033[33m";
const std::string ANSI_GREEN   = "\033[32m";

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
        car_dims_ = {0.17, 0.16, 0.075, 0.075};
        front_padding_ = 0.6;
        side_padding_ = 0.4;
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

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::map<std::string, std::string> conflict_info_; // Key: 멈춘 차, Value: 원인 차

    std::vector<double> car_dims_;
    double front_padding_, side_padding_, approach_range_sq_;
    Geo::Vec2 fourway_center_;
    double fourway_len_, fourway_app_r_sq_, fourway_box_half_len_;
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;
    double t3_box_x_min_, t3_box_x_max_, t3_1_y_min_, t3_1_y_max_, t3_2_y_min_, t3_2_y_max_;

    void update_conflict_status(const std::string& stopped_cav, const std::string& cause_vehicle) {
        if (conflict_info_.count(stopped_cav) && conflict_info_[stopped_cav] == cause_vehicle) return;
        conflict_info_[stopped_cav] = cause_vehicle;
        RCLCPP_INFO(get_logger(), "%s[CONFLICT START] %s stopped by %s%s",
                    ANSI_YELLOW.c_str(), stopped_cav.c_str(), cause_vehicle.c_str(), ANSI_RESET.c_str());
    }

    void clear_conflict_status(const std::string& stopped_cav) {
        if (conflict_info_.count(stopped_cav)) {
            RCLCPP_INFO(get_logger(), "%s[CONFLICT END] %s resumed (was stopped by %s)%s",
                        ANSI_CYAN.c_str(), stopped_cav.c_str(), conflict_info_[stopped_cav].c_str(), ANSI_RESET.c_str());
            conflict_info_.erase(stopped_cav);
        }
    }

    bool is_approaching(const Vehicle& spot, const Geo::Vec2& target_pos) {
        Geo::Vec2 vec = target_pos - spot.pos;
        return vec.dot({std::cos(spot.z), std::sin(spot.z)}) > 0;
    }

    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double front_ext, double side_pad) {
        double f = car_dims_[0] + front_ext;
        double r = car_dims_[1];
        double l = car_dims_[2] + side_pad;
        double rt = car_dims_[3] + side_pad;
        std::vector<Geo::Vec2> locals = {{f, l}, {f, -rt}, {-r, -rt}, {-r, l}};
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

    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v; v.id = id; v.is_cav = is_cav;
        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (vehicles_.count(id)) {
                    vehicles_[id].pos = {msg->pose.position.x, msg->pose.position.y};
                    vehicles_[id].z = msg->pose.orientation.z;
                    vehicles_[id].active = true;
                }
            }
        );
        if (is_cav) {
            v.pub_stop = create_publisher<std_msgs::msg::Bool>("/" + id + "/cmd_stop", rclcpp::SensorDataQoS());
            v.pub_change_way = create_publisher<std_msgs::msg::Bool>("/" + id + "/change_waypoint", rclcpp::SensorDataQoS());
        }
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s", id.c_str());
    }

    bool is_in_t3_zone(const Geo::Vec2& pos) {
        return (pos.x >= t3_box_x_min_ && pos.x <= t3_box_x_max_) &&
               ((pos.y >= t3_1_y_min_ && pos.y <= t3_1_y_max_) ||
                (pos.y >= t3_2_y_min_ && pos.y <= t3_2_y_max_));
    }
    bool is_in_round_zone(const Vehicle& v){ return (v.pos - round_center_).dist_Sq() <= round_app_r_sq_; }
    bool is_in_fourway_zone(const Vehicle& v){ return (v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_; }
    bool is_in_danger_zone(const Vehicle& v) {
        if ((v.pos - round_center_).dist_Sq() <= round_app_r_sq_) return true;
        if ((v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_) return true;
        if (is_in_t3_zone(v.pos)) return true;
        return false;
    }

    bool is_parallel(const Vehicle& my_cav, const Vehicle& target){
        double dz = std::abs(std::abs(my_cav.z - target.z));
        while (dz > M_PI) dz -= 2.0 * M_PI;
        dz = std::abs(dz);
        return (dz < 0.40) || (dz > (M_PI - 0.40));
    }

    // 데드락 해결 로직 포함된 충돌 감지
    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, std::string& log_msg, bool& deadlock_override) {
        double dist_to_round = (my_cav.pos - round_center_).dist_Sq();
        if (dist_to_round <= (round_radius_* round_radius_) && !is_approaching(my_cav, round_center_)) return false;

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue;
            if ((my_cav.pos - target.pos).dist_Sq() > approach_range_sq_) continue;

            // 1. 순수 기하학적 충돌 감지
            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto side_box   = get_vehicle_corners(my_cav, 0.0, side_padding_);
            auto full_box   = get_vehicle_corners(my_cav, front_padding_, side_padding_);
            auto front_box  = get_vehicle_corners(my_cav, front_padding_, 0.0);
            bool is_full  = Geo::check_obb_intersection(full_box, target_box);
            bool is_front = Geo::check_obb_intersection(front_box, target_box);

            // is_next_lane 계산
            double vehicle_width = car_dims_[2] + car_dims_[3];
            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            double side_dist = std::abs(-std::sin(my_cav.z) * dx + std::cos(my_cav.z) * dy);
            bool is_next_lane = is_parallel(my_cav, target) && (side_dist > vehicle_width * 1.2);

            bool collision_detected = false;
            std::string temp_log = "";

            if (is_in_t3_zone(my_cav.pos)) {
                if (tid == "CAV_03" || tid == "CAV_04") {
                    if (my_cav.id == "CAV_01" && is_next_lane) {
                        collision_detected = true; temp_log = "T3_NEXT_LANE";
                    } else if (my_cav.id == "CAV_02" && is_full && !is_next_lane) {
                        collision_detected = true; temp_log = "T3_FULL";
                    }
                }
            }
            else if (is_in_fourway_zone(my_cav)) {
                // !is_parallel -> !is_next_lane 변경
                if (is_front && !is_next_lane) {
                    collision_detected = true;
                    temp_log = "4WAY_FRONT";
                }
            }

            if (collision_detected) {
                // 데드락 해소 로직
                if (conflict_info_.count(tid) && conflict_info_[tid] == my_cav.id) {
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

    void control_loop() {
        if (vehicles_.empty()) return;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            // 1. 위험 지역 밖 처리
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

            // --- 위험 지역 내부 로직 ---
            bool my_cav_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";
            bool deadlock_active = false;
            std::string zone = is_in_t3_zone(my_cav.pos) ? "T3" :
                               (is_in_fourway_zone(my_cav) ? "4WAY" : "ROUND");

            if (my_cav.forced_stop) {
                my_cav_stop = true; cause_id = my_cav.stop_cause; log_msg = "FORCED";
            }
            else {
                // 1. 충돌 감지
                if (is_conflict(my_cav, cause_id, log_msg, deadlock_active)) {
                    my_cav_stop = true;
                }
                else {
                    // 2. 구역별 로직 체크
                    if (zone == "4WAY") {
                        if (check_fourway_rect_split(my_cav)) {
                            my_cav_stop = true; cause_id = "4WAY_BLOCK"; log_msg = "4WAY_FULL";
                        }
                    }
                    else if (zone == "ROUND") {
                        if (check_roundabout(my_cav)) {
                            my_cav_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                            my_cav.has_entered_roundabout = false;
                        } else {
                            double dist_sq = (my_cav.pos - round_center_).dist_Sq();
                            if (dist_sq <= round_app_r_sq_ * 1.1 &&
                                dist_sq > (round_radius_ * round_radius_) &&
                                !my_cav.has_entered_roundabout) {

                                // --- 기본 inside 기준 제거: inside_cavs 사용 안 함 ---
                                bool go_inside = false;  // 기본은 original

                                // --- 페어별 예외 규칙만 사용 ---
                                if (my_cav.id == "CAV_01" &&
                                    vehicles_.count("CAV_04") &&
                                    vehicles_["CAV_04"].active &&
                                    (vehicles_["CAV_04"].pos - round_center_).dist_Sq() <= round_app_r_sq_) {
                                    go_inside = true;
                                }
                                else if (my_cav.id == "CAV_02" &&
                                         vehicles_.count("CAV_03") &&
                                         vehicles_["CAV_03"].active &&
                                         (vehicles_["CAV_03"].pos - round_center_).dist_Sq() <= round_app_r_sq_) {
                                    go_inside = true;
                                }
                                else if (my_cav.id == "CAV_04" &&
                                         vehicles_.count("CAV_01") &&
                                         vehicles_["CAV_01"].active &&
                                         (vehicles_["CAV_01"].pos - round_center_).dist_Sq() <= round_app_r_sq_) {
                                    go_inside = false; // 명시적으로 original
                                }
                                else if (my_cav.id == "CAV_03" &&
                                         vehicles_.count("CAV_02") &&
                                         vehicles_["CAV_02"].active &&
                                         (vehicles_["CAV_02"].pos - round_center_).dist_Sq() <= round_app_r_sq_) {
                                    go_inside = false; // 명시적으로 original
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

            // 상태 변경 로그 출력
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
                // 계속 정지 상태면 정보 갱신
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

    int count_cavs_in_roundabout() {
        int count = 0;
        double r_sq = round_radius_ * round_radius_;
        for (const auto& [id, v] : vehicles_) {
            if (v.is_cav && v.active &&
                (v.pos - round_center_).dist_Sq() <= r_sq) count++;
        }
        return count;
    }

    bool check_roundabout(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        double r_sq = round_radius_ * round_radius_;
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_))
            return false;

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

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
