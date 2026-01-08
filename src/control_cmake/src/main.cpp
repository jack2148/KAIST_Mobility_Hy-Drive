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

        conflict_range_ = 0.5;
        lateral_padding_ = 0.5;

        approach_range_ = 2.0;
        approach_range_sq_ = approach_range_ * approach_range_;

        fourway_center_ = {-2.333, 0.0};
        fourway_r_sq_ = 0.8 * 0.8;
        fourway_app_r_sq_ = 1.5 * 1.5;

        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.3;

        tmr_discovery_ = create_wall_timer(std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this));

        tmr_control_ = create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&MainTrafficController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Controller Started.");
    }

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::unordered_set<std::string> fourway_inside_;

    std::vector<double> car_dims_;
    double conflict_range_;
    double lateral_padding_;

    double approach_range_;
    double approach_range_sq_;

    Geo::Vec2 fourway_center_;
    double fourway_r_sq_, fourway_app_r_sq_;

    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;

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

    void control_loop() {
        if (vehicles_.empty()) return;
        const double PARALLEL_THRESHOLD = 0.35; 

        for (auto& [id, v] : vehicles_) v.forced_stop = false;

        double round_protect_sq = (round_radius_ + 0.2) * (round_radius_ + 0.2);

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            bool should_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";

            // 1. 차량 간 충돌 감지
            for (const auto& [tid, target] : vehicles_) {
                if (tid == my_id || !target.active) continue;

                double dist_sq = (my_cav.pos - target.pos).magSq();
                if (dist_sq > approach_range_sq_) continue;

                double yaw_diff = std::abs(my_cav.yaw - target.yaw);
                while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
                yaw_diff = std::abs(yaw_diff);
                bool is_parallel = (yaw_diff < PARALLEL_THRESHOLD) || (yaw_diff > (M_PI - PARALLEL_THRESHOLD));

                auto target_box = get_vehicle_corners(target, 0.0, 0.0);
                auto front_box  = get_vehicle_corners(my_cav, conflict_range_, 0.0);
                
                bool is_front = Geo::check_obb_intersection(front_box, target_box);
                bool is_side = false;
                if (!is_front) {
                    auto full_box = get_vehicle_corners(my_cav, conflict_range_, lateral_padding_);
                    is_side = Geo::check_obb_intersection(full_box, target_box);
                }

                double t_dist_round = (target.pos - round_center_).magSq();
                bool target_in_round = (t_dist_round <= round_protect_sq);

                if (is_front) {
                    should_stop = true; log_msg = "FRONT CONFLICT with " + tid;
                } else if (is_side) {
                    if (!is_parallel) {
                        if (target_in_round) continue; 
                        else {
                            if (target.is_cav) {
                                vehicles_[tid].forced_stop = true;
                                vehicles_[tid].stop_cause = "FORCED_BY_" + my_id;
                            }
                            log_msg = "SIDE CONTACT -> I GO";
                        }
                    }
                }
                if (should_stop) {
                    if (target.is_cav && target.stop_cause == my_id) { should_stop = false; continue; }
                    cause_id = tid; break;
                }
            }

            if (!should_stop && my_cav.forced_stop) {
                should_stop = true; cause_id = my_cav.stop_cause; log_msg = "YIELDING";
            }

            // 2. 교차로 로직
            if (!should_stop) {
                if (check_fourway(my_cav)) {
                    should_stop = true; cause_id = "4WAY_FULL"; log_msg = "4WAY_FULL";
                } else if (check_roundabout(my_cav)) {
                    should_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                }
            }

            if (my_cav.is_stopped != should_stop) {
                if (should_stop) RCLCPP_INFO(get_logger(), "%s[%s] STOP: %s%s", ANSI_RED.c_str(), my_id.c_str(), log_msg.c_str(), ANSI_RESET.c_str());
                else RCLCPP_INFO(get_logger(), "%s[%s] GO: Path Clear%s", ANSI_BLUE.c_str(), my_id.c_str(), ANSI_RESET.c_str());
            }

            my_cav.is_stopped = should_stop;
            if (should_stop && my_cav.stop_cause.empty()) my_cav.stop_cause = cause_id;
            else if (!should_stop) my_cav.stop_cause.clear();

            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg; msg.data = should_stop; my_cav.pub_stop->publish(msg);
            }
        }
    }

    bool is_approaching(const Vehicle& v, const Geo::Vec2& target_pos) {
        Geo::Vec2 to = target_pos - v.pos;
        return to.dot({std::cos(v.yaw), std::sin(v.yaw)}) > 0;
    }

    bool check_fourway(const Vehicle& v) {
        double dist_sq = (v.pos - fourway_center_).magSq();
        if (dist_sq > fourway_app_r_sq_) { fourway_inside_.erase(v.id); return false; }
        if (dist_sq <= fourway_r_sq_) fourway_inside_.insert(v.id);
        else fourway_inside_.erase(v.id);

        if (!is_approaching(v, fourway_center_)) return false;
        if (fourway_inside_.count(v.id)) return false;
        return fourway_inside_.size() >= 3;
    }

    // HV가 차량(v)의 진입을 막고 있는지 확인하는 함수 (내부 사용)
    bool is_blocked_by_hv(const Vehicle& v) {
        double rad_check_left = 0.2 / round_radius_;
        double rad_check_right = 2.1 / round_radius_;
        double v_angle = std::atan2(v.pos.y - round_center_.y, v.pos.x - round_center_.x);

        for (const auto& [tid, target] : vehicles_) {
            if (tid == v.id || !target.active || target.is_cav) continue;
            // 회전교차로 안에 있는 HV만 체크
            if ((target.pos - round_center_).magSq() > round_radius_ * round_radius_) continue;

            double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
            double diff = t_angle - v_angle;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;

            if ((diff > 0 && diff < rad_check_left) || (diff <= 0 && diff > -rad_check_right)) return true;
        }
        return false;
    }

    // [수정된 로직] 기회 기반 + 동시 진입 방지
    bool check_roundabout(const Vehicle& me) {
        double dist_sq = (me.pos - round_center_).magSq();
        double r_sq = round_radius_ * round_radius_;

        // 접근 중이 아니거나 이미 안에 있으면 통과
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(me, round_center_)) return false;

        // 1. 이미 안에 있는 CAV가 있으면 무조건 대기 (안전 제일)
        for (const auto& [tid, target] : vehicles_) {
            if (target.is_cav && target.active && (target.pos - round_center_).magSq() <= r_sq) return true;
        }

        // 2. 내 진입로가 HV에 의해 막혀있으면 대기
        if (is_blocked_by_hv(me)) return true;

        // 3. [동시 진입 방지] 나와 똑같이 "진입 가능한 상태(HV 없음)"인 다른 CAV와 비교
        for (const auto& [tid, target] : vehicles_) {
            if (tid == me.id || !target.is_cav || !target.active) continue;

            // 상대도 접근 중인가?
            double t_dist_sq = (target.pos - round_center_).magSq();
            if (t_dist_sq > round_app_r_sq_ || t_dist_sq <= r_sq || !is_approaching(target, round_center_)) continue;

            // 상대가 HV 때문에 못 들어가는 상황이면, 걔는 경쟁자가 아님 (내가 먼저 감)
            if (is_blocked_by_hv(target)) continue;

            // 여기까지 왔으면 'target'도 지금 당장 들어갈 수 있는 상태임.
            // 둘 중 하나만 들어가야 하므로, 거리와 ID로 승자를 정함. 
            // 상대가 더 가까우면 내가 양보
            if (t_dist_sq < dist_sq - 0.01) return true;

            // 거리가 비슷하면 ID 빠른 놈이 먼저 (동시 진입 절대 불가)
            if (std::abs(t_dist_sq - dist_sq) <= 0.01) {
                if (tid < me.id) return true; 
            }
        }

        return false; // 방해물도 없고, 경쟁에서도 이겼으므로 진입
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}