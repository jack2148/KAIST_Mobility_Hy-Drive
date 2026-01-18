#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>

// 2D 벡터 연산 및 OBB(Oriented Bounding Box) 충돌 감지를 위한 네임스페이스
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
    Geo::Vec2 pos{0.0, 0.0};
    double z = 0.0;
    bool active = false;
    bool is_stopped = false;
    bool forced_stop = false;
    std::string stop_cause = "";
    
    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    
    // Publisher
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_way;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_roundabout;
};

class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        // 차량 크기 정보 (Front, Rear, Left, Right)
        car_info = {0.17, 0.16, 0.075, 0.075};
        
        // OBB 충돌 감지용 패딩
        front_padding_ = 0.8;
        side_padding_ = 0.8;

        // 충돌 감지 범위
        approach_range_sq_ = 2.5 * 2.5;

        // 회전교차로 관련 변수 (진입 판단용)
        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4;

        tmr_discovery_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this)
        );
        tmr_control_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MainTrafficController::control_loop, this)
        );

        RCLCPP_INFO(get_logger(), "Controller Started: CAV Only Mode (No HV).");
    }

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::map<std::string, std::string> conflict_info_;

    std::vector<double> car_info;
    double front_padding_, side_padding_, approach_range_sq_;
    
    // 회전교차로 변수
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;

    void update_conflict_status(const std::string& stopped_cav, const std::string& cause_vehicle) {
        if (conflict_info_.count(stopped_cav) && conflict_info_[stopped_cav] == cause_vehicle) return;
        conflict_info_[stopped_cav] = cause_vehicle;
    }

    void clear_conflict_status(const std::string& stopped_cav) {
        if (conflict_info_.count(stopped_cav)) {
            conflict_info_.erase(stopped_cav);
        }
    }

    bool is_approaching(const Vehicle& spot, const Geo::Vec2& target_pos) {
        Geo::Vec2 vec = target_pos - spot.pos;
        return vec.dot({std::cos(spot.z), std::sin(spot.z)}) > 0;
    }

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

    void discover_vehicles() {
        auto topic_map = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_map) {
            if (name.empty() || name[0] != '/') continue;
            std::string id = "";
            
            // HV 관련 로직 삭제 -> CAV만 탐색
            if (name.size() > 7) continue;
            if (name.substr(1, 4) == "CAV_") {
                std::string num_str = name.substr(5, 2);
                try { id = "CAV_" + num_str; } catch (...) { continue; }
            }
            
            if (!id.empty() && !vehicles_.count(id)) {
                register_vehicle(id, name);
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic) {
        Vehicle v; v.id = id;
        
        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (vehicles_.count(id)) {
                    Vehicle& veh = vehicles_[id];
                    veh.pos = {msg->pose.position.x, msg->pose.position.y};
                    veh.z = msg->pose.orientation.z;
                    veh.active = true;
                }
            }
        );

        // 모든 차량이 CAV이므로 Publisher 생성
        v.pub_stop = create_publisher<std_msgs::msg::Bool>("/" + id + "/cmd_stop", rclcpp::SensorDataQoS());
       
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered CAV: %s", id.c_str());
    }

    bool is_parallel(const Vehicle& my_cav, const Vehicle& target){
        double dz = std::abs(std::abs(my_cav.z - target.z));
        while (dz > M_PI) dz -= 2.0 * M_PI;
        dz = std::abs(dz);
        return (dz < 0.40) || (dz > (M_PI - 0.40));
    }

    // OBB 기반 충돌 감지 (CAV vs CAV)
    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, bool& deadlock_override) {
        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue;
            if ((my_cav.pos - target.pos).dist_Sq() > approach_range_sq_) continue;

            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto full_box = get_vehicle_corners(my_cav, front_padding_, side_padding_); 

            bool is_full = Geo::check_obb_intersection(full_box, target_box);

            double vehicle_width = car_info[2] + car_info[3];
            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            double side_dist = std::abs(-std::sin(my_cav.z) * dx + std::cos(my_cav.z) * dy);
            bool is_next_lane = is_parallel(my_cav, target) && (side_dist > vehicle_width * 1.2) && side_dist < vehicle_width*2.0;

            if (is_full && !is_next_lane) {
                if (conflict_info_.count(tid) && conflict_info_[tid] == my_cav.id) {
                    deadlock_override = true;
                    continue; 
                }
                conflict_id = tid;
                return true;
            }
        }
        return false;
    }

    // 회전교차로 진입 로직 (다른 CAV가 내부에 있을 경우 확인)
    bool check_roundabout_entry_safety(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        double r_sq = round_radius_ * round_radius_;

        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_)) {
            return false;
        }

        double v_angle = std::atan2(my_cav.pos.y - round_center_.y, my_cav.pos.x - round_center_.x);

        double check_left = 2.1 / round_radius_;
        double check_right = 0.4 / round_radius_;

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue; // target은 무조건 CAV

            double t_dist_sq = (target.pos - round_center_).dist_Sq();
            if (t_dist_sq <= r_sq) {
                double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
                double diff = t_angle - v_angle;
                
                while (diff > M_PI) diff -= 2.0 * M_PI;
                while (diff < -M_PI) diff += 2.0 * M_PI;

                if ((diff > 0 && diff < check_right) || (diff <= 0 && diff > -check_left)) {
                    return true;
                }
            }
        }
        return false;
    }

    void control_loop() {
        if (vehicles_.empty()) return;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.active) continue;

            double dist_from_center = std::sqrt((my_cav.pos - round_center_).dist_Sq());
            bool inside_roundabout = (dist_from_center <= round_radius_);
            
            if (my_cav.pub_is_roundabout) {
                std_msgs::msg::Bool round_msg;
                round_msg.data = inside_roundabout;
                my_cav.pub_is_roundabout->publish(round_msg);
            }

            bool my_cav_stop = false;
            std::string cause_id = "NONE";
            bool deadlock_active = false;

            // 1. 강제 정지 명령 확인
            if (my_cav.forced_stop) {
                my_cav_stop = true; cause_id = my_cav.stop_cause;
            }
            // 2. OBB 충돌 감지 (CAV 간)
            else if (is_conflict(my_cav, cause_id, deadlock_active)) {
                my_cav_stop = true;
            }
            // 3. 회전교차로 진입 안전 확인
            else if (check_roundabout_entry_safety(my_cav)) {
                my_cav_stop = true; cause_id = "ROUND_CAV_YIELD";
            }

            if (!my_cav_stop) {
                clear_conflict_status(my_id);
            } else {
                update_conflict_status(my_id, cause_id);
            }

            my_cav.is_stopped = my_cav_stop;
            if (my_cav_stop && my_cav.stop_cause.empty()) {
                my_cav.stop_cause = cause_id;
            } else if (!my_cav_stop) {
                my_cav.stop_cause.clear();
            }

            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg; msg.data = my_cav_stop;
                my_cav.pub_stop->publish(msg);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}
