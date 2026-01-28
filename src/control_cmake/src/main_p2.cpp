/**
 * Problem 2: V40 - Safe Escape (Prevent Clipping)
 * * [V39 유지] Smart Gap & L3 Filtering
 * * [V40 추가] Escape Cornering Control
 * - 문제: Escape 시 속도를 높이면 회전 반경이 커져서, 차선 변경 중 원래 차선 앞차 충돌.
 * - 해결: 장애물이 가까우면(2.0m 미만) 속도를 1.0m/s 이하로 제한하여 날카롭게 회전.
 */

#include <algorithm>
#include <cmath>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <set>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;

struct Point { double x; double y; };
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

class MainP2Controller : public rclcpp::Node {
public:
    MainP2Controller() : Node("main_p2_controller"), qos_(10) {
        qos_.best_effort();
        qos_.durability_volatile();

        this->declare_parameter("filter_dist", 3.0);
        filter_dist_ = this->get_parameter("filter_dist").as_double();
        
        this->declare_parameter("csv_path_1", "tool/JSp2Lane1.csv");
        this->declare_parameter("csv_path_2", "tool/JSp2Lane2.csv");
        this->declare_parameter("merge_csv_path", "tool/JSp2merge.csv");

        std::string path1 = this->get_parameter("csv_path_1").as_string();
        std::string path2 = this->get_parameter("csv_path_2").as_string();
        std::string merge_path = this->get_parameter("merge_csv_path").as_string();

        load_waypoints(path1, lane1_points_);
        load_waypoints(path2, lane2_points_);
        load_waypoints(merge_path, merge_points_);
        calculate_merge_bounds();

        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Problem 2 Controller V40 (Safe Escape) Started.");

        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MainP2Controller::discover_vehicles, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MainP2Controller::control_loop, this));
    }

private:
    std::vector<Point> lane1_points_, lane2_points_, merge_points_;
    rclcpp::Time start_time_;
    double merge_min_x_ = 9999.0, merge_max_x_ = -9999.0;
    double merge_min_y_ = 9999.0, merge_max_y_ = -9999.0;
    rclcpp::TimerBase::SharedPtr discovery_timer_, control_timer_;
    rclcpp::QoS qos_;
    std::map<std::string, VehicleState> vehicle_db_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_, hv_subs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> lane_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> speed_pubs_;
    double filter_dist_;

    void load_waypoints(const std::string& path, std::vector<Point>& target_vec) {
        std::ifstream file(path);
        if (!file.is_open()) return;
        std::string line;
        std::getline(file, line);
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) try { target_vec.push_back({std::stod(row[0]), std::stod(row[1])}); } catch (...) {}
        }
    }

    void calculate_merge_bounds() {
        if (merge_points_.empty()) return;
        for (const auto& p : merge_points_) {
            if (p.x < merge_min_x_) merge_min_x_ = p.x;
            if (p.x > merge_max_x_) merge_max_x_ = p.x;
            if (p.y < merge_min_y_) merge_min_y_ = p.y;
            if (p.y > merge_max_y_) merge_max_y_ = p.y;
        }
        merge_min_x_ -= 1.0; merge_max_x_ += 1.0;
        merge_min_y_ -= 1.0; merge_max_y_ += 1.0;
    }

    bool is_in_merge_zone(double x, double y) {
        if (merge_points_.empty()) return false;
        if (x < merge_min_x_ || x > merge_max_x_ || y < merge_min_y_ || y > merge_max_y_) return false;
        double min_d = std::numeric_limits<double>::max();
        for (const auto& p : merge_points_) {
            double d = std::hypot(x - p.x, y - p.y);
            if (d < min_d) min_d = d;
        }
        return (min_d < 0.7);
    }

    double get_distance_to_path(double x, double y, const std::vector<Point>& points) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p : points) {
            double dist = std::hypot(x - p.x, y - p.y);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }

    int identify_lane(double x, double y) {
        if (lane1_points_.empty() || lane2_points_.empty()) return 2;
        double d1 = get_distance_to_path(x, y, lane1_points_);
        double d2 = get_distance_to_path(x, y, lane2_points_);
        
        bool is_l1 = (d1 < 2.0);
        bool is_l2 = (d2 < 2.0);

        if (is_l1 && !is_l2) return 1;
        if (!is_l1 && is_l2) return 2;
        if (is_l1 && is_l2) return (d1 < d2) ? 1 : 2;
        
        return -1; 
    }

    void clean_stale_vehicles() {
        rclcpp::Time now = this->now();
        for (auto it = vehicle_db_.begin(); it != vehicle_db_.end();) {
            if ((now - it->second.last_update).seconds() > 0.5) it = vehicle_db_.erase(it);
            else ++it;
        }
    }

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
                if (id.find("HV") != std::string::npos) continue; 
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, std::stoi(m[3]), VehicleType::CAV);
            } else if (std::regex_search(name, m, hv_re)) {
                std::string id = m[2].str();
                if (vehicle_db_.count(id) == 0) register_vehicle(id, name, std::stoi(m[3]), VehicleType::HV);
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, int num, VehicleType type) {
        vehicle_db_[id] = {0, 0, 0, 0, this->now(), false, num, false, type};
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { update_state(id, msg); });
        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
            speed_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(id + "/cmd_speed", qos_);
            lane_pubs_[id] = this->create_publisher<std_msgs::msg::Int32>(id + "/cmd_lane", qos_);
        } else {
            hv_subs_[id] = sub;
        }
    }

    void update_state(const std::string& id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto& v = vehicle_db_[id];
        rclcpp::Time current_time = this->now();
        if (v.last_update.get_clock_type() != current_time.get_clock_type()) { v.last_update = current_time; return; }
        double dt = (current_time - v.last_update).seconds();
        if (v.is_active && dt > 0.001) {
            double dx = msg->pose.position.x - v.x;
            double dy = msg->pose.position.y - v.y;
            v.speed = (v.speed * 0.7) + ((std::hypot(dx, dy) / dt) * 0.3); 
        }
        double qz = msg->pose.orientation.z, qw = msg->pose.orientation.w;
        v.yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
        v.x = msg->pose.position.x; v.y = msg->pose.position.y;
        v.last_update = current_time; v.is_active = true;
    }

    void control_loop() {
        if (vehicle_db_.empty()) return;
        clean_stale_vehicles();

        std::set<double> hv_speeds;
        for (auto const& [id, v] : vehicle_db_) if (v.type == VehicleType::HV && v.is_active && v.speed > 0.1) 
            hv_speeds.insert(std::round(v.speed * 10.0) / 10.0);

        double v1 = 1.0, v2 = 1.2;
        if (!hv_speeds.empty()) { v1 = *hv_speeds.begin(); v2 = *hv_speeds.rbegin(); }
        v1 = std::min(v1, 2.0); v2 = std::min(v2, 2.0);

        std::vector<std::string> active_cavs;
        for (auto const& [id, v] : vehicle_db_) if (v.type == VehicleType::CAV && v.is_active && id.find("HV") == std::string::npos) 
            active_cavs.push_back(id);

        for (const auto& my_id : active_cavs) {
            auto& my_car = vehicle_db_[my_id];
            
            int current_lane_raw = identify_lane(my_car.x, my_car.y);
            int current_lane = (current_lane_raw == -1) ? 2 : current_lane_raw; 

            double dynamic_safe_gap = std::max(3.5, my_car.speed * 2.5);
            double dynamic_return_gap = std::max(5.0, my_car.speed * 3.5);

            double f_dist_l1 = 999.0, r_dist_l1 = 999.0, f_dist_l2 = 999.0, r_dist_l2 = 999.0;
            double my_cos = std::cos(my_car.yaw), my_sin = std::sin(my_car.yaw);

            for (const auto& [other_id, other_car] : vehicle_db_) {
                if (my_id == other_id || !other_car.is_active) continue;
                double dx = other_car.x - my_car.x;
                double dy = other_car.y - my_car.y;
                double raw_dist = std::hypot(dx, dy);
                if (raw_dist > 25.0) continue; 

                int other_lane_id = identify_lane(other_car.x, other_car.y);
                if (other_lane_id == -1) continue; 

                double longi = dx * my_cos + dy * my_sin;
                if (other_lane_id == 1) {
                    if (longi >= 0) f_dist_l1 = std::min(f_dist_l1, raw_dist);
                    else r_dist_l1 = std::min(r_dist_l1, raw_dist);
                } else {
                    if (longi >= 0) f_dist_l2 = std::min(f_dist_l2, raw_dist);
                    else r_dist_l2 = std::min(r_dist_l2, raw_dist);
                }
            }

            auto adapt_speed_to_front = [&](double distance, double max_s) -> double {
                if (distance < 0.45) return 0.0;
                if (distance < 3.0) return std::min(max_s, std::max(0.3, max_s * (distance / 3.0)));
                return max_s;
            };

            int target_lane = current_lane;
            double target_speed = v2 * 1.5;
            std::string status_msg = "Cruising";

            bool in_merge_zone = is_in_merge_zone(my_car.x, my_car.y);
            
            if (in_merge_zone && current_lane == 1) {
                double required_front_gap = std::max(4.0, my_car.speed * 2.5);

                if (r_dist_l2 < 2.0 || f_dist_l2 < required_front_gap) {
                    target_lane = 1; 
                    double yield_speed = v2 * 0.7;
                    target_speed = adapt_speed_to_front(f_dist_l1, yield_speed);
                    
                    if (f_dist_l2 < required_front_gap) status_msg = "Yielding (Front Gap)";
                    else status_msg = "Yielding (Unsafe Rear)";
                } else {
                    target_lane = 2; 
                    double computed_speed = adapt_speed_to_front(f_dist_l2, v2);
                    if (f_dist_l2 < 1.5) target_speed = computed_speed;
                    else target_speed = std::max(0.8, computed_speed); 
                    status_msg = "MERGE";
                }
            } else {
                if (current_lane == 2) {
                    if (f_dist_l2 < dynamic_safe_gap * 0.8) {
                        bool rear_danger = (r_dist_l2 < 3.0);
                        double req_gap = rear_danger ? (dynamic_safe_gap * 0.5) : dynamic_safe_gap;
                        double req_rear_l1 = rear_danger ? 0.5 : 1.0; 

                        if (f_dist_l1 > req_gap && r_dist_l1 > req_rear_l1) {
                            target_lane = 1; 
                            double overtake_speed = adapt_speed_to_front(f_dist_l1, v2);
                            
                            // [V40 FIX: Safe Escape]
                            // 탈출 시 앞(L2) 장애물이 가까우면(< 2.0m) 속도를 제한하여
                            // 회전 반경을 줄임 (충돌 방지)
                            if (f_dist_l2 < 2.0) {
                                // 장애물이 가까우면 최대 1.0m/s로 제한하되, 
                                // 최소한의 조향 속도(0.5m/s)는 유지
                                target_speed = std::max(0.5, std::min(1.0, overtake_speed));
                            } else {
                                // 장애물이 멀면 시원하게 가속해서 탈출
                                target_speed = rear_danger ? std::max(overtake_speed, my_car.speed) : overtake_speed;
                            }
                            
                            status_msg = rear_danger ? "Escape L2" : "Overtake";
                        } else { 
                            target_speed = adapt_speed_to_front(f_dist_l2, v2); 
                            status_msg = "Blocked L2"; 
                        }
                    } else {
                        target_speed = adapt_speed_to_front(f_dist_l2, v2);
                    }
                } else {
                    bool l1_blocked = (f_dist_l1 < dynamic_safe_gap * 0.8);
                    bool l2_clear = (f_dist_l2 > dynamic_return_gap && r_dist_l2 > dynamic_safe_gap);
                    if (l1_blocked) {
                        if (l2_clear) { target_lane = 2; status_msg = "Return"; }
                        else { target_speed = adapt_speed_to_front(f_dist_l1, v1); status_msg = "Blocked L1"; }
                    } else if ((r_dist_l1 < 3.0 || f_dist_l2 > dynamic_return_gap * 1.5) && l2_clear) {
                        target_lane = 2; status_msg = "Yield";
                    }
                }
            }

            target_speed = std::max(0.0, std::min(target_speed, 2.0));
            if (lane_pubs_.count(my_id)) lane_pubs_[my_id]->publish(std_msgs::msg::Int32().set__data(target_lane));
            if (speed_pubs_.count(my_id)) speed_pubs_[my_id]->publish(std_msgs::msg::Float64().set__data(target_speed));
            if (stop_pubs_.count(my_id)) stop_pubs_[my_id]->publish(std_msgs::msg::Bool().set__data(false));

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[%s] L%d->%d | Spd:%.2f | L1(F:%.1f R:%.1f) L2(F:%.1f R:%.1f) | GapReq:%.1f | %s",
                my_id.c_str(), current_lane, target_lane, target_speed, 
                f_dist_l1, r_dist_l1, f_dist_l2, r_dist_l2, dynamic_safe_gap, status_msg.c_str());
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainP2Controller>());
    rclcpp::shutdown();
    return 0;
}