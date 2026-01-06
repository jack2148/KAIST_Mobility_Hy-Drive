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

using std::placeholders::_1;

// 색상 정의
const std::string ANSI_RESET  = "\033[0m";
const std::string ANSI_GREEN  = "\033[32m";
const std::string ANSI_YELLOW = "\033[33m";
const std::string ANSI_BLUE   = "\033[34m";
const std::string ANSI_RED    = "\033[31m";

enum class VehicleType { CAV, HV };

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

struct ZoneAngles { double start, end; };

class MainTrafficController : public rclcpp::Node
{
public:
    MainTrafficController() : Node("main_traffic_controller"), qos_(10) {
        qos_.best_effort(); qos_.durability_volatile();

        this->declare_parameter("filter_dist", 2.0); 
        filter_dist_ = this->get_parameter("filter_dist").as_double();

        this->declare_parameter("interaction_dist", 1.0);
        double interaction_dist = this->get_parameter("interaction_dist").as_double();
        max_interaction_dist_sq_ = interaction_dist * interaction_dist;

        // 차량 길이 0.33m 반영 (여유 거리 0.2m 포함)
        this->declare_parameter("conflict_dist", 0.33 + 0.2);
        conflict_dist_ = this->get_parameter("conflict_dist").as_double();

        this->declare_parameter("parallel_angle_deg", 3.0);
        double deg = this->get_parameter("parallel_angle_deg").as_double();
        parallel_angle_rad_ = deg * M_PI / 180.0;

        fourway_x_ = -2.333; fourway_y_ = 0.0; fourway_radius_ = 0.8;
        round_x_ = 1.667; round_y_ = 0.0; round_radius_ = 1.025;

        setup_roundstop_zones();

        discovery_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MainTrafficController::discover_vehicles, this));
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MainTrafficController::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Main Controller Started. FilterDist: %.2f", filter_dist_);
    }

private:
    const double APPROACH_RADIUS = 1.0; 
    const double YAW_RATE_THRESHOLD = 0.15;
    
    double filter_dist_;
    double max_interaction_dist_sq_; 
    double conflict_dist_; 
    double parallel_angle_rad_;

    rclcpp::TimerBase::SharedPtr discovery_timer_, control_timer_;
    rclcpp::QoS qos_;
    double fourway_x_, fourway_y_, fourway_radius_;
    double round_x_, round_y_, round_radius_;
    std::map<int, ZoneAngles> rs_zones_; 

    std::map<std::string, VehicleState> vehicle_db_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_;
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;

    void setup_roundstop_zones() {
        auto calc_angles = [&](double x1, double y1, double x2, double y2) -> ZoneAngles {
            return { std::atan2(y1 - round_y_, x1 - round_x_), std::atan2(y2 - round_y_, x2 - round_x_) };
        };
        rs_zones_[1] = calc_angles(0.6906125545501709, 0.312959134578, 1.4520390033721924, -1.0022773742675781);
        ZoneAngles zone_23 = calc_angles(1.9554425477981567, 0.9834802746772766, 0.7294758558273315, 0.41508838534355164);
        rs_zones_[2] = zone_23; rs_zones_[3] = zone_23;
        rs_zones_[4] = calc_angles(0.7692194581031799, 0.49519041180610657, 1.1041407585144043, -0.8568485975265503);
        rs_zones_[0] = rs_zones_[1]; 
    }

    std::pair<double, double> get_relative_pos(const VehicleState& my, const VehicleState& target) {
        double dx = target.x - my.x;
        double dy = target.y - my.y;
        return { dx * std::cos(my.yaw) + dy * std::sin(my.yaw), -dx * std::sin(my.yaw) + dy * std::cos(my.yaw) };
    }

    bool is_in_fourway_critical(double x, double y) const { return std::hypot(x - fourway_x_, y - fourway_y_) <= fourway_radius_; }
    bool is_in_fourway_approach(double x, double y) const { return std::hypot(x - fourway_x_, y - fourway_y_) <= APPROACH_RADIUS; }
    bool is_in_round_critical(double x, double y) const { return std::hypot(x - round_x_, y - round_y_) <= round_radius_; }
    bool is_in_round_approach(double x, double y) const { return std::hypot(x - round_x_, y - round_y_) <= APPROACH_RADIUS; }
    bool is_in_roundstop_zone(double x, double y, int id_num) const {
        int key = rs_zones_.count(id_num) ? id_num : 1; 
        const auto& zone = rs_zones_.at(key);
        double ang = std::atan2(y - round_y_, x - round_x_);
        return (ang >= zone.start || ang <= zone.end);
    }

    void discover_vehicles() {
        auto topic_names_and_types = this->get_topic_names_and_types();
        std::regex cav_regex(R"((.*)/?(CAV_?(\d+))/?.*$)"); std::regex hv_regex(R"((.*)/?(HV_?(\d+))/?.*$)"); 
        for (const auto& [name, types] : topic_names_and_types) {
             bool is_pose = false; for (const auto& type : types) if (type.find("PoseStamped") != std::string::npos) is_pose = true;
             if(!is_pose) continue;
             std::smatch match;
             if (std::regex_search(name, match, cav_regex)) {
                 if (vehicle_db_.count(match[2].str()) == 0) register_vehicle(match[2].str(), name, std::stoi(match[3].str()), VehicleType::CAV);
             } else if (std::regex_search(name, match, hv_regex)) {
                 if (vehicle_db_.count(match[2].str()) == 0) register_vehicle(match[2].str(), name, std::stoi(match[3].str()), VehicleType::HV);
             }
        }
    }
    void register_vehicle(const std::string& id, const std::string& topic_name, int num, VehicleType type) {
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic_name, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ update_state(id, msg); });
        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
        } else {
            hv_subs_[id] = sub;
        }
        vehicle_db_[id] = {0,0,0,0,0, this->now(), false, num, false, type};
        RCLCPP_INFO(this->get_logger(), "Registered %s", id.c_str());
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
            v.yaw_rate = v.yaw_rate * 0.6 + (std::abs(diff) / dt) * 0.4;
        }
        v.x = msg->pose.position.x; v.y = msg->pose.position.y;
        v.yaw = current_yaw; v.last_update = this->now(); v.is_active = true;
    }

    void control_loop() {
        if (vehicle_db_.empty()) return;

        std::map<std::string, bool> next_stop_state;
        std::map<std::string, std::string> stop_reasons;
        
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV) {
                next_stop_state[id] = false; stop_reasons[id] = "NONE";
            }
        }

        std::vector<std::string> active_cavs;
        for(auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);
        }

        for(const auto& id_a : active_cavs) {
            auto& state_a = vehicle_db_[id_a];
            bool stop_command = false;

            // 교차로 정보
            bool in_fourway_critical = is_in_fourway_critical(state_a.x, state_a.y);
            bool approaching_fourway = (!in_fourway_critical) && is_in_fourway_approach(state_a.x, state_a.y);
            bool in_round_critical = is_in_round_critical(state_a.x, state_a.y);
            bool approaching_round = (!in_round_critical) && is_in_round_approach(state_a.x, state_a.y);

            for(auto const& [id_b, state_b] : vehicle_db_) {
                if(id_a == id_b || !state_b.is_active) continue;

                double dist_sq = std::pow(state_b.x - state_a.x, 2) + std::pow(state_b.y - state_a.y, 2);
                if (dist_sq > max_interaction_dist_sq_) continue; 

                auto rel_pos_b_from_a = get_relative_pos(state_a, state_b);
                double b_in_a_x = rel_pos_b_from_a.first; 

                // [Conflict Check with Anti-Flickering]
                if (b_in_a_x > 0.0 && b_in_a_x < conflict_dist_) {
                    
                    double yaw_diff = std::abs(state_a.yaw - state_b.yaw);
                    while(yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI; 
                    yaw_diff = std::abs(yaw_diff); 

                    bool is_parallel = (yaw_diff < parallel_angle_rad_); 
                    bool is_opposite = (yaw_diff > (M_PI - parallel_angle_rad_)); 

                    if (!is_parallel && !is_opposite) {
                        bool should_stop = false;

                        if (!state_b.is_stopped) {
                            should_stop = true;
                        }
                        else {
                            if (state_a.is_stopped) {
                                if (state_a.id_num > state_b.id_num) should_stop = true; 
                                else should_stop = false; 
                            }
                            else {
                                should_stop = false;
                            }
                        }

                        if (should_stop) {
                            stop_command = true;
                            std::stringstream ss;
                            ss << "CONFLICT [RelX:" << std::fixed << std::setprecision(2) << b_in_a_x 
                               << ", Ang:" << (int)(yaw_diff * 180.0/M_PI) << "deg] vs " << id_b;
                            if (state_b.is_stopped) ss << " (Deadlock Yield)";
                            stop_reasons[id_a] = ss.str();
                        }
                    }
                }
            }

            // [기존 교차로 로직]
            if (!stop_command) {
                if (in_fourway_critical || approaching_fourway) {
                      for(auto const& [id_b, state_b] : vehicle_db_) {
                        if(id_a == id_b) continue;
                        if(is_in_fourway_critical(state_b.x, state_b.y) && approaching_fourway && std::abs(state_b.yaw_rate) > YAW_RATE_THRESHOLD) {
                             stop_command = true; stop_reasons[id_a] = "4WAY (Wait Turn " + id_b + ")";
                        }
                        if(in_fourway_critical && is_in_fourway_critical(state_b.x, state_b.y) && state_a.id_num > state_b.id_num) {
                             stop_command = true; stop_reasons[id_a] = "4WAY (Yield ID " + id_b + ")";
                        }
                      }
                }
                else if (in_round_critical || approaching_round) {
                      for(auto const& [id_b, state_b] : vehicle_db_) {
                        if(id_a == id_b) continue;
                        
                        // [수정] 진입 시(approaching_round)에만 HV 체크
                        if(approaching_round) {
                            if(state_b.type==VehicleType::HV && is_in_roundstop_zone(state_b.x, state_b.y, state_b.id_num)) {
                                 stop_command = true; stop_reasons[id_a] = "ROUND (Wait HV " + id_b + ")";
                            }
                        }

                        // [유지] 내부 충돌 방지: 이미 내부에 있다면(in_round_critical) ID 낮은 차량 양보
                        if(state_b.type==VehicleType::CAV && in_round_critical && is_in_round_critical(state_b.x, state_b.y) && state_a.id_num > state_b.id_num) {
                             stop_command = true; stop_reasons[id_a] = "ROUND (Yield ID " + id_b + ")";
                        }
                      }
                }
            }

            if (stop_command) next_stop_state[id_a] = true;
        }

        for(auto& [id, should_stop] : next_stop_state) {
            auto& v = vehicle_db_[id];
            if (v.is_stopped != should_stop) {
                std::string reason = stop_reasons[id];
                if (should_stop) {
                    if (reason.find("CONFLICT") != std::string::npos)
                        RCLCPP_INFO(this->get_logger(), "%s[%s] STOP: %s%s", ANSI_RED.c_str(), id.c_str(), reason.c_str(), ANSI_RESET.c_str());
                    else
                        RCLCPP_INFO(this->get_logger(), "%s[%s] STOP: %s%s", ANSI_GREEN.c_str(), id.c_str(), reason.c_str(), ANSI_RESET.c_str());
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s[%s] GO: Path Clear%s", ANSI_BLUE.c_str(), id.c_str(), ANSI_RESET.c_str());
                }
            }
            v.is_stopped = should_stop;
            if (stop_pubs_.count(id)) {
                std_msgs::msg::Bool msg;
                msg.data = should_stop;
                stop_pubs_[id]->publish(msg);
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
