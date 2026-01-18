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

struct Point { double x; double y; };

class MainP2Controller : public rclcpp::Node
{
public:
    MainP2Controller() : Node("main_p2_controller"), qos_(10)
    {
        qos_.best_effort();
        qos_.durability_volatile();
        this->declare_parameter("csv_path_1", "tool/fastp2.csv"); 
        this->declare_parameter("csv_path_2", "tool/slowp2.csv"); 
        load_waypoints(this->get_parameter("csv_path_1").as_string(), lane1_points_);
        load_waypoints(this->get_parameter("csv_path_2").as_string(), lane2_points_);
        
        discovery_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MainP2Controller::discover_vehicles, this));
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MainP2Controller::control_loop, this));
    }

private:
    struct VehicleState { double x, y, yaw; int id_num; int target_lane = 1; double mem_dist = 999.0; int mem_cnt = 0; };
    std::vector<Point> lane1_points_, lane2_points_;
    std::map<std::string, VehicleState> vehicle_db_;
    const int MAX_MEMORY_FRAMES = 15; 
    std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> cav_subs_, hv_subs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> stop_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> lane_pubs_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> speed_pubs_;
    rclcpp::TimerBase::SharedPtr discovery_timer_, control_timer_;
    rclcpp::QoS qos_;

    const double ZONE_31_X = 5.083; const double ZONE_31_Y = 0.666; 
    const double ZONE_3_X  = 4.53;  const double ZONE_3_Y  = -2.55;  
    const double TRIGGER_RADIUS = 3.0; 

    void load_waypoints(const std::string& path, std::vector<Point>& v) {
        std::ifstream f(path); if (!f.is_open()) return;
        std::string l; std::getline(f, l); 
        while (std::getline(f, l)) {
            std::stringstream ss(l); std::string c; std::vector<std::string> r;
            while (std::getline(ss, c, ',')) r.push_back(c);
            if (r.size() >= 2) try { v.push_back({std::stod(r[0]), std::stod(r[1])}); } catch (...) {}
        }
    }

    double get_distance_to_path(double x, double y, const std::vector<Point>& pts) {
        if (pts.empty()) return 999.0; double min_d = 999.0;
        for (size_t i = 0; i < pts.size(); i += 5) {
            double d = std::hypot(x - pts[i].x, y - pts[i].y); if (d < min_d) min_d = d;
        }
        return min_d;
    }

    int identify_vehicle_lane(double x, double y) {
        return (get_distance_to_path(x, y, lane1_points_) < get_distance_to_path(x, y, lane2_points_)) ? 1 : 2;
    }

    void discover_vehicles() {
        auto topics = this->get_topic_names_and_types();
        std::regex c_re(R"((.*)/?(CAV_?(\d+))/?.*$)"); std::regex h_re(R"((.*)/?(HV_?(\d+))/?.*$)");    
        for (const auto& [name, types] : topics) {
            std::smatch m; if (std::regex_search(name, m, c_re) || std::regex_search(name, m, h_re)) {
                std::string id = m[2].str(); if (vehicle_db_.count(id) == 0) register_vehicle(id, name, std::stoi(m[3]));
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, int num) {
        vehicle_db_[id] = {0,0,0, num};
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(topic, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                auto& v = vehicle_db_[id]; v.x = msg->pose.position.x; v.y = msg->pose.position.y;
                double qz = msg->pose.orientation.z, qw = msg->pose.orientation.w;
                v.yaw = std::atan2(2.0*(qw*qz), 1.0-2.0*(qz*qz));
            });
        if (id.find("CAV") != std::string::npos) {
            cav_subs_[id] = sub; stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
            speed_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(id + "/cmd_speed", qos_);
            lane_pubs_[id] = this->create_publisher<std_msgs::msg::Int32>(id + "/cmd_lane", qos_);
        } else { hv_subs_[id] = sub; }
    }

    void control_loop() {
        if (vehicle_db_.empty()) return;
        for(auto& [my_id, my_car] : vehicle_db_) {
            if (my_id.find("CAV") == std::string::npos) continue;

            double d31 = std::hypot(my_car.x - ZONE_31_X, my_car.y - ZONE_31_Y);
            double d3  = std::hypot(my_car.x - ZONE_3_X,  my_car.y - ZONE_3_Y);
            
            if (d31 < TRIGGER_RADIUS) my_car.target_lane = 2;
            else if (d3 < TRIGGER_RADIUS || (my_car.x < 4.6 && my_car.target_lane == 2)) my_car.target_lane = 1;

            double front_dist = 999.0; bool side_risk = false; double lateral_for_log = 0.0;
            double my_cos = std::cos(my_car.yaw), my_sin = std::sin(my_car.yaw);
            
            int current_lane = identify_vehicle_lane(my_car.x, my_car.y);
            bool is_changing = (current_lane != my_car.target_lane) || (d31 < TRIGGER_RADIUS) || (my_car.x < 4.8 && my_car.x > 3.0);

            for(const auto& [other_id, other_car] : vehicle_db_) {
                if(my_id == other_id || other_car.id_num == 0 || (other_car.id_num >= 31 && other_car.id_num <= 36)) continue;
                double rx = other_car.x - my_car.x, ry = other_car.y - my_car.y;
                double dist = std::hypot(rx, ry); if (dist < 0.1 || dist > 15.0) continue; 
                double longi = rx * my_cos + ry * my_sin;
                double lat = std::abs(-rx * my_sin + ry * my_cos);
                
                // ★ [수정 핵심] 차선 판별 이전에 관련성 체크
                bool is_relevant = false;
                if (is_changing) {
                    // 차선 변경 중에는 차선 ID 무관, 내 경로 1.2m 폭 안의 모든 차 감시
                    if (lat < 1.2) is_relevant = true;
                } else {
                    // 정속 주행 중에는 목표 차선 일치 차량만 감시
                    int o_lane = identify_vehicle_lane(other_car.x, other_car.y);
                    if (o_lane == my_car.target_lane && lat < 0.15) is_relevant = true;
                }

                if (is_relevant) {
                    if (is_changing) {
                        // 대각선 상태에서도 직선 거리(dist)로 정확히 파악
                        if (longi > -0.2 && dist < front_dist) { front_dist = dist; lateral_for_log = lat; }
                    } else {
                        if (longi > -0.1 && longi < front_dist) { front_dist = longi; lateral_for_log = lat; }
                    }
                    
                    // 측방 위험 감지 범위 확장
                    double side_lat_limit = is_changing ? 0.8 : 0.2;
                    double side_longi_limit = is_changing ? 1.5 : 1.0;
                    if (longi > -0.8 && longi < side_longi_limit && lat < side_lat_limit) side_risk = true;
                }
            }

            if (front_dist < 999.0) { my_car.mem_dist = front_dist; my_car.mem_cnt = 0; }
            else if (my_car.mem_cnt < MAX_MEMORY_FRAMES) { front_dist = my_car.mem_dist; my_car.mem_cnt++; }

            double target_speed = 2.0; bool should_stop = false;
            if (side_risk) { target_speed = 0.0; should_stop = true; }
            else if (front_dist < 1.2) {
                target_speed = std::max(0.0, (front_dist - 0.6) * 1.5);
                if (front_dist < 0.4) { target_speed = 0.0; should_stop = true; }
            }

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "[%s] Target:L%d | F:%.2f | L:%.2f | CHG:%d", 
                my_id.c_str(), my_car.target_lane, front_dist, lateral_for_log, is_changing);

            if (lane_pubs_.count(my_id)) lane_pubs_[my_id]->publish(std_msgs::msg::Int32().set__data(my_car.target_lane));
            if (speed_pubs_.count(my_id)) speed_pubs_[my_id]->publish(std_msgs::msg::Float64().set__data(target_speed));
            if (stop_pubs_.count(my_id)) stop_pubs_[my_id]->publish(std_msgs::msg::Bool().set__data(should_stop));
        }
    }
};
int main(int argc, char * argv[]) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<MainP2Controller>()); rclcpp::shutdown(); return 0; }