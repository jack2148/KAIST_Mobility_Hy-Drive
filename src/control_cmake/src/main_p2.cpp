/**
 * Problem 2: V25 - Soft Start 최적화 (초기 속도 1.3 대응)
 * 주요 기능: 차량 발견, 차선 식별, 병합 구역 판단 및 속도/차선 제어
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

// --- 데이터 구조체 ---
struct Point {
    double x;
    double y;
};

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

// --- 메인 컨트롤러 클래스 ---
class MainP2Controller : public rclcpp::Node {
public:
    MainP2Controller() : Node("main_p2_controller"), qos_(10) {
        // QoS 설정
        qos_.best_effort();
        qos_.durability_volatile();

        // 파라미터 선언 및 로드
        this->declare_parameter("filter_dist", 3.0);
        filter_dist_ = this->get_parameter("filter_dist").as_double();
        
        this->declare_parameter("csv_path_1", "tool/fastp2.csv");
        this->declare_parameter("csv_path_2", "tool/slowp2.csv");
        this->declare_parameter("merge_csv_path", "tool/JSp2merge.csv");

        std::string path1 = this->get_parameter("csv_path_1").as_string();
        std::string path2 = this->get_parameter("csv_path_2").as_string();
        std::string merge_path = this->get_parameter("merge_csv_path").as_string();

        // 웨이포인트 로드
        load_waypoints(path1, lane1_points_);
        load_waypoints(path2, lane2_points_);
        load_waypoints(merge_path, merge_points_);

        calculate_merge_bounds();

        // 시작 시간 기록
        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Problem 2 Controller V25 (Soft Start 1.3) Started.");

        // 타이머 설정
        discovery_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MainP2Controller::discover_vehicles, this));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MainP2Controller::control_loop, this));
    }

private:
    // 멤버 변수
    std::vector<Point> lane1_points_;
    std::vector<Point> lane2_points_;
    std::vector<Point> merge_points_;
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

    // --- 유틸리티 함수 ---

    // CSV 파일로부터 웨이포인트 로드
    void load_waypoints(const std::string& path, std::vector<Point>& target_vec) {
        std::ifstream file(path);
        if (!file.is_open()) return;
        std::string line;
        std::getline(file, line); // 헤더 건너뛰기
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) {
                try {
                    target_vec.push_back({std::stod(row[0]), std::stod(row[1])});
                } catch (...) {}
            }
        }
    }

    // 병합 구역 경계 계산
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

    // 차량이 병합 구역에 있는지 판단
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

    // 경로와 현재 위치 사이의 최단 거리 반환
    double get_distance_to_path(double x, double y, const std::vector<Point>& points) {
        double min_dist = std::numeric_limits<double>::max();
        for (const auto& p : points) {
            double dist = std::hypot(x - p.x, y - p.y);
            if (dist < min_dist) min_dist = dist;
        }
        return min_dist;
    }

    // 현재 위치를 기반으로 차선 ID(1 또는 2) 식별
    int identify_lane(double x, double y) {
        if (lane1_points_.empty() || lane2_points_.empty()) return 2;
        double d1 = get_distance_to_path(x, y, lane1_points_);
        double d2 = get_distance_to_path(x, y, lane2_points_);
        return (d1 < d2) ? 1 : 2;
    }

    // 업데이트가 오래된 차량 정보 제거
    void clean_stale_vehicles() {
        rclcpp::Time now = this->now();
        for (auto it = vehicle_db_.begin(); it != vehicle_db_.end();) {
            if ((now - it->second.last_update).seconds() > 0.5) {
                it = vehicle_db_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // --- 통신 및 상태 관리 ---

    // 새로운 차량 토픽 탐색 및 등록
    void discover_vehicles() {
        auto topic_names = this->get_topic_names_and_types();
        std::regex cav_re(R"((.*)/?(CAV_?(\d+))/?.*$)");
        std::regex hv_re(R"((.*)/?(HV_?(\d+))/?.*$)");

        for (const auto& [name, types] : topic_names) {
            bool is_pose = false;
            for (const auto& t : types) {
                if (t.find("PoseStamped") != std::string::npos) is_pose = true;
            }
            if (!is_pose) continue;

            std::smatch m;
            if (std::regex_search(name, m, cav_re)) {
                std::string id = m[2].str();
                if (vehicle_db_.count(id) == 0) {
                    register_vehicle(id, name, std::stoi(m[3]), VehicleType::CAV);
                }
            } else if (std::regex_search(name, m, hv_re)) {
                std::string id = m[2].str();
                if (vehicle_db_.count(id) == 0) {
                    register_vehicle(id, name, std::stoi(m[3]), VehicleType::HV);
                }
            }
        }
    }

    // 차량 등록 및 Pub/Sub 생성
    void register_vehicle(const std::string& id, const std::string& topic, int num, VehicleType type) {
        vehicle_db_[id] = {0, 0, 0, 0, this->now(), false, num, false, type};
        auto sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, qos_, [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                update_state(id, msg);
            });

        if (type == VehicleType::CAV) {
            cav_subs_[id] = sub;
            stop_pubs_[id] = this->create_publisher<std_msgs::msg::Bool>(id + "/cmd_stop", qos_);
            speed_pubs_[id] = this->create_publisher<std_msgs::msg::Float64>(id + "/cmd_speed", qos_);
            lane_pubs_[id] = this->create_publisher<std_msgs::msg::Int32>(id + "/cmd_lane", qos_);
        } else {
            hv_subs_[id] = sub;
        }
    }

    // 수신된 Pose 데이터를 기반으로 차량 상태 업데이트
    void update_state(const std::string& id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        auto& v = vehicle_db_[id];
        rclcpp::Time current_time = this->now();

        if (v.last_update.get_clock_type() != current_time.get_clock_type()) {
            v.last_update = current_time;
            return;
        }

        double dt = (current_time - v.last_update).seconds();
        if (v.is_active && dt > 0.001) {
            double dx = msg->pose.position.x - v.x;
            double dy = msg->pose.position.y - v.y;
            double dist = std::hypot(dx, dy);
            double raw_speed = dist / dt;
            v.speed = (v.speed * 0.7) + (raw_speed * 0.3); // Low-pass filter
        }

        double qz = msg->pose.orientation.z, qw = msg->pose.orientation.w;
        v.yaw = std::atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
        v.x = msg->pose.position.x;
        v.y = msg->pose.position.y;
        v.last_update = current_time;
        v.is_active = true;
    }

    // --- 제어 루프 ---

    void control_loop() {
        if (vehicle_db_.empty()) return;
        clean_stale_vehicles();

        // [Soft Start 로직] 초기 급발진 방지 (시작 후 2초간 속도 제한)
        double time_since_start = (this->now() - start_time_).seconds();
        double soft_start_limit = 1.3;
        if (time_since_start < 2.0) {
            soft_start_limit = 0.4;
        }

        std::vector<std::string> active_cavs;
        std::set<double> hv_speeds;
        bool hv_found = false;

        for (auto const& [id, v] : vehicle_db_) {
            if (v.type == VehicleType::CAV && v.is_active) active_cavs.push_back(id);
            if (v.type == VehicleType::HV && v.is_active && v.speed > 0.1) {
                hv_speeds.insert(std::round(v.speed * 10.0) / 10.0);
                hv_found = true;
            }
        }

        double v1 = 0.0, v2 = 0.0;
        if (!hv_speeds.empty()) {
            v1 = *hv_speeds.begin();
            v2 = *hv_speeds.rbegin();
        }

        // 각 CAV에 대한 개별 제어 계산
        for (const auto& my_id : active_cavs) {
            auto& my_car = vehicle_db_[my_id];
            int current_lane = identify_lane(my_car.x, my_car.y);

            double MAX_SPEED = soft_start_limit;
            double OVERTAKE_DIST = 4.0;
            double SAFE_GAP_FRONT = 1.5;
            double SAFE_GAP_REAR = 1.2;

            int target_lane = current_lane;
            double target_speed = MAX_SPEED;
            bool should_stop = false;
            std::string status_msg = "Cruising";

            // 초기 웜업 단계
            if (!hv_found && time_since_start < 1.0) {
                target_speed = 0.2;
                status_msg = "Warming Up...";
            }

            // 주변 차량과의 거리 계산 (Longitudinal)
            double f_dist_l1 = 999.0, r_dist_l1 = 999.0;
            double f_dist_l2 = 999.0, r_dist_l2 = 999.0;
            double my_cos = std::cos(my_car.yaw), my_sin = std::sin(my_car.yaw);

            for (const auto& [other_id, other_car] : vehicle_db_) {
                if (my_id == other_id || !other_car.is_active) continue;

                double dx = other_car.x - my_car.x;
                double dy = other_car.y - my_car.y;
                double raw_dist = std::hypot(dx, dy);

                if (raw_dist > 8.0) continue;

                int other_lane_id = identify_lane(other_car.x, other_car.y);
                
                // L2 차선 오프셋 필터링
                if (other_lane_id == 2 && get_distance_to_path(other_car.x, other_car.y, lane2_points_) > 0.45) 
                    continue;

                double longi = dx * my_cos + dy * my_sin;
                if (other_lane_id == 1) {
                    if (longi >= 0) f_dist_l1 = std::min(f_dist_l1, raw_dist);
                    else r_dist_l1 = std::min(r_dist_l1, raw_dist);
                } else {
                    if (longi >= 0) f_dist_l2 = std::min(f_dist_l2, raw_dist);
                    else r_dist_l2 = std::min(r_dist_l2, raw_dist);
                }
            }

            // 거리 기반 속도 결정 람다
            auto calculate_traffic_speed = [&](double distance) -> double {
                if (distance < 0.5) return 0.0;
                if (distance < 1.5) return 0.6;
                return MAX_SPEED;
            };

            bool in_merge_zone = is_in_merge_zone(my_car.x, my_car.y);
            double boost_spd = std::min(1.8, MAX_SPEED + 0.3);

            // --- 의사결정 로직 ---
            if (in_merge_zone) {
                if (current_lane == 1) {
                    if (f_dist_l1 > 3.0 || r_dist_l2 < 3.0) {
                        target_lane = 1;
                        target_speed = std::min(boost_spd, calculate_traffic_speed(f_dist_l1));
                        status_msg = "Merge Zone (Stay L1)";
                    } else {
                        target_lane = 2;
                        target_speed = std::min(boost_spd, calculate_traffic_speed(f_dist_l2));
                        status_msg = "!MERGE ZONE BOOST (Return)!";
                    }
                } else {
                    target_lane = 2;
                    target_speed = std::min(boost_spd, calculate_traffic_speed(f_dist_l2));
                    status_msg = "!MERGE ZONE BOOST!";
                }
            } else {
                if (current_lane == 2) {
                    if (f_dist_l2 < OVERTAKE_DIST) {
                        if (f_dist_l1 > SAFE_GAP_FRONT && r_dist_l1 > SAFE_GAP_REAR) {
                            target_lane = 1;
                            target_speed = calculate_traffic_speed(f_dist_l1);
                            status_msg = "Overtaking (-> Right)";
                        } else {
                            target_speed = calculate_traffic_speed(f_dist_l2);
                            status_msg = "Blocked L2 (Traffic)";
                        }
                    } else {
                        target_speed = MAX_SPEED;
                        status_msg = "Cruising L2";
                    }
                } else { // current_lane == 1
                    bool l1_blocked = (f_dist_l1 < OVERTAKE_DIST);
                    bool l2_clear = (f_dist_l2 > 8.0 && r_dist_l2 > 6.0);

                    if (l1_blocked) {
                        if (l2_clear) {
                            target_lane = 2;
                            target_speed = 0.6;
                            status_msg = "Return (-> Left)";
                        } else {
                            target_speed = calculate_traffic_speed(f_dist_l1);
                            status_msg = "Blocked L1 (Traffic)";
                        }
                    } else {
                        if ((r_dist_l1 < 2.0 || f_dist_l2 > 15.0) && l2_clear) {
                            target_lane = 2;
                            status_msg = "Yield (-> Left)";
                        } else {
                            target_speed = MAX_SPEED;
                            status_msg = "Cruising L1";
                        }
                    }
                }
            }

            // 속도 최종 클램핑
            target_speed = std::max(0.0, std::min(target_speed, 1.8));

            // 정보 출력 (Throttle 적용)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                 "[%s] L%d->%d | Spd:%.2f | F:%.1f/%.1f R:%.1f/%.1f | v1:%.1f v2:%.1f | %s",
                                 my_id.c_str(), current_lane, target_lane, target_speed, f_dist_l1, f_dist_l2,
                                 r_dist_l1, r_dist_l2, v1, v2, status_msg.c_str());

            // 명령 발행
            if (lane_pubs_.count(my_id))
                lane_pubs_[my_id]->publish(std_msgs::msg::Int32().set__data(target_lane));
            if (speed_pubs_.count(my_id))
                speed_pubs_[my_id]->publish(std_msgs::msg::Float64().set__data(target_speed));
            if (stop_pubs_.count(my_id))
                stop_pubs_[my_id]->publish(std_msgs::msg::Bool().set__data(should_stop));
        }
    }
};

// --- 메인 함수 ---
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainP2Controller>());
    rclcpp::shutdown();
    return 0;
}