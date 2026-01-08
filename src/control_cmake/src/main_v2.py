import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from shapely.geometry import Polygon, Point
import math
import re
from dataclasses import dataclass

@dataclass
class VehicleData:
    id_str: str
    is_cav: bool
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    active: bool = False

class SimpleTrafficController(Node):
    def __init__(self):
        super().__init__('main_traffic_controller')

        # 1. QoS 설정
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 2. 데이터 관리
        self.vehicles = {}
        self.subs = {}
        self.stop_pubs = {}
        
        # 상태 관리: { 멈춘차ID: 원인차ID }
        self.stopped_cavs = {} 
        # 사지교차로 진입 차량 관리 (ID Set)
        self.fourway_inside = set()

        # 차량 제원 [Front, Rear, Left, Right]
        self.car_info = [0.17, 0.16, 0.075, 0.075] 
        self.conflict_range = 0.1

        # 3. 교차로 영역 설정 (Shapely Point & Buffer 활용)
        self.fourway_center = Point(-2.333, 0.0)
        self.fourway_zone = self.fourway_center.buffer(0.8)       # Critical Zone
        self.fourway_app_zone = self.fourway_center.buffer(1.5)   # Approach Zone

        self.round_center = Point(1.667, 0.0)
        self.round_radius = 1.025
        self.round_app_zone = self.round_center.buffer(2.0)       # Approach Zone

        # 4. 정규식 컴파일
        self.cav_regex = re.compile(r"(.*)/?(CAV_(\d{2}))/?.*pose")
        self.hv_regex = re.compile(r"(.*)/?(HV_(\d{2}))/?.*pose")

        # 5. 타이머 실행
        self.create_timer(1.0, self.discover_vehicles)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Traffic Controller Started")

    def discover_vehicles(self):
        """새로운 차량 토픽 스캔 및 등록"""
        for name, types in self.get_topic_names_and_types():
            if 'geometry_msgs/msg/PoseStamped' not in types:
                continue
            
            # 이미 등록된 차량 스킵
            if any(vid in name for vid in self.vehicles):
                continue

            # CAV 감지 (01~04)
            m_cav = self.cav_regex.search(name)
            if m_cav:
                v_id, num = m_cav.group(2), int(m_cav.group(3))
                if 1 <= num <= 4:
                    self.register_vehicle(v_id, name, True)
                continue

            # HV 감지 (19~36)
            m_hv = self.hv_regex.search(name)
            if m_hv:
                v_id, num = m_hv.group(2), int(m_hv.group(3))
                if 19 <= num <= 36:
                    self.register_vehicle(v_id, name, False)

    def register_vehicle(self, v_id, topic, is_cav):
        self.get_logger().info(f"Registered: {v_id}")
        self.vehicles[v_id] = VehicleData(v_id, is_cav)
        
        # 위치 구독 (Lambda Capture 주의)
        self.subs[v_id] = self.create_subscription(
            PoseStamped, topic, 
            lambda msg, vid=v_id: self.update_pose(msg, vid), 
            self.qos
        )
        
        # 정지 명령 퍼블리셔 (CAV만)
        if is_cav:
            self.stop_pubs[v_id] = self.create_publisher(Bool, f"/{v_id}/cmd_stop", self.qos)

    def update_pose(self, msg, v_id):
        if v_id in self.vehicles:
            v = self.vehicles[v_id]
            v.x, v.y = msg.pose.position.x, msg.pose.position.y
            v.yaw = msg.pose.orientation.z
            v.active = True

    def get_vehicle_poly(self, v, margin=0.0):
        """차량의 충돌 박스(Polygon) 생성 헬퍼"""
        front, rear, left, right = self.car_info
        # 내 차 기준 코너 좌표
        corners = [(front + margin, left), (front + margin, -right), (-rear, -right), (-rear, left)]
        
        # 회전 변환 및 절대 좌표로 이동
        poly_points = []
        cos_y, sin_y = math.cos(v.yaw), math.sin(v.yaw)
        for lx, ly in corners:
            nx = v.x + (lx * cos_y - ly * sin_y)
            ny = v.y + (lx * sin_y + ly * cos_y)
            poly_points.append((nx, ny))
            
        return Polygon(poly_points)

    def check_physical_conflict(self, my_cav, target):
        """두 차량 간 물리적 충돌 여부 확인"""
        my_poly = self.get_vehicle_poly(my_cav, self.conflict_range)
        target_poly = self.get_vehicle_poly(target)
        return my_poly.intersects(target_poly)

    def check_fourway(self, my_cav):
        """사지교차로 로직: 2대 이상 진입 시 진입 금지"""
        my_point = Point(my_cav.x, my_cav.y)
        v_id = my_cav.id_str

        # 1. 교차로 내부 상태 업데이트
        if self.fourway_zone.contains(my_point):
            self.fourway_inside.add(v_id)
        else:
            self.fourway_inside.discard(v_id) # 나가면 제거

        # 2. 접근 중일 때 판단
        if self.fourway_app_zone.contains(my_point):
            # 교차로를 향해 가고 있는지 확인 (Rel X > 0)
            dx, dy = self.fourway_center.x - my_cav.x, self.fourway_center.y - my_cav.y
            rel_x = dx * math.cos(my_cav.yaw) + dy * math.sin(my_cav.yaw)

            if rel_x > 0:
                # 이미 내가 안에 있다면 멈출 필요 없음
                if v_id in self.fourway_inside:
                    return False
                # 안에 다른 차들이 2대 이상이면 진입 금지 (병목 방지)
                if len(self.fourway_inside) >= 2:
                    return True

        return False

    def check_roundabout(self, my_cav, all_vehicles):
        """회전교차로 로직: 진입 시 부채꼴 영역 내 차량 확인"""
        my_point = Point(my_cav.x, my_cav.y)
        
        # 접근 구역 아니면 패스
        if not self.round_app_zone.contains(my_point):
            return False

        # 부채꼴 생성 (호 길이 1.0m)
        radius = self.round_radius
        arc_len = 1.0
        half_angle = (arc_len / radius) / 2.0
        
        # 내 차 위치에서 중심을 바라보는 각도
        angle_to_center = math.atan2(my_cav.y - self.round_center.y, my_cav.x - self.round_center.x)
        
        sector_points = [(self.round_center.x, self.round_center.y)]
        start, end = angle_to_center - half_angle, angle_to_center + half_angle
        
        for i in range(11):
            theta = start + (end - start) * (i / 10.0)
            px = self.round_center.x + radius * math.cos(theta)
            py = self.round_center.y + radius * math.sin(theta)
            sector_points.append((px, py))
            
        sector_poly = Polygon(sector_points)

        # 부채꼴 내 장애물 확인
        for target in all_vehicles:
            if target.id_str == my_cav.id_str: continue
            if sector_poly.contains(Point(target.x, target.y)):
                return True # 정지
                
        return False

    def control_loop(self):
        if not self.vehicles: return

        active_cavs = [v for v in self.vehicles.values() if v.is_cav and v.active]
        all_vehicles = [v for v in self.vehicles.values() if v.active]
        next_stopped = {} # 이번 루프 정지 상태 임시 저장

        for my_cav in active_cavs:
            should_stop = False
            cause_id = "NONE"

            # 1. 차량 간 충돌 검사
            for target in all_vehicles:
                if my_cav.id_str == target.id_str: continue

                # 기본 물리 충돌 체크
                is_conflict = self.check_physical_conflict(my_cav, target)
                
                if is_conflict:
                    # CAV 간 데드락 처리
                    if target.is_cav and (target.id_str in self.stopped_cavs):
                        cause = self.stopped_cavs[target.id_str]
                        # 상대가 나 때문에 멈췄으면 나는 지나감 (양보받음)
                        if cause == my_cav.id_str:
                            continue
                    
                    should_stop = True
                    cause_id = target.id_str
                    break # 충돌 감지되면 즉시 루프 종료

            # 2. 교차로 로직 (차량 간 충돌이 없을 때만 수행하여 불필요한 연산 방지)
            if not should_stop:
                if self.check_fourway(my_cav):
                    should_stop = True
                    cause_id = "4WAY_FULL"
                elif self.check_roundabout(my_cav, all_vehicles):
                    should_stop = True
                    cause_id = "ROUND_YIELD"

            # 3. 상태 저장 및 명령 발행
            if should_stop:
                self.stopped_cavs[my_cav.id_str] = cause_id
            else:
                self.stopped_cavs.pop(my_cav.id_str, None) # 정지 해제

            if my_cav.id_str in self.stop_pubs:
                self.stop_pubs[my_cav.id_str].publish(Bool(data=should_stop))

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrafficController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
