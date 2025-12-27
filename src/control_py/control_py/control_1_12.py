import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Accel, PoseStamped
import pandas as pd
import numpy as np
import math


class PathTrackerNode(Node):
    def __init__(self):
        super().__init__('path_tracker_node')
        

        self.subscription = self.create_subscription(
            PoseStamped, '/Ego_pose', self.listener_callback, qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Accel, '/Accel', qos_profile_sensor_data)
        
        # CSV 경로 파일 로드
        csv_path = 'tool/p1_1.csv'
        df = pd.read_csv(csv_path)
        self.waypoints = df[['X', 'Y']].to_numpy()
        
        # === 속도 제어 파라미터 ===
        self.max_speed = 0.4  # 직진은 최대 속력으로 할거임
        self.min_speed = 0.1  # 회전은 min~max 사이
        self.speed_lookahead = 45
        self.steer_threshold_low = 0.2 # 이 각도 이하면 최대 속도
        self.steer_threshold_high = 0.6 # 이 각도 이상이면 최소 속도

        # 경로 탐색 최적화를 위한 인덱스 저장
        self. current_pose_idx = 0 # 현재 위치 인덱스 
        self.is_initialized = False # 초기 위치 탐색 여부

    def listener_callback(self, msg):

        # 현재 좌표, 방향
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_yaw = msg.pose.orientation.z 
        
        rear_x = current_x - 0.16 * math.cos(current_yaw) # 뒷바퀴 중심 x좌표.  차량 회전축~후방 길이 = 0.16인데 뒷바퀴 중심 몰라서 임의로 0.12라고 침
        rear_y = current_y - 0.16* math.sin(current_yaw)
        
        # 1. 차량 현재 위치 갱신
        if not self.is_initialized: # 초기 현재 위치 탐색 : 현재 위치와 가장 가까운 waypoint 탐색함 (18~21 사이는 0~250안에 들어있음)
            dx = self.waypoints[:250, 0] - rear_x
            dy = self.waypoints[:250, 1] - rear_y
            dists = np.hypot(dx, dy) # np.hypot(변1, 변2) -> 직각삼각형의 빗변 길이.    => 해당 waypoint ~ 현재 위치간의 거리 계산
            current_idx = np.argmin(dists) # np.argmin : 최솟값의 index 반환
            self.is_initialized = True 
            
        else: # 주행 중 현재 위치 갱신
            search_range = 10 # 탐색 범위 (현재 위치 기준 이후 지점들만)
            candidate_idxs = [] # 현재 위치 후보의 인덱스 리스트
            for i in range(search_range): # 경로 끝까지 도달해도 다시 처음부터 시작하도록 함
                idx = (self.current_pose_idx + i) % len(self.waypoints)
                candidate_idxs.append(idx)
            
                # 후보군 좌표 추출
                candidate_points = self.waypoints[candidate_idxs]
                dx = candidate_points[:, 0] - rear_x
                dy = candidate_points[:, 1] - rear_y
                dists = np.hypot(dx, dy) 
                current_idx = np.argmin(dists)
                current_idx = candidate_idxs[current_idx]
                
        self.current_pose_idx = current_idx
            

        # 2. 타겟 위치 갱신
        lookahead_step = 25 # 차량 뒷바퀴 + Lookahead만큼 앞부분 보기 (cm 단위)
        target_idx = self.current_pose_idx + lookahead_step
        
        # 인덱스가 길이를 넘어가면 처음으로 순환
        if target_idx >= len(self.waypoints):
            target_idx = target_idx % len(self.waypoints)
            
        goal_pos = self.waypoints[target_idx] # [x, y] 목표 위치 정의
        

        # 조향각, 선속력 계산
        steering_angle = self.calculate_goal_rad(rear_x, rear_y, current_yaw, goal_pos[0], goal_pos[1])
        target_vel = self.calculate_adaptive_speed(steering_angle)


        # 메시지 발행
        accel_msg = Accel()
        accel_msg.linear.x = float(target_vel)
        accel_msg.angular.z = float(steering_angle)
        
        self.publisher_.publish(accel_msg)
        
        # 상태 로깅 (디버깅용)
        self.get_logger().info(
            f'Idx:{target_idx} | Steer:{steering_angle:.1f}rad | Vel:{target_vel:.2f} m/s'
        )

    # 조향각에 따라 속력 조절 : 조향각 클 수록 속력 낮춤
    def calculate_adaptive_speed(self, steer_angle):
        abs_steer = abs(steer_angle)
        
        max_speed = self.max_speed
        min_speed = self.min_speed 

        if abs_steer < self.steer_threshold_low:
            return max_speed
        elif abs_steer > self.steer_threshold_high:
            return min_speed
        else:
            # 선형 보간 (Linear Interpolation)
            ratio = (abs_steer - self.steer_threshold_low) / (self.steer_threshold_high - self.steer_threshold_low)
            return max_speed - ratio * (max_speed - min_speed)

    def normalize_angle(self, angle):
        """ 각도를 -pi ~ pi 범위로 정규화 """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def calculate_goal_rad(self, rx, ry, ca, gx, gy): # rear_x, rear_y, current_angle, goal_x, goal_y
        
        # 파라미터 정의
        L = 0.211 # 차량 앞바퀴~뒷바퀴 거리 (축거) 

        dx = gx - rx
        dy = gy - ry
        ld = math.sqrt(dx**2 + dy**2)
        
        if ld == 0: return 0.0

        back_goal_rad = math.atan2(dy, dx) # 뒷바퀴-목표지점 잇는 직선 방향의 절대각도
        
        
        alpha = back_goal_rad - ca # 뒷바퀴-목표지점 잇는 직선과 차량이 바라보는 방향 직선 사이의 각도
        
        # 각도 정규화
        alpha = self.normalize_angle(alpha)

        # Pure Pursuit 공식 
        return math.atan2(2*L*math.sin(alpha), ld)


def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
