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
        self.max_speed = 0.3  # 직진 속력 (최대 속력)
        self.min_speed = 0.1  # 회전 속력 (최소 속력)
        self.steer_threshold_low = np.deg2rad(10) # 이 각도 이하면 최대 속도
        self.steer_threshold_high = np.deg2rad(25) # 이 각도 이상이면 최소 속도

        # 경로 탐색 최적화를 위한 인덱스 저장
        self. current_pose_idx = 0 # 현재 위치 인덱스
        self.is_initialized = False # 초기 위치 탐색 여부

    def listener_callback(self, msg):

        # 현재 좌표, 방향
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_yaw = msg.pose.orientation.z
        
        # 1. 차량 현재 위치 갱신
        if not self.is_initialized: # 초기 현재 위치 탐색 : 현재 위치와 가장 가까운 waypoint 탐색함 (18~21 사이는 0~250안에 들어있음)
            dx = self.waypoints[:250, 0] - current_x
            dy = self.waypoints[:250, 1] - current_y
            dists = np.hypot(dx, dy) # np.hypot(변1, 변2) -> 직각삼각형의 빗변 길이.    => 해당 waypoint ~ 현재 위치간의 거리 계산
            nearest_idx = np.argmin(dists) # np.argmin : 최솟값의 index 반환
            self.current_pose_idx = nearest_idx
            self.is_initialized = True 

        else: # 주행 중 현재 위치 갱신
            search_range = 10 # 탐색 범위 (현재 위치 기준 이후 지점들만)
            candidate_idxs = [] # 현재 위치 후보의 인덱스 리스트
            for i in range(search_range): # 경로 끝까지 도달해도 다시 처음부터 시작하도록 함
                idx = (self.current_pose_idx + i) % len(self.waypoints)
                candidate_idxs.append(idx)
            
            # 후보군 좌표 추출
            candidate_points = self.waypoints[candidate_idxs]
            dx = candidate_points[:, 0] - current_x
            dy = candidate_points[:, 1] - current_y
            dists = np.hypot(dx, dy)
            
            # 가장 가까운 점 찾기
            min_dist_idx = np.argmin(dists)
            current_idx = candidate_idxs[min_dist_idx]
            
            self.current_pose_idx = current_idx

        
        # 2. 타겟 위치 갱신
        lookahead_step = 16 # Lookahead만큼 앞부분 보기 (cm 단위)
        target_idx = self.current_pose_idx + lookahead_step
        
        # 인덱스가 길이를 넘어가면 처음으로 순환
        if target_idx >= len(self.waypoints):
            target_idx = target_idx % len(self.waypoints)
            
        goal_pos = self.waypoints[target_idx] # [x, y] 목표 위치 정의
        

        # 조향각 계산 (현재 타겟 기준 - 실제 조향용)
        steering_angle = self.calculate_goal_rad(
            {'x': current_x, 'y': current_y}, 
            current_yaw, 
            {'x': goal_pos[0], 'y': goal_pos[1]}
        )
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

    def calculate_goal_rad(self, current_pos, current_yaw, goal_pos):
        
        # 파라미터 정의
        L = 0.33 # 차량 길이.
        
        # 입력 형식 맞추기
        cx, cy = current_pos['x'], current_pos['y']
        gx, gy = goal_pos['x'], goal_pos['y']

        posx_back = cx - 0.14 * math.cos(current_yaw) # 뒷바퀴 중심 x좌표
        posy_back = cy - 0.14 * math.sin(current_yaw) # 뒷바퀴 중심 y좌표
        
        dx = gx - posx_back
        dy = gy - posy_back
        ld = math.sqrt(dx**2 + dy**2)
        
        if ld == 0: return 0.0

        back_goal_rad = math.atan2(dy, dx) # 뒷바퀴-목표지점 잇는 직선 방향의 절대각도
        
        
        alpha = back_goal_rad - current_yaw # 뒷바퀴-목표지점 잇는 직선과 차량이 바라보는 방향 직선 사이의 각도
        
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
