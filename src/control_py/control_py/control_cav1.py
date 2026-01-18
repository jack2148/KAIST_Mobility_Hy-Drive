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
        
        # Ego_pose 구독
        self.subscription = self.create_subscription(
            PoseStamped, '/Ego_pose', self.listener_callback, qos_profile_sensor_data)
        
        # 제어 명령 발행
        self.publisher_ = self.create_publisher(Accel, '/Accel', qos_profile_sensor_data)
        
        # CSV 경로 파일 로드
        csv_path = '/home/ym/Mobility_Challenge_Simulator/tool/ablean_cav1.csv'
        df = pd.read_csv(csv_path)
        self.waypoints = df[['X', 'Y', 'Lean']].to_numpy()
        
        # === Stanley 파라미터 ===
        self.k = 2.0         # 횡방향 오차 게인
        self.k_soft = 1.0    # 저속 주행 시 분모 0 방지 상수
        self.max_steer = np.deg2rad(45)  # 최대 조향각 (45도로 상향 조정 추천)
        self.wheelbase = 0.33 # 축거 (차량 전륜-후륜 거리, m)
        
        # === 속도 제어 파라미터 ===
        self.max_speed = 0.3  # 직진 시 최대 속도
        self.min_speed = 0.1  # 급커브 시 최소 속도 (이하로 떨어지면 정지할 수 있으니 주의)
        self.steer_threshold_low = np.deg2rad(10) # 이 각도 이하면 최대 속도
        self.steer_threshold_high = np.deg2rad(25) # 이 각도 이상이면 최소 속도

        # 경로 탐색 최적화를 위한 인덱스 저장
        self.last_nearest_idx = 0

    def listener_callback(self, msg):
        # 1. 현재 차량 상태 추출
        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_yaw = msg.pose.orientation.z
        
        # 임시 속도 (조향각 계산용, 실제 속도는 조향각 계산 후 결정)
        # 스탠리 수식의 분모(v)에 들어갈 값은 너무 작으면 발산하므로 최소값 보장
        calc_vel = max(self.min_speed, 1.0) 

        # 2. 스탠리 알고리즘 수행 (필요한 조향각 계산)
        raw_steer_angle, target_idx, error_front_axle = self.stanley_control(current_x, current_y, current_yaw, calc_vel)

        # 3. [핵심] 조향각 기반 적응형 속도 결정
        target_vel = self.calculate_adaptive_speed(raw_steer_angle)

        # 4. 최종 제어 명령 생성
        # 조향각 제한 적용 (최종 명령용)
        final_steer = np.clip(raw_steer_angle, -self.max_steer, self.max_steer)
        
        # 조향각(delta) -> 각속도(wz) 변환
        # 속도가 줄어들면 동일한 조향각이어도 요구되는 각속도(wz)는 줄어듦
        wz = (target_vel / self.wheelbase) * math.tan(final_steer)
        wz = np.clip(wz, -3.0, 3.0) 

        accel_msg = Accel()
        accel_msg.linear.x = float(target_vel)
        accel_msg.angular.z = float(wz)
        
        self.publisher_.publish(accel_msg)
        
        # 5. 상태 로깅
        self.get_logger().info(
            f'Idx:{target_idx} | ERR:{error_front_axle:.2f} | '
            f'Steer:{math.degrees(final_steer):.1f}° | Vel:{target_vel:.2f} m/s'
        )

    def calculate_adaptive_speed(self, steer_angle):
        """ 조향각 크기에 따라 속도를 조절하는 함수 """
        abs_steer = abs(steer_angle)
        
        if abs_steer < self.steer_threshold_low:
            return self.max_speed
        elif abs_steer > self.steer_threshold_high:
            return self.min_speed
        else:
            # 선형 보간 (Linear Interpolation)
            ratio = (abs_steer - self.steer_threshold_low) / (self.steer_threshold_high - self.steer_threshold_low)
            return self.max_speed - ratio * (self.max_speed - self.min_speed)

    def normalize_angle(self, angle):
        """ 각도를 -pi ~ pi 범위로 정규화 """
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def stanley_control(self, cx, cy, cyaw, cv):
        """
        Stanley Steering Control Algorithm
        """
        # 1. 앞바퀴 중심 좌표 계산
        fx = cx + self.wheelbase * math.cos(cyaw)
        fy = cy + self.wheelbase * math.sin(cyaw)

        # 2. [수정됨] 가장 가까운 경로점(Nearest Point) 찾기
        # 이전 인덱스 기준 앞뒤로 넉넉하게(예: 50개) 탐색하여 놓치는 일이 없도록 함
        search_window = 50 
        
        start_idx = max(self.last_nearest_idx - 10, 0) # 지나온 길은 조금만 탐색
        end_idx = min(self.last_nearest_idx + search_window, len(self.waypoints)) # 앞쪽 길은 넓게 탐색
        
        # 만약 경로가 끝에 다다랐다면 전체 범위를 다시 확인하거나(순환형), 끝 점을 유지
        if self.last_nearest_idx >= len(self.waypoints) - 1:
             start_idx = max(len(self.waypoints) - 50, 0)
             end_idx = len(self.waypoints)

        dx_list = [fx - wx for wx in self.waypoints[start_idx:end_idx, 0]]
        dy_list = [fy - wy for wy in self.waypoints[start_idx:end_idx, 1]]
        d_list = np.hypot(dx_list, dy_list)
        
        # 탐색된 구간 내에서의 최소값 인덱스
        min_idx = np.argmin(d_list)
        
        # 실제 전체 경로 상의 인덱스로 변환
        nearest_idx = start_idx + min_idx
        self.last_nearest_idx = nearest_idx

        # [Lookahead 적용] (기존 코드 유지)
        lookahead_offset = 3 
        target_idx = min(nearest_idx + lookahead_offset, len(self.waypoints) - 1)
        
        # ... (이하 로직 동일)


        # 3. 횡방향 오차(Cross Track Error) 계산 (Target Point 기준)
        map_x = self.waypoints[target_idx][0]
        map_y = self.waypoints[target_idx][1]
        map_yaw = self.waypoints[target_idx][2] 

        # 오차 거리 (여기서는 nearest distance를 쓰거나 target distance를 쓸 수 있음. Lookahead 효과를 위해 target 사용 시 오차가 커 보일 수 있음)
        # 안정성을 위해 오차 크기 자체는 nearest point 기준으로 하되, 방향만 target을 볼 수도 있음.
        # 여기서는 간단히 target point 기준으로 계산합니다.
        dx = fx - map_x
        dy = fy - map_y
        error_front_axle = math.hypot(dx, dy)

        # 오차 부호 결정
        path_vec_x = math.cos(map_yaw)
        path_vec_y = math.sin(map_yaw)
        
        # 수직 성분 내적
        if (path_vec_x * dy - path_vec_y * dx) > 0:
            error_front_axle = -error_front_axle 
        else:
            error_front_axle = error_front_axle

        # 4. 스탠리 조향 법칙
        # (1) 헤딩 오차
        theta_e = self.normalize_angle(map_yaw - cyaw)
        
        # (2) 횡방향 오차 보정항
        theta_d = math.atan2(self.k * error_front_axle, cv + self.k_soft)
        
        # (3) 최종 조향각 (제한 전 Raw 값 반환)
        delta = theta_e + theta_d

        return delta, target_idx, error_front_axle

def main(args=None):
    rclpy.init(args=args)
    node = PathTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
