import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Accel
import csv
import math

class SimpleTracker(Node):
    def __init__(self):
        super().__init__('simple_tracker')

        # 1. 제어 파라미터 (이전 가변 로직 포함)
        self.max_v = 1.0
        self.min_v = 0.3
        self.k_ld = 0.1
        self.min_ld = 0.2
        self.v_gain = 0.5
        
        self.current_v = 0.5
        self.last_idx = 0 

        # 2. 완주 관리 변수 추가
        self.lap_count = 0
        self.max_laps = 5
        self.is_finished = False

        # 경로 로드
        self.path_x = []
        self.path_y = []
        self.csv_path = '/home/philoshan/Mobility_Challenge_Simulator/tool/p1_1_path.csv'
        self.load_path()

        # QoS 설정
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                         durability=DurabilityPolicy.VOLATILE, 
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        
        self.sub = self.create_subscription(PoseStamped, '/Ego_pose', self.pose_callback, qos)
        self.pub = self.create_publisher(Accel, '/Accel', 10)

        self.get_logger().info(f"Tracker Started. Goal: {self.max_laps} Laps.")

    def load_path(self):
        try:
            with open(self.csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader, None)
                for row in reader:
                    if row:
                        self.path_x.append(float(row[0]))
                        self.path_y.append(float(row[1]))
            self.get_logger().info(f"Path Loaded: {len(self.path_x)} points.")
        except Exception as e:
            self.get_logger().error(f"File Error: {e}")

    def pose_callback(self, msg):
        if not self.path_x or self.is_finished:
            return

        cx = msg.pose.position.x
        cy = msg.pose.position.y
        q = msg.pose.orientation
        cyaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

        # 가변 Ld 계산
        ld = (self.k_ld * self.current_v) + self.min_ld

        # 1. 목표점 찾기
        target_idx = self.last_idx
        for i in range(self.last_idx, len(self.path_x)):
            dist = math.hypot(self.path_x[i] - cx, self.path_y[i] - cy)
            if dist >= ld:
                target_idx = i
                break
        
        self.last_idx = target_idx

        # [핵심] 완주 판단 및 루프 로직
        # 마지막 인덱스 근처에 도달했는지 확인 (남은 점이 5개 미만일 때)
        if self.last_idx >= len(self.path_x) - 5:
            self.lap_count += 1
            self.get_logger().info(f"★ Lap {self.lap_count} Completed! ★")
            
            if self.lap_count >= self.max_laps:
                self.stop_robot()
                return
            else:
                # 다음 바퀴를 위해 인덱스 초기화
                self.last_idx = 0
                return

        # 2. 제어량 계산 (Alpha, Velocity, Omega)
        tx, ty = self.path_x[target_idx], self.path_y[target_idx]
        alpha = math.atan2(ty - cy, tx - cx) - cyaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        target_v = self.max_v * math.exp(-self.v_gain * abs(alpha))
        self.current_v = max(self.min_v, target_v)
        omega = (2.0 * self.current_v * math.sin(alpha)) / ld

        # 3. 명령 발행
        self.publish_cmd(self.current_v, omega)

    def publish_cmd(self, v, w):
        cmd = Accel()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)

    def stop_robot(self):
        self.is_finished = True
        self.publish_cmd(0.0, 0.0)
        self.get_logger().info("All laps completed. Stopping robot and exiting...")
        # 잠시 후 노드 종료를 유도 (선택 사항)
        # self.destroy_node() 

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()