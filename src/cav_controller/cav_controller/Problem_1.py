#!/usr/bin/env python3
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Accel


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class StanleyFollower(Node):
    """
    - CSV 웨이포인트(여러 구간)를 이어붙여 하나의 경로로 구성
    - /Ego_pose를 받아 Stanley로 조향각(delta) 계산
    - delta를 max_steer로 제한한 뒤, 자전거 모델로 omega로 변환
    - /Accel로 (v, omega) publish
    - yaw는 msg.pose.orientation.z 를 "이미 yaw"라고 가정하고 그대로 사용
    - (추가) 마지막 웨이포인트에 도달하면 last_idx를 0으로 리셋하여 무한 반복(랩 주행)
    """

    SEGMENT_ALIAS = {(1, 3): "1_3.csv"}  # 필요 없으면 {} 로 비워도 됩니다.

    def __init__(self):
        super().__init__("stanley_follower")

        # ===== Parameters =====
        self.declare_parameter("waypoint_dir", "tool/waypoint")

        # ✅ 18->21로 시작, 59->18로 돌아와서 한 바퀴(끝점=18)
        self.declare_parameter(
            "route_nodes",
            [18, 21, 51, 46, 40, 63, 34, 27, 31, 1, 3, 7, 9, 56, 59, 18],
        )

        # 주행/차량 파라미터
        self.declare_parameter("speed", 2.0)          # [m/s]
        self.declare_parameter("wheelbase", 0.211)    # [m] 앞/뒤 축거리(휠베이스)
        self.declare_parameter("L_front", 0.15)       # [m] 전륜점(가상) 오프셋
        self.declare_parameter("heading_lookahead", 3)

        # Stanley 파라미터
        self.declare_parameter("k_cte", 4.5)
        self.declare_parameter("eps", 0.5)
        self.declare_parameter("k_steer", 1.0)        # stanley_angle에 곱하는 조향 gain
        self.declare_parameter("max_steer", 0.9)      # [rad] 최대 조향각 (약 51.6도)

        # 최근접점 탐색 최적화
        self.declare_parameter("search_window", 400)
        self.declare_parameter("back_allow", 2)

        # ✅ 랩(한바퀴) 완료 판정 파라미터 (추가)
        self.declare_parameter("lap_finish_margin", 5)    # last_idx가 끝에서 몇 개 이내일 때만 랩 판정
        self.declare_parameter("lap_finish_dist", 0.25)   # [m] 마지막 웨이포인트에 이 거리 이내면 랩 완료

        waypoint_dir = str(self.get_parameter("waypoint_dir").value)
        self.route_nodes = [int(x) for x in list(self.get_parameter("route_nodes").value)]

        self.v = float(self.get_parameter("speed").value)
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.L_front = float(self.get_parameter("L_front").value)
        self.heading_lookahead = int(self.get_parameter("heading_lookahead").value)

        self.k_cte = float(self.get_parameter("k_cte").value)
        self.eps = float(self.get_parameter("eps").value)
        self.k_steer = float(self.get_parameter("k_steer").value)
        self.max_steer = float(self.get_parameter("max_steer").value)

        self.search_window = int(self.get_parameter("search_window").value)
        self.back_allow = int(self.get_parameter("back_allow").value)

        self.lap_finish_margin = int(self.get_parameter("lap_finish_margin").value)
        self.lap_finish_dist = float(self.get_parameter("lap_finish_dist").value)

        if len(self.route_nodes) < 2:
            raise ValueError("route_nodes는 최소 2개 이상 필요합니다. 예: [18, 21, ...]")

        if self.wheelbase <= 1e-6:
            raise ValueError("wheelbase는 0보다 커야 합니다. 예: 0.211")

        if self.max_steer <= 0.0:
            raise ValueError("max_steer는 0보다 커야 합니다. 예: 0.9")

        self.waypoint_dir = self._resolve_waypoint_dir(waypoint_dir)
        self.waypoints = self._load_route_points(self.waypoint_dir, self.route_nodes)
        if len(self.waypoints) < 2:
            raise RuntimeError("웨이포인트가 2개 미만입니다. CSV 내용을 확인하세요.")

        self.last_idx = 0
        self.lap_count = 0

        self.get_logger().info(
            f"Loaded WPs={len(self.waypoints)} | dir='{self.waypoint_dir}' | route_nodes={self.route_nodes}"
        )
        self.get_logger().info(
            f"v={self.v:.2f} wheelbase={self.wheelbase:.3f} max_steer={self.max_steer:.3f}rad({math.degrees(self.max_steer):.1f}deg)"
        )

        # ROS I/O
        self.pub_cmd = self.create_publisher(Accel, "/Accel", qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/Ego_pose", self.cb, qos_profile_sensor_data)

    # ----------------------------
    # Path helpers
    # ----------------------------
    def _resolve_waypoint_dir(self, waypoint_dir: str) -> Path:
        p = Path(waypoint_dir)
        if p.is_absolute() and p.exists():
            return p.resolve()

        candidates = []

        cwd = Path.cwd().resolve()
        for parent in [cwd] + list(cwd.parents)[:6]:
            candidates.append((parent / waypoint_dir).resolve())

        script_dir = Path(__file__).resolve().parent
        for parent in [script_dir] + list(script_dir.parents)[:8]:
            candidates.append((parent / waypoint_dir).resolve())

        for c in candidates:
            if c.exists() and c.is_dir():
                return c

        raise FileNotFoundError(f"waypoint_dir를 찾을 수 없습니다: '{waypoint_dir}' (cwd={cwd})")

    def _read_csv_points(self, csv_path: Path):
        """
        CSV (x,y) 로드 + NaN/inf 필터링
        """
        pts = []
        skipped = 0

        with csv_path.open("r", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    x = float(row[0].strip())
                    y = float(row[1].strip())
                except ValueError:
                    continue

                if (not math.isfinite(x)) or (not math.isfinite(y)):
                    skipped += 1
                    continue

                pts.append((x, y))

        if not pts:
            raise ValueError(f"CSV에 유효한 좌표가 없습니다: {csv_path}")

        if skipped > 0:
            self.get_logger().warn(f"Filtered non-finite rows: {skipped} (NaN/inf) in {csv_path.name}")

        return pts

    def _segment_csv_name(self, a: int, b: int) -> str:
        return self.SEGMENT_ALIAS.get((a, b), f"{a}_{b}.csv")

    def _load_route_points(self, waypoint_dir: Path, route_nodes):
        all_pts = []
        eps2 = 1e-12

        for i in range(len(route_nodes) - 1):
            a = route_nodes[i]
            b = route_nodes[i + 1]
            name = self._segment_csv_name(a, b)
            csv_path = waypoint_dir / name
            if not csv_path.exists():
                raise FileNotFoundError(f"구간 CSV가 없습니다: {csv_path}")

            pts = self._read_csv_points(csv_path)

            # 연결부 중복 제거
            if all_pts:
                lx, ly = all_pts[-1]
                fx, fy = pts[0]
                if (lx - fx) ** 2 + (ly - fy) ** 2 < eps2:
                    pts = pts[1:]

            all_pts.extend(pts)

        return all_pts

    # ----------------------------
    # Geometry helpers
    # ----------------------------
    def _nearest_index(self, x: float, y: float, start_idx: int) -> int:
        n = len(self.waypoints)

        if self.search_window <= 0:
            i0, i1 = 0, n - 1
        else:
            i0 = max(0, start_idx - self.back_allow)
            i1 = min(n - 1, start_idx + self.search_window)

        best_i = i0
        best_d2 = float("inf")
        for i in range(i0, i1 + 1):
            wx, wy = self.waypoints[i]
            dx = wx - x
            dy = wy - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        return best_i

    def _path_heading(self, idx: int) -> float:
        n = len(self.waypoints)
        idx = max(0, min(idx, n - 2))
        x1, y1 = self.waypoints[idx]
        x2, y2 = self.waypoints[idx + 1]
        return math.atan2(y2 - y1, x2 - x1)

    def _signed_cte_to_segment(self, idx: int, x: float, y: float) -> float:
        n = len(self.waypoints)
        idx = max(0, min(idx, n - 2))
        x0, y0 = self.waypoints[idx]
        x1, y1 = self.waypoints[idx + 1]

        sx, sy = (x1 - x0), (y1 - y0)
        seg_len2 = sx * sx + sy * sy
        if seg_len2 < 1e-12:
            return 0.0

        px, py = (x - x0), (y - y0)
        t = (px * sx + py * sy) / seg_len2
        t = 0.0 if t < 0.0 else 1.0 if t > 1.0 else t

        cx = x0 + t * sx
        cy = y0 + t * sy

        cross_z = sx * (y - y0) - sy * (x - x0)
        dist = math.hypot(x - cx, y - cy)
        return -dist if cross_z > 0.0 else +dist

    # ----------------------------
    # Lap helper (추가)
    # ----------------------------
    def _try_wrap_lap(self, xf: float, yf: float, idx: int) -> int:
        """
        마지막 웨이포인트 근처까지 왔으면 last_idx를 0으로 리셋해서 다시 시작(무한 반복)
        """
        n = len(self.waypoints)
        if n < 2:
            return idx

        if idx < (n - 1 - self.lap_finish_margin):
            return idx

        lx, ly = self.waypoints[-1]
        if math.hypot(lx - xf, ly - yf) > self.lap_finish_dist:
            return idx

        # 랩 완료 -> 리셋
        self.lap_count += 1
        self.last_idx = 0
        self.get_logger().info(f"LAP DONE -> restart (lap_count={self.lap_count})")

        # 리셋 직후 재탐색(즉시 18_21로 다시 붙게)
        return self._nearest_index(xf, yf, 0)

    # ----------------------------
    # Callback
    # ----------------------------
    def cb(self, msg: PoseStamped):
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        yaw = msg.pose.orientation.z  # "이미 yaw"라고 가정

        # 전륜점(가상)
        xf = cx + self.L_front * math.cos(yaw)
        yf = cy + self.L_front * math.sin(yaw)

        # nearest idx
        idx = self._nearest_index(xf, yf, self.last_idx)
        if idx < self.last_idx - self.back_allow:
            idx = self.last_idx - self.back_allow
        self.last_idx = idx

        # ✅ 랩 반복(추가)
        idx = self._try_wrap_lap(xf, yf, idx)
        self.last_idx = idx

        # heading error
        path_yaw = self._path_heading(idx + self.heading_lookahead)
        heading_err = wrap_to_pi(path_yaw - yaw)

        # cte
        cte = self._signed_cte_to_segment(idx, xf, yf)

        # Stanley 조향각(기본 형태)
        cte_term = math.atan2(self.k_cte * cte, (self.v + self.eps))
        stanley_angle = wrap_to_pi(heading_err + cte_term)

        # 조향 gain + 최대 조향각 제한
        delta_raw = self.k_steer * stanley_angle
        delta_cmd = clamp(delta_raw, -self.max_steer, +self.max_steer)

        # 자전거 모델로 omega 변환
        omega_cmd = (self.v / self.wheelbase) * math.tan(delta_cmd)

        cmd = Accel()
        cmd.linear.x = float(self.v)
        cmd.angular.z = float(omega_cmd)
        self.pub_cmd.publish(cmd)

        self.get_logger().info(
            f"idx={idx} ego=({cx:+.3f},{cy:+.3f}) yaw={yaw:+.3f} | "
            f"cte={cte:+.3f} head_err={heading_err:+.3f} stan={stanley_angle:+.3f} | "
            f"delta={delta_cmd:+.3f}rad({math.degrees(delta_cmd):+.1f}deg) v={self.v:.2f} w={omega_cmd:+.3f} | lap={self.lap_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StanleyFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
