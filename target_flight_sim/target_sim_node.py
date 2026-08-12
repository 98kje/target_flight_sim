#!/usr/bin/env python3
"""target_sim_node — 표적 기체 비행 시뮬레이터.

Publishes
─────────
  /target/global_position [sensor_msgs/NavSatFix]     lat/lon/alt, @ update_hz
  /target/velocity        [geometry_msgs/TwistStamped] ENU 속도 [m/s], @ update_hz
  /target/status          [std_msgs/String]           "EN_ROUTE"/"ARRIVED" 등 (상태 전이 시 1회)

목표 도달 후 동작은 on_arrival 파라미터로 결정:
  'hold'    : 목표점에서 정지, 계속 위치/속도(0) 발행
  'loop'    : loop_pause_s 만큼 대기 후 출발점으로 복귀해 재출발 (반복 시나리오)
  'reverse' : 도착 즉시 반대 방향(목표→출발)으로 왕복 비행
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, String

from target_flight_sim.geo import enu_to_llh, llh_to_enu


class TargetSimNode(Node):

    def __init__(self):
        super().__init__('target_sim')

        self.declare_parameter('update_hz',      10.0)
        self.declare_parameter('start_lat_deg',  37.0084494)
        self.declare_parameter('start_lon_deg', 127.9466327)
        self.declare_parameter('start_alt_m',     30.0)
        self.declare_parameter('goal_lat_deg',   37.0040055)
        self.declare_parameter('goal_lon_deg',  127.9215439)
        self.declare_parameter('goal_alt_m',      30.0)
        self.declare_parameter('speed_mps',       12.5)
        self.declare_parameter('on_arrival',     'hold')   # hold | loop | reverse
        self.declare_parameter('loop_pause_s',     3.0)

        self._hz         = float(self.get_parameter('update_hz').value)
        self._speed      = float(self.get_parameter('speed_mps').value)
        self._on_arrival = str(self.get_parameter('on_arrival').value)
        self._pause_s    = float(self.get_parameter('loop_pause_s').value)

        start_lat = self.get_parameter('start_lat_deg').value
        start_lon = self.get_parameter('start_lon_deg').value
        start_alt = self.get_parameter('start_alt_m').value
        goal_lat  = self.get_parameter('goal_lat_deg').value
        goal_lon  = self.get_parameter('goal_lon_deg').value
        goal_alt  = self.get_parameter('goal_alt_m').value

        # 출발점을 ENU 원점.
        self._ref = (start_lat, start_lon, start_alt)
        self._start_enu = np.zeros(3)
        self._goal_enu  = llh_to_enu(goal_lat, goal_lon, goal_alt, *self._ref)

        leg = self._goal_enu - self._start_enu
        self._dist_total = float(np.linalg.norm(leg))
        self._unit = leg / self._dist_total if self._dist_total > 1e-6 \
            else np.array([1.0, 0.0, 0.0])

        self._dist      = 0.0     # 출발점 기준 진행 거리 [0, dist_total]
        self._dir_sign   = 1.0    # +1: 출발→목표, -1: 목표→출발 (reverse 모드)
        self._state      = 'EN_ROUTE'
        self._arrived_at = None

        self._pub_pos = self.create_publisher(NavSatFix, '/target/global_position', 10)
        self._pub_vel = self.create_publisher(TwistStamped, '/target/velocity', 10)
        self._pub_sta = self.create_publisher(String, '/target/status', 10)

        self.get_logger().info(
            f'[TargetSim] start=({start_lat:.6f},{start_lon:.6f},{start_alt:.1f}) '
            f'goal=({goal_lat:.6f},{goal_lon:.6f},{goal_alt:.1f}) '
            f'dist={self._dist_total:.0f}m speed={self._speed:.1f}m/s '
            f'on_arrival={self._on_arrival}')

        self.create_timer(1.0 / self._hz, self._tick)

    def _publish_status(self, text: str):
        self._pub_sta.publish(String(data=text))
        self.get_logger().info(f'[TargetSim] {text}')

    def _tick(self):
        dt = 1.0 / self._hz

        if self._state == 'PAUSED':
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._arrived_at >= self._pause_s:
                self._dist = 0.0
                self._dir_sign = 1.0
                self._state = 'EN_ROUTE'
                self._publish_status('EN_ROUTE')
            self._publish(vel_scale=0.0)
            return

        if self._state != 'ARRIVED':
            self._dist += self._dir_sign * self._speed * dt

            if self._dir_sign > 0 and self._dist >= self._dist_total:
                self._dist = self._dist_total
                if self._on_arrival == 'hold':
                    self._state = 'ARRIVED'
                elif self._on_arrival == 'loop':
                    self._state = 'PAUSED'
                    self._arrived_at = self.get_clock().now().nanoseconds * 1e-9
                elif self._on_arrival == 'reverse':
                    self._dir_sign = -1.0
                self._publish_status('ARRIVED')

            elif self._dir_sign < 0 and self._dist <= 0.0:
                self._dist = 0.0
                self._dir_sign = 1.0
                self._publish_status('EN_ROUTE')

        vel_scale = 0.0 if self._state == 'ARRIVED' else self._dir_sign
        self._publish(vel_scale=vel_scale)

    def _publish(self, vel_scale: float):
        pos_enu = self._start_enu + self._unit * self._dist
        lat, lon, alt = enu_to_llh(*pos_enu, *self._ref)
        stamp = self.get_clock().now().to_msg()

        pos_msg = NavSatFix()
        pos_msg.header = Header(stamp=stamp, frame_id='wgs84')
        pos_msg.latitude  = lat
        pos_msg.longitude = lon
        pos_msg.altitude  = alt
        self._pub_pos.publish(pos_msg)

        vel = self._unit * self._speed * vel_scale
        vel_msg = TwistStamped()
        vel_msg.header = Header(stamp=stamp, frame_id='enu')
        vel_msg.twist.linear.x = float(vel[0])   # East
        vel_msg.twist.linear.y = float(vel[1])   # North
        vel_msg.twist.linear.z = float(vel[2])   # Up
        self._pub_vel.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
