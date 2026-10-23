#!/usr/bin/env python3
"""Lidar obstacle avoidance: picks the freest direction, slows near things,
reverses out of dead ends, turns on the spot when boxed in."""
import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# ---- Settings you can change ----
FORWARD_SPEED = 0.15    # m/s normal speed
MIN_SPEED = 0.06        # m/s when squeezing through or close to something
BACK_SPEED = -0.10      # m/s when reversing out of a dead end
TURN_SPEED = 0.6        # rad/s maximum turning rate
HALF_WIDTH = 0.16       # m: robot half width (0.10) + safety margin (0.06)
LOOKAHEAD = 1.2         # m: how far ahead we look
STOP_CLEAR = 0.28       # m: nearer than this straight ahead = blocked
TURN_EXIT = 0.70        # m: stop turning in place when the way ahead is this clear
BACK_MIN_REAR = 0.25    # m: stop reversing when this close behind
BACK_MAX_TIME = 2.5     # s: never reverse longer than this
TURN_MAX_TIME = 8.0     # s: never turn in place longer than this
WANDER = 0.30           # how strongly it wanders (0 = only reacts to obstacles)

ANGLES = [math.radians(a) for a in range(-180, 181, 10)]


def to_points(ranges, angle_min, inc, rmin, rmax):
    pts = []
    for i, r in enumerate(ranges):
        if math.isfinite(r) and rmin <= r <= rmax:
            a = angle_min + i * inc
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def clear_along(points, theta, cap=LOOKAHEAD):
    """Free distance along heading theta for a corridor as wide as the robot."""
    c, s = math.cos(theta), math.sin(theta)
    best = cap
    for x, y in points:
        l = x * c + y * s
        if l <= 0.0:
            continue
        w = -x * s + y * c
        if abs(w) < HALF_WIDTH and l < best:
            best = l
    return best


class Avoider:
    def __init__(self):
        self.mode = 'DRIVE'
        self.t_mode = 0.0
        self.last_sign = 0
        self.turn_dir = 1
        self.wander = 0.0           # slow random preference for turning left or right
        self.wander_until = 0.0

    def _set(self, mode, now):
        self.mode = mode
        self.t_mode = now

    def update(self, ranges, angle_min, inc, rmin, rmax, now):
        """Return (linear, angular) for one lidar scan."""
        pts = to_points(ranges, angle_min, inc, rmin, rmax)
        front = clear_along(pts, 0.0)

        if self.mode == 'BACKUP':
            rear = clear_along(pts, math.pi)
            if rear < BACK_MIN_REAR or now - self.t_mode > BACK_MAX_TIME:
                self._pick_turn(pts)
                self._set('TURN', now)
                return 0.0, self.turn_dir * TURN_SPEED
            return BACK_SPEED, 0.0

        if self.mode == 'TURN':
            if front > TURN_EXIT or now - self.t_mode > TURN_MAX_TIME:
                self._set('DRIVE', now)
            else:
                return 0.0, self.turn_dir * TURN_SPEED

        # ---- DRIVE: pick the best heading in front of us ----
        if now > self.wander_until:           # every 10-25 s pick a new gentle bias
            self.wander = random.choice([-1.0, 0.0, 1.0]) * WANDER
            self.wander_until = now + random.uniform(10.0, 25.0)
        best_theta, best_score, best_clear = 0.0, -1e9, 0.0
        for th in ANGLES:
            if abs(th) > math.radians(90):
                continue
            c = clear_along(pts, th)
            score = c - 0.35 * abs(th) + self.wander * th
            if self.last_sign and th * self.last_sign > 0:
                score += 0.10           # small preference to keep the same side
            if score > best_score:
                best_theta, best_score, best_clear = th, score, c
        if front >= LOOKAHEAD * 0.95 and self.wander == 0.0:
            best_theta, best_clear = 0.0, front   # open ahead: go straight

        if best_clear < STOP_CLEAR:
            rear = clear_along(pts, math.pi)
            if rear > BACK_MIN_REAR + 0.1:
                self._set('BACKUP', now)
                return BACK_SPEED, 0.0
            self._pick_turn(pts)
            self._set('TURN', now)
            return 0.0, self.turn_dir * TURN_SPEED

        self.last_sign = 0 if abs(best_theta) < 1e-6 else (1 if best_theta > 0 else -1)
        ang = max(-TURN_SPEED, min(TURN_SPEED, 1.5 * best_theta))
        if abs(best_theta) > math.radians(50):
            return 0.0, math.copysign(TURN_SPEED, best_theta)     # turn on the spot first
        speed = FORWARD_SPEED * max(0.4, min(1.0, (best_clear - STOP_CLEAR) / 0.6))
        if best_clear < LOOKAHEAD * 0.6:
            speed = min(speed, FORWARD_SPEED)
        speed = max(speed, MIN_SPEED)
        return speed, ang

    def _pick_turn(self, pts):
        best_th, best_c = 0.0, -1.0
        for th in ANGLES:
            if abs(th) < math.radians(30):
                continue
            c = clear_along(pts, th)
            if c > best_c + 1e-9:
                best_th, best_c = th, c
        self.turn_dir = 1 if best_th >= 0 else -1


class AvoiderNode(Node):
    def __init__(self):
        super().__init__('avoider')
        self.brain = Avoider()
        self.last_mode = ''
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.get_logger().info('Avoider started. Press Ctrl+C to stop.')

    def on_scan(self, scan):
        now = self.get_clock().now().nanoseconds * 1e-9
        lin, ang = self.brain.update(scan.ranges, scan.angle_min,
                                     scan.angle_increment,
                                     scan.range_min, scan.range_max, now)
        if self.brain.mode != self.last_mode:
            self.last_mode = self.brain.mode
            self.get_logger().info('Mode: ' + self.last_mode)
        cmd = Twist()
        cmd.linear.x = float(lin)
        cmd.angular.z = float(ang)
        self.pub.publish(cmd)

    def stop(self):
        self.pub.publish(Twist())


def main():
    rclpy.init()
    node = AvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
