#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('f1tenth_reactive_obs')

        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.current_gap = None

        # Publicadores y Suscriptores
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, qos_sensor)

        # Telemetría obligatoria cada 5 segundos (Independiente de SHOW_LOGS)
        self.telemetry_timer = self.create_timer(5.0, self._telemetry_callback)

        # Configuración de Logs Internos del Algoritmo
        self.SHOW_LOGS = False
        self.LOG_PERIOD_SECONDS = 0.5
        self.last_log_time = self.get_clock().now()

        # --- Parámetros de Follow the Gap ---
        self.PREPROCESS_MAX_DIST = 10.0
        self.DISPARITY_THRESHOLD = 0.8
        self.CAR_WIDTH_METERS = 0.40
        self.SAFETY_MARGIN_METERS = 0.19
        self.CAR_RADIUS_METERS = (self.CAR_WIDTH_METERS / 2.0) + self.SAFETY_MARGIN_METERS
        self.BUBBLE_TIE_MARGIN_METERS = 0.15
        self.MAX_BUBBLE_ANGLE_RAD = 0.70
        self.BEST_POINT_CENTER_BLEND = 0.18  
        self.prev_steering_angle = 0.0
        self.GAP_TRACK_TOLERANCE_INDICES = 15
        self.MAX_STEER_DELTA_RAD = 0.2
        self.EMERGENCY_MAX_STEER_DELTA_RAD = 0.45
        self.MIN_ANGLE_RAD = -1.22
        self.MAX_ANGLE_RAD = 1.22
        self.MAX_TARGET_ANGLE_RAD = 0.9
        self.MAX_TIE_TOLERANCE_METERS = 0.30
        self.GAP_CONTINUITY_WEIGHT = 0.6   
        self.GAP_SIZE_TIE_TOLERANCE_INDICES = 35
        self.GAP_EDGE_MARGIN_MIN_INDICES = 6
        self.MIN_VIABLE_GAP_INDICES = 20

        # Parámetros de Velocidad Dinámica
        self.DIST_SPEED_CAPS = [
            (0.5, 1.4),
            (0.8, 2.6),
            (1.2, 4.2),
        ]
        self.FRONT_CONE_HALF_ANGLE_RAD = 0.25
        self.MID_CONE_HALF_ANGLE_RAD = 0.60
        self.FRONT_DIST_SPEED_CAPS = [
            (0.65, 1.2),
            (1.20, 2.5),
            (2.00, 4.0),
            (3.00, 5.5),
        ]
        self.FRONT_EMERGENCY_DIST = 0.65
        self.EMERGENCY_MIN_SPEED = 0.15
        self.MAX_SPEED = 6.0

        self.MAX_LAPS = 10
        self.LAP_ORIGIN_RADIUS = 2.0
        self.lap_count = 0
        self.lap_start_time = None
        self.lap_times = []
        self.race_finished = False

        self.origin_set = False
        self.origin_x = None
        self.origin_y = None
        self.max_dist_from_origin = 0.0

        self.current_speed = 0.0

        self.get_logger().info(
            '\n🏎️   F1Tenth Reactive Follow Gap Inicializado \n'
            f'Objetivo: {self.MAX_LAPS} vueltas aplicando evasión de obstáculos dinámicos\n'
        )

    def _effective_car_radius(self):
        speed_factor = 1.0 + (self.current_speed / self.MAX_SPEED) * 0.5
        return self.CAR_RADIUS_METERS * speed_factor

    def odom_callback(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

        if not self.origin_set:
            self.origin_x = px
            self.origin_y = py
            self.origin_set = True
            self.lap_start_time = time.monotonic()
            self.get_logger().info(
                f'\n📍  Origen de carrera fijado en: ({px:.2f}, {py:.2f})\n'
            )
            return

        if self.race_finished:
            return

        dist_to_origin = math.hypot(px - self.origin_x, py - self.origin_y)
        if dist_to_origin > self.max_dist_from_origin:
            self.max_dist_from_origin = dist_to_origin

        now = time.monotonic()

        if (self.max_dist_from_origin > 8.0 
                and dist_to_origin < self.LAP_ORIGIN_RADIUS):

            lap_duration = now - self.lap_start_time
            self.lap_times.append(lap_duration)
            self.lap_count += 1
            self.lap_start_time = now
            self.max_dist_from_origin = 0.0

            self.get_logger().info(
                f'\n\n🏁  ¡VUELTA {self.lap_count}/{self.MAX_LAPS} completada!'
                f'\nTiempo de Vuelta: {lap_duration:.3f}s'
            )

            if self.lap_count >= self.MAX_LAPS:
                self.race_finished = True
                self.get_logger().info('\n🏁 ¡CARRERA COMPLETADA!')
                resumen = "\nRESUMEN DE LA CARRERA:\n"
                for idx, t in enumerate(self.lap_times):
                    resumen += f"• Vuelta {idx + 1}: {t:.3f}s\n"
                resumen += f"🏆 Mejor Vuelta: {min(self.lap_times):.3f}s\n"
                self.get_logger().info(resumen)

                self.publish_drive(0.0, 0.0)

    def _telemetry_callback(self):
        if self.race_finished or self.lap_start_time is None:
            return
        now = time.monotonic()
        elapsed = now - self.lap_start_time
        self.get_logger().info(
            f'\n[TEL] Lap: {self.lap_count + 1} | T.Actual: {elapsed:.2f}s | V.Actual: {self.current_speed:.2f} m/s'
        )

    def scan_callback(self, scan_msg):
        if self.race_finished:
            self.publish_drive(0.0, 0.0)
            return

        start_index = int((self.MIN_ANGLE_RAD - scan_msg.angle_min) / scan_msg.angle_increment)
        end_index = int((self.MAX_ANGLE_RAD - scan_msg.angle_min) / scan_msg.angle_increment)

        start_index = max(0, start_index)
        end_index = min(len(scan_msg.ranges) - 1, end_index)

        proc_ranges = list(scan_msg.ranges[start_index:end_index + 1])
        total_samples = len(proc_ranges)

        for i in range(len(proc_ranges)):
            if np.isnan(proc_ranges[i]) or np.isinf(proc_ranges[i]):
                proc_ranges[i] = 0.0
            elif proc_ranges[i] > self.PREPROCESS_MAX_DIST:
                proc_ranges[i] = self.PREPROCESS_MAX_DIST

        raw_ranges = list(proc_ranges)

        disparities_detected = 0
        for i in range(len(proc_ranges) - 1):
            diff = proc_ranges[i + 1] - proc_ranges[i]
            if abs(diff) > self.DISPARITY_THRESHOLD:
                disparities_detected += 1
                near_idx = i if diff > 0 else i + 1
                near_dist = max(proc_ranges[near_idx], 0.1)

                alpha_inflado = min(
                    math.atan2(self._effective_car_radius(), near_dist),
                    self.MAX_BUBBLE_ANGLE_RAD
                )
                indices_a_cambiar = int(math.ceil(alpha_inflado / scan_msg.angle_increment))

                direction = 1 if diff > 0 else -1
                for j in range(1, indices_a_cambiar + 1):
                    target_idx = near_idx + (direction * j)
                    if 0 <= target_idx < len(proc_ranges):
                        if proc_ranges[target_idx] > proc_ranges[near_idx]:
                            proc_ranges[target_idx] = proc_ranges[near_idx]

        min_distance = min(raw_ranges)

        front_start_idx = max(0, int((-self.FRONT_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        front_end_idx = min(len(raw_ranges) - 1, int((self.FRONT_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        front_section = raw_ranges[front_start_idx:front_end_idx + 1]
        min_distance_front = min(front_section) if front_section else min_distance

        mid_start_idx = max(0, int((-self.MID_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        mid_end_idx = min(len(raw_ranges) - 1, int((self.MID_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        mid_section = raw_ranges[mid_start_idx:mid_end_idx + 1]
        min_distance_mid = min(mid_section) if mid_section else min_distance

        if min_distance_front <= self.FRONT_EMERGENCY_DIST:
            steer = self.escape_direction(raw_ranges)
            delta = steer - self.prev_steering_angle
            delta = max(-self.EMERGENCY_MAX_STEER_DELTA_RAD, min(self.EMERGENCY_MAX_STEER_DELTA_RAD, delta))
            target_angle = self.prev_steering_angle + delta
            target_angle = max(-self.MAX_TARGET_ANGLE_RAD, min(self.MAX_TARGET_ANGLE_RAD, target_angle))
            self.prev_steering_angle = target_angle
            emergency_speed = max(self.EMERGENCY_MIN_SPEED, min(0.4, (min_distance_front / self.FRONT_EMERGENCY_DIST) * 0.4))
            self.publish_drive(emergency_speed, target_angle)
            return

        if min_distance > 0.0:
            alpha_bubble = min(math.atan2(self._effective_car_radius(), min_distance), self.MAX_BUBBLE_ANGLE_RAD)
            bubble_width_indices = int(math.ceil(alpha_bubble / scan_msg.angle_increment))
            margin_indices = int(total_samples * 0.10) 
            
            near_indices = [
                i for i, d in enumerate(raw_ranges)
                if d <= min_distance + self.BUBBLE_TIE_MARGIN_METERS and margin_indices < i < (total_samples - margin_indices)
            ]
            
            for near_idx in near_indices:
                start_bubble = max(0, near_idx - bubble_width_indices)
                end_bubble = min(len(proc_ranges), near_idx + bubble_width_indices + 1)
                for j in range(start_bubble, end_bubble):
                    proc_ranges[j] = 0.0

        prev_idx = int(round((self.prev_steering_angle - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        max_start, max_end = self.find_max_gap(proc_ranges, prev_idx)
        gap_size = max_end - max_start

        if gap_size < self.MIN_VIABLE_GAP_INDICES:
            steer = self.escape_direction(raw_ranges)
            delta = steer - self.prev_steering_angle
            delta = max(-self.MAX_STEER_DELTA_RAD, min(self.MAX_STEER_DELTA_RAD, delta))
            target_angle = self.prev_steering_angle + delta
            self.prev_steering_angle = target_angle
            fallback_speed = max(self.EMERGENCY_MIN_SPEED, min(1.0, min_distance))
            self.publish_drive(fallback_speed, target_angle)
            return

        start_neighbor_idx = max(0, max_start - 1)
        end_neighbor_idx = min(len(raw_ranges) - 1, max_end + 1)

        start_margin_indices = max(self.GAP_EDGE_MARGIN_MIN_INDICES, self._dynamic_edge_margin(raw_ranges[start_neighbor_idx], scan_msg.angle_increment))
        end_margin_indices = max(self.GAP_EDGE_MARGIN_MIN_INDICES, self._dynamic_edge_margin(raw_ranges[end_neighbor_idx], scan_msg.angle_increment))

        trimmed_start = max_start + start_margin_indices
        trimmed_end = max_end - end_margin_indices
        if trimmed_start > trimmed_end:
            trimmed_start, trimmed_end = max_start, max_end

        gap_section = proc_ranges[trimmed_start:trimmed_end + 1]
        if len(gap_section) == 0:
            self.publish_drive(0.5, -0.4)
            return

        max_val = max(gap_section)
        candidate_indices = [idx for idx, v in enumerate(gap_section) if v >= max_val - self.MAX_TIE_TOLERANCE_METERS]
        best_idx_in_gap = int(round(sum(candidate_indices) / len(candidate_indices)))
        best_idx_global = trimmed_start + best_idx_in_gap

        gap_center_idx = (trimmed_start + trimmed_end) / 2.0
        best_idx_global = int(round((1 - self.BEST_POINT_CENTER_BLEND) * best_idx_global + self.BEST_POINT_CENTER_BLEND * gap_center_idx))
        best_idx_global = max(trimmed_start, min(trimmed_end, best_idx_global))

        raw_target_angle = self.MIN_ANGLE_RAD + (best_idx_global * scan_msg.angle_increment)

        delta = raw_target_angle - self.prev_steering_angle
        delta = max(-self.MAX_STEER_DELTA_RAD, min(self.MAX_STEER_DELTA_RAD, delta))
        target_angle = self.prev_steering_angle + delta
        target_angle = max(-self.MAX_TARGET_ANGLE_RAD, min(self.MAX_TARGET_ANGLE_RAD, target_angle))
        self.prev_steering_angle = target_angle

        target_velocity = self.compute_velocity(target_angle, min_distance_mid, min_distance_front)

        if self.SHOW_LOGS:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.last_log_time).nanoseconds / 1e9

            if elapsed_time >= self.LOG_PERIOD_SECONDS:
                self.get_logger().info(
                    f"\n--- TELEMETRÍA REACTIVA [Vuelta: {self.lap_count}/{self.MAX_LAPS}] ---\n"
                    f" >> COMANDO DE CONTROL: Ángulo Giro: {target_angle:.3f} rad | Velocidad: {target_velocity:.1f} m/s\n"
                    f"--------------------------------------------------"
                )
                self.last_log_time = current_time

        self.publish_drive(target_velocity, target_angle)

    def compute_velocity(self, target_angle, min_distance_mid, min_distance_front):
        if abs(target_angle) > 0.5:
            v_angle = 2.0
        elif abs(target_angle) > 0.30:
            v_angle = 3.5
        elif abs(target_angle) > 0.15:
            v_angle = 5.0
        else:
            v_angle = self.MAX_SPEED

        v_dist = self.MAX_SPEED
        for dist_thresh, speed_cap in self.DIST_SPEED_CAPS:
            if min_distance_mid < dist_thresh:
                v_dist = speed_cap
                break

        v_dist_front = self.MAX_SPEED
        for dist_thresh, speed_cap in self.FRONT_DIST_SPEED_CAPS:
            if min_distance_front < dist_thresh:
                v_dist_front = speed_cap
                break

        base_speed = min(v_angle, v_dist, v_dist_front)

        if (abs(target_angle) < 0.05
                and min_distance_front > 4.0
                and min_distance_mid > 3.0):
            return min(self.MAX_SPEED + 1.0, v_dist, v_dist_front)

        return base_speed

    def _dynamic_edge_margin(self, obstacle_dist, angle_increment):
        obstacle_dist = max(obstacle_dist, 0.1)
        alpha = min(math.atan2(self._effective_car_radius(), obstacle_dist), self.MAX_BUBBLE_ANGLE_RAD)
        return int(math.ceil(alpha / angle_increment))

    def escape_direction(self, ranges):
        mid = len(ranges) // 2
        right_space = sum(ranges[:mid])
        left_space = sum(ranges[mid:])
        return 0.4 if left_space > right_space else -0.4

    def _gap_score(self, gap, total_len):
        size = gap[1] - gap[0]
        center = (gap[0] + gap[1]) / 2.0
        lateral_offset_indices = abs(center - total_len / 2.0)
        return size - self.GAP_CONTINUITY_WEIGHT * lateral_offset_indices

    def find_max_gap(self, ranges, prev_idx=None):
        gaps = []
        current_start = None

        for i in range(len(ranges)):
            if ranges[i] > 0.1:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    gaps.append((current_start, i - 1))
                    current_start = None

        if current_start is not None:
            gaps.append((current_start, len(ranges) - 1))

        if not gaps:
            return 0, 0

        total_len = len(ranges)
        biggest = max(gaps, key=lambda g: self._gap_score(g, total_len))

        if prev_idx is None or self.current_gap is None:
            self.current_gap = biggest
            return biggest

        tracked = None
        for g in gaps:
            if (g[0] - self.GAP_TRACK_TOLERANCE_INDICES) <= prev_idx <= (g[1] + self.GAP_TRACK_TOLERANCE_INDICES):
                tracked = g
                break

        if tracked is None:
            self.current_gap = biggest
            return biggest

        tracked_size = tracked[1] - tracked[0]
        biggest_score = self._gap_score(biggest, total_len)

        if biggest_score > tracked_size + self.GAP_SIZE_TIE_TOLERANCE_INDICES:
            self.current_gap = biggest
            return biggest

        self.current_gap = tracked
        return tracked

    def publish_drive(self, velocity, steering_angle):
        self.current_speed = velocity
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.speed = velocity
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.publish_drive(0.0, 0.0)
        reactive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
