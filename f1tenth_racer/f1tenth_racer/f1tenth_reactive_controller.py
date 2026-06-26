import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

MAX_LAPS          = 10         
BASE_SPEED        = 5.2        
SPEED_INCREMENT   = 1.0   
MAX_SPEED         = 12.0    
ABSOLUTE_MAX_SPEED = 10.0

BUBBLE_RADIUS     = 0.45      
MIN_GAP_WIDTH_DEG = 12.0 
SCAN_FIELD_DEG    = 180.0  

STEERING_GAIN     = 0.65   
MAX_STEER_RAD     = 0.42   
SPEED_STEER_DECAY = 0.40  

LAP_ORIGIN_RADIUS = 2.0  
LAP_COOLDOWN_S    = 3.0

class F1TenthReactiveController(Node):

    def __init__(self):
        super().__init__('f1tenth_reactive_controller')

        self.lap_count        = 0
        self.race_finished    = False

        self.origin_set       = False
        self.origin_x         = 0.0
        self.origin_y         = 0.0

        self.max_dist_from_origin = 0.0   
        self.last_lap_time_s  = None
        self.lap_start_time   = time.monotonic()
        self.last_lap_trigger = -LAP_COOLDOWN_S

        self.last_steering_angle = 0.0
        self.steering_smoothing   = 0.65  
        
        self.lap_times        = []
        self.current_speed    = 0.0

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_sensor
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_callback, qos_sensor
        )

        self.telemetry_timer = self.create_timer(
            5, self._telemetry_callback
        )

        self.get_logger().info(
            '\n🏎️   F1Tenth Reactive Controller iniciado — '
            f'{MAX_LAPS} vueltas\n'
        )

    def _target_speed(self) -> float:
        v = BASE_SPEED + self.lap_count * SPEED_INCREMENT
        return min(v, MAX_SPEED)

    def _scan_callback(self, scan: LaserScan):
        if self.race_finished:
            self._publish_stop()
            return

        ranges = np.array(scan.ranges, dtype=np.float32)
        n_total = len(ranges)

        half_fov_rad = math.radians(SCAN_FIELD_DEG / 2.0)
        angle_min    = scan.angle_min
        angle_max    = scan.angle_max
        angle_inc    = scan.angle_increment

        idx_center = int(round((0.0 - angle_min) / angle_inc))
        half_n     = int(round(half_fov_rad / angle_inc))
        
        i_start    = max(0,       idx_center - half_n)
        i_end      = min(n_total, idx_center + half_n)

        fov_ranges  = ranges[i_start:i_end].copy()
        n_fov       = len(fov_ranges)

        max_range = scan.range_max if (0.0 < scan.range_max < 30.0) else 10.0
        fov_ranges = np.where(np.isnan(fov_ranges) | np.isinf(fov_ranges), max_range, fov_ranges)
        fov_ranges = np.clip(fov_ranges, scan.range_min, max_range)

        min_idx   = int(np.argmin(fov_ranges))
        min_dist  = fov_ranges[min_idx]

        if min_dist > 0.05:
            bubble_half = int(math.ceil(BUBBLE_RADIUS / (min_dist * angle_inc)))
        else:
            bubble_half = n_fov // 4

        b_start = max(0,     min_idx - bubble_half)
        b_end   = min(n_fov, min_idx + bubble_half + 1)
        fov_ranges[b_start:b_end] = 0.0

        free_threshold = max(0.6, min_dist * 0.9)   
        free_mask      = fov_ranges > free_threshold

        best_start  = 0
        best_end    = 0
        best_width  = 0
        cur_start   = None

        for i, free in enumerate(free_mask):
            if free and cur_start is None:
                cur_start = i
            if (not free or i == n_fov - 1) and cur_start is not None:
                cur_end   = i if not free else i + 1
                cur_width = cur_end - cur_start
                if cur_width > best_width:
                    best_width  = cur_width
                    best_start  = cur_start
                    best_end    = cur_end
                cur_start = None

        min_gap_indices = int(math.radians(MIN_GAP_WIDTH_DEG) / angle_inc)
        if best_width < max(1, min_gap_indices):
            self._publish_drive(0.0, 0.8)
            return

        gap_ranges = fov_ranges[best_start:best_end]
        
        center_idx = (best_start + best_end) // 2
        farthest_rel_idx = int(np.argmax(gap_ranges))
        farthest_idx = best_start + farthest_rel_idx
        
        target_idx = int(0.70 * center_idx + 0.30 * farthest_idx)

        global_idx = i_start + target_idx
        target_angle_rad = angle_min + (global_idx * angle_inc)

        if abs(target_angle_rad) < math.radians(2.5):
            target_steering = 0.0
        else:
            if abs(target_angle_rad) < math.radians(6.0):
                current_gain = STEERING_GAIN * 0.4
            else:
                current_gain = STEERING_GAIN * 1.0
            
            target_steering = current_gain * target_angle_rad

        steering = float(self.steering_smoothing * self.last_steering_angle + 
                         (1.0 - self.steering_smoothing) * target_steering)
        
        steering = float(np.clip(steering, -MAX_STEER_RAD, MAX_STEER_RAD))
        self.last_steering_angle = steering  

        v_base = self._target_speed()
        look_ahead_dist = float(np.mean(gap_ranges))
        
        if look_ahead_dist > 7.0 and abs(steering) < 0.08:
            velocity = v_base + 1.5 
        else:
            steer_ratio = abs(steering) / MAX_STEER_RAD
            distance_factor = np.clip(look_ahead_dist / 6.0, 0.35, 1.0) 
            velocity = float(v_base * (1.0 - SPEED_STEER_DECAY * steer_ratio) * distance_factor)
        
        velocity = min(velocity, ABSOLUTE_MAX_SPEED)
        velocity = max(1.2, velocity)
        self._publish_drive(steering, velocity)

    def _odom_callback(self, odom: Odometry):
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y

        if not self.origin_set:
            self.origin_x  = px
            self.origin_y  = py
            self.origin_set = True
            self.lap_start_time = time.monotonic()
            self.get_logger().info(
                f'\n📍  Origen de carrera fijado en: ({px:.2f}, {py:.2f})\n'
            )
            return

        if self.race_finished:
            return

        dist = math.hypot(px - self.origin_x, py - self.origin_y)

        if dist > self.max_dist_from_origin:
            self.max_dist_from_origin = dist

        now = time.monotonic()

        if (self.max_dist_from_origin > 8.0 
                and dist < LAP_ORIGIN_RADIUS  
                and (now - self.last_lap_trigger) > LAP_COOLDOWN_S):

            lap_time = now - self.lap_start_time
            self.last_lap_time_s  = lap_time
            self.lap_times.append(lap_time)
            self.lap_count       += 1
            self.last_lap_trigger = now
            self.lap_start_time   = now
            self.max_dist_from_origin = 0.0   

            self.get_logger().info(
                f'\n\n🏁  ¡VUELTA {self.lap_count}/{MAX_LAPS} completada!'
                f'\nTiempo de Vuelta: {lap_time:.3f}s'
                f'\nSiguiente Velocidad Base: {self._target_speed():.2f} m/s\n'
            )

            if self.lap_count >= MAX_LAPS:
                self.race_finished = True
                self.get_logger().info('\n🏁 ¡CARRERA COMPLETADA!')
                
                resumen = "\nRESUMEN DE LA CARRERA:\n"
                for idx, t in enumerate(self.lap_times):
                    resumen += f"• Vuelta {idx+1}: {t:.3f}s\n"
                resumen += f"🏆 Mejor Vuelta: {min(self.lap_times):.3f}s\n"
                self.get_logger().info(resumen)
                
                self._publish_stop()

    def _telemetry_callback(self):
        if self.race_finished:
            return
        now = time.monotonic()
        elapsed = now - self.lap_start_time
        self.get_logger().info(
            f'\n[TEL] Vuelta: {self.lap_count+1} | T.Actual: {elapsed:.2f}s | V.Actual: {self.current_speed:.2f} m/s'
        )

    def _publish_drive(self, steering_angle: float, speed: float):
        self.current_speed = speed
        msg                              = AckermannDriveStamped()
        msg.header.stamp                 = self.get_clock().now().to_msg()
        msg.header.frame_id              = 'base_link'
        msg.drive.steering_angle         = steering_angle
        msg.drive.speed                  = speed
        self.drive_pub.publish(msg)

    def _publish_stop(self):
        self._publish_drive(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = F1TenthReactiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()