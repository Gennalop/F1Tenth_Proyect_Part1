import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

MAX_LAPS = 10
LAP_ORIGIN_RADIUS = 2.0
LAP_COOLDOWN_S = 3.0

class F1TenthLearningNode(Node):
    def __init__(self):
        super().__init__('f1tenth_learning_node')
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, qos_sensor)
        
        self.telemetry_timer = self.create_timer(5.0, self._telemetry_callback)
        
        self.state = 'EXPLORING'
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.waypoints = []  
        self.origin_set = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.max_dist_from_origin = 0.0
        
        self.lap_count = 0
        self.lap_start_time = None
        self.last_lap_trigger = -LAP_COOLDOWN_S
        self.lap_times = []
        self.last_sent_speed = 0.0

        self.lookahead_gain = 0.42  
        self.min_lookahead = 1.1   
        self.max_lookahead = 4.2 
        self.max_speed = 9.5       
        self.racing_speed_scaling = 1.0 

        self.last_wp_idx = 0

        self.p_gain = 0.38            
        self.d_gain = 0.32            
        self.wall_follow_speed = 4.0  
        self.last_error = 0.0

        self.get_logger().info(
            '\n🏎️   F1Tenth Learning Node Inicializado \n'
            f'Modo Inicial: EXPLORACIÓN (Vuelta 1) | Objetivo: {MAX_LAPS} vueltas\n'
        )

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self.origin_set:
            self.origin_x = self.current_x
            self.origin_y = self.current_y
            self.origin_set = True
            self.lap_start_time = time.monotonic()
            self.last_lap_trigger = time.monotonic()
            self.get_logger().info(
                f'\n📍  Origen de carrera fijado en: ({self.origin_x:.2f}, {self.origin_y:.2f})\n'
            )
            return

        if self.state == 'EXPLORING':
            if not self.waypoints or math.hypot(self.current_x - self.waypoints[-1][0], self.current_y - self.waypoints[-1][1]) > 0.30:
                self.waypoints.append([self.current_x, self.current_y, self.wall_follow_speed])

        dist_to_origin = math.hypot(self.current_x - self.origin_x, self.current_y - self.origin_y)
        if dist_to_origin > self.max_dist_from_origin:
            self.max_dist_from_origin = dist_to_origin

        now = time.monotonic()
        
        if (self.max_dist_from_origin > 8.0 
                and dist_to_origin < LAP_ORIGIN_RADIUS  
                and (now - self.last_lap_trigger) > LAP_COOLDOWN_S):

            lap_duration = now - self.lap_start_time
            self.lap_times.append(lap_duration)
            self.lap_count += 1
            self.last_lap_trigger = now
            self.lap_start_time = now
            self.max_dist_from_origin = 0.0  
            self.last_wp_idx = 0 

            self.get_logger().info(
                f'\n\n🏁  ¡VUELTA {self.lap_count}/{MAX_LAPS} completada!'
                f'\nTiempo de Vuelta: {lap_duration:.3f}s\n'
            )

            if self.state == 'EXPLORING':
                self.get_logger().info("\n\nCambiando de MODO...\n")
                self.state = 'OPTIMIZING'
                self.optimize_trajectory()
                self.state = 'RACING'
                self.get_logger().info("\n\n INICIANDO MODO CARRERA\n")

            elif self.lap_count >= MAX_LAPS:
                self.state = 'FINISHED'
                self._print_final_summary()

    def _telemetry_callback(self):
        if self.state in ['FINISHED', 'OPTIMIZING'] or self.lap_start_time is None:
            return
        now = time.monotonic()
        elapsed = now - self.lap_start_time
        self.get_logger().info(
            f'\n[TEL] Lap: {self.lap_count+1} | T.Actual: {elapsed:.2f}s | V.Actual: {self.last_sent_speed:.2f} m/s | M: {self.state}'
        )

    def _print_final_summary(self):
        self.get_logger().info('\n🏁 ¡CARRERA COMPLETADA!')
        self.get_logger().info('📊 RESUMEN DE LA CARRERA:')
        for idx, t in enumerate(self.lap_times):
            self.get_logger().info(f' • Vuelta {idx+1}: {t:.3f}s')
        if len(self.lap_times) > 1:
            self.get_logger().info(f'🏆 Mejor Vuelta: {min(self.lap_times[1:]):.3f}s\n')
     
    def get_range(self, ranges, angle):
        angle_rad = math.radians(angle)
        idx = int((angle_rad - getattr(self, 'angle_min', -math.pi/2)) / getattr(self, 'angle_increment', math.pi/180))
        if 0 <= idx < len(ranges):
            return ranges[idx]
        return None

    def scan_callback(self, msg):
        if not hasattr(self, 'angle_min'):
            self.angle_min = msg.angle_min
            self.angle_increment = msg.angle_increment

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        
        if self.state == 'FINISHED':
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.last_sent_speed = 0.0
            self.drive_pub.publish(drive_msg)
            return

        if self.state == 'EXPLORING':
            ranges = np.array(msg.ranges)
            dist_left = self.get_range(ranges, 45)
            dist_right = self.get_range(ranges, -45)
            
            if dist_left is None or dist_right is None:
                self.last_sent_speed = self.wall_follow_speed * 0.5
                drive_msg.drive.speed = self.last_sent_speed
                self.drive_pub.publish(drive_msg)
                return

            error = dist_left - dist_right
            error_diff = error - self.last_error
            self.last_error = error
            
            steering_angle = self.p_gain * error + self.d_gain * error_diff
            steering_angle = np.clip(steering_angle, -0.4, 0.4)
            
            self.last_sent_speed = self.wall_follow_speed
            drive_msg.drive.speed = self.last_sent_speed
            drive_msg.drive.steering_angle = steering_angle
            self.drive_pub.publish(drive_msg)
        
        elif self.state == 'RACING':
            if not self.waypoints:
                return
                
            num_wps = len(self.waypoints)
            lookahead_dist = max(self.min_lookahead, min(self.max_lookahead, self.last_sent_speed * self.lookahead_gain))
            
            target_wp = None
            closest_wp_idx = self.last_wp_idx
            search_range = 150 
            
            for i in range(search_range):
                idx = (self.last_wp_idx + i) % num_wps
                wp = self.waypoints[idx]
                
                dx = wp[0] - self.current_x
                dy = wp[1] - self.current_y
                
                local_x = dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
                local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
                
                if local_x > 0.0:
                    dist = math.hypot(dx, dy)
                    if dist >= lookahead_dist:
                        target_wp = wp
                        closest_wp_idx = idx
                        target_local_y = local_y
                        break
            
            if target_wp is None:
                closest_wp_idx = (self.last_wp_idx + 1) % num_wps
                target_wp = self.waypoints[closest_wp_idx]
                dx = target_wp[0] - self.current_x
                dy = target_wp[1] - self.current_y
                target_local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

            self.last_wp_idx = closest_wp_idx

            L2 = lookahead_dist ** 2
            steering_angle = (2 * target_local_y) / L2
            
            if self.last_sent_speed > 5.0:
                steering_angle *= 0.85 

            steering_angle = np.clip(steering_angle, -0.4189, 0.4189)

            steer_ratio = abs(steering_angle) / 0.4189
            if steer_ratio > 0.4:
                target_speed = target_wp[2] * (1.0 - 0.35 * steer_ratio)
            else:
                target_speed = target_wp[2] * self.racing_speed_scaling

            drive_msg.drive.speed = target_speed
            drive_msg.drive.steering_angle = steering_angle
            self.last_sent_speed = target_speed
            self.drive_pub.publish(drive_msg)
        
    def optimize_trajectory(self):
        n = len(self.waypoints)
        if n < 5:
            return
            
        optimized_wps = []
        points_x = [wp[0] for wp in self.waypoints]
        points_y = [wp[1] for wp in self.waypoints]
        
        smooth_x = np.convolve(points_x, np.ones(9)/9, mode='same')
        smooth_y = np.convolve(points_y, np.ones(9)/9, mode='same')
        
        for i in range(n):
            p_prev = [smooth_x[(i - 1) % n], smooth_y[(i - 1) % n]]
            p_curr = [smooth_x[i], smooth_y[i]]
            p_next = [smooth_x[(i + 1) % n], smooth_y[(i + 1) % n]]
            
            a = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            b = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            c = math.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1])
            
            s = (a + b + c) / 2.0
            area = math.sqrt(max(1e-6, s * (s - a) * (s - b) * (s - c)))
            R = (a * b * c) / (4.0 * area) if area > 1e-4 else 100.0
            
            if R > 6.0:
                speed = self.max_speed
            else:
                speed = max(3.0, self.max_speed * (R / 6.0))
                
            optimized_wps.append([p_curr[0], p_curr[1], speed])
            
        for i in range(n - 2, 0, -1):
            optimized_wps[i][2] = min(optimized_wps[i][2], optimized_wps[i+1][2] + 0.15)

        self.waypoints = optimized_wps
        self.get_logger().info(f"-> {n} posiciones de carrera calculadas y optimizadas.")

def main(args=None):
    rclpy.init(args=args)
    node = F1TenthLearningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()