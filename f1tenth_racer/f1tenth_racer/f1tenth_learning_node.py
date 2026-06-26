import rclpy
from rclpy.node import Node
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class F1TenthLearningNode(Node):
    def __init__(self):
        super().__init__('f1tenth_learning_node')
        
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        
        self.state = 'EXPLORING'
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.waypoints = []  
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        
        self.lap_counter = 0
        self.min_lap_distance = 12.0  
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        
        self.lap_start_time = None
        self.lap_times = []

        self.lookahead_gain = 0.35  
        self.min_lookahead = 0.6    
        self.max_speed = 7.0        
        self.racing_speed_scaling = 1.0 

        self.p_gain = 0.6            
        self.d_gain = 0.25            
        self.wall_follow_speed = 1.3  
        self.last_error = 0.0
        
        self.was_behind_gate = True 

        self.get_logger().info("=========================================================")
        self.get_logger().info(" NODO F1TENTH INICIALIZADO - MODO: EXPLORACIÓN (VUELTA 1)")
        self.get_logger().info("=========================================================")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.last_x = self.current_x
            self.last_y = self.current_y
            self.lap_start_time = time.time()
            return

        dx = self.current_x - self.last_x
        dy = self.current_y - self.last_y
        self.distance_traveled += math.hypot(dx, dy)
        self.last_x = self.current_x
        self.last_y = self.current_y

        if self.state == 'EXPLORING':
            if not self.waypoints or math.hypot(self.current_x - self.waypoints[-1][0], self.current_y - self.waypoints[-1][1]) > 0.10:
                self.waypoints.append([self.current_x, self.current_y, self.wall_follow_speed])

        dx_start = self.current_x - self.start_x
        dy_start = self.current_y - self.start_y
        local_x_start = dx_start * math.cos(-self.start_yaw) - dy_start * math.sin(-self.start_yaw)
        
        is_ahead = (local_x_start >= 0.0)

        if is_ahead and self.was_behind_gate and self.distance_traveled > self.min_lap_distance:
            current_time = time.time()
            lap_duration = current_time - self.lap_start_time
            self.lap_times.append(lap_duration)
            
            self.lap_counter += 1
            
            self.get_logger().info("---------------------------------------------------------")
            self.get_logger().info(f"¡VUELTA {self.lap_counter} COMPLETADA!")
            self.get_logger().info(f"Tiempo de la vuelta: {lap_duration:.3f} segundos")
            self.get_logger().info("---------------------------------------------------------")
            
            self.lap_start_time = current_time
            self.distance_traveled = 0.0

            if self.state == 'EXPLORING':
                self.get_logger().info("Cambiando a MODO: OPTIMIZANDO TRAYECTORIA...")
                self.state = 'OPTIMIZING'
                self.optimize_trajectory()
                self.state = 'RACING'
                self.get_logger().info("¡OPTIMIZACIÓN COMPLETADA! -> INICIANDO MODO CARRERA DE ALTA VELOCIDAD")
                self.get_logger().info("---------------------------------------------------------")
                
            elif self.state == 'RACING' and self.lap_counter >= 10:
                self.get_logger().info("=========================================================")
                self.get_logger().info("¡COMPETICIÓN FINALIZADA! 10 VUELTAS COMPLETADAS.")
                for idx, t in enumerate(self.lap_times):
                    mode = "Reconocimiento" if idx == 0 else f"Carrera Rápida"
                    self.get_logger().info(f" Vuelta {idx+1} ({mode}): {t:.3f} s")
                self.get_logger().info(f" MEJOR TIEMPO DE CARRERA: {min(self.lap_times[1:]):.3f} s")
                self.get_logger().info("=========================================================")
                self.state = 'FINISHED'

        self.was_behind_gate = not is_ahead

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
        
        if self.state == 'FINISHED':
            drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            return

        if self.state == 'EXPLORING':
            ranges = np.array(msg.ranges)
            dist_left = self.get_range(ranges, 45)
            dist_right = self.get_range(ranges, -45)
            
            if dist_left is None or dist_right is None:
                drive_msg.drive.speed = self.wall_follow_speed * 0.5
                self.drive_pub.publish(drive_msg)
                return

            error = dist_left - dist_right
            error_diff = error - self.last_error
            self.last_error = error
            
            steering_angle = self.p_gain * error + self.d_gain * error_diff
            steering_angle = np.clip(steering_angle, -0.4, 0.4)
            
            drive_msg.drive.speed = self.wall_follow_speed
            drive_msg.drive.steering_angle = steering_angle
            self.drive_pub.publish(drive_msg)
            
        elif self.state == 'RACING':
            if not self.waypoints:
                return
                
            current_speed = getattr(self, 'last_sent_speed', self.max_speed * 0.5)
            lookahead_dist = max(self.min_lookahead, current_speed * self.lookahead_gain)
            
            target_wp = None
            min_dist = float('inf')
            
            for wp in self.waypoints:
                dist = math.hypot(wp[0] - self.current_x, wp[1] - self.current_y)
                if dist >= lookahead_dist and dist < min_dist:
                    min_dist = dist
                    target_wp = wp
            
            if target_wp is None:
                target_wp = self.waypoints[-1]

            dx = target_wp[0] - self.current_x
            dy = target_wp[1] - self.current_y
            
            local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            L2 = lookahead_dist ** 2
            steering_angle = (2 * local_y) / L2
            steering_angle = np.clip(steering_angle, -0.4189, 0.4189)
            
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
        
        smooth_x = np.convolve(points_x, np.ones(5)/5, mode='same')
        smooth_y = np.convolve(points_y, np.ones(5)/5, mode='same')
        
        for i in range(n):
            p_prev = [smooth_x[(i - 3) % n], smooth_y[(i - 3) % n]]
            p_curr = [smooth_x[i], smooth_y[i]]
            p_next = [smooth_x[(i + 3) % n], smooth_y[(i + 3) % n]]
            
            a = math.hypot(p_curr[0] - p_prev[0], p_curr[1] - p_prev[1])
            b = math.hypot(p_next[0] - p_curr[0], p_next[1] - p_curr[1])
            c = math.hypot(p_next[0] - p_prev[0], p_next[1] - p_prev[1])
            
            s = (a + b + c) / 2.0
            area = math.sqrt(max(1e-6, s * (s - a) * (s - b) * (s - c)))
            R = (a * b * c) / (4.0 * area) if area > 1e-4 else 100.0
            
            if R > 3.5:
                speed = self.max_speed
            else:
                speed = max(2.3, self.max_speed * (R / 3.5))
                
            optimized_wps.append([p_curr[0], p_curr[1], speed])
            
        for i in range(n - 2, 0, -1):
            optimized_wps[i][2] = min(optimized_wps[i][2], optimized_wps[i+1][2] + 0.15)

        self.waypoints = optimized_wps
        self.get_logger().info(f"-> {n} posiciones de carrera calculadas y optimizadas.")

def main(args=None):
    rclpy.init(args=args)
    node = F1TenthLearningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()