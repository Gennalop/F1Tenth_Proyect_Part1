import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

MAX_LAPS           = 10      
BASE_SPEED         = 9.0        

LAP_ORIGIN_RADIUS  = 2.0

MAX_RANGE          = 10.0
DISPARITY_THRESH   = 0.6
CAR_WIDHT          = 0.30
SAFETY_MARGIN      = 0.15   

SIDE_ANGLE_DEG     = 90.0   
THETA_DEG          = 45.0   
LOOKAHEAD          = 1.2    

KP                 = 0.30
KD                 = 0.02

STEER_MAX_RAD      = 0.4189 

STRAIGHT_BOOST_SPEED     = 14.0  
STRAIGHT_BOOST_MIN_DIST  = 7.0 
STRAIGHT_BOOST_MAX_STEER = 0.35  


class F1TenthReactiveController(Node):

    def __init__(self):
        super().__init__('f1tenth_reactive_controller')

        self.enable_logs = False

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_sensor)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, qos_sensor)
        
        self.telemetry_timer = self.create_timer(5.0, self._telemetry_callback)

        self.lap_count = 0
        self.lap_start_time = None
        self.lap_times = []
        self.race_finished = False

        self.origin_set = False
        self.origin_x = None
        self.origin_y = None
        self.max_dist_from_origin = 0.0 

        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        self.current_speed = 0.0
        self.scan_cycle = 0 

        self.get_logger().info(
            '\n🏎️   F1Tenth Reactive Controller Inicializado \n'
            f'Objetivo: {MAX_LAPS} vueltas a velocidad constante de {BASE_SPEED} m/s\n'
        )

    def _target_speed(self) -> float:
        return float(BASE_SPEED)
    
    def scan_callback(self, scan):
        if self.race_finished:
            self._publish_drive(0.0, 0.0)
            return

        self.scan_cycle += 1
        
        ranges_raw = np.array(scan.ranges)
        raw_min = np.nanmin(ranges_raw) if len(ranges_raw) > 0 else 0.0
        raw_max = np.nanmax(ranges_raw) if len(ranges_raw) > 0 else 0.0

        ranges = np.nan_to_num(ranges_raw, nan=MAX_RANGE, posinf=MAX_RANGE, neginf=0.0)
        ranges = np.clip(ranges, 0.0, MAX_RANGE)
        
        ranges_clean = self._clean_disparities(ranges, scan.angle_increment)

        side_angle = math.radians(SIDE_ANGLE_DEG)
        theta = math.radians(THETA_DEG)
        left_proj = self._get_wall_projection(ranges_clean, scan.angle_min, scan.angle_increment, 'left', side_angle, theta)
        right_proj = self._get_wall_projection(ranges_clean, scan.angle_min, scan.angle_increment, 'right', side_angle, theta)

        CRITICAL_WALL_DIST = 0.65 

        if left_proj is not None and right_proj is not None:
            if right_proj < CRITICAL_WALL_DIST:
                error = CAR_WIDHT * 2.5  
                case_desc = "EVASIÓN CRÍTICA: Muro derecho demasiado cerca"
            elif left_proj < CRITICAL_WALL_DIST:
                error = -CAR_WIDHT * 2.5 
                case_desc = "EVASIÓN CRÍTICA: Muro izquierdo demasiado cerca"
            else:
                error = left_proj - right_proj
                case_desc = "Ambas paredes visibles (Centrado normal)"
        elif left_proj is not None:
            error = -CAR_WIDHT * 1.5  
            case_desc = "Solo pared IZQUIERDA visible"
        elif right_proj is not None:
            error = CAR_WIDHT * 1.5   
            case_desc = "Solo pared DERECHA visible"
        else:
            error = self.prev_error
            case_desc = "Ceguera total: Usando error previo"

        error = np.clip(error, -1.0, 1.0)

        now_time = self.get_clock().now()
        dt = (now_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0: dt = 1e-3
        self.prev_time = now_time

        derivative = (error - self.prev_error) / dt
        derivative = np.clip(derivative, -5.0, 5.0)
        self.prev_error = error

        num_rays = len(ranges_clean)
        fov_indices = int((math.radians(15.0) / scan.angle_increment))
        center_idx = num_rays // 2
        start_idx = max(0, center_idx - fov_indices)
        end_idx = min(num_rays, center_idx + fov_indices)
        front_dist_critica = float(np.min(ranges_clean[start_idx:end_idx]))

        current_target_speed = self._target_speed()
        speed_scale = BASE_SPEED / max(current_target_speed, 1e-3)
        
        steering_angle_raw = (KP * error + KD * derivative) * speed_scale
        if front_dist_critica > 3.0:
            max_giro_seguro = STEER_MAX_RAD * 0.8
            steering_angle = np.clip(steering_angle_raw, -max_giro_seguro, max_giro_seguro)
        else:
            steering_angle = np.clip(steering_angle_raw, -STEER_MAX_RAD, STEER_MAX_RAD)
        
        boost_fov_indices = int((math.radians(3.0) / scan.angle_increment))
        b_start = max(0, center_idx - boost_fov_indices)
        b_end = min(num_rays, center_idx + boost_fov_indices)
        dist_para_boost = float(np.mean(ranges_clean[b_start:b_end]))

        v_max = self._target_speed()
        v_min = 1.5  

        steer_ratio = abs(steering_angle) / STEER_MAX_RAD
        speed_cmd = v_max - (v_max - v_min) * (steer_ratio ** 2)

        if front_dist_critica < 1.8:
            speed_cmd = 0.5  
            speed_reason = "FRENO DE EMERGENCIA: Obstáculo inminente en cono frontal"
        elif front_dist_critica < 4.5:
            speed_cmd = min(speed_cmd, 4.5)
            speed_reason = "Frenado preventivo por curva cercana"
        else:
            speed_reason = "Velocidad de carrera"
            if dist_para_boost > STRAIGHT_BOOST_MIN_DIST and front_dist_critica >= 5.0 and steer_ratio < STRAIGHT_BOOST_MAX_STEER:
                speed_cmd = STRAIGHT_BOOST_SPEED
                speed_reason = "BOOST: Recta despejada"
                #self.get_logger().info(
                #    f'\nBOOST\n'
                #)

        speed_cmd = max(0.5, min(STRAIGHT_BOOST_SPEED, speed_cmd))

        if self.enable_logs and (self.scan_cycle % 20 == 0):
            self.get_logger().info(
                f"\n=== [SCAN CICLO #{self.scan_cycle}] ====================\n"
                f" 1. LiDAR Inicial: Min={raw_min:.2f}m, Max={raw_max:.2f}m"
                f" 2. Proyecciones (Lookahead {LOOKAHEAD}m): Izq={f'{left_proj:.2f}m' if left_proj else 'None'} | Der={f'{right_proj:.2f}m' if right_proj else 'None'}\n"
                f" 3. Estrategia: {case_desc} -> Error Calculado = {error:.3f}\n"
                f" 4. Control PD: dt={dt:.4f}s | Derivativa={derivative:.3f} | Volante crudo={math.degrees(np.clip(steering_angle_raw, -STEER_MAX_RAD, STEER_MAX_RAD)):.1f}° | Volante filtrado={math.degrees(steering_angle):.1f}°\n"
                f" 5. Motor: Frente={front_dist_critica:.2f}m | Target={speed_cmd:.2f} m/s ({speed_reason})\n"
                f"======================================================="
            )

        self._publish_drive(steering_angle, speed_cmd)

    def odom_callback(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y

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

        dist_to_origin = math.hypot(px - self.origin_x, py - self.origin_y)
        if dist_to_origin > self.max_dist_from_origin:
            self.max_dist_from_origin = dist_to_origin

        now = time.monotonic()

        if (self.max_dist_from_origin > 8.0 
            and dist_to_origin < LAP_ORIGIN_RADIUS):

            lap_duration = now - self.lap_start_time
            self.lap_times.append(lap_duration)
            self.lap_count += 1
            self.lap_start_time = now
            self.max_dist_from_origin = 0.0  

            self.get_logger().info(
                f'\n\n🏁  ¡VUELTA {self.lap_count}/{MAX_LAPS} completada!'
                f'\nTiempo de Vuelta: {lap_duration:.3f}s'
            )

            if self.lap_count >= MAX_LAPS:
                self.race_finished = True
                self.get_logger().info('\n🏁 ¡CARRERA COMPLETADA!')
                resumen = "\nRESUMEN DE LA CARRERA:\n"
                for idx, t in enumerate(self.lap_times):
                    resumen += f"• Vuelta {idx+1}: {t:.3f}s\n"
                resumen += f"🏆 Mejor Vuelta: {min(self.lap_times):.3f}s\n"
                self.get_logger().info(resumen)
                
                self._publish_drive(0.0, 0.0)

    def _publish_drive(self, steering_angle: float, speed: float):
        self.current_speed = speed
        msg                              = AckermannDriveStamped()
        msg.header.stamp                 = self.get_clock().now().to_msg()
        msg.header.frame_id              = 'base_link'
        msg.drive.steering_angle         = steering_angle
        msg.drive.speed                  = speed
        self.drive_pub.publish(msg)

    def _telemetry_callback(self):
        if self.race_finished or self.lap_start_time is None:
            return
        now = time.monotonic()
        elapsed = now - self.lap_start_time
        self.get_logger().info(
            f'\n[TEL] Lap: {self.lap_count+1} | T.Actual: {elapsed:.2f}s | V.Actual: {self.current_speed:.2f} m/s'
        )

    def _get_wall_projection(self, ranges, angle_min, angle_increment, side, side_angle, theta):
        if side == 'right': angle_b, angle_c = -side_angle, -(side_angle - theta)
        else: angle_b, angle_c = side_angle, (side_angle - theta)
        idx_b = int(round((angle_b - angle_min) / angle_increment))
        idx_c = int(round((angle_c - angle_min) / angle_increment))
        if not (0 <= idx_b < len(ranges) and 0 <= idx_c < len(ranges)):
            return None
        dist_b = float(ranges[idx_b])
        dist_c = float(ranges[idx_c])
        if dist_b >= MAX_RANGE * 0.95 or dist_c >= MAX_RANGE * 0.95:
            return None
        alpha = math.atan2(dist_c * math.cos(theta) - dist_b, dist_c * math.sin(theta))
        current_distance = dist_b * math.cos(alpha)
        projected_distance = current_distance + LOOKAHEAD * math.sin(alpha)
        return projected_distance
    
    def _clean_disparities(self, ranges, angle_increment):
        out = ranges.copy()
        car_radius = (CAR_WIDHT / 2.0) + SAFETY_MARGIN

        for i in range(len(out) - 1):
            diff = out[i + 1] - out[i]
            if abs(diff) > DISPARITY_THRESH:
                near_idx = i if diff > 0 else i + 1
                near_dist = max(out[near_idx], 0.1)
                alpha_inflado = math.atan2(car_radius, near_dist)
                indices_a_cambiar = int(math.ceil(alpha_inflado / angle_increment))
                direction = 1 if diff > 0 else -1
                for j in range(1, indices_a_cambiar + 1):
                    target_idx = near_idx + (direction * j)
                    if 0 <= target_idx < len(out):
                        if out[target_idx] > out[near_idx]:
                            out[target_idx] = out[near_idx]
        return out


def main(args=None):
    rclpy.init(args=args)
    node = F1TenthReactiveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()