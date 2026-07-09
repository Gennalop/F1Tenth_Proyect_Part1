#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

# Mensajes estándar de ROS 2 para F1TENTH
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('f1tenth_reactive_obs')

        # --- CONFIGURACIÓN DE SUSCRIPTORES Y PUBLICADORES ---
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # --- CONFIGURACIÓN DE TELEMETRÍA ---
        self.SHOW_LOGS = True            # Cambiar a False para desactivar por completo
        self.LOG_PERIOD_SECONDS = 0.5    # Tiempo en segundos entre cada impresión de telemetría

        # Guardará el tiempo de ROS en el que se imprimió el último log
        self.last_log_time = self.get_clock().now()

        # --- HIPERPARÁMETROS DEL ALGORITMO ---
        self.PREPROCESS_MAX_DIST = 4.0     # Distancia máxima a considerar
        self.DISPARITY_THRESHOLD = 0.8    # Umbral en metros para activar Disparity Extender

        # Geometría real del auto: el radio de seguridad ahora se calcula con
        # atan2(car_radius, dist) en vez de la aproximación lineal
        # radio/(dist*angle_increment), que pierde precisión justo a distancias
        # cortas (como los ~0.45m que se ven en curvas cerradas).
        self.CAR_WIDTH_METERS = 0.40       # Ancho real del auto (ajusta a tu chasis)
        self.SAFETY_MARGIN_METERS = 0.18   # Margen extra de seguridad (antes 0.15: subido un poco
                                            # para separar más al auto de las paredes en curvas)
        self.CAR_RADIUS_METERS = (self.CAR_WIDTH_METERS / 2.0) + self.SAFETY_MARGIN_METERS

        # Rate limiter de dirección: evita que el ángulo objetivo salte bruscamente
        # de un frame a otro (esto es lo que causaba el zigzag pared-a-pared en
        # rectas: pequeño ruido hacía que el "mejor punto" saltara de lado).
        self.prev_steering_angle = 0.0
        self.MAX_STEER_DELTA_RAD = 0.12   # cuánto puede cambiar el ángulo por scan

        # Límites del campo de visión (FOV) delantero en radianes (~ 70 grados a cada lado)
        self.MIN_ANGLE_RAD = -1.22
        self.MAX_ANGLE_RAD = 1.22

        # Tolerancia para considerar "empatados" dos puntos como máximo del hueco.
        # Con esto agrupamos toda la meseta de puntos lejanos y tomamos su CENTRO,
        # en vez de quedarnos con el primer índice que encuentre .index(max(...)).
        self.MAX_TIE_TOLERANCE_METERS = 0.10

        # NUEVO (fix #1 - zigzag): tolerancia para considerar "empatados" dos
        # HUECOS distintos por tamaño. Si dos huecos difieren en menos de esta
        # cantidad de índices, no se consideran "el más grande y punto": se
        # elige el que esté más cerca del hueco que se venía siguiendo. Esto
        # evita que el auto salte de un hueco a otro (izquierda-derecha) por
        # un empate momentáneo entre dos aberturas parecidas al pasar un
        # obstáculo.
        self.GAP_SIZE_TIE_TOLERANCE_INDICES = 15

        # NUEVO (fix #2 - pegado a esquinas): en vez de un margen angular FIJO
        # (que recorta poca distancia real justo cuando el obstáculo está
        # cerca, que es cuando más margen hace falta), calculamos el margen
        # de cada borde del hueco con el mismo criterio atan2(car_radius,
        # dist) que ya usamos en la burbuja y el disparity extender. Así el
        # margen crece solo cuando el obstáculo del borde está cerca (curva
        # cerrada) y se achica cuando está lejos (recta), en vez de al revés.
        self.GAP_EDGE_MARGIN_MIN_INDICES = 3  # piso mínimo, por si el borde da un ángulo ridículamente chico

        # Umbrales de distancia (FOV completo, 140°) -> tope de velocidad.
        # Se mantienen más permisivos: solo importan para pasajes MUY angostos,
        # ya no son responsables de frenar por paredes laterales en rectas.
        self.DIST_SPEED_CAPS = [
            (0.5, 1.2),
            (0.8, 2.2),
            (1.2, 3.5),
        ]

        # Cono frontal angosto (en radianes, a cada lado de 0) usado EXCLUSIVAMENTE
        # para frenar por peligro real de frente (p.ej. la esquina de una curva).
        # No usa paredes laterales de un pasillo recto para frenar.
        self.FRONT_CONE_HALF_ANGLE_RAD = 0.35  # ~20 grados a cada lado

        # NUEVO (fix #3 - más velocidad): topes bastante más permisivos que la
        # vez anterior. FRONT_EMERGENCY_DIST se sube un poco (de 0.35 a 0.45)
        # para compensar: a más velocidad, más distancia de frenado hace
        # falta para reaccionar a un obstáculo repentino de frente. Es el
        # único número que NO debería bajarse al mismo tiempo que se sube
        # la velocidad.
        self.FRONT_DIST_SPEED_CAPS = [
            (0.45, 1.3),
            (0.8, 2.4),
            (1.2, 3.4),
            (1.8, 4.6),
        ]
        self.FRONT_EMERGENCY_DIST = 0.45  # antes 0.35: compensa la mayor velocidad de crucero

        self.MAX_SPEED = 6.0   # antes 4.5

        # --- VARIABLES PARA EL CONTROL DE VUELTAS ---
        self.lap_count = 0
        self.prev_x = 0.0
        self.is_past_start_line = False

        self.get_logger().info("Nodo 'f1tenth_reactive_obs' inicializado con éxito.")

    def odom_callback(self, msg):
        """
        Lógica simple para contar 10 vueltas basada en la odometría.
        """
        current_x = msg.pose.pose.position.x

        if self.prev_x < 0.0 <= current_x:
            if not self.is_past_start_line:
                self.lap_count += 1
                self.get_logger().info(f"¡Vuelta completada! Vueltas totales: {self.lap_count}/10")
                self.is_past_start_line = True
        elif current_x < -0.5:
            self.is_past_start_line = False

        self.prev_x = current_x

        if self.lap_count >= 10:
            self.get_logger().warn("¡Se han completado 10 vueltas! Deteniendo el vehículo de forma segura.")
            self.publish_drive(0.0, 0.0)

    def scan_callback(self, scan_msg):
        """
        Callback principal del LiDAR. Procesa los datos y calcula la dirección del vehículo.
        """
        if self.lap_count >= 10:
            return

        # 1. Obtener los índices del arreglo 'ranges' correspondientes al FOV delantero
        start_index = int((self.MIN_ANGLE_RAD - scan_msg.angle_min) / scan_msg.angle_increment)
        end_index = int((self.MAX_ANGLE_RAD - scan_msg.angle_min) / scan_msg.angle_increment)

        start_index = max(0, start_index)
        end_index = min(len(scan_msg.ranges) - 1, end_index)

        # Extraer el sub-arreglo delantero
        proc_ranges = list(scan_msg.ranges[start_index:end_index + 1])
        total_samples = len(proc_ranges)

        # 2. Preprocesamiento: Eliminar ruido y recortar distancias máximas
        for i in range(len(proc_ranges)):
            if np.isnan(proc_ranges[i]) or np.isinf(proc_ranges[i]):
                proc_ranges[i] = 0.0
            elif proc_ranges[i] > self.PREPROCESS_MAX_DIST:
                proc_ranges[i] = self.PREPROCESS_MAX_DIST

        # Snapshot CRUDO (post limpieza de NaN/inf y recorte a 4.0m, pero ANTES
        # de que el Disparity Extender empiece a poner ceros). Esto es clave:
        # si calculamos min_distance sobre proc_ranges DESPUÉS del paso 3, vamos
        # a agarrar los ceros que nosotros mismos pusimos ahí, no una lectura real.
        raw_ranges = list(proc_ranges)

        # 3. Extensión de Disparidades (Disparity Extender)
        # Usa atan2(car_radius, dist) para calcular cuántos índices hay que
        # "tapar" para que el auto no pase rozando el borde de un obstáculo
        # cercano. A diferencia de la aproximación lineal anterior, esto es
        # preciso también a distancias cortas. Solo infla HACIA el lado del
        # hueco (no ambos lados a ciegas) y nunca reduce un punto que ya era
        # más cercano que el obstáculo detectado.
        disparities_detected = 0
        for i in range(len(proc_ranges) - 1):
            diff = proc_ranges[i + 1] - proc_ranges[i]
            if abs(diff) > self.DISPARITY_THRESHOLD:
                disparities_detected += 1
                near_idx = i if diff > 0 else i + 1
                near_dist = max(proc_ranges[near_idx], 0.1)

                alpha_inflado = math.atan2(self.CAR_RADIUS_METERS, near_dist)
                indices_a_cambiar = int(math.ceil(alpha_inflado / scan_msg.angle_increment))

                direction = 1 if diff > 0 else -1
                for j in range(1, indices_a_cambiar + 1):
                    target_idx = near_idx + (direction * j)
                    if 0 <= target_idx < len(proc_ranges):
                        if proc_ranges[target_idx] > proc_ranges[near_idx]:
                            proc_ranges[target_idx] = proc_ranges[near_idx]

        # 4. Encontrar el punto más cercano REAL (sobre raw_ranges, no sobre el
        # proc_ranges ya zonificado) y dibujar la "Burbuja de Seguridad"
        min_distance = min(raw_ranges)
        min_idx = raw_ranges.index(min_distance)

        # Distancia mínima en el cono frontal angosto (para frenado por peligro real de frente)
        front_start_idx = max(0, int(
            (-self.FRONT_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment
        ))
        front_end_idx = min(len(raw_ranges) - 1, int(
            (self.FRONT_CONE_HALF_ANGLE_RAD - self.MIN_ANGLE_RAD) / scan_msg.angle_increment
        ))
        front_section = raw_ranges[front_start_idx:front_end_idx + 1]
        min_distance_front = min(front_section) if front_section else min_distance

        if min_distance_front <= self.FRONT_EMERGENCY_DIST:
            # Peligro real de frente (p.ej. esquina de una curva encima nuestro).
            # Escapamos hacia el lado con más espacio, no siempre hacia el mismo lado.
            steer = self.escape_direction(raw_ranges)
            self.get_logger().error(
                f"¡CRÍTICO: obstáculo de frente a {min_distance_front:.2f} m! Evasión hacia {steer:.2f} rad."
            )
            self.prev_steering_angle = steer  # evita un salto brusco al salir de la emergencia
            self.publish_drive(0.4, steer)
            return

        if min_distance > 0.0:
            alpha_bubble = math.atan2(self.CAR_RADIUS_METERS, min_distance)
            bubble_width_indices = int(math.ceil(alpha_bubble / scan_msg.angle_increment))
            start_bubble = max(0, min_idx - bubble_width_indices)
            end_bubble = min(len(proc_ranges), min_idx + bubble_width_indices + 1)

            for j in range(start_bubble, end_bubble):
                proc_ranges[j] = 0.0

        # 5. Encontrar el "Gap" más grande.
        # FIX #1 (zigzag): le pasamos el índice del ángulo previo para que, si
        # hay varios huecos de tamaño parecido, se prefiera el más cercano al
        # que se venía siguiendo, en vez de saltar de uno a otro por un
        # empate momentáneo.
        prev_idx = int(round((self.prev_steering_angle - self.MIN_ANGLE_RAD) / scan_msg.angle_increment))
        max_start, max_end = self.find_max_gap(proc_ranges, prev_idx)
        gap_size = max_end - max_start

        # 6. Encontrar el mejor punto dentro de ese Gap.
        # FIX #2 (pegado a esquinas): recortamos un margen de ambos bordes del
        # hueco antes de buscar el mejor punto. El margen ahora es DINÁMICO
        # (atan2(car_radius, dist_obstaculo_del_borde)), igual que la burbuja
        # y el disparity extender: más margen cuando el obstáculo del borde
        # está cerca (curva cerrada), menos cuando está lejos (recta). Con un
        # margen fijo, en curvas cerradas casi no se recortaba distancia real
        # y el auto seguía rozando la pared; con margen dinámico sí se aleja.
        start_neighbor_idx = max(0, max_start - 1)
        end_neighbor_idx = min(len(raw_ranges) - 1, max_end + 1)

        start_margin_indices = max(
            self.GAP_EDGE_MARGIN_MIN_INDICES,
            self._dynamic_edge_margin(raw_ranges[start_neighbor_idx], scan_msg.angle_increment)
        )
        end_margin_indices = max(
            self.GAP_EDGE_MARGIN_MIN_INDICES,
            self._dynamic_edge_margin(raw_ranges[end_neighbor_idx], scan_msg.angle_increment)
        )

        trimmed_start = max_start + start_margin_indices
        trimmed_end = max_end - end_margin_indices
        if trimmed_start > trimmed_end:
            # El hueco es demasiado angosto para recortar bordes; usamos el original.
            trimmed_start, trimmed_end = max_start, max_end

        gap_section = proc_ranges[trimmed_start:trimmed_end + 1]
        if len(gap_section) == 0:
            self.get_logger().error("¡CRÍTICO: Sin huecos seguros disponibles! Ejecutando maniobra de emergencia.")
            self.publish_drive(0.5, -0.4)
            return

        # --- Tomar el CENTRO de la meseta de puntos lejanos, ---
        # --- no el primer índice que .index(max(...)) devolvería.          ---
        max_val = max(gap_section)
        candidate_indices = [
            idx for idx, v in enumerate(gap_section)
            if v >= max_val - self.MAX_TIE_TOLERANCE_METERS
        ]
        best_idx_in_gap = int(round(sum(candidate_indices) / len(candidate_indices)))
        best_idx_global = trimmed_start + best_idx_in_gap
        distancia_al_frente = proc_ranges[best_idx_global]

        # 7. Convertir el índice de regreso a ángulo en radianes
        raw_target_angle = self.MIN_ANGLE_RAD + (best_idx_global * scan_msg.angle_increment)

        # Rate limiter: no dejamos que el ángulo cambie más de MAX_STEER_DELTA_RAD
        # por scan. Esto es lo que corta el zigzag pared-a-pared: antes, un pequeño
        # cambio en qué índice "empataba" como máximo podía mandar el ángulo de
        # +0.02 a -0.15 de un frame a otro sin transición, y a 4 m/s eso se
        # traduce en un volantazo real.
        delta = raw_target_angle - self.prev_steering_angle
        delta = max(-self.MAX_STEER_DELTA_RAD, min(self.MAX_STEER_DELTA_RAD, delta))
        target_angle = self.prev_steering_angle + delta
        self.prev_steering_angle = target_angle

        # 8. Estrategia de velocidad reactiva
        target_velocity = self.compute_velocity(target_angle, min_distance, min_distance_front)

        # --- IMPRESIÓN DE LOGS DE MONITOREO CON CONTROL DE TIEMPO ---
        if self.SHOW_LOGS:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.last_log_time).nanoseconds / 1e9

            if elapsed_time >= self.LOG_PERIOD_SECONDS:
                self.get_logger().info(
                    f"\n--- TELEMETRÍA REACTIVA [Vuelta: {self.lap_count}/10] ---\n"
                    f" Muestras FOV: {total_samples} | Disparidades: {disparities_detected}\n"
                    f" Obstáculo más cercano (FOV completo): {min_distance:.2f} m (Índice: {min_idx})\n"
                    f" Obstáculo más cercano (cono frontal ±20°): {min_distance_front:.2f} m\n"
                    f" Gap Max Detectado: Índices [{max_start} a {max_end}] | Tamaño: {gap_size}\n"
                    f" Gap tras recorte de bordes: [{trimmed_start} a {trimmed_end}] "
                    f"(margen inicio: {start_margin_indices} idx | margen fin: {end_margin_indices} idx)\n"
                    f" Puntos candidatos a 'mejor punto': {len(candidate_indices)} (centro tomado)\n"
                    f" Distancia en Punto Elegido: {distancia_al_frente:.2f} m\n"
                    f" >> COMANDO DE CONTROL: Ángulo Giro: {target_angle:.3f} rad | Velocidad: {target_velocity:.1f} m/s\n"
                    f"--------------------------------------------------"
                )
                self.last_log_time = current_time

        self.publish_drive(target_velocity, target_angle)

    def compute_velocity(self, target_angle, min_distance, min_distance_front):
        """
        Velocidad = mínimo entre tres topes:
          1) ángulo de giro (igual que antes)
          2) distancia mínima en TODO el FOV (140°) - permisivo, solo para
             pasajes realmente angostos, ya no frena por paredes laterales.
          3) distancia mínima en el CONO FRONTAL angosto (~40° total) - estricto,
             porque una lectura corta ahí sí es peligro real de choque de frente
             (p.ej. la esquina de una curva que se acerca).
        """
        # Tope por ángulo
        if abs(target_angle) > 0.35:
            v_angle = 2.5   # antes 1.5
        elif abs(target_angle) > 0.15:
            v_angle = 4.0   # antes 2.5
        else:
            v_angle = self.MAX_SPEED

        # Tope por distancia global (FOV completo)
        v_dist = self.MAX_SPEED
        for dist_thresh, speed_cap in self.DIST_SPEED_CAPS:
            if min_distance < dist_thresh:
                v_dist = speed_cap
                break

        # Tope por distancia frontal (cono angosto)
        v_dist_front = self.MAX_SPEED
        for dist_thresh, speed_cap in self.FRONT_DIST_SPEED_CAPS:
            if min_distance_front < dist_thresh:
                v_dist_front = speed_cap
                break

        return min(v_angle, v_dist, v_dist_front)

    def _dynamic_edge_margin(self, obstacle_dist, angle_increment):
        """
        Calcula cuántos índices recortar en un borde del hueco, en función de
        qué tan cerca está el obstáculo que generó ese borde (mismo criterio
        atan2(car_radius, dist) que la burbuja de seguridad y el disparity
        extender). Un obstáculo cerca (curva cerrada) da un margen más
        grande; uno lejos (recta) da un margen chico.
        """
        obstacle_dist = max(obstacle_dist, 0.1)
        alpha = math.atan2(self.CAR_RADIUS_METERS, obstacle_dist)
        return int(math.ceil(alpha / angle_increment))

    def escape_direction(self, ranges):
        """
        Determina hacia qué lado girar en una maniobra de emergencia, en vez de
        usar siempre el mismo ángulo fijo. Compara el espacio acumulado en la
        mitad "derecha" (índices bajos, ángulos negativos) contra la mitad
        "izquierda" (índices altos, ángulos positivos) y gira hacia el lado
        con más espacio disponible.
        """
        mid = len(ranges) // 2
        right_space = sum(ranges[:mid])
        left_space = sum(ranges[mid:])
        return 0.4 if left_space > right_space else -0.4

    def find_max_gap(self, ranges, prev_idx=None):
        """
        Encuentra TODOS los huecos (segmentos contiguos con distancia > 0.1).

        FIX #1 (zigzag): si hay más de un hueco de tamaño similar (dentro de
        GAP_SIZE_TIE_TOLERANCE_INDICES), no nos quedamos ciegamente con el
        más grande: elegimos el que esté más cerca del hueco que se venía
        siguiendo (prev_idx). Esto evita que el auto salte de un hueco a otro
        de un frame a otro por un empate momentáneo entre dos aberturas
        parecidas (la causa típica del "mira a la izquierda y vuelve a la
        derecha" al pasar un obstáculo).
        """
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

        max_size = max(end - start for start, end in gaps)
        candidate_gaps = [
            (start, end) for start, end in gaps
            if (end - start) >= max_size - self.GAP_SIZE_TIE_TOLERANCE_INDICES
        ]

        if prev_idx is None or len(candidate_gaps) == 1:
            return max(candidate_gaps, key=lambda g: g[1] - g[0])

        def dist_to_prev(gap):
            center = (gap[0] + gap[1]) / 2.0
            return abs(center - prev_idx)

        return min(candidate_gaps, key=dist_to_prev)

    def publish_drive(self, velocity, steering_angle):
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