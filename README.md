# F1Tenth Learning Node
 
Sistema autónomo de aprendizaje en línea para ROS 2 que combina un controlador reactivo (Wall Following) para explorar el circuito en la primera vuelta y un seguidor de trayectoria (Pure Pursuit) para las vueltas de carrera.
 
## Enfoque — Wall Following + Pure Pursuit
 
**Vuelta 1 — Exploración (Wall Following PD)**
El robot se guía reactivamente comparando la distancia a las paredes izquierda y derecha con el LiDAR (±45°). Durante este recorrido registra waypoints cada 30 cm.
 
**Al completar la vuelta — Optimización**
La ruta grabada se suaviza y se le asigna velocidad.
 
**Vueltas 2 en adelante — Carrera (Pure Pursuit)**
El robot sigue los waypoints optimizados.
 
## Estructura
 
```
F1TenthLearningNode
├── odom_callback()       # Actualiza posición, registra waypoints, detecta vueltas
├── scan_callback()       # Wall Following (EXPLORING) o Pure Pursuit (RACING)
└── optimize_trajectory() # Suavizado + asignación de velocidades por curvatura
```
 
## Ejecución
 
```bash
# Compilar
colcon build
source install/setup.bash
 
# Ejecutar
ros2 run f1tenth_racer learning_node
```
