# F1Tenth Learning Node
 
Sistema autónomo de aprendizaje en línea para ROS 2 que combina un controlador reactivo (Wall Following) para explorar el circuito en la primera vuelta y un seguidor de trayectoria (Pure Pursuit) para las vueltas de carrera.

## Enfoque — Wall Following + Pure Pursuit
 
**Vuelta 1 — Exploración (Wall Following PD)**
El robot se guía reactivamente comparando la distancia a las paredes izquierda y derecha con el LiDAR (±45°). Durante este recorrido registra waypoints cada 30 cm.
 
**Al completar la vuelta — Optimización**
La ruta grabada se suaviza y se le asigna velocidad.
 
**Vueltas 2 en adelante — Carrera (Pure Pursuit)**
El robot sigue los waypoints optimizados.
 
## Estructura del paquete

```
f1tenth_racer/                        # Carpeta raíz del repositorio (paquete ROS 2)
├── f1tenth_racer/                    # Paquete Python interno
│   ├── __init__.py
│   ├── f1tenth_learning_node.py	# Lógica
├── resource/
├── test/
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

## Requisitos previos

- **ROS 2 Humble** instalado y configurado.
- Un workspace de colcon existente (por ejemplo `~/f1tenth_ws`).
- **Simulador F1TENTH** instalado y funcionando. Puedes usar el siguiente repositorio:
  [https://github.com/widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

## Configuración del mapa
 
El simulador F1TENTH necesita que le indiques qué mapa usar. Debes reemplazar el mapa por defecto (`levine`) por el que se encuentra en la carpeta `map/` de este repositorio.
 
1. **Copia los archivos del mapa** (`.png` y `.yaml`) desde la carpeta `map/` de este repositorio hacia la carpeta de mapas del simulador.

2. **Actualiza la ruta del mapa** en el archivo de configuración del simulador:
```
   ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```
   Modifica la línea `map_path`, reemplazando el nombre del mapa (`levine`) por el nombre `BrandsHatch_map`.
   ```yaml
   map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map'
```
 
3. **Recompila el simulador** para que tome los cambios:
```bash
   cd ~/F1Tenth-Repository
   colcon build
   source install/setup.bash
```

## Instalación

### 1. Clonar el repositorio

Clona el paquete dentro de la carpeta `src` de tu workspace de ROS 2:

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/Gennalop/F1Tenth_Proyect_Part1
```

### 2. Compilar el paquete

```bash
cd ~/f1tenth_ws
colcon build
```

### 3. Configurar el entorno

Cada vez que abras una nueva terminal, recuerda hacer source del workspace:

```bash
source install/setup.bash
```

## Ejecución

Con el simulador o el vehículo F1TENTH ya corriendo y publicando `/scan` y `/ego_racecar/odom`, ejecuta el nodo:

```bash
ros2 run f1tenth_racer learning_node
```

## Tópicos

| Tópico | Tipo | Rol |
|---|---|---|
| `/scan` | `LaserScan` | Entrada — LiDAR |
| `/ego_racecar/odom` | `Odometry` | Entrada — Posición |
| `/drive` | `AckermannDriveStamped` | Salida — Velocidad y dirección |

## Videos

En la carpeta `Videos/` se incluyen grabaciones de pruebas del controlador en el simulador, útiles como referencia del comportamiento esperado o puedes acceder por medio del siguiente enlace:

- Simulación de LearningController: https://youtu.be/hlk6b5DTihs
