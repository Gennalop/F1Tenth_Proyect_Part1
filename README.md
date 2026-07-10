# P1.ReactiveController — Controlador Puramente Reactivo

Esta rama forma parte del proyecto **F1TENTH - Proyecto de Control Autónomo** (ver [rama `main`](../../tree/main) para la visión general del repositorio). Aquí se implementa la **Parte 1**: un controlador puramente reactivo que permite al vehículo recorrer la pista sin obstáculos, tomando decisiones de conducción en tiempo real a partir de las lecturas del LIDAR.

## Enfoque del controlador

El controlador se basa en un esquema reactivo de **seguimiento de paredes (wall following) asistido por proyección de distancias y control PD**, con el siguiente flujo general:

1. **Preprocesamiento del LIDAR**: se limpian valores inválidos (`NaN`, `inf`) del escaneo y se acotan las distancias a un rango máximo de trabajo.
2. **Suavizado de disparidades**: de forma similar a Follow the Gap, se detectan saltos bruscos entre lecturas consecutivas (bordes de obstáculos o esquinas de la pista) y se "infla" el punto más cercano, evitando que la trayectoria calculada pase demasiado cerca de un borde.
3. **Proyección de paredes izquierda y derecha**: usando dos rayos del LIDAR a un ángulo lateral y otro con una diferencia angular (`theta`), se estima la distancia proyectada hacia cada pared considerando un horizonte de anticipación (*lookahead*), lo que permite predecir la posición del vehículo respecto a la pista un instante hacia adelante.
4. **Cálculo del error de centrado**: se compara la proyección de la pared izquierda contra la derecha para obtener un error de posición lateral. Si solo una pared es visible, o ninguna, el controlador aplica reglas de respaldo (sesgar el error hacia el lado contrario o mantener el último error válido) para no perder la referencia.
5. **Control PD sobre el ángulo de giro**: el error de centrado (y su derivada) se convierte en un ángulo de giro mediante un controlador Proporcional-Derivativo, limitado a un ángulo máximo de dirección.
6. **Evasión crítica de paredes**: si alguna proyección lateral indica que el vehículo está demasiado cerca de una pared, se fuerza un error de corrección más agresivo para alejarlo antes de que el error normal del PD reaccione.
7. **Velocidad dinámica**: la velocidad se ajusta en función del ángulo de giro (menos velocidad a mayor giro) y de la distancia libre en el cono frontal, incluyendo frenado preventivo ante curvas cercanas, freno de emergencia ante obstáculos inminentes y un modo *boost* que incrementa la velocidad en tramos rectos y despejados.

## Estructura del repositorio

```
├── Map/
│   ├── BrandsHatch_map.png         # Imagen del mapa sin obstáculos
│   └── BrandsHatch_map.yaml        # Configuración del mapa
├── Videos/                           # Grabaciones de pruebas en el simulador
├── f1tenth_racer/
│   ├── f1tenth_racer/
│   │   ├── __init__.py
│   │   └── f1tenth_reactive_controller.py   # Nodo único del controlador reactivo
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
└── README.md
```

- **`f1tenth_reactive_controller.py`**: nodo único del vehículo ego. Se suscribe a las lecturas del LIDAR (`/scan`) y a la odometría (`/ego_racecar/odom`), y publica los comandos de conducción en `/drive`. Además, lleva el conteo de vueltas y registra telemetría periódica en consola.

## Requisitos previos

- **ROS 2 Humble** instalado y configurado ([guía oficial de instalación](https://docs.ros.org/en/humble/Installation.html)).
- Un workspace de colcon existente (por ejemplo `~/f1tenth_ws`).
- **Simulador F1TENTH** instalado y funcionando, disponible en: [https://github.com/widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

## Instalación

1. **Clona esta rama del repositorio** dentro de la carpeta `src` de tu workspace:

   ```bash
   cd ~/f1tenth_ws/src
   git clone --branch P1.ReactiveControler --single-branch https://github.com/Gennalop/F1Tenth_Proyect_Part1.git f1tenth_racer
   ```

2. **Compila el paquete**:

   ```bash
   cd ~/f1tenth_ws
   colcon build
   source install/setup.bash
   ```

## Configuración del mapa

Al igual que en las demás ramas, el simulador debe usar el mapa correspondiente a esta parte del proyecto en lugar del mapa por defecto (`levine`). En este caso **no es necesario configurar un segundo vehículo**, ya que la pista no incluye obstáculos ni un oponente.

1. **Copia los archivos del mapa** (`BrandsHatch_map.png` y `BrandsHatch_map.yaml`) desde la carpeta `Map/` de esta rama hacia la carpeta de mapas del simulador `~/F1Tenth-Repository/src/f1tenth_gym_ros/maps`.

2. **Edita el archivo de configuración del simulador**:

   ```
   ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   ```

   Y actualiza la ruta del mapa, manteniendo un único agente (no se requiere oponente):

   ```yaml
   # map parameters
   map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map'
   map_img_ext: '.png'

   # opponent parameters
   num_agent: 1

   # ego starting pose on map
   sx: 0.0
   sy: 0.0
   stheta: 0.0
   ```

   > Reemplaza `<tu_usuario>` por tu nombre de usuario del sistema.

3. **Recompila el simulador** para que tome los cambios:

   ```bash
   cd ~/F1Tenth-Repository
   colcon build
   source install/setup.bash
   ```

## Ejecución

1. **Levanta el simulador** en una primera terminal:

   ```bash
   cd ~/F1Tenth-Repository
   source install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```

2. **En una segunda terminal**, ejecuta el controlador reactivo:

   ```bash
   cd ~/f1tenth_ws
   source install/setup.bash
   ros2 run f1tenth_racer reactive_controller
   ```

## Videos

En la carpeta `Videos/` se incluyen grabaciones de pruebas del controlador en el simulador, útiles como referencia del comportamiento esperado o puedes acceder por medio del siguiente enlace:

- Simulación de ReactiveController: https://youtu.be/BvCblsCIRD0 (Mejor tiempo - Vuelta10 = 69.698)
