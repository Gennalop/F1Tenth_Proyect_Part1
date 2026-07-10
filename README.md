# P2.ReactiveWithObstacles — Controlador Reactivo con Evasión de Obstáculos
 
Esta rama forma parte del proyecto **F1TENTH - Proyecto de Control Autónomo** (ver [rama `main`](../../tree/main) para la visión general del repositorio). Aquí se implementa la **Parte 2**: un controlador reactivo basado en **Follow the Gap**, capaz de recorrer la pista evitando tanto obstáculos estáticos del mapa como un segundo vehículo presente en la simulación.
 
## Enfoque del controlador
 
El controlador implementa una variante del algoritmo **Follow the Gap (FTG)**, cuyo funcionamiento general es el siguiente:
 
1. **Preprocesamiento del LIDAR**: se recorta el escaneo a un rango angular frontal de interés y se limpian valores inválidos (`NaN`, `inf`) o distancias excesivas.
2. **Detección de disparidades**: se identifican saltos bruscos entre lecturas consecutivas (bordes de obstáculos) y se "infla" el punto más cercano de cada disparidad, simulando el radio del vehículo para evitar que la trayectoria pase demasiado cerca del obstáculo.
3. **Burbuja de seguridad**: alrededor del punto más cercano detectado en todo el escaneo se genera una burbuja adicional que anula las lecturas cercanas a esa dirección, descartando esa zona como posible gap.
4. **Búsqueda del gap máximo**: sobre las lecturas restantes se buscan los espacios libres (gaps) contiguos, priorizando el más ancho y mejor alineado con el centro del vehículo. Se aplica además un seguimiento del gap anterior para dar continuidad a la trayectoria y evitar cambios bruscos de dirección entre ciclos de control.
5. **Selección del punto objetivo**: dentro del gap elegido se recorta un margen dinámico en los bordes (proporcional a la distancia del obstáculo adyacente) y se calcula el punto óptimo como una mezcla entre el punto más lejano del gap y su centro geométrico.
6. **Conversión a comando de control**: el índice del punto objetivo se traduce a un ángulo de giro, limitando la velocidad de cambio del ángulo entre ciclos para suavizar la conducción.
7. **Modos de emergencia**: si la distancia frontal cae por debajo de un umbral crítico, o si no se encuentra un gap viable, el controlador activa una maniobra de escape (girando hacia el lado con más espacio libre) reduciendo la velocidad.
8. **Velocidad dinámica**: la velocidad objetivo se ajusta en función del ángulo de giro y de la distancia libre en los conos frontal y medio del LIDAR, permitiendo acelerar en tramos despejados y frenar ante obstáculos o curvas cerradas.

## Estructura del repositorio
 
```
├── Map/
│   ├── BrandsHatch_map_obs.png      # Imagen del mapa con obstáculos
│   └── BrandsHatch_map_obs.yaml     # Configuración del mapa
├── Videos/                          # Grabaciones de pruebas en el simulador
├── f1tenth_racer/
│   ├── f1tenth_racer/
│   │   ├── __init__.py
│   │   ├── f1tenth_reactive_controller.py   # Nodo que controla el vehículo obstáculo
│   │   └── f1tenth_reactive_obs.py          # Nodo del controlador reactivo principal (ego)
│   ├── resource/
│   ├── test/
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
└── README.md
```
 
- **`f1tenth_reactive_obs.py`**: nodo principal del vehículo ego. Implementa el algoritmo Follow the Gap descrito arriba, se suscribe a `/scan` y `/ego_racecar/odom`, y publica comandos de conducción en `/drive`. Además, lleva el conteo de vueltas y registra telemetría periódica en consola.
- **`f1tenth_reactive_controller.py`**: nodo que controla al vehículo adicional, usado como obstáculo dinámico dentro de la pista.
- **`Map/`**: contiene el mapa `BrandsHatch_map_obs`, que incluye obstáculos estáticos y que debe configurarse en el simulador antes de ejecutar el proyecto.

## Instalación
 
1. **Clona esta rama del repositorio** dentro de la carpeta `src` de tu workspace:
```bash
   cd ~/f1tenth_ws/src
   git clone --branch P2.ReactiveWithObstacles --single-branch https://github.com/Gennalop/F1Tenth_Proyect_Part1.git f1tenth_racer
```
 
2. **Compila el paquete**:
```bash
   cd ~/f1tenth_ws
   colcon build f1tenth_racer
   source install/setup.bash
```
## Configuración del mapa
 
El simulador debe usar el mapa con obstáculos incluido en esta rama en lugar del mapa por defecto (`levine`):
 
1. **Copia los archivos del mapa** (`BrandsHatch_map_obs.png` y `BrandsHatch_map_obs.yaml`) desde la carpeta `Map/` de esta rama hacia la carpeta de mapas del simulador `~/F1Tenth-Repository/src/f1tenth_gym_ros/maps`.
 
2. **Edita el archivo de configuración del simulador**:
```
   ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
```
   Y actualiza los siguientes parámetros para apuntar al nuevo mapa y habilitar un segundo agente (el vehículo obstáculo):
 
```yaml
   # map parameters
   map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/BrandsHatch_map_obs'
   map_img_ext: '.png'
 
   # opponent parameters
   num_agent: 2
 
   # ego starting pose on map
   sx: 0.0
   sy: -0.4
   stheta: 0.0
 
   # opp starting pose on map
   sx1: -0.25
   sy1: 0.5
   stheta1: 0.0
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
 
2. **En una segunda terminal**, ejecuta el controlador reactivo del vehículo ego:
```bash
   cd ~/f1tenth_ws
   source install/setup.bash
   ros2 run f1tenth_racer reactive_obs
```
 
3. **En una tercera terminal**, ejecuta el controlador del vehículo obstáculo:
```bash
   cd ~/f1tenth_ws
   source install/setup.bash
   ros2 run f1tenth_racer reactive_controller
```
 
Con esto, en el simulador deberían aparecer ambos vehículos: el ego navegando la pista mediante Follow the Gap mientras evita obstáculos estáticos y al segundo vehículo, y el vehículo adicional siendo manejado por su propio nodo controlador.
 
## Videos
 
En la carpeta `Videos/` se incluyen grabaciones de pruebas del controlador en el simulador, útiles como referencia del comportamiento esperado. También puedes acceder siguiendo el enlace a continuación:
- Simulación ReactiveFollowGap: https://youtu.be/qeA9m8qlLKE

