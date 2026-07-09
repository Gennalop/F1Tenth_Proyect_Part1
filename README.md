# F1TENTH - Proyecto de Control Autónomo

Este repositorio contiene el desarrollo de controladores para un vehículo autónomo simulado en el entorno **F1TENTH**, implementados sobre **ROS 2**. El proyecto está dividido en dos partes, cada una desarrollada en ramas separadas.

## Estructura del proyecto

El desarrollo se organiza en ramas, ya que cada parte y variante del proyecto requiere su propia configuración de mapa, nodos y parámetros. El detalle específico de cada nodo, sus archivos y el mapa utilizado se documentan en el README correspondiente de cada rama.

### Parte 1: Conducción en pista sin obstáculos

En esta parte se desarrolló un controlador para que el vehículo recorra una pista sin obstáculos. Se implementaron dos enfoques distintos, cada uno en su propia rama:

- **`P1.ReactiveControler`**: Controlador puramente reactivo, que toma decisiones de conducción en tiempo real a partir de las lecturas del sensor (LIDAR).

- **`P1.LearningControler`**: Controlador mixto reactivo, que primero recorre la pista guardando *waypoints* (puntos de referencia del camino) y luego los utiliza para seguir la trayectoria.

### Parte 2: Conducción con obstáculos y otro vehículo

En esta parte se incorporaron obstáculos estáticos en el mapa, además de un segundo vehículo en la pista.

- **`P2.ReactiveWithObstacles`**: Controlador basado en Following the Gap adaptado para detectar y evitar obstáculos en el mapa.

> Para más detalles sobre la implementación, nodos y archivos específicos de cada enfoque, revisa el README de la rama correspondiente.

## Requisitos previos

- **ROS 2 Humble** instalado y configurado. Puedes seguir la guía oficial de instalación aquí:
  [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)
- Un workspace de colcon existente (por ejemplo `~/f1tenth_ws`).
- **Simulador F1TENTH** instalado y funcionando. Puedes usar el siguiente repositorio:
  [https://github.com/widegonz/F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository)

## Configuración del mapa

El simulador F1TENTH necesita que le indiques qué mapa usar. Debes reemplazar el mapa por defecto (`levine`) por el correspondiente a cada rama, el cual se encuentra en la carpeta `map/` de dicha rama.

1. **Copia los archivos del mapa** (`.png` y `.yaml`) desde la carpeta `map/` de la rama correspondiente hacia la carpeta de mapas del simulador F1TENTH `~/F1Tenth-Repository/src/f1tenth_gym_ros/maps`

3. **Actualiza la ruta del mapa** en el archivo de configuración del simulador:
   ```
   ~/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml
   ```
   Modifica la línea `map_path`, reemplazando el nombre del mapa (`levine`) por el nombre del mapa correspondiente (indicado en el README de cada rama).
   ```yaml
   map_path: '/home/<tu_usuario>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/<nombre_del_mapa>'
   ```

4. **Recompila el simulador** para que tome los cambios:
   ```bash
   cd ~/F1Tenth-Repository
   colcon build
   source install/setup.bash
   ```

## Ramas del repositorio

| Rama | Descripción |
|------|-------------|
| `P1.ReactiveControler` | Controlador reactivo puro - Parte 1 |
| `P1.LearningControler` | Controlador mixto con waypoints - Parte 1 |
| `P2.ReactiveWithObstacles` | Controlador reactivo con obstáculos y otro vehículo - Parte 2 |
