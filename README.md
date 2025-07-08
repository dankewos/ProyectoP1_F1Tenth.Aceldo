# Proyecto P1 - F1TENTH: Controlador Reactivo Follow the Gap

## Descripción general

Este proyecto implementa un **controlador reactivo** para un vehículo autónomo tipo F1TENTH en un entorno simulado con ROS 2. El enfoque utilizado es **Follow the Gap**, un algoritmo que analiza datos LiDAR para identificar el espacio más seguro por donde avanzar y ajustar en tiempo real el ángulo de giro y la velocidad. Además del controlador, tiene integrado un sistema de **conteo de vueltas** y **cronometraje**, que detecta automáticamente cuando el vehículo completa una vuelta y registra su duración. Al alcanzar 10 vueltas, el sistema finaliza con un mensaje e imprime el **tiempo de vuelta más corto**.

---

## Enfoque utilizado: Follow the Gap + Temporizador y contador de vueltas

El algoritmo **Follow the Gap** identifica los obstáculos más cercanos usando LiDAR, elimina una burbuja de seguridad alrededor de ellos, y encuentra los segmentos del campo de visión que están despejados ("gaps"). Luego:

1. **Selecciona el mejor gap** basándose en:
   - Distancia promedio
   - Ancho del gap
   - Cercanía al centro del campo de visión

2. **Calcula el punto objetivo** dentro de ese gap (mezclando el punto más lejano con el centro del gap).

3. **Determina el ángulo de giro (steering)** hacia ese punto objetivo.

4. **Calcula la velocidad óptima** según:
   - La distancia a obstáculos frontales
   - El ángulo de giro (más cerrado = más lento)

---

## Explicación del archivo que controla este comportamiento del vehiculo autonomo. (Se lo puede encontrar como `proyecto.py`)

Este script contiene el nodo ROS 2 `ReactiveFollowGap`, que ejecuta el controlador reactivo y el sistema de conteo de vueltas. A continuación, se resumen sus partes principales:

### 🔹 Inicialización (`__init__`)
- Se definen los tópicos `/scan`, `/drive` y `/ego_racecar/odom`.
- Se establecen los parámetros del controlador, como velocidad base, umbrales de seguridad, filtros y radio de burbuja.
- Se crean los `publishers` y `subscribers`.

### 🔹 `preprocess_lidar()`
- Filtra y suaviza los datos LiDAR eliminando valores inválidos y ruidos.

### 🔹 `create_safety_bubble()`
- Borra datos LiDAR cercanos al obstáculo más próximo, creando una "burbuja de seguridad" para evitar colisiones.

### 🔹 `find_gaps()`
- Detecta zonas del escaneo donde no hay obstáculos (gaps) usando un umbral mínimo de seguridad.

### 🔹 `select_best_gap()`
- Selecciona el gap más conveniente combinando distancia, ancho y proximidad al centro del campo visual.

### 🔹 `get_target_point()`
- Dentro del gap elegido, calcula el índice del punto objetivo al que apuntará el vehículo.

### 🔹 `calculate_steering_angle()`
- Calcula el ángulo de giro necesario para alcanzar el punto objetivo.
- Aplica filtro suavizado para evitar cambios bruscos.

### 🔹 `calculate_speed()`
- Ajusta la velocidad del vehículo basándose en:
  - Proximidad a obstáculos frontales.
  - Ángulo de giro.

### 🔹 `lidar_callback()`
- Es el `callback` principal del controlador.
- Procesa el escaneo LiDAR y publica los comandos de movimiento (`AckermannDriveStamped`).

### 🔹 `odom_callback()`
- Se activa con la odometría del vehículo.
- Detecta si el carro entra a la "zona de inicio" y cuenta una vuelta solo si ya había salido antes.
- Mide el tiempo de cada vuelta y guarda el más corto.
- Al llegar a 10 vueltas, muestra el mensaje final con el tiempo más rápido.

---

## Estructura del Código

F1TENTH-REPOSITORY/
├── src/
│   ├── controllers/
│   │   ├── controllers/
│   │   │   ├── mpc_node.py
│   │   │   ├── pid_node.py
│   │   │   ├── proyecto.py         ← Código del controlador Follow the Gap
│   │   │   ├── purepursuit_node.py
│   │   │   ├── rrt_node.py
│   │   │   └── ttc_node.py
│   │   ├── resource/
│   │   │   └── controllers/
│   │   ├── test/
│   │   │   ├── test_copyright.py
│   │   │   ├── test_flake8.py
│   │   │   └── test_pep257.py
│   │   ├── package.xml
│   │   ├── setup.cfg
│   │   └── setup.py
│
│   ├── f1tenth_gym_ros/
│   │   ├── config/
│   │   │   └── sim.yaml             ← Posición inicial y parámetros del mapa
│   │   ├── f1tenth_gym_ros/
│   │   │   └── __init__.py
│   │   ├── launch/
│   │   │   ├── gym_bridge_launch.py
│   │   │   ├── gym_bridge.rviz
│   │   │   ├── ego_racecar.xacro
│   │   │   └── opp_racecar.xacro
│   │   ├── maps/
│   │   │   ├── levin.png
│   │   │   ├── levin.yaml
│   │   │   ├── levinB.png
│   │   │   ├── levinB.yaml
│   │   │   ├── SaoPaulo_map.png
│   │   │   ├── SaoPaulo_map.yaml
│   │   │   ├── Spielberg_map.png
│   │   │   ├── Spielberg_map.yaml
│   │   │   ├── Spielberg_obs_map.png
│   │   │   └── Spielberg_obs_map.yaml

---

## Instrucciones de Ejecución

### 1. Clonar el repositorio

```bash
git clone https://github.com/dankewos/ProyectoP1_F1Tenth.Aceldo.git
```

**### 2. Acceder a la ubicacion del repositorio**
```bash
cd ProyectoP1_F1Tenth.Aceldo
```
**### 3. Compilar y cargar el entorno**
```bash
colcon build
source install/setup.bash
```
**### 4. Ejecutar la simulacion y el archivo que contiene el proyecto**
```bash
ros2 launch f1tenth_gym_ros simulator_launch.py
ros2 run controllers followgap
```

**### Resultado Esperado**
El vehículo navega autónomamente sin chocar con las paredes.

Se imprimen en consola:

Vuelta completada

Tiempo de cada vuelta

Tiempo más corto

Mensaje final al llegar a 10 vueltas

**###Video de Demostración**
https://youtu.be/DYyZG_X7Nd0
🎥 Demostración completa del controlador Follow the Gap con sistema de conteo de vueltas y temporizador en el simulador F1TENTH.


Autor: Grazia Aceldo
Curso: Vehículos No Tripulados
Universidad: Escuela Superior Politecnica del Litoral
Repositorio: https://github.com/dankewos/ProyectoP1_F1Tenth.Aceldo
