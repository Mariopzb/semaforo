# 🤖 PuzzleBot — Traffic Light Navigation System

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![Jetson](https://img.shields.io/badge/Jetson-Orin-76B900?style=for-the-badge&logo=nvidia&logoColor=white)

**Sistema autónomo de navegación por trayectorias con detección de semáforos en tiempo real**

</div>

---

## 📋 Tabla de Contenidos

- [Descripción General](#-descripción-general)
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Flujo de Datos ROS2](#-flujo-de-datos-ros2)
- [Nodos](#-nodos)
  - [turtle\_odometry](#1--turtle_odometrypy)
  - [path\_generator](#2--path_generatorpy)
  - [turtle\_closeloop](#3--turtle_closelooppy)
  - [semaforop](#4--semaforoppy)
- [Tópicos](#-tópicos)
- [Parámetros Clave](#-parámetros-clave)
- [Instalación y Uso](#-instalación-y-uso)
- [Configuración de la Cámara](#-configuración-de-la-cámara)
- [Figuras de Trayectoria](#-figuras-de-trayectoria)
- [Bugs Conocidos y Soluciones](#-bugs-conocidos-y-soluciones)

---

## 🧠 Descripción General

Este proyecto implementa un sistema de navegación autónoma para el robot **PuzzleBot** sobre **ROS2 Humble** corriendo en una **Jetson Orin**. El robot es capaz de:

- 📍 Seguir trayectorias geométricas (cuadrado, triángulo, trapecio) con control de lazo cerrado
- 🚦 Detectar semáforos en tiempo real mediante visión por computadora (HSV + OpenCV)
- 🛑 Modular su velocidad según el estado del semáforo (verde / amarillo / rojo)
- 📡 Estimar su posición en el espacio usando odometría de encoders

---

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────────┐
│                        JETSON ORIN                              │
│                                                                 │
│  ┌──────────────┐     ┌──────────────┐     ┌────────────────┐  │
│  │   CÁMARA     │     │  PATH        │     │   ODOMETRÍA    │  │
│  │  cpp_camera  │     │  GENERATOR   │     │  turtle_odom   │  │
│  │  (C++ node)  │     │  (Python)    │     │  (Python)      │  │
│  └──────┬───────┘     └──────┬───────┘     └───────┬────────┘  │
│         │                   │                      │           │
│    /video_source/raw    /next_point              /odom         │
│         │                   │                      │           │
│         ▼                   ▼                      ▼           │
│  ┌──────────────┐     ┌──────────────┐             │           │
│  │  SEMÁFORO    │     │   LAZO       │◄────────────┘           │
│  │  semaforop   │     │   CERRADO    │                         │
│  │  (Python)    │     │  turtle_cl.  │                         │
│  └──────┬───────┘     └──────┬───────┘                         │
│         │  /cmd_vel_raw      │                                 │
│         │◄───────────────────┘                                 │
│         │                                                       │
│    /cmd_vel                                                     │
│         │                                                       │
│         ▼                                                       │
│  ┌──────────────┐                                               │
│  │  PUZZLEBOT   │  ← Motores físicos                           │
│  │  serial_node │                                               │
│  └──────────────┘                                               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔄 Flujo de Datos ROS2

```
/camera_publisher
      │
      ▼
/video_source/raw ──────────────────────► /semaforop ◄─── /cmd_vel_raw
                                               │                 ▲
                                          /cmd_vel               │
                                               │           /turtle_closeloop
                                               ▼                 ▲
                                     /puzzlebot_serial_node      │
                                         │         │         /next_point
                                    /VelocityEncR  │              ▲
                                    /VelocityEncL  │              │
                                         │         │        /path_generator
                                         ▼         │              ▲
                                   /turtle_odometry│              │
                                         │         └──────────────┘
                                         ▼
                                       /odom ──────────────► /turtle_closeloop
                                                 └──────────► /path_generator
```

---

## 📦 Nodos

### 1. 🧭 `turtle_odometry.py`

**Propósito:** Estima la posición del robot integrando las velocidades angulares de los encoders.

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `r` | `0.052 m` | Radio de la rueda |
| `L` | `0.18 m` | Distancia entre ruedas |
| `rate` | `100 Hz` | Frecuencia de actualización |

**Suscripciones:**

| Tópico | Tipo | QoS |
|--------|------|-----|
| `/VelocityEncR` | `std_msgs/Float32` | sensor_data |
| `/VelocityEncL` | `std_msgs/Float32` | sensor_data |

**Publicaciones:**

| Tópico | Tipo |
|--------|------|
| `/odom` | `turtlesim/Pose` |

**Modelo cinemático:**
```
v     = r · (ωR + ωL) / 2
ω     = r · (ωR - ωL) / L
x    += dt · v · cos(θ)
y    += dt · v · sin(θ)
θ    += dt · ω
```

---

### 2. 🗺️ `path_generator.py`

**Propósito:** Genera la secuencia de puntos objetivo y los publica uno a uno conforme el robot los alcanza.

**Figuras disponibles** (descomentar en el código):

| Figura | Puntos |
|--------|--------|
| 🟦 Cuadrado | `[2,0] → [2,2] → [0,2] → [0,0]` |
| 🔺 Triángulo | `[2,0] → [1,1.732] → [0,0]` |
| 🔷 Trapecio | `[2,0] → [1.5,1] → [0.5,1] → [0,0]` |

**Suscripciones:**

| Tópico | Tipo |
|--------|------|
| `/odom` | `turtlesim/Pose` |

**Publicaciones:**

| Tópico | Tipo | Frecuencia |
|--------|------|-----------|
| `/next_point` | `turtlesim/Pose` | 2 Hz |

**Lógica:** Avanza al siguiente punto cuando la distancia euclidiana al objetivo es `< 0.1 m`.

---

### 3. 🎮 `turtle_closeloop.py`

**Propósito:** Controlador de lazo cerrado que lleva al robot de su posición actual al punto objetivo. Usa una máquina de estados de 2 fases.

**Máquina de estados:**

```
        ┌─────────────────────────────────────────┐
        │                                         │
        ▼                                         │
  ┌──────────┐   llegó al ángulo   ┌──────────┐   │ llegó al punto
  │ state1   │ ──────────────────► │ state2   │ ──┘
  │ girar    │                     │ avanzar  │
  └──────────┘                     └──────────┘
        ▲                               │
        │    nuevo punto recibido       │ punto alcanzado
        └───────────────────────────────┘
                    stop
```

**Ganancias del controlador:**

| Parámetro | Valor | Descripción |
|-----------|-------|-------------|
| `Kv` | `0.5` | Ganancia velocidad lineal |
| `Kw` | `1.2` | Ganancia velocidad angular |
| `v_limit` | `0.4 m/s` | Velocidad máxima |
| `tolerance_distance` | `0.02 m` | Tolerancia de posición |
| `tolerance_angle` | `0.03 rad` | Tolerancia de ángulo |

**Suscripciones:**

| Tópico | Tipo |
|--------|------|
| `/odom` | `turtlesim/Pose` |
| `/next_point` | `turtlesim/Pose` |

**Publicaciones:**

| Tópico | Tipo |
|--------|------|
| `/cmd_vel_raw` | `geometry_msgs/Twist` |

---

### 4. 🚦 `semaforop.py`

**Propósito:** Detecta el color del semáforo en la imagen de la cámara y modula la velocidad del robot según el estado actual.

**Máquina de estados del semáforo:**

```
                  ve amarillo
  ┌─────────┐ ──────────────────► ┌──────────┐
  │  GREEN  │                     │  YELLOW  │
  │  x1.0   │ ◄────────────────── │  x0.5    │
  └─────────┘   ve verde          └──────────┘
       ▲         (tras rojo)             │
       │                                │ ve rojo
       │                                ▼
       │                         ┌──────────┐
       └─────────────────────────│   RED    │
            ve verde             │  x0.0    │
            (seen_red=True)      └──────────┘
```

**Rangos HSV de detección:**

| Color | H min | H max | S min | V min |
|-------|-------|-------|-------|-------|
| 🟢 Verde | 40 | 80 | 50 | 50 |
| 🟡 Amarillo | 20 | 35 | 100 | 100 |
| 🔴 Rojo | 0–10 / 170–180 | — | 150 | 120 |

**Parámetro ajustable:**

```bash
# Cambiar distancia de detección en tiempo real
ros2 param set /traffic_supervisor min_area 1500
```

| `min_area` | Distancia aprox. |
|-----------|-----------------|
| 600 | ~1.5 m |
| 1000 | ~1.2 m (default) |
| 1500 | ~1.0 m |
| 3000 | ~0.7 m |

**Suscripciones:**

| Tópico | Tipo |
|--------|------|
| `/video_source/raw` | `sensor_msgs/Image` |
| `/cmd_vel_raw` | `geometry_msgs/Twist` |

**Publicaciones:**

| Tópico | Tipo |
|--------|------|
| `/cmd_vel` | `geometry_msgs/Twist` |

---

## 📡 Tópicos

| Tópico | Tipo | Publicado por | Suscrito por |
|--------|------|--------------|--------------|
| `/video_source/raw` | `sensor_msgs/Image` | `camera_publisher` | `semaforop` |
| `/VelocityEncR` | `std_msgs/Float32` | `puzzlebot_serial_node` | `turtle_odometry` |
| `/VelocityEncL` | `std_msgs/Float32` | `puzzlebot_serial_node` | `turtle_odometry` |
| `/odom` | `turtlesim/Pose` | `turtle_odometry` | `turtle_closeloop`, `path_generator` |
| `/next_point` | `turtlesim/Pose` | `path_generator` | `turtle_closeloop` |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | `turtle_closeloop` | `semaforop` |
| `/cmd_vel` | `geometry_msgs/Twist` | `semaforop` | `puzzlebot_serial_node` |

---

## ⚙️ Parámetros Clave

### Cámara (`cpp_camera`)

```xml
<param name="board"        value="jorin" />
<param name="mode"         value="2" />      <!-- 1280x720 -->
<param name="fps"          value="10" />     <!-- 100ms latencia -->
<param name="Compression"  value="true" />
<param name="JPEG_quality" value="50" />
```

| Mode | Resolución | FPS | Uso recomendado |
|------|-----------|-----|-----------------|
| 0 | 640×480 | 30 | Pruebas rápidas |
| **2** | **1280×720** | **30** | **Detección semáforo** ✅ |
| 5 | 1920×1080 | 30 | Alta precisión |

---

## 🚀 Instalación y Uso

### Crear el paquete desde cero

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python mi_paquete \
  --dependencies rclpy geometry_msgs sensor_msgs turtlesim std_msgs cv_bridge
```

### Copiar los nodos

```bash
cp turtle_odometry.py  ~/ros2_ws/src/mi_paquete/mi_paquete/
cp turtle_closeloop.py ~/ros2_ws/src/mi_paquete/mi_paquete/
cp path_generator.py   ~/ros2_ws/src/mi_paquete/mi_paquete/
cp semaforop.py        ~/ros2_ws/src/mi_paquete/mi_paquete/
```

### Registrar los entry points en `setup.py`

```python
entry_points={
    'console_scripts': [
        'turtle_odometry  = mi_paquete.turtle_odometry:main',
        'turtle_closeloop = mi_paquete.turtle_closeloop:main',
        'path_generator   = mi_paquete.path_generator:main',
        'semaforop        = mi_paquete.semaforop:main',
    ],
},
```

### Compilar y lanzar

```bash
cd ~/ros2_ws
colcon build --packages-select mi_paquete
source install/setup.bash
ros2 launch mi_paquete launch.xml
```

> 💡 **Tip:** Agrega `source ~/ros2_ws/install/setup.bash` a tu `~/.bashrc` para no olvidarlo nunca.

---

## 📷 Configuración de la Cámara

El nodo de cámara corre en C++ (`cpp_camera`) separado del paquete Python. Se lanza independientemente y publica en `/video_source/raw`.

```bash
# Lanzar la cámara por separado
ros2 launch cpp_camera camera.xml
```

---

## 📐 Figuras de Trayectoria

Para cambiar la figura, edita `path_generator.py` y descomenta el bloque deseado:

```python
# CUADRADO 2x2 metros
self.point_list = [
    [2.0, 0.0],
    [2.0, 2.0],
    [0.0, 2.0],
    [0.0, 0.0]
]

# TRIÁNGULO equilátero ~2m de lado
# self.point_list = [
#     [2.0, 0.0],
#     [1.0, 1.732],
#     [0.0, 0.0]
# ]

# TRAPECIO
# self.point_list = [
#     [2.0, 0.0],
#     [1.5, 1.0],
#     [0.5, 1.0],
#     [0.0, 0.0]
# ]
```

Luego reconstruye:
```bash
colcon build --packages-select mi_paquete && source install/setup.bash
```

---

## 🐛 Bugs Conocidos y Soluciones

| Bug | Causa | Solución |
|-----|-------|----------|
| `AttributeError: 'property' object has no attribute 'x'` | `stop_msg = Twist` sin `()` | Cambiar a `stop_msg = Twist()` |
| `AttributeError: 'function' object has no attribute 'info'` | `get_logger` sin `()` | Cambiar a `get_logger()` |
| `RCLError: rcl_shutdown already called` | `rclpy.shutdown()` llamado dos veces | Agregar guard `if rclpy.ok():` |
| Nodos duplicados (`/next_point1`) | Launch file lanzado dos veces | `pkill -f ros2` y relanzar |
| Cambios en `.py` no se reflejan | Falta `colcon build` | Siempre rebuildar tras editar |
| Robot no se mueve aunque `/cmd_vel_raw` tiene datos | `speed_multiplier = 0.0` por falsa detección | Ajustar `min_area` o rangos HSV |

---

## 👥 Autores

> Proyecto desarrollado para el curso de Robótica Móvil — **Jetson Orin + PuzzleBot**

---

<div align="center">

**Hecho con 🤖 y mucho `colcon build`**

</div>
