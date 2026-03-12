# Colmena Robótica Colaborativa — Tesis de Licenciatura

Fork del repositorio del curso **"Robots Móviles 2026-2"** (https://github.com/mnegretev/Mobile-Robots-2026-2) de la Facultad de Ingeniería, UNAM,
adaptado para el desarrollo de una tesis sobre **Multi-Agent Path Finding (MAPF) con robots terrestres**.

## Descripción

Este proyecto implementa una colmena robótica colaborativa con múltiples robots diferenciales
en simulación (ROS 2 Jazzy + Gazebo Harmonic). Los robots deben coordinarse para desplazarse
a zonas objetivo evitando colisiones entre ellos y con obstáculos del entorno.

### Escenarios de prueba

| Escenario | Descripción |
|---|---|
| `towers` | Torres cilíndricas — navegación en campo abierto con obstáculos |
| `rocks`  | Rocas dispersas — simulación de terreno irregular |
| `maze`   | Laberinto de dos corredores — conflictos MAPF en espacios estrechos |

## Stack

- **ROS 2** Jazzy Jalisco
- **Gazebo** Harmonic (gz-sim 8)
- **Ubuntu** 24.04

## Requisitos

- Ubuntu 24.04
- Acceso a Internet y permisos de `sudo`
- ROS 2 Jazzy y Gazebo Harmonic

## Instalación

```bash
sudo apt update
sudo apt install -y git
git clone <url>
cd Mobile-Robots-2026-2
chmod +x Setup.sh
./Setup.sh
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
```

`Setup.sh` está pensado para una instalación limpia de Ubuntu 24.04. El script:

- configura el repositorio oficial de ROS 2 Jazzy
- instala ROS 2 Desktop, Gazebo Harmonic y dependencias del proyecto
- ejecuta `rosdep install` sobre `ros2_ws/src`
- compila el workspace con `colcon build`

Si `rosdep` ya estaba inicializado en la máquina, el script lo reutiliza.

## Verificación rápida

```bash
ros2 pkg list | grep swarm_bringup
ros2 launch swarm_bringup scenario.launch.py scenario:=towers
```

## Uso

### Lanzar un escenario

```bash
ros2 launch swarm_bringup scenario.launch.py scenario:=towers
ros2 launch swarm_bringup scenario.launch.py scenario:=rocks
ros2 launch swarm_bringup scenario.launch.py scenario:=maze
```

### Control remoto + monitor

```bash
ros2 run swarm_bringup swarm_teleop_gui.py
```

Muestra posición (x, y, θ), velocidades (vx, ω) y heading en tiempo real de los 3 robots,
con D-pad para control manual de cada uno.

## Estructura del repositorio

```
ros2_ws/src/
├── hardware/
│   └── justina_description/   # URDF del robot diferencial Justina
├── navigation/
│   ├── motion_planning/        # Utilidades de planeación
│   ├── navig_msgs/             # Mensajes ROS personalizados
│   ├── path_follower/          # Seguidor de trayectorias (Pure Pursuit)
│   └── path_planner/          # Planeador de rutas
└── swarm_bringup/             # Paquete principal de la tesis
    ├── launch/
    │   ├── scenario.launch.py  # Lanza cualquiera de los 3 escenarios
    │   └── swarm.launch.py     # 3 robots en mundo vacío (dev/debug)
    └── scripts/
        ├── swarm_teleop_gui.py # GUI de control + monitor
        └── swarm_monitor.py    # Monitor de posiciones (consola)
```

---

## Créditos

Este repositorio es un fork del trabajo del **Dr. Marco Negrete**:

> Dr. Marco Negrete
> Profesor Titular A — Jefe del Departamento de Procesamiento de Señales
> Facultad de Ingeniería, UNAM
> marco.negrete@ingenieria.unam.edu
> https://mnegretev.info

Repositorio original: https://github.com/mnegretev/Mobile-Robots-2026-2
