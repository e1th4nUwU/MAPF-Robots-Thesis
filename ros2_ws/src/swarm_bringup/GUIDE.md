# Guia Tecnica: Multi-Robot Swarm en ROS 2 Jazzy + Gazebo Harmonic

> **Tesis:** Colmena Robotica Colaborativa: Planificacion de Rutas para Colmenas de Robots Terrestres
> **Autor:** Eithan Treviño | **Asesor:** Dr. Marco Negrete, LIRA, FI-UNAM
> **Stack:** ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8), Nav2, Python, Ubuntu 24.04

---

## Tabla de Contenidos

0. [Instalacion Desde Ubuntu 24.04 Limpio](#0-instalacion-desde-ubuntu-2404-limpio)
1. [Panorama General](#1-panorama-general)
2. [Analisis del Repositorio Original](#2-analisis-del-repositorio-original)
3. [El Robot: Justina Base](#3-el-robot-justina-base)
4. [Arquitectura Multi-Robot](#4-arquitectura-multi-robot)
5. [Archivos Creados: Explicacion Linea por Linea](#5-archivos-creados-explicacion-linea-por-linea)
6. [Como Lanzar y Probar](#6-como-lanzar-y-probar)
7. [Debugging de Topics](#7-debugging-de-topics)
8. [Siguientes Pasos](#8-siguientes-pasos)

---

## 0. Instalacion Desde Ubuntu 24.04 Limpio

Esta seccion asume una maquina recien instalada con **Ubuntu 24.04**, sin ROS 2, sin Gazebo y sin dependencias previas del curso.

### 0.1 Que instala el proyecto

El script `Setup.sh` ahora deja lista la maquina para trabajar con este repositorio. Hace lo siguiente:

1. habilita el repositorio `universe` de Ubuntu
2. agrega el repositorio oficial de **ROS 2 Jazzy**
3. instala **ROS 2 Desktop**, **Gazebo Harmonic**, Nav2, MoveIt, `ros_gz`, SUMO y utilidades de desarrollo
4. instala dependencias Python del proyecto, incluyendo `python3-tk` para la GUI `swarm_teleop_gui.py`
5. ejecuta `rosdep install` sobre `ros2_ws/src`
6. compila el workspace con `colcon build`
7. agrega a `~/.bashrc` el source de ROS y del workspace

### 0.2 Instalacion paso a paso

```bash
sudo apt update
sudo apt install -y git

git clone <url-del-repo>
cd Mobile-Robots-2026-2
chmod +x Setup.sh
./Setup.sh
```

Cuando termine, abre una terminal nueva. Si no quieres reiniciar la shell, ejecuta manualmente:

```bash
source /opt/ros/jazzy/setup.bash
source <ruta-al-repo>/ros2_ws/install/setup.bash
```

### 0.3 Verificacion minima

```bash
ros2 pkg list | grep swarm_bringup
ros2 pkg list | grep ros_gz_bridge
ros2 launch swarm_bringup scenario.launch.py scenario:=towers
```

Si el ultimo comando abre Gazebo y aparecen los 3 robots, la instalacion quedo funcional.

### 0.4 Notas importantes

- El script esta pensado para **Ubuntu 24.04**. No esta preparado para 22.04 ni para otras distribuciones.
- `Setup.sh` usa `sudo`, conexion a Internet y acceso al repositorio oficial de ROS.
- Si `rosdep` ya estaba inicializado en la maquina, el script lo detecta y no vuelve a correr `rosdep init`.
- El script compila el workspace automaticamente; no hace falta correr `colcon build` por separado despues de instalar.

---

## 1. Panorama General

### Que estamos construyendo

Un sistema donde **3 robots diferenciales identicos** conviven en un mismo mundo simulado, cada uno con sus propios sensores y comunicacion independiente, pero coordinados por un nodo central.

### Por que es no-trivial

En ROS 2, si lanzas 3 robots sin cuidado, los tres publican en `/odom`, `/cmd_vel`, `/scan`, etc. Sus datos se mezclan y no puedes distinguir quien es quien. La solucion es **namespaces**: cada robot vive en su propio "directorio" de topics (`/robot1/odom`, `/robot2/odom`, `/robot3/odom`).

Ademas, Gazebo Harmonic y ROS 2 son **sistemas separados** que no se ven entre si. Necesitas un **bridge** (puente) que traduzca los topics de Gazebo a topics de ROS 2 y viceversa.

### Diagrama de arquitectura

```
+------------------------------------------------------------------+
|                      GAZEBO HARMONIC                             |
|                   (un solo proceso)                              |
|                                                                  |
|  +----------+       +----------+       +----------+              |
|  |  robot1  |       |  robot2  |       |  robot3  |              |
|  | DiffDrive|       | DiffDrive|       | DiffDrive|              |
|  |  LiDAR   |       |  LiDAR   |       |  LiDAR   |              |
|  +----+-----+       +----+-----+       +----+-----+              |
|       |                  |                  |                    |
|  /model/robot1/*    /model/robot2/*    /model/robot3/*           |
+-------+------------------+------------------+--------------------+
        |                  |                  |
   gz topics          gz topics          gz topics
        |                  |                  |
  +-----+------+    +-----+------+    +-----+------+
  | bridge_1   |    | bridge_2   |    | bridge_3   |
  | (ros_gz)   |    | (ros_gz)   |    | (ros_gz)   |
  +-----+------+    +-----+------+    +-----+------+
        |                  |                  |
   /robot1/*          /robot2/*          /robot3/*
   (ROS 2)            (ROS 2)            (ROS 2)
        |                  |                  |
        +------------------+------------------+
                           |
                +----------+----------+
                |   swarm_monitor.py  |
                | (suscrito a los 3   |
                |  topics /*/odom)    |
                +---------------------+
```

---

## 2. Analisis del Repositorio Original

### Estructura relevante

```
ros2_ws/src/
  hardware/
    justina_description/       <-- URDF del robot
      urdf/
        justina.xacro          <-- Archivo principal (ensambla las partes)
        justina_base.xacro     <-- Base diferencial (lo que usamos)
        justina_upper.xacro    <-- Torso + cabeza + camara (NO lo usamos)
    house_simul/               <-- Mundo de Gazebo (la casa)
      worlds/
        house.world            <-- Archivo SDF del mundo
      config/
        gz_bridge.yaml         <-- Bridge original (para 1 robot con brazo)
      launch/
        house_simul.launch.py  <-- Launch original (complejo, usa MoveIt)
    xarm/                      <-- Brazo manipulador (NO lo usamos)
```

### Hallazgo clave: el brazo ya estaba deshabilitado

En `justina.xacro` linea 4:

```xml
<!-- <xacro:include filename="$(find justina_description)/urdf/justina_upper.xacro"/> -->
```

Esa linea esta **comentada**. Significa que el Dr. Negrete ya dejo la opcion de usar solo la base. No tuvimos que modificar ningun archivo del repositorio original.

### Por que no reutilizamos el launch original

El launch `house_simul.launch.py` invoca `mbot_demo` que carga:
- MoveIt (planificacion de brazo manipulador)
- xarm controllers (controladores del brazo)
- SRDF (descripcion semantica del brazo)

Nada de eso aplica para la colmena. Ademas, ese launch solo soporta 1 robot. Necesitabamos uno nuevo desde cero.

---

## 3. El Robot: Justina Base

### Que es un archivo Xacro

Xacro ("XML Macro") es una extension de URDF que permite usar variables, matematicas y macros. En vez de escribir numeros a mano, puedes hacer:

```xml
<xacro:property name="wheel_radius" value="0.07"/>
<cylinder radius="${wheel_radius}" length="${wheel_width}"/>
```

Cuando se procesa, `${wheel_radius}` se reemplaza por `0.07`. Esto hace el modelo parametrico y facil de modificar.

### Anatomia de justina_base.xacro

```
justina_base.xacro
|
|-- Propiedades (lineas 6-45)
|   Variables numericas: dimensiones, masas, inercias
|
|-- Links (lineas 47-165)
|   |-- base_link         (frame de referencia, sin geometria)
|   |-- body_link         (caja azul: 0.45 x 0.35 x 0.1 m)
|   |-- left_wheel_link   (cilindro negro: r=0.07, w=0.04)
|   |-- right_wheel_link  (cilindro negro: identico)
|   |-- caster_link       (esfera negra: r=0.07, friccion ~0)
|   |-- laser_link        (cilindro gris: el LiDAR)
|
|-- Joints (lineas 167-206)
|   |-- base_to_body      (fixed: base_link -> body_link)
|   |-- left_wheel_joint  (continuous: gira sin limite)
|   |-- right_wheel_joint (continuous: gira sin limite)
|   |-- caster_joint      (fixed: la bola no tiene motor)
|   |-- laser_joint       (fixed: el LiDAR esta pegado al cuerpo)
|
|-- Gazebo: Friccion (lineas 208-232)
|   Llantas: mu=1.0 (alta friccion, no patinan)
|   Caster:  mu=0.000001 (casi cero, se desliza libremente)
|
|-- Gazebo: Plugins (lineas 234-321)
    |-- DiffDrive    (control de velocidad + odometria)
    |-- JointState   (publica posiciones de joints)
    |-- gpu_lidar    (sensor LiDAR simulado)
```

### Que es un Link

Un **link** es una pieza rigida del robot. Cada link tiene 3 componentes:

- **visual**: como se VE (la geometria renderizada en Gazebo)
- **collision**: como CHOCA (puede ser mas simple que la visual, para performance)
- **inertial**: como se MUEVE (masa + tensor de inercia para la fisica)

Si omites `inertial`, Gazebo tratara la pieza como si tuviera masa cero y el robot "explotara" en la simulacion.

### Que es un Joint

Un **joint** conecta dos links y define como se mueven entre si:

| Tipo | Descripcion | Ejemplo en Justina |
|------|-------------|---------------------|
| `fixed` | No se mueve, union rigida | base_to_body, laser_joint |
| `continuous` | Gira sin limite (como una llanta) | left_wheel_joint |
| `revolute` | Gira con limites (min/max angulo) | (no usado en la base) |
| `prismatic` | Se desliza linealmente | (no usado) |

### El plugin DiffDrive

Este es el "cerebro" del movimiento. En `justina_base.xacro` lineas 236-248:

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <right_joint>right_wheel_joint</right_joint>
  <left_joint>left_wheel_joint</left_joint>
  <wheel_separation>${wheel_separation}</wheel_separation>
  <wheel_radius>${wheel_radius}</wheel_radius>
  <topic>cmd_vel</topic>              <!-- Escucha comandos de velocidad -->
  <odom_topic>odom</odom_topic>       <!-- Publica odometria -->
</plugin>
```

**Como funciona:**
1. Recibe un mensaje `Twist` en el topic `cmd_vel` (velocidad lineal + angular)
2. Calcula las velocidades de cada llanta usando la cinematica diferencial:
   - `v_left  = v_linear - v_angular * wheel_separation / 2`
   - `v_right = v_linear + v_angular * wheel_separation / 2`
3. Aplica esas velocidades a los joints de las llantas
4. Calcula la odometria (posicion estimada) integrando las velocidades y la publica

**Topics relativos en Gazebo Harmonic:**
Cuando escribes `<topic>cmd_vel</topic>`, Gazebo automaticamente le antepone el nombre del modelo. Si el robot se llama `robot1` en Gazebo, el topic real sera `/model/robot1/cmd_vel`. Esto es clave para multi-robot: cada instancia tiene sus propios topics sin configuracion extra.

### El sensor LiDAR

En lineas 294-321:

```xml
<sensor name="laser" type="gpu_lidar">
  <topic>scan</topic>
  <lidar>
    <scan>
      <horizontal>
        <samples>480</samples>           <!-- 480 rayos -->
        <min_angle>-2.0944</min_angle>   <!-- -120 grados -->
        <max_angle>2.0944</max_angle>    <!-- +120 grados -->
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>                     <!-- No ve mas cerca de 10 cm -->
      <max>10.0</max>                    <!-- Alcance maximo 10 m -->
    </range>
  </lidar>
</sensor>
```

Simula un LiDAR 2D con campo de vision de 240 grados. Sera util para la deteccion de obstaculos y la navegacion.

---

## 4. Arquitectura Multi-Robot

### El problema fundamental

Gazebo y ROS 2 son sistemas **completamente independientes**:

- **Gazebo** tiene su propio sistema de mensajeria (`gz::msgs`) y sus propios topics
- **ROS 2** tiene el suyo (`std_msgs`, `nav_msgs`, etc.) con DDS como middleware

No se ven entre si. El paquete `ros_gz_bridge` es el intermediario que:
1. Se suscribe a un topic de Gazebo
2. Convierte el mensaje al formato ROS 2
3. Lo publica en un topic de ROS 2
(y viceversa para comandos como `cmd_vel`)

### Por que un bridge por robot

Cada robot en Gazebo genera topics con su nombre de modelo:

| Robot en Gazebo | Topic de odometria en Gazebo | Topic deseado en ROS 2 |
|-----------------|------------------------------|------------------------|
| robot1 | `/model/robot1/odom` | `/robot1/odom` |
| robot2 | `/model/robot2/odom` | `/robot2/odom` |
| robot3 | `/model/robot3/odom` | `/robot3/odom` |

Cada bridge necesita un archivo YAML que le diga exactamente que topic de Gazebo mapear a que topic de ROS 2. Como los nombres son diferentes para cada robot, necesitamos un YAML por robot.

### Namespaces en ROS 2

Un **namespace** es como un directorio para topics. Si un nodo corre en el namespace `/robot1`, todos sus topics se prefijan automaticamente:

```
Sin namespace:           Con namespace /robot1:
  /odom           -->      /robot1/odom
  /cmd_vel        -->      /robot1/cmd_vel
  /scan           -->      /robot1/scan
```

En el launch file, usamos `PushRosNamespace('robot1')` para lograr esto.

### Robot State Publisher (RSP)

Cada robot necesita su propio RSP. Este nodo:
1. Lee el URDF
2. Escucha `/joint_states` (posiciones de las llantas)
3. Calcula y publica las transformadas TF (donde esta cada pieza del robot en 3D)

Sin TF, herramientas como RViz y Nav2 no pueden visualizar ni navegar el robot.

El parametro `frame_prefix` es crucial para multi-robot:
```python
'frame_prefix': f'{name}/',  # ej: 'robot1/'
```
Esto convierte `base_link` en `robot1/base_link`, evitando colisiones entre los 3 TF trees.

---

## 5. Archivos Creados: Explicacion Linea por Linea

### 5.1 package.xml

```xml
<package format="3">
  <name>swarm_bringup</name>
```

Define el paquete ROS 2. Cada paquete es una unidad de software con nombre unico.

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
```

Usamos CMake como sistema de build (vs `ament_python` que es la otra opcion).
Se eligio CMake porque es consistente con los demas paquetes del repo.

```xml
  <exec_depend>justina_description</exec_depend>
  <exec_depend>house_simul</exec_depend>
```

`exec_depend` = "necesito este paquete instalado para EJECUTAR". No para compilar (`build_depend`), porque nuestro paquete no tiene codigo C++ que compilar. Solo necesita los archivos de los otros paquetes en runtime.

### 5.2 CMakeLists.txt

```cmake
install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)
```

Copia las carpetas al directorio de instalacion. Sin esto, `ros2 launch` no encontraria los archivos.

```cmake
install(
  PROGRAMS scripts/swarm_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`PROGRAMS` (vs `FILES`) preserva el permiso de ejecucion. `lib/${PROJECT_NAME}` es donde ROS 2 busca ejecutables cuando haces `ros2 run swarm_bringup swarm_monitor.py`.

### 5.3 gz_bridge_robotN.yaml

Cada archivo mapea 5 topics para un robot:

```yaml
# 1. CLOCK: Gazebo -> ROS
#    Para que ROS use el tiempo de simulacion (no el del reloj de tu PC).
#    use_sim_time:=True en los nodos depende de esto.
- ros_topic_name: "robot1/clock"
  gz_topic_name: "/clock"
  direction: GZ_TO_ROS

# 2. ODOMETRIA: Gazebo -> ROS
#    Posicion y velocidad estimadas del robot.
#    El plugin DiffDrive la calcula integrando las velocidades de las llantas.
- ros_topic_name: "robot1/odom"
  gz_topic_name: "/model/robot1/odom"
  direction: GZ_TO_ROS

# 3. VELOCIDAD: ROS -> Gazebo
#    Comandos de movimiento. Tu nodo publica aqui, Gazebo mueve el robot.
- ros_topic_name: "robot1/cmd_vel"
  gz_topic_name: "/model/robot1/cmd_vel"
  direction: ROS_TO_GZ

# 4. LIDAR: Gazebo -> ROS
#    Datos del sensor laser. El topic en Gazebo es largo porque incluye
#    world/model/link/sensor en la ruta.
- ros_topic_name: "robot1/scan"
  gz_topic_name: "/world/default/model/robot1/link/laser_link/sensor/laser/scan"
  direction: GZ_TO_ROS

# 5. JOINT STATES: Gazebo -> ROS
#    Posiciones angulares de las llantas. El RSP los necesita para TF.
- ros_topic_name: "robot1/joint_states"
  gz_topic_name: "/world/default/model/robot1/joint_state"
  direction: GZ_TO_ROS
```

**Nota sobre `direction`:**
- `GZ_TO_ROS`: Gazebo es el productor, ROS es el consumidor (sensores, odometria)
- `ROS_TO_GZ`: ROS es el productor, Gazebo es el consumidor (comandos de velocidad)

### 5.4 swarm.launch.py

Un launch file es un script Python que describe que nodos lanzar y con que parametros. Equivale a abrir 10+ terminales a mano.

#### Constantes de robots

```python
ROBOTS = [
    {'name': 'robot1', 'x': '-2.0', 'y': '-1.0', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '-2.0', 'y':  '0.0', 'yaw': '0.0'},
    {'name': 'robot3', 'x': '-2.0', 'y':  '1.0', 'yaw': '0.0'},
]
```

Posiciones iniciales separadas 1 metro en Y para que no se spawnen encima.
El `name` es critico: se usa como nombre del modelo en Gazebo Y como namespace en ROS.

#### Procesamiento del URDF

```python
xacro_file = os.path.join(justina_pkg, 'urdf', 'justina.xacro')
robot_description_content = xacro.process_file(xacro_file).toxml()
```

`xacro.process_file()` expande todas las variables y macros del Xacro, produciendo un URDF puro (XML plano). Se procesa **una sola vez** porque los 3 robots usan el mismo modelo.

#### Lanzar Gazebo

```python
gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    ])),
    launch_arguments={'gz_args': f'-r -v 3 {gz_world}'}.items(),
)
```

- `IncludeLaunchDescription`: equivale a un `#include` -- ejecuta otro launch file
- `ros_gz_sim/gz_sim.launch.py`: launch file generico que arranca Gazebo
- `-r`: "run" -- empieza la simulacion inmediatamente (sin pausar)
- `-v 3`: verbosidad nivel 3 (util para debug, reduce a 1 en produccion)

#### Bridge del reloj global

```python
clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='clock_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
)
```

Sintaxis especial del bridge inline: `topic_ros@tipo_ros[tipo_gz`
- `[` significa GZ_TO_ROS (el bracket "apunta" hacia la izquierda, hacia ROS)
- `]` significaria ROS_TO_GZ

Solo necesitamos UN bridge de reloj (no uno por robot) porque el tiempo es global.

#### GroupAction + PushRosNamespace

```python
group = GroupAction([
    PushRosNamespace(name),
    Node(
        package='robot_state_publisher',
        ...
        parameters=[{
            'frame_prefix': f'{name}/',
        }],
    ),
])
```

`GroupAction` agrupa nodos. `PushRosNamespace` hace que todos los nodos dentro del grupo usen ese namespace. El RSP dentro del grupo de `robot1` publicara en `/robot1/tf` en vez de `/tf`.

`frame_prefix` agrega un prefijo a los nombres de los frames TF. Sin esto, los 3 robots publicarian un frame llamado `base_link` y TF no sabria cual es cual.

#### Spawn de entidades

```python
spawn = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', f'{name}/robot_description',
        '-name', name,
        '-x', robot['x'], '-y', robot['y'], '-z', '0.5', '-Y', robot['yaw'],
    ],
)
```

- `ros_gz_sim create`: le pide a Gazebo que cree una entidad
- `-topic`: lee el URDF de este topic ROS (lo publica el RSP)
- `-name`: nombre unico en Gazebo (**debe coincidir** con los bridge configs)
- `-z 0.5`: spawn a 50cm de altura para que caiga al suelo (evita que aparezca dentro del piso)

### 5.5 swarm_monitor.py

#### Estructura del nodo

```python
class SwarmMonitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
```

`Node` es la clase base de ROS 2. Todo nodo hereda de ella. `'swarm_monitor'` es el nombre del nodo (visible en `ros2 node list`).

#### Subscripciones con lambda

```python
for name in ROBOT_NAMES:
    self.create_subscription(
        Odometry,                                      # tipo de mensaje
        f'/{name}/odom',                               # topic
        lambda msg, n=name: self._odom_callback(msg, n), # callback
        10,                                            # QoS (buffer size)
    )
```

**Por que `n=name`:** Sin esto, la lambda capturaria la *variable* `name`, no su *valor*. Al final del loop, `name == 'robot3'` y las 3 lambdas llamarian al callback con `'robot3'`. El truco `n=name` captura el valor actual en cada iteracion. Esto es un clasico "gotcha" de Python.

**QoS 10:** Quality of Service. Mantiene los ultimos 10 mensajes en buffer. Si el callback es lento y llegan mensajes rapido, los mas viejos se descartan. 10 es el default convencional para topics de odometria.

#### Conversion de quaternion a yaw

```python
def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
```

ROS representa orientaciones como **quaterniones** (x, y, z, w) en vez de angulos de Euler. Los quaterniones evitan el "gimbal lock" y son mas estables numericamente. Pero para un robot en el piso, solo nos importa el **yaw** (rotacion alrededor de Z). Esta formula extrae justo eso.

#### Timer vs callback directo

```python
self.create_timer(1.0, self._print_positions)
```

El plugin DiffDrive publica odometria a **100 Hz**. Si imprimieramos en cada callback, la terminal se inundaria. El timer imprime un resumen cada 1 segundo, mientras los callbacks solo actualizan el diccionario en memoria.

---

## 6. Como Lanzar y Probar

### Compilar

```bash
cd ~/Desktop/Mobile-Robots-2026-2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to swarm_bringup
source install/setup.bash
```

`--packages-up-to` compila el paquete y todas sus dependencias que no esten compiladas.

### Terminal 1: Lanzar la simulacion

```bash
source ~/Desktop/Mobile-Robots-2026-2/ros2_ws/install/setup.bash
ros2 launch swarm_bringup swarm.launch.py
```

Deberias ver Gazebo abrirse con la casa y 3 robots azules en posiciones diferentes.

### Terminal 2: Ejecutar el monitor

```bash
source ~/Desktop/Mobile-Robots-2026-2/ros2_ws/install/setup.bash
ros2 run swarm_bringup swarm_monitor.py
```

Deberia imprimir cada segundo algo como:

```
[INFO] [swarm_monitor]: --- Swarm Positions ---
[INFO] [swarm_monitor]:   robot1: x=-2.000  y=-1.000  yaw=+0.0 deg
[INFO] [swarm_monitor]:   robot2: x=-2.000  y=+0.000  yaw=+0.0 deg
[INFO] [swarm_monitor]:   robot3: x=-2.000  y=+1.000  yaw=+0.0 deg
```

### Terminal 3: Mover un robot

```bash
source ~/Desktop/Mobile-Robots-2026-2/ros2_ws/install/setup.bash

# Mover robot1 hacia adelante a 0.5 m/s:
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Girar robot2 en su lugar:
ros2 topic pub /robot2/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.5}}'

# Detener robot1 (Ctrl+C en el pub anterior, o):
ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

### Verificar topics

```bash
# Ver todos los topics activos:
ros2 topic list

# Deberias ver algo como:
#   /robot1/odom
#   /robot1/cmd_vel
#   /robot1/scan
#   /robot2/odom
#   /robot2/cmd_vel
#   ...

# Escuchar odometria de robot1 en tiempo real:
ros2 topic echo /robot1/odom

# Ver frecuencia de publicacion:
ros2 topic hz /robot1/odom
```

---

## 7. Debugging de Topics

### Problema mas comun: "el topic no tiene datos"

Esto pasa cuando los nombres de topics del bridge no coinciden con los que Gazebo genera. Los nombres exactos dependen de:

1. **Nombre del world** en el SDF (`<world name='default'>`)
2. **Nombre del modelo** en Gazebo (el `-name` del spawn)
3. **Nombre del topic** en el plugin (ej: `<odom_topic>odom</odom_topic>`)

### Como diagnosticar

```bash
# Ver TODOS los topics que Gazebo esta publicando:
gz topic -l

# Buscar topics de un robot especifico:
gz topic -l | grep robot1

# Escuchar un topic de Gazebo directamente (sin pasar por ROS):
gz topic -e -t /model/robot1/odom
```

Si `gz topic -l` muestra `/model/robot1/odom` pero el bridge config dice `/model/robot1/odometry`, el bridge no conectara nada. **Ajusta el YAML para que coincida exactamente.**

### Tabla de topics esperados en Gazebo

| Recurso | Patron del topic en Gazebo |
|---------|----------------------------|
| cmd_vel | `/model/<name>/cmd_vel` |
| odom | `/model/<name>/odom` |
| joint_states | `/world/default/model/<name>/joint_state` |
| LiDAR | `/world/default/model/<name>/link/laser_link/sensor/laser/scan` |
| clock | `/clock` (global, no por modelo) |

> **Importante:** Si el world se llamara `house` en vez de `default`, los topics con `/world/default/` cambiarian a `/world/house/`. Siempre verifica con `gz topic -l`.

---

## 8. Siguientes Pasos

Segun el plan de trabajo del Dr. Negrete:

### Paso 4: Comunicacion entre robots (actual -> siguiente)

Expandir `swarm_monitor.py` para:
- Publicar las posiciones de los 3 robots en un topic compartido (`/swarm/positions`)
- Implementar un mensaje custom o usar `PoseArray`
- Que cada robot "sepa" donde estan los demas

### Paso 5: Tarea colaborativa

Ideas iniciales:
- **Formacion**: los 3 robots mantienen un triangulo equilatero mientras se mueven
- **Cobertura**: cubrir un area dividiendola en 3 zonas
- **Rendezvous**: llegar los 3 a un punto desde posiciones diferentes sin colisionar

### Paso 6: Algoritmos MAPF

Multi-Agent Path Finding. Algoritmos a investigar:
- **CBS (Conflict-Based Search)**: optimo, escala bien para pocos agentes
- **ORCA (Optimal Reciprocal Collision Avoidance)**: reactivo, bueno para tiempo real
- **Priority-Based Planning**: simple, asigna prioridades y planifica secuencialmente
- **A* con reserva de tiempo**: extiende A* al espacio (x, y, t)

---

## Estructura final del paquete

```
ros2_ws/src/swarm_bringup/
|-- CMakeLists.txt
|-- package.xml
|-- GUIDE.md                      <-- Este archivo
|-- launch/
|   +-- swarm.launch.py           <-- Lanza Gazebo + 3 robots + bridges
|-- config/
|   |-- gz_bridge_robot1.yaml     <-- Bridge topics para robot1
|   |-- gz_bridge_robot2.yaml     <-- Bridge topics para robot2
|   +-- gz_bridge_robot3.yaml     <-- Bridge topics para robot3
+-- scripts/
    +-- swarm_monitor.py          <-- Nodo que monitorea posiciones
```
