# Mobile-Robots-2026-2
Software para el curso "Robots Móviles" de la Facultad de Ingeniería, UNAM, 2026-2

## Requerimientos

* Ubuntu 24.04: https://ubuntu.com/download/desktop/thank-you?version=24.04.3&architecture=amd64&lts=true
* ROS Jazzy Jalisco: https://docs.ros.org/en/jazzy/Installation.html
* Google DeepMind MuJoCo: https://mujoco.readthedocs.io/en/latest/programming/#building-from-source

## Instalación

Nota: se asume que ya se tiene instalado Ubuntu, MuJoCo y ROS.

* $ cd
* $ git clone https://github.com/mnegretev/Mobile-Robots-2026-2/
* $ cd Mobile-Robots-2026-2
* $ sudo apt update
* $ sudo apt upgrade
* $ ./Setup.sh
* $ cd ros2_ws
* $ echo "alias cb='colcon build && source install/local_setup.bash'" >> ~/.bashrc
* $ source ~/.bashrc
* $ cb

## Pruebas

Para probar que todo se instaló y compiló correctamente:

* $ cd ~/Mobile-Robots-2026-1/ros2_ws
* $ cb
* $ ros2 launch house_simul house_simul.launch.py
  
Y en otra terminal:

* $ cd ~/Mobile-Robots-2026-1/ros2_ws
* $ cb
* $ ros2 launch motion_planning motion_planning.launch.py

Si todo se instaló y compiló correctamente, se debería ver un visualizador como el siguiente:
![rviz](https://github.com/mnegretev/Mobile-Robots-2026-2/blob/main/Media/rviz2.png)

Un ambiente simulado como el siguiente:
![gazebo](https://github.com/mnegretev/Mobile-Robots-2026-2/blob/main/Media/gz.png)

Y una GUI como la siguiente:
![GUIExample](https://github.com/mnegretev/Mobile-Robots-2026-2/blob/main/Media/gui.png)

## Contacto
Dr. Marco Negrete<br>
Profesor Titular A<br>
Jefe del Departamento de Procesamiento de Señales<br>
Facultad de Ingeniería, UNAM <br>
marco.negrete@ingenieria.unam.edu<br>
mnegretev.info<br>
https://mnegretev.info<br>

