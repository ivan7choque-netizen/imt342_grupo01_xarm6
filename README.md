# Primer Parcial Práctico: Cinemática Directa e Inversa con ROS 2 Jazzy

**Materia:** IMT-342 Robótica -- Universidad Católica Boliviana ``San Pablo''  
**Docente:** Carlos Daniel Aguilar Mancachi  
**Grupo:** Grupo 1  
**Autores:** 
* Iván Alberto Choque Alba
* Rodrigo Murillo
**Robot Asignado:** UFACTORY xArm6 (6 Grados de Libertad)

---

## 1. Objetivo del Trabajo
Desarrollar, implementar y validar de forma teórica y práctica la cinemática directa (FK) mediante matrices de transformación homogénea basadas en la convención Denavit-Hartenberg estándar, y la cinemática inversa (IK) iterativa basada en el Jacobiano posicional y la pseudoinversa de un manipulador serial en ROS 2 Jazzy.

---

## 2. Software y Versiones Requeridas
* **Sistema Operativo:** Ubuntu 24.04 LTS
* **Middleware de Comunicaciones:** ROS 2 Jazzy
* **RMW por defecto:** CycloneDDS (`rmw_cyclonedds_cpp`)
* **Librerías Python:** NumPy, rclpy

---

## 3. Estructura del Repositorio
El repositorio está organizado conforme a los estándares de ROS 2, excluyendo carpetas de compilación temporales (`build/`, `install/`, `log/`):

grupo_01_xarm6_ws/
|-- src/
|   |-- grupo01_xarm6_bringup/
|   |   |-- launch/
|   |   |   `-- display.launch.py
|   |   `-- package.xml
|   |
|   |-- grupo01_xarm6_kinematics/
|   |   |-- grupo01_xarm6_kinematics/
|   |   |   |-- fk_node.py
|   |   `--   `-- ik_node.py
|   |   |-- package.xml
|   |   `-- setup.py
|   |
|   `-- xarm_ros2/ (Paquete oficial de descripción de UFACTORY)
|-- instalar.sh
|-- entorno.sh
|-- README.md
`-- .gitignore

---

## 4. Instrucciones de Clonación e Instalación (Computadora Nueva)

Para clonar y poner en marcha el proyecto en un entorno limpio con Ubuntu 24.04 y ROS 2 Jazzy, ejecute:

git clone https://github.com/ivan7choque-netizen/imt342_grupo01_xarm6.git
cd imt342_grupo01_xarm6

chmod +x instalar.sh abrir.sh recompilar.sh verificar.sh
./instalar.sh

El script `instalar.sh` se encarga de instalar las dependencias necesarias de ROS 2, clonar el repositorio oficial de descripción de UFACTORY, compilar el paquete bringup y configurar el middleware CycloneDDS.

---

## 5. Ejecución del Simulador (RViz2)

Para cargar el modelo del xArm6 en RViz2 junto con el control visual de articulaciones:

cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 launch grupo01_xarm6_bringup display.launch.py

*(O alternativamente usando el script rápido `./abrir.sh`)*.

---

## 6. Ejecución de los Nodos de Cinemática

### Cinemática Directa (FK)
El nodo `fk_node` se suscribe a `/joint_states`, evalúa las matrices Denavit-Hartenberg (DH) correspondientes al xArm6 (incluyendo los desfases angulares geométricos de las articulaciones 2 y 3) e imprime la posición cartesiana $(x, y, z)$ en consola.

En una nueva terminal:
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 run grupo01_xarm6_kinematics fk_node

### Cinemática Inversa (IK)
*Nota importante:* Antes de ejecutar la IK, asegúrese de cerrar la ventana de *Joint State Publisher GUI* para evitar conflictos de publicación simultánea sobre el tópico de estados articulares.

En una nueva terminal, ejecute el nodo de cinemática inversa:
cd ~/grupo_01_xarm6_ws
source entorno.sh
ros2 run grupo01_xarm6_kinematics ik_node

Para enviar un objetivo cartesiano $(x, y, z)$ al efector final del robot, publique un mensaje en el tópico `/target`:
ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.30, y: 0.20, z: 0.40}" --once

---

## 7. Tópicos Clave de ROS 2
* `/joint_states` (`sensor_msgs/msg/JointState`): Publica y lee la configuración angular de las 6 articulaciones del manipulador.
* `/target` (`geometry_msgs/msg/Point`): Recibe las coordenadas espaciales deseadas para resolver la posición mediante el Jacobiano numérico y la pseudoinversa.

---

## 8. Consideraciones Particulares y Errores Conocidos
* **Desfases angulares (Caso especial del xArm6):** Debido a la geometría propia del codo del xArm6, las transformaciones de las articulaciones 2 y 3 requieren los desfases de $\theta_2 = q_2 - 1.3849$ rad y $\theta_3 = q_3 + 1.3849$ rad ($\approx 79.35^\circ$), los cuales han sido debidamente integrados en los nodos de cálculo analítico y numérico para asegurar concordancia exacta con RViz2.
