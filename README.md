# hello_moveit & yolo_moveit

Este repositorio contiene dos nodos ROS 2 Humble que integran MoveIt para control de un robot Panda con pinza, demostrando movimiento básico y manipulación con detecciones YOLO 3D.

---

## Paquetes

### 1. `hello_moveit`

Nodo simple que:

- Controla la pinza para abrirla y cerrarla.
- Planifica un movimiento para acercar el brazo robótico a un objeto en coordenadas fijas.
- Realiza un descenso cartesiano directo hacia el objeto.
- Cierra la pinza para simular la sujeción.

---

### 2. `yolo_moveit`

Nodo avanzado que:

- Se suscribe a detecciones 3D publicadas en el tópico `/yolo/detections_3d` (mensaje `yolo_msgs/msg/DetectionArray`).
- Permite seleccionar por consola la clase de objeto a manipular.
- Planea y ejecuta movimiento para acercarse a la posición detectada del objeto.
- Abre la pinza, desciende hacia el objeto y cierra la pinza para sujetarlo.

---

## Requisitos

- ROS 2 Humble
- MoveIt 2
- Mensajes personalizados `yolo_msgs` con detecciones 3D (asegúrate de tener este paquete)
- Robot Panda configurado con los grupos `panda_arm` y `hand`

---

## Cómo compilar

Clona el repositorio dentro de tu workspace ROS 2 y compila con colcon:

```bash
colcon build --packages-select hello_moveit yolo_moveit
source install/setup.bash


## Cómo ejecutar

### Nodo hello_moveit

```bash
ros2 run hello_moveit hello_moveit
```

##Nodo yolo_moveit

```bash
ros2 run yolo_moveit yolo_moveit
```

## Descripción del código

- `hello_moveit` es un ejecutable que realiza un movimiento básico de aproximación y manipulación a un objeto fijo en el espacio.

- `yolo_moveit` es un nodo que escucha detecciones de objetos 3D y realiza manipulación selectiva en función de la clase elegida por el usuario.
