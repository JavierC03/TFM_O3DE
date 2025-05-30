# manipulation_o3de: test_moveit & yolo_moveit

Este repositorio contiene dos nodos ROS 2 Humble que integran MoveIt para controlar un robot Panda con pinza, demostrando movimiento básico y manipulación basada en detecciones YOLO 3D.

---

## Paquete

### `manipulation_o3de`

Contiene dos nodos ejecutables:

---

### 1. `test_moveit`

Nodo simple principalmente destinado a realizar pruebas que:

- Controla la pinza para abrirla y cerrarla.
- Planifica un movimiento para acercar el brazo robótico a un objeto en coordenadas fijas.
- Realiza un descenso cartesiano directo hacia el objeto.
- Cierra la pinza para simular la sujeción.

---

### 2. `yolo_moveit`

Nodo avanzado que:

- Se suscribe a detecciones 3D publicadas en el tópico `/yolo/detections_3d` (mensaje `yolo_msgs/msg/DetectionArray`).
- Permite seleccionar por consola la clase de objeto a manipular.
- Planea y ejecuta un movimiento para acercarse a la posición detectada del objeto.
- Abre la pinza, desciende hacia el objeto y la cierra para sujetarlo.

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
colcon build --packages-select manipulation_o3de
source install/setup.bash

```

## Cómo ejecutar

### Nodo hello_moveit

```bash
ros2 run hello_moveit hello_moveit
```

## Nodo yolo_moveit

```bash
ros2 run yolo_moveit yolo_moveit
```

## Descripción del código

- `hello_moveit` es un ejecutable que realiza un movimiento básico de aproximación y manipulación a un objeto fijo en el espacio.

- `yolo_moveit` es un nodo que escucha detecciones de objetos 3D y realiza manipulación selectiva en función de la clase elegida por el usuario.


## Script auxiliar: `crear_prefab.py`

Este script **no es un ejecutable del paquete ROS 2**, pero es una herramienta útil para preparar archivos `.prefab` modificados con posiciones aleatorias para simulaciones o entornos 3D.

### Funcionalidad principal

- Copia un archivo `.prefab` original a una carpeta destino.
- Modifica ciertas líneas del archivo para cambiar posiciones X e Y de objetos dentro del `.prefab` usando valores aleatorios dentro de rangos definidos.
- Intercambia las posiciones X e Y de ciertos objetos pequeños (denominados TOY) para mezclar su orden de forma aleatoria.
- Elimina el contenido previo de la carpeta destino antes de copiar el archivo modificado.

### Uso

Se ejecuta desde línea de comandos con los parámetros:

```bash
python3 crear_prefab.py --origen /ruta/al/archivo_original.prefab --destino /ruta/a/carpeta_destino
```
### Parámetros clave y comportamiento

- `--origen`: ruta completa al archivo `.prefab` original que se quiere copiar y modificar.

- `--destino`: carpeta donde se copiará el archivo modificado.

- Modifica líneas específicas del archivo que corresponden a posiciones X e Y de cubos y objetos pequeños TOY.

- Usa rangos predefinidos para generar valores aleatorios para las posiciones modificadas.

- Muestra mensajes en consola indicando los cambios realizados y posibles advertencias si las líneas indicadas no existen.

Este script facilita la generación rápida de prefabs con variabilidad en las posiciones de objetos para probar diferentes configuraciones de escenarios.
