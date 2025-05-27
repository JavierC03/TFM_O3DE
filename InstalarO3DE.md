# Introducción

Este documento recoge el proceso de instalación, configuración y
primeros pasos con *Open 3D Engine* (O3DE) en entorno Linux, así como su
integración con *ROS 2 Robotic Manipulation Template*, una plantilla de
simulación para brazos robóticos basada en MoveIt 2.

El objetivo es proporcionar una guía práctica orientada a facilitar la
puesta en marcha del entorno.

# Requisitos Previos

Antes de proceder con la creación y configuración del proyecto, es
fundamental asegurarse de que se cumplen los requisitos previos tanto
para Open 3D Engine (O3DE) como para la plantilla *ROS 2 Robotic
Manipulation Template*.

Para la instalación de O3DE, se deben consultar y seguir las
instrucciones oficiales disponibles en:

<https://development--o3deorg.netlify.app/docs/welcome-guide/requirements/>

En esta página se detallan las dependencias necesarias para diferentes
sistemas operativos, así como recomendaciones sobre versiones de
compiladores, librerías y herramientas asociadas.

Respecto a la plantilla *ROS 2 Robotic Manipulation Template*, es
necesario disponer de un entorno ROS 2 Humble funcional y configurado,
junto con las dependencias específicas del paquete de simulación. Para
instalar estas dependencias, se puede ejecutar el siguiente comando
(adaptando `$ROS_DISTRO` a la versión de ROS 2 instalada):

``` bash
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-resources ros-${ROS_DISTRO}-depth-image-proc
``` 

En mi caso particular, la versión de ROS 2 utilizada es `humble`, por lo
que el comando específico es:

``` bash
sudo apt install ros-humble-moveit ros-humble-moveit-resources ros-humble-depth-image-proc
``` 

Se recomienda seguir las guías oficiales de ROS 2 y MoveIt 2 para
asegurar una correcta instalación y configuración.

# Instalación

Open 3D Engine (O3DE) es un motor de juego y simulación de código
abierto que, para aprovechar al máximo sus capacidades y flexibilidad,
se recomienda instalar compilándolo directamente desde su código fuente.
Esta opción garantiza el acceso a las últimas funcionalidades y una
integración más directa con el entorno de desarrollo.

Sin embargo, debido a la complejidad del proceso de compilación y las
múltiples dependencias implicadas, en determinados entornos y
configuraciones puede resultar complicado completar esta instalación
desde cero.

En mi caso particular, tras realizar varias pruebas exhaustivas con la
compilación desde fuente en un entorno Linux, se presentaron
dificultades que hicieron inviable este método. Por ello, opté por
instalar O3DE utilizando los paquetes precompilados oficiales. Esta
alternativa permite una instalación más ágil.

# Creación del Proyecto

Una vez instalado Open 3D Engine (O3DE), el siguiente paso es la
creación de un proyecto de simulación. Existen dos métodos principales
para llevar a cabo esta tarea:

  - **Creación mediante la interfaz gráfica de O3DE Editor**

  - **Creación a través de línea de comandos**

A continuación, se describen ambos métodos con detalle para facilitar su
uso en diferentes contextos.

## Creación mediante la interfaz gráfica de O3DE Editor

Para iniciar la creación de un proyecto desde la interfaz gráfica de
O3DE Editor, una vez lanzado el editor desde la consola con el entorno
de ROS 2 correctamente cargado, se debe seguir el siguiente
procedimiento:

1.  En la pantalla de inicio de O3DE Editor, hacer clic en el botón
    **New Project...** situado en la parte derecha.

2.  Aparecerá una nueva pantalla en la que será necesario definir:
    
      - **Nombre del proyecto**.
    
      - **Ruta de almacenamiento**, que se actualizará automáticamente
        según el nombre asignado. Por defecto, si el proyecto se llama
        `NewProject`, la ruta será `/home/user/O3DE/Projects/NewProject`.

3.  En la parte inferior, se deberá seleccionar una plantilla de
    proyecto. Si la plantilla requerida aún no está descargada, al
    seleccionarla aparecerá una opción que permitirá descargarla
    directamente desde el editor.

4.  Tras seleccionar la plantilla y pulsar en **Create**, es posible que
    el proceso muestre un error indicando la ausencia de ciertas *Gems*
    necesarias.

5.  En este caso, será necesario regresar a la pantalla de inicio del
    editor, acceder al menú de **Gems** y buscar manualmente las *Gems*
    indicadas en el mensaje de error.

6.  Este proceso puede repetirse varias veces, ya que podrían surgir
    dependencias adicionales no resueltas inicialmente. Una vez todas
    las *Gems* requeridas estén correctamente añadidas, se podrá
    proceder nuevamente a la creación del proyecto.

7.  Si la creación es exitosa, se mostrará una notificación indicando
    que el proyecto requiere compilación. Desde la pantalla de inicio,
    el proyecto aparecerá listado y se habilitará la opción de
    **compilarlo**.

8.  Al ejecutar la compilación, si no se presentan errores, el proyecto
    quedará listo para ser utilizado desde el editor.

## Creación de un proyecto ROS 2 en O3DE desde línea de comandos

La creación de un proyecto de simulación utilizando ROS 2 en O3DE puede
realizarse íntegramente desde la línea de comandos. Este procedimiento
permite configurar correctamente el entorno, evitar errores comunes y
automatizar la integración con ROS 2.

A continuación, se describe paso a paso el proceso recomendado:

### Instalación de dependencias ROS 2

En primer lugar, es necesario instalar las dependencias de ROS 2 que
permitirán la correcta integración con O3DE. Para ello, se deben
instalar los siguientes paquetes, utilizando la variable de entorno
`$ROS_DISTRO` que corresponde a la distribución de ROS 2 instalada:

``` bash
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-control-msgs ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-vision-msgs
```

En mi caso particular, utilizo la distribución `humble`, por lo que los
comandos equivalentes son:

``` bash
sudo apt install ros-humble-ackermann-msgs ros-humble-control-msgs ros-humble-nav-msgs ros-humble-gazebo-msgs ros-humble-xacro ros-humble-vision-msgs
```

Adicionalmente, para evitar posibles errores en el manejo de
transformaciones, se recomienda instalar el siguiente paquete:

``` bash
sudo apt install ros-humble-tf2-ros
```

### Clonado de O3DE Extras y registro de Gem

Se procede a clonar el repositorio `o3de-extras` desde GitHub:

``` bash
git clone https://github.com/o3de/o3de-extras
```

Luego, se procede a registrar las *Gems* correspondientes a ROS 2. Para
ello, es necesario indicar la ruta donde se ha clonado el repositorio
`o3de-extras` y la ruta donde está instalado O3DE.

En mi caso particular, las rutas utilizadas son las siguientes (estas
rutas pueden variar según la instalación de cada usuario):

``` bash
O3DE_EXTRAS_HOME=${HOME}/o3de-extras
O3DE_HOME=/opt/O3DE/24.09.2
```

Se registra la *Gem ROS2*:

``` bash
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/ROS2
```

### Registro de todas las Gems y plantillas

Para asegurarse de que todas las *Gems* y plantillas estén disponibles,
ejecutar:

``` bash
cd ${O3DE_EXTRAS_HOME}
git lfs install && git lfs pull
${O3DE_HOME}/scripts/o3de.sh register --all-gems-path ${O3DE_EXTRAS_HOME}/Gems/
${O3DE_HOME}/scripts/o3de.sh register --all-templates-path ${O3DE_EXTRAS_HOME}/Templates/
```

### Creación del proyecto

El proyecto se crea utilizando la plantilla `Ros2ProjectTemplate`. A
continuación se muestra un ejemplo de cómo se podrían definir variables
para el nombre del proyecto y la ruta donde se desea crear, aunque en
realidad no definí estas variables previamente, y el comando se puede
ejecutar directamente especificando las rutas completas.

``` bash
PROJECT_NAME=ProyectoPrueba
PROJECT_PATH=${HOME}/projects/${PROJECT_NAME}
${O3DE_HOME}/scripts/o3de.sh create-project --project-path $PROJECT_PATH --template-path ${O3DE_EXTRAS_HOME}/Templates/Ros2ProjectTemplate
```

Estos valores corresponden a mi caso particular, pero pueden variar
según la configuración y preferencias de cada usuario. Se recomienda
adaptar tanto el nombre del proyecto como la ruta (`–project-path`) y la
ruta de la plantilla (`–template-path`) según la ubicación y nombres
deseados.

### Compilación del proyecto

Dentro del directorio del proyecto, ejecutar:

``` bash
cd $PROJECT_PATH
cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON
cmake --build build/linux --config profile --target ${PROJECT_NAME} Editor ${PROJECT_NAME}.Assets
```

Es posible que la segunda ejecución de `cmake –build` arroje errores de
permisos en directorios internos de O3DE. En tal caso, ejecutar:

``` bash
sudo chmod -R 777 /opt/O3DE/24.09.1/bin/Linux/profile/Default/
```

Posteriormente, volver a ejecutar el comando de compilación:

``` bash
cmake --build build/linux --config profile --target ${PROJECT_NAME} Editor ${PROJECT_NAME}.Assets
```

# Ejecución del proyecto

Una vez completada la compilación, el proyecto puede abrirse mediante
O3DE utilizando el procedimiento habitual descrito en la sección de
interfaz gráfica.

Al iniciar O3DE, en la pantalla principal aparecerá una lista con los
proyectos disponibles. Es necesario seleccionar el proyecto
correspondiente y pulsar el botón **Open Editor**. Cabe destacar que, si
la compilación se realizó correctamente, esta será la única opción
habilitada para acceder al proyecto.

Al abrir el Editor, se mostrará una ventana para elegir el simulador o
entorno que se desea editar. En esta ventana, se debe hacer clic en el
botón **Open…**, lo que abrirá el explorador de niveles del proyecto.

Dentro de la estructura de carpetas, se encontrará una carpeta
denominada **Levels**, la cual contiene dos archivos. De entre ellos, se
debe seleccionar la carpeta **RoboticManipulation** y, posteriormente,
pulsar el botón **Open** para cargar el nivel correspondiente en el
editor.

Una vez abierto el editor del simulador, para iniciar la ejecución es
necesario pulsar el botón **Play**, representado por un triángulo
ubicado en la esquina superior derecha de la interfaz. Al comenzar la
simulación, se debe abrir una terminal y dirigirse al directorio raíz
del proyecto. Desde dicha ubicación, se lanza el entorno de ROS 2
ejecutando el comando:

``` bash
ros2 launch Examples/panda_moveit_config_demo.launch.py
``` 

Este procedimiento inicia la configuración y el nodo de demostración
para el control del brazo robótico dentro del simulador.
