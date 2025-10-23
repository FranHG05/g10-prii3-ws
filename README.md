<h2>Sprint 2 — Control del JetBot mediante Gazebo</h2>
<h3>1) Clonar el repositorio</h3>

```bash
git clone https://github.com/FranHG05/g10-prii3-ws.git
```

<h3>2) Compilar el workspace</h3>

```bash
cd ~/g10-prii3-ws

colcon build

source install/setup.bash  
```


<h3>3) Lanzar Gazebo (Terminal 1)</h3>

Configurar modelo TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger
```
Lanzar mundo vacío en Gazebo

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

<h3>4) Lanzar el nodo que dibuja el 10 (Terminal 2) </h3>

```bash
cd ~/g10-prii3-ws
source install/setup.bash
ros2 launch g10_prii3_move_turtlebot draw_turtlebot.launch.py
```

<h3>5) Controlar la ejecución desde otra terminal SSH (Terminal 3)</h3>

Ejecutar los siguientes comandos:

<h4>Pausar el movimiento</h4>

```bash
ros2 service call /pause std_srvs/srv/Empty
```
<h4>Reanudar el movimiento</h4>

```bash
ros2 service call /resume std_srvs/srv/Empty
```

<h4>Reiniciar el dibujo</h4>

```bash
ros2 service call /reset std_srvs/srv/Empty
```



<h2>Sprint 2 — Control del JetBot mediante ROS2</h2>
<h3>1) Clonar el repositorio</h3>

```bash
git clone https://github.com/FranHG05/g10-prii3-ws.git
```

<h3>2) Compilar el workspace</h3>

```bash
cd ~/g10-prii3-ws

colcon build
```

<h3>3) Preparar el entorno en 3 terminales</h3>

Abrir terminal y conectarse al JetBot usando su IP:

```bash
ssh -X jetbot@172.16.190.154
```
El usuario y contraseña son:

```bash
Usuario: jetbot
Contraseña: jetbot
```
Cargar el entorno:
```bash
source install/setup.bash
```

<h3>4) Iniciar el driver del JetBot (Terminal 1)</h3>

Este nodo es el que permite mover físicamente los motores del JetBot y recibir datos de sus sensores.
```bash
ros2 run jetbot_pro_ros2 jetbot
```

<h3>5) Ejecutar el nodo principal del JetBot (Terminal 2) </h3>

```bash
ros2 launch g10_prii3_move_jetbot draw_jetbot.launch.py
```

<h3>6) Controlar la ejecución desde otra terminal SSH (Terminal 3)</h3>

Ejecutar los siguientes comandos:

<h4>Pausar el movimiento</h4>

```bash
ros2 service call /pause std_srvs/srv/Empty
```
<h4>Reanudar el movimiento</h4>

```bash
ros2 service call /resume std_srvs/srv/Empty
```

<h4>Reiniciar el dibujo</h4>

```bash
ros2 service call /reset std_srvs/srv/Empty
```


<h3>Estructura del proyecto</h3>


```bash
g10-prii3-ws/
├── README.md
└── src/
    ├── g10_prii3_move_jetbot/
    │   ├── g10_prii3_move_jetbot/
    │   │   ├── draw_number.py
    │   │   └── __init__.py
    │   ├── launch/
    │   │   └── draw_jetbot.launch.py
    │   ├── package.xml
    │   ├── resource/
    │   │   └── g10_prii3_move_jetbot
    │   ├── setup.py
    │   └── test/
    ├── g10_prii3_move_turtlebot/
    └── g10_prii3_turtlesim/

```



### Sprint 1

1) Clonar el repositorio:
```bash
git clone https://github.com/FranHG05/g10-prii3-ws
```
2) Compilar el workspace:
```bash
cd ~/g10-prii3-ws
colcon build
```
3) Cargar el entorno en dos terminales:
```bash
cd ~/g10-prii3-ws
source install/setup.bash
```
4) Ejecutar el nodo mediante el launch (terminal 1):
```bash
ros2 launch g10_prii3_turtlesim draw_turtle.launch.py
```

5) Control en la ejecución:

Pausar el dibujo (terminal 2):
```bash
ros2 service call /pause std_srvs/srv/Empty
```
Reanudar el dibujo (terminal 2):
```bash
ros2 service call /resume std_srvs/srv/Empty
```
Reiniciar el dibujo (terminal 2):
```bash
ros2 service call /reset std_srvs/srv/Empty
```
