<h2>Sprint 3 — Fase Beta del Sprint</h2>
<h3>1) Conectarse al JetBot con su IP (Terminal 1, 2 y 3)</h3>

```bash
ssh -X jetbot@172.16.190.154
```
El usuario y contraseña son:

```bash
Usuario: jetbot
Contraseña: jetbot
```

<h3>2) Clonar el repositorio (Terminal 1)</h3>

```bash
git clone https://github.com/FranHG05/g10-prii3-ws.git
```

<h3>3) Compilar el workspace y cargar el entorno ESTE ES PARA turtlebot(Terminal 1)</h3>

```bash
cd ~/g10-prii3-ws

colcon build --packages-select g10_prii3_nav_turtlebot

source install/setup.bash  
```

<h3>4) Gazebo turtlebot</h3>

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

<h3>5) Rviz ESTE COMANDO ES PARA turtlebot OJO</h3>

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/marolc/g10-prii3-ws/src/g10_prii3_nav_turtlebot/worlds/map.yaml
```

<h3>6) Iniciar el driver del JetBot (Terminal 2)</h3>

Este nodo es el que permite mover físicamente los motores del JetBot y recibir datos del LiDAR.
```bash
ros2 run jetbot_pro_ros2 jetbot
```

<h2>Sprint 2 — Simulación con Gazebo</h2>
<h3>1) Clonar el repositorio (Terminal 1)</h3>

```bash
git clone https://github.com/FranHG05/g10-prii3-ws.git
```

<h3>2) Compilar el workspace y cargar el entorno (Terminal 1)</h3>

```bash
cd ~/g10-prii3-ws

colcon build --packages-select g10_prii3_move_turtlebot

source install/setup.bash  
```


<h3>3) Lanzar Gazebo (Terminal 2)</h3>

Configurar modelo TurtleBot3

```bash
export TURTLEBOT3_MODEL=burger
```
Lanzar mundo vacío en Gazebo

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

<h3>4.1) Lanzar el launch que dibuja el 10 y para antes de chocar (Terminal 1) </h3>

Se ha añadido en el launch un cubo para demostrar la parada, para que siga dibujando el 10 debe eliminarse manualmente en el Gazebo.

```bash
ros2 launch g10_prii3_move_turtlebot draw_turtlebot.launch.py
```

<h3>4.2) Lanzar el launch que dibuja el 10 y esquiva (Terminal 1) </h3>

```bash
ros2 launch g10_prii3_move_turtlebot obstacle_avoidance.launch.py
```

<h3>5) Controlar la ejecución (Terminal 3) </h3>

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


<h2>Sprint 2 — Movimiento del JetBot</h2>
<h3>1) Conectarse al JetBot con su IP (Terminal 1, 2 y 3)</h3>

```bash
ssh -X jetbot@172.16.190.154
```
El usuario y contraseña son:

```bash
Usuario: jetbot
Contraseña: jetbot
```

<h3>2) Clonar el repositorio (Terminal 1)</h3>

```bash
git clone https://github.com/FranHG05/g10-prii3-ws.git
```

<h3>3) Compilar el workspace y cargar el entorno (Terminal 1)</h3>

```bash
cd ~/g10-prii3-ws

colcon build --packages-select g10_prii3_move_jetbot

source install/setup.bash  
```

<h3>4) Iniciar el driver del JetBot (Terminal 2)</h3>

Este nodo es el que permite mover físicamente los motores del JetBot y recibir datos del LiDAR.
```bash
ros2 run jetbot_pro_ros2 jetbot
```

<h3>5.1) Lanzar el launch que dibuja el 10 y para antes de chocar (Terminal 1) </h3>

```bash
ros2 launch g10_prii3_move_turtlebot draw_turtlebot.launch.py
```

<h3>5.2) Lanzar el launch que dibuja el 10 y esquiva (Terminal 1) </h3>

```bash
ros2 launch g10_prii3_move_turtlebot obstacle_avoidance.launch.py
```

<h3>6) Controlar la ejecución (Terminal 3)</h3>

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
