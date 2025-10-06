Sprint 1: 

1) Clonar el repositorio:
git clone https://github.com/FranHG05/g10-prii3-ws

2) Compilar el workspace:
cd ~/g10-prii3-ws
colcon build

3) Cargar el entorno en dos terminales:
source install/setup.bash

4) Ejecutar el nodo mediante el launch (terminal 1):
ros2 launch g10_prii3_turtlesim draw_turtle.launch.py

5) Pausar, reanudar y reiniciar el dibujo (terminal 2):
ros2 service call /pause std_srvs/srv/Empty
ros2 service call /resume std_srvs/srv/Empty
ros2 service call /reset std_srvs/srv/Empty
