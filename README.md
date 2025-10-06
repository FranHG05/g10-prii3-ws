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
