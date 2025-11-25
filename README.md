Dockerı başlatmak için:

PC tarafı(Gazebo/rviz):
docker compose --profile sim up --build

Jetson tarafı(robot):
docker compose --profile robot up --build

Dcker kapatma:
docker compose --profile robot/sim down

Colcon Build:
cd /workspace/ros2_ws
colcon build --symlink-install

Build sonrası source:
source install/setup.bash

Node u başlatma:
ros2 run motor_control_node motor_control_node
