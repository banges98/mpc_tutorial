# mpc_tutorial

### Requirements
- Ubuntu 20.04 + ROS Noetic  
- Python 3.8+  
- MORAI Simulator (ROS Bridge 사용)

### Dependencies
- sudo apt update
- pip3 install cvxpy    # MPC에 사용되는 OSQP solver 포함
- git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git   # MORAI Simulator 메시지 패키지

### Build
```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# 실행 권한 부여
chmod +x ~/catkin_ws/src/mpc_tutorial/scripts/*.py
```

### Run
```bash
# 1) 경로 발행
roslaunch mpc_tutorial path_publisher.launch

# 2) MPC 실행
rosrun mpc_tutorial mpc.py

# 3) RViz 시각화
roslaunch mpc_tutorial rviz_mpc.launch
```

