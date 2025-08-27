# mpc_tutorial

# Requirements 
$ sudo apt update
$ pip3 install cvxpy    # mpc에 사용되는 OSQP solver가 포함되어 있는 라이브러리
$ git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git     # morai simulator에 사용되는 메시지

# Build
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

# 실행 권한 부여
chmod +x ~/catkin_ws/src/mpc_tutorial/scripts/*.py

# 코드 실행
$ roslaunch mpc_tutorial path_publisher.launch      # 경로 발행
$ rosrun mpc_tutorial mpc.py                        # mpc 실행 코드
$ roslaunch mpc_tutorial rviz_mpc.launch            # 경로 시각화
