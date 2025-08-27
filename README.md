# mpc_tutorial

MORAI Sim (K-City) 환경에서 Model Predictive Control(MPC) 데모를 실행하기 위한 패키지입니다.  

---

## Requirements

- Ubuntu 20.04 + ROS Noetic  
- Python 3.8+  
- MORAI Simulator (ROS Bridge 사용)

### Dependencies
```bash
sudo apt update
pip3 install cvxpy    # MPC에 사용되는 OSQP solver 포함
git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs.git   # MORAI Simulator 메시지 패키지
