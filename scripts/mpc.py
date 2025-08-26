#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import cvxpy as op
from std_msgs.msg import Float64, Int32
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Vector3, PoseArray, Pose, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

class MPC:
    def __init__(self):
        ## Initialize
        rospy.init_node('mpc', anonymous=True)
        ## Subsriber
        rospy.Subscriber("/local_path", Path, self.PathCallback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.EgoStatusCallback)
        rospy.Subscriber("/gps_utm_odom", Odometry, self.GPSCallback)
        rospy.Subscriber("/imu", Imu, self.IMUCallback)
        ## Publisher
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.visual_pub = rospy.Publisher('/prediction_state', PoseArray, queue_size=1)
        self.cubic_path_pub = rospy.Publisher('/cubic_path', Path, queue_size=1)
        self.cost_pub = rospy.Publisher('/cost', Float64, queue_size=1)
        ## Vehicle Parameter
        self.m = 1500 # kg
        self.Caf = 10000 # N/rad
        self.Car = 10000 # N/rad
        self.lf = 1.1 # m
        self.lr = 1.6 # m
        self.Iz = 2600 # kgm^2
        ## Control Parameter
        self.reference_velocity = 7 # m/s
        self.Hz = 30
        self.Np = 30
        self.Nc = 15
        ## Objective Function Weight
        p_11 = 10
        p_33 = 100 - p_11
        q_11 = 10
        q_33 = 100 - q_11
        r_11 = self.lr**2 / (self.lf**2 + self.lr**2)
        r_22 = self.lf**2 / (self.lf**2 + self.lr**2)
        self.P = np.diag([p_11, 0.00, p_33, 0.00]) # State Error Penalty Weight
        self.Q = np.diag([q_11, 0.00, q_33, 0.00]) # Terminal State Error Penalty Weight
        self.R = np.diag([r_11, r_22]) # Control Input Penalty Weight
        self.sum_P = sum(sum(self.P))
        self.sum_Q = sum(sum(self.Q))
        self.sum_R = sum(sum(self.R))
        self.S = np.diag([1.00]) # Lateral Acceleration Penalty Weight
        self.tracking_gain = 1.0
        self.k_q = 1.0
        self.control_gain = self.tracking_gain *7/150  * self.sum_P / self.sum_R * self.reference_velocity # * self.reference_velocity / 5
        # self.control_gain = self.tracking_gain * 7/100  * self.sum_P / self.sum_R * self.reference_velocity
        self.P = self.P * self.tracking_gain
        self.Q = self.Q * self.tracking_gain * self.k_q
        self.R = self.R * self.control_gain
        ## System Hertz & Prediction Step Time
        self.dt_sys = 1/float(self.Hz)
        self.pid = Pid(self.dt_sys)
        self.dt_pred = self.dt_sys
        ## Vehicle Model
        self.vehicle_model = VehicleModel(self.dt_pred, self.m, self.Caf, self.Caf, self.lf, self.lr, self.Iz)
        ## States
        self.is_gps = False
        self.is_imu = False
        self.is_path = False
        self.is_vehicle = False
        self.path = Path()
        ## Simulator
        self.cmd_msg = CtrlCmd()
        self.cmd_msg.longlCmdType = 1
        self.cmd_msg.accel = 0
        self.cmd_msg.brake = 0
        self.cmd_msg.steering = 0

        self.current_postion = Vector3()
        self.V = Vector3()
        self.a = Vector3()
        self.vehicle_yaw = Float64()
        self.vehicle_yaw_rate = Float64()
        self.a_x = Float64()
        self.a_y = Float64()

        self.max_wheel_angle = np.deg2rad(45.0)
        self.delta_wheel_angle = self.max_wheel_angle * 2 / 1.5 / self.Hz
        self.max_lateral_acceleration = 1.0
        self.u_prev = 0.0
        self.error_simple = True
        self.X = None

    def PathCallback(self, msg):
        self.is_path = True
        self.path = msg

    def EgoStatusCallback(self, msg):
        self.is_vehicle = True
        self.V = msg.velocity
        self.a = msg.acceleration

    def GPSCallback(self, msg):
        self.is_gps = True
        odom_quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y
    
    def IMUCallback(self, msg):
        self.is_imu = True
        self.a_x = msg.linear_acceleration.x
        self.a_y = msg.linear_acceleration.y
        self.vehicle_yaw_rate = msg.angular_velocity.z
    
    def predict_x(self):
        t = np.linspace(0, self.Np, self.Np+1) * self.dt_pred
        return self.V.x * t + 0.5 * self.a.x * t ** 2

    def cubic_path_tarcking_model(self, path, predict_x):
        X_ref = np.zeros((4,self.Np+1))
        cubic_path = np.polyfit(
            np.squeeze(np.array(path[0, :])),
            np.squeeze(np.array(path[1, :])),
            3
            )
        coefficient_d = np.polyder(cubic_path)
        coefficient_dd = np.polyder(coefficient_d)
        y_ref = np.polyval(cubic_path, predict_x)
        y_ref_d = np.polyval(coefficient_d, predict_x)
        yaw_ref = np.polyval(coefficient_d, predict_x)
        yawrate_ref = np.polyval(coefficient_dd, predict_x)
        X_ref[0,:] = y_ref
        X_ref[1,:] = y_ref_d
        X_ref[2,:] = self.pi_2_pi(yaw_ref)
        X_ref[3,:] = self.pi_2_pi(yawrate_ref)
        ### path visualization ###
        resolution = 0.1
        path_x = 30
        axis_x = np.arange(-5, path_x + resolution, resolution)
        path_y = np.polyval(cubic_path, axis_x)
        cubicPath=Path()
        cubicPath.header.stamp = rospy.Time.now()
        cubicPath.header.frame_id = 'base_link'
        for i in range(len(axis_x)):
            tmp_pose=PoseStamped()
            tmp_pose.header.stamp = rospy.Time.now()
            tmp_pose.header.frame_id = 'base_link'
            tmp_pose.pose.position.x = axis_x[i]
            tmp_pose.pose.position.y = path_y[i]
            tmp_pose.pose.position.z = 0
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            cubicPath.poses.append(tmp_pose)
        self.cubic_path_pub.publish(cubicPath)
        return X_ref

    def pi_2_pi(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def deg_2_deg(self, angle):
        return (angle + 180) % (2 * 180) - 180
    
    def mps2kph(self, velocity):
        return velocity * 3600 / 1000
    
    def kph2mps(self, velocity):
        return velocity * 1000 / 3600
    
    def linear_control(self, A, B, C, x, u):
        X = A @ x + B @ u
        Y = C @ (X - x)
        return X, Y
    
    def run(self):
        try:
            startTime = time.time()
            Vx = self.V.x
            Vy = self.V.y
            V = np.sqrt(Vx**2 + Vy**2)
            if self.is_gps and self.is_imu and self.is_path and  self.is_vehicle:
                y = 0
                yaw  = 0
                yawrate = self.vehicle_yaw_rate
                y_dot = Vx*np.sin(yawrate*self.dt_sys)
                x_p = []
                y_p = []
                for point in self.path.poses:
                    x_p.append(point.pose.position.x)
                    y_p.append(point.pose.position.y)
                path_matrix = np.matrix([x_p, y_p])
                self.X = np.array(np.matrix([y, y_dot, yaw, yawrate]).T)
                A, B, C = self.vehicle_model.update_state(Vx)
                predict_x = self.predict_x()
                X_ref = self.cubic_path_tarcking_model(path_matrix, predict_x) # 4 x (N+1)
                X_pred = self.X
                cost = 0
                constraints = []
                U = op.Variable((2, self.Np))
                for i in range(self.Np):
                    u = op.reshape(U[:, i], (2, 1))
                    X_ref_i = X_ref[:, i].reshape(4, 1)
                    cost += op.quad_form(X_ref_i - X_pred, self.P)
                    X_pred, Y_pred = self.linear_control(A, B, C, X_pred, u)
                    # cost += op.quad_form(Y_pred, self.S)
                    cost += op.quad_form(u, self.R)
                X_ref_i = X_ref[:, -1].reshape(4, 1)
                cost += op.quad_form(X_ref_i - X_pred, self.Q)
                constraints += [op.abs(U[0]) <= self.max_wheel_angle]
                # constraints += [U[1,:] == 0]
                constraints += [op.abs(U[0,self.Nc] - self.u_prev) <= self.delta_wheel_angle]
                # for k in range(1, self.Np):
                #     constraints += [op.abs(U[k] - U[k - 1]) <= self.delta_wheel_angle]
                problem = op.Problem(op.Minimize(cost), constraints)
                solution = problem.solve(solver=op.OSQP, max_iter=10000, eps_abs=1e-5, eps_rel=1e-5, warm_start=True, verbose=False)
                cost_value = problem.value
                self.cost_pub.publish(cost_value)
                U_set = U.value.reshape((2,self.Np))
                u_f = U_set[0, self.Nc]
                self.u_prev = u_f
                self.cmd_msg.steering = u_f
                self.viz_pose(A, B, C, U_set, predict_x)
                acc_cmd = min(self.pid.get_output(self.reference_velocity, V), 1)
                if acc_cmd > 0:
                    self.cmd_msg.accel = acc_cmd
                    self.cmd_msg.brake = 0.0
                else:
                    self.cmd_msg.accel = 0.0
                    self.cmd_msg.brake = -acc_cmd
            else:
                print("GPS:", self.is_gps, " |  IMU:", self.is_imu, " |  Path:", self.is_path, " |  Vehicle:", self.is_vehicle)
                self.cmd_msg.steering = 0.0
                self.cmd_msg.accel = 0.0
                self.cmd_msg.brake = -1
            self.ctrl_pub.publish(self.cmd_msg)
            endTime = time.time()
            duration = endTime - startTime
            Hz = 1 / duration
            print(
                f"\nMPC\n"
                f"{'Prediction horizon':<22}: {int(self.Np):6d}\n"
                f"{'Control horizon':<22}: {int(self.Nc):6d}\n"
                f"{'Hertz':<22}: {Hz:9.2f}\n"
                f"{'Target velocity':<22}: {self.reference_velocity:9.3f}\n"
                f"{'Vehicle velocity':<22}: {V:9.3f}\n"
                f"{'Cmd_accel':<22}: {self.cmd_msg.accel:9.3f}\n"
                f"{'Cmd_brake':<22}: {self.cmd_msg.brake:9.3f}\n"
                f"{'Cmd_steering':<22}: {np.rad2deg(self.cmd_msg.steering):9.3f}\n"
                + "=" * 40
            )
        except TypeError as e:
            print("Error:", e)

    def viz_pose(self, A, B, C, U, predict_x):
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        X_pred = self.X
        for i in range(self.Np):
            u = U[:,i].reshape(2,1)
            X_pred, _ = self.linear_control(A, B, C, X_pred, u)
            y = X_pred[0,0]
            yaw = X_pred[2,0]
            pose = Pose()
            pose.position.x = predict_x[i+1]
            pose.position.y = y
            pose.position.z = 0.0
            quaternion = quaternion_from_euler(0.0, 0.0, yaw)
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.poses.append(pose)
        self.visual_pub.publish(pose_array)

class VehicleModel:
    def __init__(self, dt, m, Caf, Car, lf, lr, Iz):
        self.dt_pred = dt
        self.A = None
        self.B = None
        self.C = None
        self.m = m
        self.Caf = Caf
        self.Car = Car
        self.lf = lf
        self.lr = lr
        self.Iz = Iz

    def dynamic_vehicle_model(self, Vx):
        m = self.m
        Caf = self.Caf
        Car = self.Car
        lf = self.lf
        lr = self.lr
        Iz = self.Iz
        if Vx < 1:
            Vx = 1
        A_c = np.matrix([
                    [0, 1, 0, 0],
                    [0, -2 * (Caf + Car) / (m * Vx), 0, -Vx -2 * (Caf * lf - Car * lr) / (m * Vx)],
                    [0, 0, 0, 1],
                    [0, -2 * (Caf  * lf - Car * lr) / (Iz * Vx), 0, -2 * (Caf * lf**2 + Car * lr**2) / (Iz * Vx)]
                    ])
        B_c = np.matrix([
                [0, 0],
                [2 * Caf / m, 2 * Car / m],
                [0,0],
                [2 * Caf * lf / Iz, -2 * Car * lr / Iz],
                ])
        C_c = np.matrix([
                [0, 1, Vx, 0],
                ])
        A_d = np.identity(4) + A_c * self.dt_pred
        B_d = B_c * self.dt_pred
        C_d = C_c
        self.A = A_d
        self.B = B_d
        self.C = C_d

    def update_state(self, velocity):
        self.dynamic_vehicle_model(velocity)
        return self.A, self.B, self.C

class Pid:
    def __init__(self, sampling_time):
        self.p_gain = 1.0   #30Hz : 1.0
        self.i_gain = 0.00  #30Hz : 0.05
        self.d_gain = 0.00   #30Hz : 0.2
        self.sampling_time = sampling_time
        self.previous_error = 0
        self.integral_error = 0
        self.i = 0

    def get_output(self, X_des_value, current_value):
        error = X_des_value - current_value
        if self.i > 5 / self.sampling_time :
            self.integral_error = 0
        else:
            self.integral_error += error * self.sampling_time
        derivative_error = (error - self.previous_error) / self.sampling_time
        output = self.p_gain * error + self.i_gain * self.integral_error + self.d_gain * derivative_error
        self.previous_error = error
        self.i += 1
        return output

if __name__ == '__main__':
    try:
        ModelPredictiveControl = MPC()
        rate = rospy.Rate(ModelPredictiveControl.Hz)  # 50hz
        while not rospy.is_shutdown():
            ModelPredictiveControl.run()
            rate.sleep()
    except rospy.ROSInterruptException as ros_error:
        print("ROS error:", ros_error)
    except op.error.SolverError as solver_error:
        print("Solver error:", solver_error)
