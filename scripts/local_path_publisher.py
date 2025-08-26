#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int16, String
from tf.transformations import euler_from_quaternion


class LocalPath(object):
    def __init__(self):
        # Publishers & Subscribers
        self.local_path_pub = rospy.Publisher("/local_path", Path, queue_size=10)
        self.gps_sub = rospy.Subscriber("/gps_utm_odom", Odometry, self.gps_callback, queue_size=10)
        self.global_path_sub = rospy.Subscriber("/global_path", Path, self.global_path_callback, queue_size=10)

        # States
        self.global_path = Path()
        self.current_position = Point()
        self.vehicle_yaw = 0.0
        self.gps_state = False
        self.path_state = False
        self.max_waypoint = 50
        self.current_waypoint = 0

    def spin(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            if self.gps_state and self.path_state:
                self.publish_local_path(self.global_path, self.local_path_pub)
            rate.sleep()

    # === Callbacks ===
    def gps_callback(self, msg):
        self.gps_state = True

        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vehicle_yaw = yaw  # rad

        p = msg.pose.pose.position
        self.current_position.x = p.x
        self.current_position.y = p.y
        self.current_position.z = p.z

    def global_path_callback(self, msg):
        self.path_state = True
        self.global_path = msg
        self.current_waypoint = self.find_nearest_point()
        rospy.loginfo("Current waypoint updated to: %d", self.current_waypoint)

    def map_to_base_link(self, gx, gy):
        # 현재 위치/자세
        cx = self.current_position.x
        cy = self.current_position.y
        yaw = self.vehicle_yaw
        c, s = math.cos(yaw), math.sin(yaw)

        # map -> base_link 변환
        dx, dy = gx - cx, gy - cy
        x_bl =  c*dx + s*dy
        y_bl = -s*dx + c*dy
        return x_bl, y_bl

    def publish_local_path(self, global_path, publisher):
        local_path = Path()
        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "base_link"  # 출력 프레임

        last_local_wp = min(self.current_waypoint + self.max_waypoint, len(global_path.poses))
        for i in range(self.current_waypoint, last_local_wp):
            gx = global_path.poses[i].pose.position.x
            gy = global_path.poses[i].pose.position.y
            x_bl, y_bl = self.map_to_base_link(gx, gy)

            ps = PoseStamped()
            ps.header.stamp = rospy.Time.now()
            ps.header.frame_id = "base_link"      # 포즈도 base_link로 통일
            ps.pose.position.x = x_bl
            ps.pose.position.y = y_bl
            ps.pose.orientation.w = 1.0
            local_path.poses.append(ps)

        publisher.publish(local_path)


    def find_nearest_point(self):
        nearest_index = -1
        min_distance = float("inf")

        for i, pose_stamped in enumerate(self.global_path.poses):
            gx = pose_stamped.pose.position.x
            gy = pose_stamped.pose.position.y
            d = math.hypot(self.current_position.x - gx, self.current_position.y - gy)
            if d < min_distance:
                min_distance = d
                nearest_index = i

        return nearest_index


if __name__ == "__main__":
    rospy.init_node("local_path_publisher")
    node = LocalPath()
    node.spin()
