#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Very very loosely based on https://gist.github.com/wkentaro/2b38d8eb914c729197c6
# Note: Parking spot assumed to be on the left and at 90 degree angle

import rospy
import math
import tf
import numpy as np
import ros_numpy as rnp
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

import tf_conversions
import tf2_ros

speed_estimate = 0  # Estimate of actual speed considering acceleration
steer_estimate = 0  # Estimate of steer angle considering steering velocity

# Path position relative to odom position
OFFSET_X = 1.0

class SimulatorNode:
    def __init__(self):
    	# Publish VESC commands in format acceptable to simulator
        self.real_cmds = rospy.Subscriber(
            "/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, self.real_cmds_callback)
        self.sim_cmds = rospy.Publisher(
            "model/vehicle_blue/cmd_vel", Twist, queue_size=10)

        # Publish odom to node that control expects
        self.odometry = rospy.Subscriber(
            "/model/vehicle_blue/odometry", Odometry, self.odom_callback)
        self.control_odometry = rospy.Publisher(
            "zed2/zed_node/odom", Odometry, queue_size=10)
		# Publish parking spot location relative to odom
        self.parking_spots = rospy.Publisher(
            "/processed/parking_spots", PointCloud2, queue_size=10)
        self.parking_spot_line_cloud = rospy.Publisher(
            "/processed/parking_spot_line_cloud", PointCloud2, queue_size=10)
            
        self.car = rospy.Publisher("/simulation/car", Marker, queue_size=10)
        self.line1 = rospy.Publisher("/simulation/line1", Marker, queue_size=10)
        self.line2 = rospy.Publisher("/simulation/line2", Marker, queue_size=10)
        self.line3 = rospy.Publisher("/simulation/line3", Marker, queue_size=10)
            
            
        # Timestamp path
        #self.path_sub = rospy.Subscriber(
            #"/parking_path", Path, self.path_callback)
        #self.path = rospy.Publisher(
            #"/simulation/parking_path", Path, queue_size=10)
    
    #def path_callback(self, msg):
        #now = rospy.Time.now()
        #pub_msg = msg
        #pub_msg.header.stamp = now
        #self.path.publish(pub_msg)

    def odom_callback(self, msg):
        now = rospy.Time.now()

        # === Make parking spot relative to the current car orientation and shifted by half of car length to get where the centre of the car needs to mocve for the front to park where desired ===
        # Extract parking spot pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        self.odom_zero = [x, y, z, roll, pitch, yaw]

        translation = np.array([[1, 0, 0, -self.odom_zero[0]], [
                                0, 1, 0, -self.odom_zero[1]], [0, 0, 1, -self.odom_zero[2]], [0, 0, 0, 1]])
        rotation_roll = np.array([[1, 0, 0, 0], [0, math.cos(-self.odom_zero[3]), -math.sin(-self.odom_zero[3]), 0], [
                                  0, math.sin(-self.odom_zero[3]), math.cos(-self.odom_zero[3]), 0], [0, 0, 0, 1]])
        rotation_pitch = np.array([[math.cos(-self.odom_zero[4]), 0, math.sin(-self.odom_zero[4]), 0], [
                                   0, 1, 0, 0], [-math.sin(-self.odom_zero[4]), 0, math.cos(-self.odom_zero[4]), 0], [0, 0, 0, 1]])
        rotation_yaw = np.array([[math.cos(-self.odom_zero[5]), -math.sin(-self.odom_zero[5]), 0, 0], [
                                 math.sin(-self.odom_zero[5]), math.cos(-self.odom_zero[5]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        # Input desired parking spot here and transform to centre of the car
        point_new_coord_sys = np.dot(rotation_roll, np.dot(rotation_pitch, np.dot(
            rotation_yaw, np.dot(translation, np.array([[2 + OFFSET_X], [4 - OFFSET_X], [0.325], [1]])))))

        new_x = point_new_coord_sys[0, 0] + 0
        new_y = point_new_coord_sys[1, 0] + 0
        point = np.array([(new_x, new_y, 0)], dtype=[
                         ('x', np.float32), ('y', np.float32), ('z', np.float32)])

        pub_msg = PointCloud2()
        pub_msg = rnp.point_cloud2.array_to_pointcloud2(point)
        pub_msg.header.stamp = now
        pub_msg.header.frame_id = "zed2_left_camera_frame"
        self.parking_spots.publish(pub_msg)

        # === Make parking spot lines relative to the current car orientation  ===
        # Input back point of desired parking line here
        point_new_coord_sys = np.dot(rotation_roll, np.dot(rotation_pitch, np.dot(
            rotation_yaw, np.dot(translation, np.array([[1.85 + OFFSET_X], [4.30 - OFFSET_X], [0.325], [1]])))))

        new_x_1 = point_new_coord_sys[0, 0] + 0
        new_y_1 = point_new_coord_sys[1, 0] + 0

        # Input another point of desired parking line here that's always visible
        point_new_coord_sys = np.dot(rotation_roll, np.dot(rotation_pitch, np.dot(
            rotation_yaw, np.dot(translation, np.array([[1.85 + OFFSET_X], [4.29 - OFFSET_X], [0.325], [1]])))))

        new_x_2 = point_new_coord_sys[0, 0] + 0
        new_y_2 = point_new_coord_sys[1, 0] + 0
        point = np.array([(new_x_1, new_y_1, 0), (new_x_2, new_y_2, 0)], dtype=[
                         ('x', np.float32), ('y', np.float32), ('z', np.float32)])

        pub_msg = PointCloud2()
        pub_msg = rnp.point_cloud2.array_to_pointcloud2(point)
        pub_msg.header.stamp = now
        self.parking_spot_line_cloud.publish(pub_msg)
        
        now = rospy.Time.now()

        # Extract parking spot pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

        pub_msg = Odometry()
        pub_msg.pose.pose.position.x = msg.pose.pose.position.x + 0.06 * math.sin(yaw)
        pub_msg.pose.pose.position.y = msg.pose.pose.position.y - 0.06 * math.cos(yaw)
        pub_msg.pose.pose.position.z = msg.pose.pose.position.z
        pub_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
        pub_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w
        pub_msg.header.frame_id = "vehicle_blue/odom"
        pub_msg.header.stamp = now
        self.control_odometry.publish(pub_msg)
		
		# Publish transform
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = now
        t.header.frame_id = "vehicle_blue/odom"
        t.child_frame_id = "zed2_left_camera_frame"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        br.sendTransform(t)
        
        marker = Marker()
        marker.header.frame_id = "vehicle_blue/odom"
        marker.header.stamp = now
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = msg.pose.pose.position.x
        marker.pose.position.y = msg.pose.pose.position.y
        marker.pose.orientation.z = msg.pose.pose.orientation.z
        marker.pose.orientation.w = msg.pose.pose.orientation.w
        marker.scale.x = 2
        marker.scale.y = 1.25
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        self.car.publish(marker)
        
        marker = Marker()
        marker.header.frame_id = "vehicle_blue/odom"
        marker.header.stamp = now
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = 3 - 1.83/2
        marker.pose.position.y = 3
        marker.scale.x = 0.05
        marker.scale.y = 2.5
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.line1.publish(marker)
        
        marker = Marker()
        marker.header.frame_id = "vehicle_blue/odom"
        marker.header.stamp = now
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = 3 + 1.83/2
        marker.pose.position.y = 3
        marker.scale.x = 0.05
        marker.scale.y = 2.5
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.line2.publish(marker)
        
        marker = Marker()
        marker.header.frame_id = "vehicle_blue/odom"
        marker.header.stamp = now
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = 3
        marker.pose.position.y = 4.25
        marker.scale.x = 1.83  # 1.25 / 1.83 = 5.8 (https://www.thezebra.com/resources/driving/average-car-size/) / 8.5 (https://blog.asphaltkingdom.com/standard-parking-space-dimensions)
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.b = 1.0
        self.line3.publish(marker)
        
    def real_cmds_callback(self, msg):
        wheelbase = 1.25  # approximate
        control_loop_period = 0.02

        global speed_estimate
        global steer_estimate

        speed_cmd = msg.drive.speed
        steer_cmd = msg.drive.steering_angle

        # 0 means as fast as possible, not 0
        if (msg.drive.acceleration == 0):
            msg.drive.acceleration = 0.05
        if (msg.drive.steering_angle_velocity == 0):
            msg.drive.steering_angle_velocity = 0.1

        # Estimate actual speed by limiting its change to what the acceleration param sets
        if (speed_cmd - speed_estimate > msg.drive.acceleration * control_loop_period):
            speed_estimate = speed_estimate + msg.drive.acceleration * control_loop_period
        elif (speed_estimate - speed_cmd > msg.drive.acceleration * control_loop_period):
            speed_estimate = speed_estimate - msg.drive.acceleration * control_loop_period
        else:
            speed_estimate = speed_cmd

        # Estimate actual steer by limiting its change to what the velocity param sets
        if (steer_cmd - steer_estimate > msg.drive.steering_angle_velocity * control_loop_period):
            steer_estimate = steer_estimate + \
                msg.drive.steering_angle_velocity * control_loop_period
        elif (steer_estimate - steer_cmd > msg.drive.steering_angle_velocity * control_loop_period):
            steer_estimate = steer_estimate - \
                msg.drive.steering_angle_velocity * control_loop_period
        else:
            steer_estimate = steer_cmd

        # Calculate angular velocity
        calculated_angular_z = speed_estimate * math.tan(steer_estimate) / wheelbase

        msg = Twist()
        msg.linear.x = speed_estimate
        msg.angular.z = calculated_angular_z
        self.sim_cmds.publish(msg)

    def run(self):
        while not rospy.is_shutdown():
            pass

if __name__ == "__main__":
    rospy.init_node("simulator")
    node = SimulatorNode()
    node.run()

