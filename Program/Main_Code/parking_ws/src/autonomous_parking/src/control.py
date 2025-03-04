#!/usr/bin/env python

import rospy
import math
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class ControlNode:
    def __init__(self):
        rospy.loginfo("Initializing ControlNode...")

        # TEMPORARY
        self.trajectory = [[0.2, 0.2]]

        # Internal states
        self.trajectory = []
        self.prev_trajectory = []
        self.first_time = True # dummy var to force trajectory reset to zero at beginning
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_speed = 0.0
        self.aeb_stop = False

        self.current_waypoint_idx = 0
        self.odom_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Tuning parameters
        self.lookahead_dist = 0.3
        self.max_speed = 0.1  # m/s
        self.aeb_ttc_threshold = 0.5  # seconds

        # Subscriptions
        # LiDAR sensor
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # Path Planning output
        rospy.Subscriber("/parking_path", Path, self.trajectory_callback)
        # Position and orientation as reported by the ZED camera
        rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.odom_callback)

        # Publisher
        self.cmd_vel_pub = rospy.Publisher("/vesc/commands/motor/speed", Float64,
                                           queue_size=10)
        self.cmd_ang_pub = rospy.Publisher("/vesc/commands/servo/position", Float64,
                                           queue_size=10)

        rospy.loginfo("ControlNode ready.")

    # -------------------------------------------------------------------------
    # 8.1. Time-to-Collision AEB (Clause C)
    # -------------------------------------------------------------------------
    def lidar_callback(self, scan_msg):
        # We check the forward sector for potential collisions
        angle_range = math.radians(30)
        angle_min = scan_msg.angle_min
        angle_incr = scan_msg.angle_increment

        ttc_list = []
        for i, r in enumerate(scan_msg.ranges):
            angle = angle_min + i * angle_incr
            if abs(angle) <= angle_range:
                # valid range
                if not math.isinf(r) and r > 0.0:
                    relative_speed = self.current_speed  # assume obstacle is stationary
                    if relative_speed > 0:
                        ttc = r / relative_speed
                        ttc_list.append(ttc)

        if ttc_list:
            min_ttc = min(ttc_list)
            if min_ttc < self.aeb_ttc_threshold:
                self.aeb_stop = True
                rospy.logwarn("AEB Activated! minTTC=%.2f" % min_ttc)
            else:
                self.aeb_stop = False
        else:
            self.aeb_stop = False

    # -------------------------------------------------------------------------
    # 8.2. Pure Pursuit Path Following
    # -------------------------------------------------------------------------
    def trajectory_callback(self, msg):
        self.trajectory = []
        for datapoint in msg.poses:
            # list of (x, y, yaw)
            self.trajectory.append((datapoint.pose.position.x, datapoint.pose.position.y, 0))

    def odom_callback(self, odom_msg):
        # Extract current pose
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        
        if self.trajectory != self.prev_trajectory or self.first_time:
            # there's a new path with new waypoints so can reset the waypoint indexer
            self.current_waypoint_idx = 0
            self.odom_zero = [x, y, orientation.x, orientation.y, orientation.z, orientation.w]
            self.prev_trajectory = self.trajectory
            first_time = False
            
        position_x = x - self.odom_zero[0]
        position_y = y - self.odom_zero[1]
        orientation_x = orientation.x - self.odom_zero[2]
        orientation_y = orientation.y - self.odom_zero[3]
        orientation_z = orientation.z - self.odom_zero[4]
        orientation_w = orientation.w - self.odom_zero[5]
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation_x, orientation_y, orientation_z, orientation_w]
        )
        
        self.current_pose = (position_x, position_y, yaw)
         
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx*vx + vy*vy)

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz control loop

        while not rospy.is_shutdown():
            if self.aeb_stop:
                # emergency stop
                speed_cmd = 0.0
                steer_cmd = 0.0

                # Publish final commands
                self.cmd_vel_pub.publish(speed_cmd)
                self.cmd_ang_pub.publish(steer_cmd)
            else:
                # follow the path if available
                if self.trajectory and self.current_waypoint_idx < len(self.trajectory):
                	# 0th waypoint is current position so start at 1
                    target = self.trajectory[self.current_waypoint_idx + 1]
                    speed_cmd, steer_cmd = self.pure_pursuit_control(target)

                    # Publish final commands
                    self.cmd_vel_pub.publish(speed_cmd)
                    self.cmd_ang_pub.publish(steer_cmd)

                    # Check if we reached the waypoint
                    dx = target[0] - self.current_pose[0]
                    dy = target[1] - self.current_pose[1]
                    dist_to_wp = math.hypot(dx, dy)
                    if dist_to_wp < 0:  # CHANGE AS NEEDED
                       self.current_waypoint_idx += 1
                else:
                    # No path or done
                    speed_cmd = 0.0
                    steer_cmd = 0.0

                    self.cmd_vel_pub.publish(speed_cmd)
                    self.cmd_ang_pub.publish(steer_cmd)

            rate.sleep()

        # Pure pursuit control assumes constant speed and controls the steering
    def pure_pursuit_control(self, target):
        """
        A basic geometric approach to track a single waypoint.
        For a full path, we iterate through the waypoint list.
        """
        (x_c, y_c, yaw_c) = self.current_pose
        (x_t, y_t, yaw_t) = target

        # heading error
        angle_to_target = math.atan2((y_t - y_c), (x_t - x_c))
        
        # flip sign as our y axis points to the left which means positive differences should force a left (negative) turn
        heading_error = -(angle_to_target - yaw_c)
        
        # normalize
        heading_error = math.atan2(
            math.sin(heading_error), math.cos(heading_error))

        # Steering
        k_steering = 1.0 # CHANGE AS NEEDED
        steering_cmd = k_steering * heading_error

        # Speed depends on heading error
        base_speed = self.max_speed
        speed_cmd = base_speed * (1.0 - min(abs(heading_error)/math.pi, 1.0))
        return speed_cmd, steering_cmd


if __name__ == "__main__":
    rospy.init_node("control_node")
    node = ControlNode()
    node.run()

