#!/usr/bin/env python

import rospy
import math
import sys
import tf
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

START_Y = -0.06


class ControlNode:
    def __init__(self):
        rospy.loginfo("Initializing ControlNode...")

        # Internal states

        # From Path Planning
        self.trajectory = []

        # Dummy trajectory to force frame of reference reset to zero if there's no path at the beginning
        self.prev_trajectory = [1]

        self.current_pose = (0.0, 0.0, 0.0)
        self.current_speed = 0.0
        self.aeb_stop = False

        self.current_waypoint_idx = 1
        self.odom_msg = Odometry()
        self.odom_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.simulated_wheelbase = 0.3 # set to 0.3 for physical opeeration and 1.25 for simulated
        self.physical_wheelbase = 0.3

        # Tuning parameters
        self.lookahead_dist = 0.3 * self.simulated_wheelbase / self.physical_wheelbase
        self.max_speed = 0.5 * self.simulated_wheelbase / self.physical_wheelbase  # m/s
        self.aeb_ttc_threshold = 0.5  # seconds

        # Subscriptions
        # LiDAR sensor
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # Path Planning output
        rospy.Subscriber("/parking_path", Path, self.trajectory_callback)
        # Position and orientation as reported by the ZED camera
        rospy.Subscriber("/zed2/zed_node/odom", Odometry, self.odom_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher(
            "/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

        rospy.loginfo("ControlNode ready.")

        # NOT IN USE AT THE MOMENT
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

        # Add each datapoint to the trajectory. Path Planning does not specify a yaw so assumed 0
        for datapoint in msg.poses:
            # List of (x, y, yaw)
            self.trajectory.append(
                (datapoint.pose.position.x, datapoint.pose.position.y, 0))

    def odom_callback(self, odom_msg):
        self.odom_msg = odom_msg

        # NOT IN USE AT THE MOMENT
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx*vx + vy*vy)

    def run(self):
        # rate = rospy.Rate(50)  # 50 Hz control loop
        
        try:
            while not rospy.is_shutdown():
                # All this needs to be run wish an updated trajectory and odom_msg so can't be in either callback
                # Extract current pose
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                z = self.odom_msg.pose.pose.position.z

                orientation = self.odom_msg.pose.pose.orientation
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                    [orientation.x, orientation.y,
                        orientation.z, orientation.w]
                )

                # Path Planning resets its frame of reference each time there's a new path so Control needs to do that too for consistency
                if self.trajectory != self.prev_trajectory:
                    self.odom_zero = [x, y, z, roll, pitch, yaw]

                    # There's a new path with new waypoints so can reset the waypoint indexer
                    # 0th waypoint is current position so start at 1
                    self.current_waypoint_idx = 1
                    self.prev_trajectory = self.trajectory
                    self.first_time = False

                # Set position to be relative to frame of reference. For an explaniation, see https://medium.com/@parkie0517/rigid-transformation-in-3d-space-translation-and-rotation-d701d8859ba8 or your MECHENG 4K03 notes
                translation = np.array([[1, 0, 0, -self.odom_zero[0]], [
                                        0, 1, 0, -self.odom_zero[1]], [0, 0, 1, -self.odom_zero[2]], [0, 0, 0, 1]])
                rotation_roll = np.array([[1, 0, 0, 0], [0, math.cos(-self.odom_zero[3]), -math.sin(-self.odom_zero[3]), 0], [
                                          0, math.sin(-self.odom_zero[3]), math.cos(-self.odom_zero[3]), 0], [0, 0, 0, 1]])
                rotation_pitch = np.array([[math.cos(-self.odom_zero[4]), 0, math.sin(-self.odom_zero[4]), 0], [
                                           0, 1, 0, 0], [-math.sin(-self.odom_zero[4]), 0, math.cos(-self.odom_zero[4]), 0], [0, 0, 0, 1]])
                rotation_yaw = np.array([[math.cos(-self.odom_zero[5]), -math.sin(-self.odom_zero[5]), 0, 0], [
                                         math.sin(-self.odom_zero[5]), math.cos(-self.odom_zero[5]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

                point_new_coord_sys = np.dot(rotation_roll, np.dot(rotation_pitch, np.dot(
                    rotation_yaw, np.dot(translation, np.array([[x], [y], [z], [1]])))))

                x = point_new_coord_sys[0, 0]
                # Moving start point relative to detected points, path planning moves points relative to starting position, so opposite sign compared to path planning
                y = point_new_coord_sys[1, 0] + START_Y
                yaw = yaw - self.odom_zero[5]

                self.current_pose = (x, y, yaw)

                # Follow the path if not done
                if self.current_waypoint_idx < len(self.trajectory):

                    target = self.trajectory[self.current_waypoint_idx]
                    # Check if we reached the waypoint
                    dx = target[0] - self.current_pose[0]
                    dy = target[1] - self.current_pose[1]
                    dist_to_wp = math.hypot(dx, dy)

                    # CHANGE AS NEEDED, must be high enough to prevent a waypoint from being driven past
                    while dist_to_wp < self.lookahead_dist:
                        self.current_waypoint_idx += 1

                        if (self.current_waypoint_idx == len(self.trajectory)):

                            # For simulation since shutting off when final point is within lookahead_dist is too soon and real car moves parking spot backwards anyways so stopping is later
                            # if dist_to_wp > self.lookahead_dist / 4:
                                # self.current_waypoint_idx -= 1
                                # target = self.trajectory[self.current_waypoint_idx]
                            break

                        target = self.trajectory[self.current_waypoint_idx]
                        # Check if we reached the waypoint
                        dx = target[0] - self.current_pose[0]
                        dy = target[1] - self.current_pose[1]
                        dist_to_wp = math.hypot(dx, dy)

                    speed_cmd, steer_cmd = self.pure_pursuit_control(
                        target)

                    # Publish final commands
                    msg = AckermannDriveStamped()
                    msg.drive.speed = speed_cmd
                    msg.drive.acceleration = 0.5 * self.simulated_wheelbase / self.physical_wheelbase
                    msg.drive.steering_angle = steer_cmd
                    msg.drive.steering_angle_velocity = 1
                    # rospy.loginfo("Speed: %f m/s. Steering %f rad.", speed_cmd, steer_cmd)
                    self.cmd_pub.publish(msg)

                else:
                    # Done
                    speed_cmd = 0.0
                    steer_cmd = 0.0

                    # Publish final commands
                    msg = AckermannDriveStamped()
                    msg.drive.speed = speed_cmd
                    msg.drive.acceleration = 0
                    msg.drive.steering_angle = steer_cmd
                    msg.drive.steering_angle_velocity = 0
                    # rospy.loginfo("Speed: %f m/s. Steering %f rad.", speed_cmd, steer_cmd)
                    self.cmd_pub.publish(msg)

                # rate.sleep()

        except KeyboardInterrupt:
            # emergency stop
            speed_cmd = 0.0
            steer_cmd = 0.0

            # Publish final commands
            msg = AckermannDriveStamped()
            msg.drive.speed = speed_cmd
            msg.drive.acceleration = 0
            msg.drive.steering_angle = steer_cmd
            msg.drive.steering_angle_velocity = 0
            # rospy.loginfo("Speed: %f m/s. Steering %f rad.", speed_cmd, steer_cmd)
            self.cmd_pub.publish(msg)
            sys.exit(0)

    # Pure pursuit control assumes constant speed and controls the steering
    def pure_pursuit_control(self, target):
        """
        A basic geometric approach to track a single waypoint.
        For a full path, we iterate through the waypoint list.
        """
        (x_c, y_c, yaw_c) = self.current_pose
        (x_t, y_t, yaw_t) = target

        # Heading error
        angle_to_target = math.atan2((y_t - y_c), (x_t - x_c))

        heading_error = (angle_to_target - yaw_c)

        # Normalize (put within range of -pi to pi)
        heading_error = math.atan2(
            math.sin(heading_error), math.cos(heading_error))

        # print(x_c, y_c, x_t, y_t, heading_error)

        # Steering
        k_steering = 0.8 # CHANGE AS NEEDED
        steering_cmd = k_steering * heading_error

        # Speed depends on heading error
        base_speed = self.max_speed
        speed_cmd = base_speed * (1.0 - min(abs(heading_error)/math.pi, 1.0))
        return speed_cmd, steering_cmd


if __name__ == "__main__":
    rospy.init_node("control_node", disable_signals=True) # disable_signals needed for KeyboardInterrupt
    node = ControlNode()
    node.run()

