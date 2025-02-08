#!/usr/bin/env python

import rospy
import math
import tf
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from messages_utils import create_twist_msg


def create_twist_msg(vel_x, vel_y):
    # create twist message
    twist_msg = Twist()

    # set velocities in twist message
    twist_msg.linear.x = vel_x
    twist_msg.linear.y = vel_y

    speed = math.sqrt(twist_msg.linear.x*twist_msg.linear.x +
                      twist_msg.linear.y*twist_msg.linear.y)

    # return twist_msg
    return speed


class ControlNode:
    def __init__(self):
        rospy.loginfo("Initializing ControlNode...")
        
        # TEMPORARY
        self.trajectory = [[0.2, 0.2]]

        # Internal states
        self.trajectory = []
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_speed = 0.0
        self.aeb_stop = False

        # Tuning parameters
        self.lookahead_dist = 0.3
        self.max_speed = 0.4  # m/s
        self.aeb_ttc_threshold = 0.5  # seconds

        # Subscriptions
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # Path Planning output
        rospy.Subscriber("/planning/refined_trajectory", String, self.trajectory_callback)
        # POSITION, VELOCITY, STEERING ANGLE

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
        try:
            self.trajectory = eval(msg.data)  # list of (x, y, yaw)
        except:
            self.trajectory = []

    def odom_callback(self, odom_msg):
        # Extract current pose
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation = odom_msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        self.current_pose = (x, y, yaw)

        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx*vx + vy*vy)

    def run(self):
        rate = rospy.Rate(20)  # 20 Hz control loop
        current_waypoint_idx = 0

        while not rospy.is_shutdown():
            if self.aeb_stop:
                # emergency stop
                twist = create_twist_msg(0.0, 0.0)
                self.cmd_vel_pub.publish(twist)
            else:
                # follow the path if available
                if self.trajectory and current_waypoint_idx < len(self.trajectory):
                    target = self.trajectory[current_waypoint_idx]
                    speed_cmd, steer_cmd = self.pure_pursuit_control(target)

                    # Publish final commands
                    twist = create_twist_msg(speed_cmd, steer_cmd)
                    self.cmd_vel_pub.publish(twist)

                    # Check if we reached the waypoint
                    dx = target[0] - self.current_pose[0]
                    dy = target[1] - self.current_pose[1]
                    dist_to_wp = math.hypot(dx, dy)
                    if dist_to_wp < 0.1:
                        current_waypoint_idx += 1
                else:
                    # No path or done
                    twist = create_twist_msg(0.0, 0.0)
                    self.cmd_vel_pub.publish(twist)

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
        heading_error = angle_to_target - yaw_c
        # normalize
        heading_error = math.atan2(
            math.sin(heading_error), math.cos(heading_error))

        # Steering
        k_steering = 1.0
        steering_cmd = k_steering * heading_error

        # Speed depends on heading error
        base_speed = self.max_speed
        speed_cmd = base_speed * (1.0 - min(abs(heading_error)/math.pi, 1.0))

        return speed_cmd, steering_cmd


if __name__ == "__main__":
    rospy.init_node("control_node")
    node = ControlNode()
    node.run()
