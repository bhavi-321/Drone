#!/usr/bin/env python3

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode
import math
import numpy as np

class DroneKitTakeoffObstacleAvoidance:
    def __init__(self):
        print("Connecting to vehicle...")
        self.vehicle = connect("127.0.0.1:14550", wait_ready=True)
        print("Connected.")

        # Home & target positions (ENU)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.target_x = 0.0
        self.target_y = 25.0
        self.target_z = 10.0

        # ROS setup
        rospy.init_node('dronekit_takeoff_mavros_obstacle_avoidance')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.rear_distance = float('inf')

        rospy.Subscriber('/range/front', Range, self.front_callback)
        rospy.Subscriber('/range/left', Range, self.left_callback)
        rospy.Subscriber('/range/right', Range, self.right_callback)
        rospy.Subscriber('/range/rear', Range, self.rear_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)

        self.twist_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.rate = rospy.Rate(10)

    # DroneKit takeoff
    def takeoff_with_dronekit(self, altitude):
        print("Arming motors...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        print("Taking off!")
        self.vehicle.simple_takeoff(altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(" Altitude:", alt)
            if alt >= altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        self.home_z = altitude

    def ensure_guided_mode(self):
        resp = self.set_mode_client(0, 'GUIDED')
        print("Set GUIDED via MAVROS:", resp.mode_sent)

    # Sensor callbacks
    def front_callback(self, msg): self.front_distance = msg.range
    def left_callback(self, msg): self.left_distance = msg.range
    def right_callback(self, msg): self.right_distance = msg.range
    def rear_callback(self, msg): self.rear_distance = msg.range
    def position_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def distance_to(self, x, y, z):
        return math.sqrt((x - self.current_x)**2 + (y - self.current_y)**2 + (z - self.current_z)**2)

    # Repulsive vector-based avoidance
    def compute_avoidance_vector(self, min_range=0.5, influence_dist=4.0):
        """
        Generates repulsive velocity vector from range sensors.
        Closer objects exert stronger repulsion.
        """
        vectors = []
        if self.front_distance < influence_dist:
            strength = (influence_dist - self.front_distance) / influence_dist
            vectors.append(np.array([-strength, 0]))  # push backward
        if self.rear_distance < influence_dist:
            strength = (influence_dist - self.rear_distance) / influence_dist
            vectors.append(np.array([strength, 0]))  # push forward
        if self.left_distance < influence_dist:
            strength = (influence_dist - self.left_distance) / influence_dist
            vectors.append(np.array([0, strength]))  # push right
        if self.right_distance < influence_dist:
            strength = (influence_dist - self.right_distance) / influence_dist
            vectors.append(np.array([0, -strength]))  # push left

        if not vectors:
            return np.array([0.0, 0.0])
        return np.sum(vectors, axis=0)

    def navigate_to_target(self, x_target, y_target, z_target, label="Target"):
        velocity = 1.5
        tolerance = 0.5
        print(f"Navigating to {label} ({x_target}, {y_target}, {z_target})...")

        while not rospy.is_shutdown():
            dist = self.distance_to(x_target, y_target, z_target)
            if dist < tolerance:
                print(f"{label} reached!")
                break

            dx = x_target - self.current_x
            dy = y_target - self.current_y
            dz = z_target - self.current_z
            norm = math.sqrt(dx**2 + dy**2 + dz**2)
            if norm == 0: 
                continue

            # Goal velocity vector
            goal_vec = np.array([dx / norm, dy / norm])

            # Avoidance vector
            avoid_vec = self.compute_avoidance_vector()

            # Blend goal + avoidance
            total_vec = goal_vec + avoid_vec * 2.5  # amplify avoidance
            total_norm = np.linalg.norm(total_vec)
            if total_norm > 0:
                total_vec = total_vec / total_norm

            twist = TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = total_vec[0] * velocity
            twist.twist.linear.y = total_vec[1] * velocity
            twist.twist.linear.z = (dz / norm) * velocity * 0.8  # z movement smooth

            self.twist_pub.publish(twist)
            self.rate.sleep()

        # Stop
        stop = TwistStamped()
        stop.header.stamp = rospy.Time.now()
        stop.twist.linear.x = 0.0
        stop.twist.linear.y = 0.0
        stop.twist.linear.z = 0.0
        self.twist_pub.publish(stop)
        rospy.sleep(1)

    def land(self):
        print("Landing...")
        descend = TwistStamped()
        descend.header.stamp = rospy.Time.now()
        descend.twist.linear.z = -0.8
        while not rospy.is_shutdown() and self.current_z > 0.3:
            self.twist_pub.publish(descend)
            self.rate.sleep()
        print("Landed successfully.")

    def run(self):
        self.takeoff_with_dronekit(self.target_z)
        self.ensure_guided_mode()
        self.vehicle.close()
        print("Switched to MAVROS.")

        # Go to target
        self.navigate_to_target(self.target_x, self.target_y, self.target_z, "Target")

        # Return home
        print("Returning home...")
        self.navigate_to_target(self.home_x, self.home_y, self.home_z, "Home")

        # Land
        self.land()

if __name__ == '__main__':
    mission = DroneKitTakeoffObstacleAvoidance()
    mission.run()
