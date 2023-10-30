#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from collections import deque
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.distance import cdist
import subprocess as sp
from MARS_msgs.msg import TargetTelemetry


class PositionBufferNode:
    def __init__(self):
        rospy.init_node('position_buffer_node', anonymous=True)
        self.buffer_size = 20  # Adjust N as needed
        self.position_buffer = deque(maxlen=self.buffer_size)
        self.frequency_threshold = 2  # Minimum desired frequency in Hz
        self.distance_threshold = 50  # Maximum allowed distance in meters
        self.last_time = rospy.Time.now()
        self.rate = rospy.Rate(5)  # 20 Hz

        rospy.Subscriber("/uav0/target_position_geolocation", TargetTelemetry, self.pose_callback)

    def pose_callback(self, pose_msg):
        self.position_buffer.append(pose_msg)

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(f"buffer size:{len(self.position_buffer)}")
            if len(self.position_buffer) > 5:
                dist_mat = self.compute_distance_matrix()
                rospy.loginfo(f"dist max: {dist_mat.max()}")
                f = (self.position_buffer[-1].timestamp.secs - self.position_buffer[0].timestamp.secs) / len(self.position_buffer)
                f = 1/f
                rospy.loginfo(f"freq: {f}")


            # buffer must be full, and frequency right, and distance within bounds
            if len(self.position_buffer) == self.buffer_size and \
                f > 3 and \
                self.check_distance_constraint(dist_mat):
                self.position_buffer = deque(maxlen=self.buffer_size) # reset buffer

                rospy.loginfo(f"buffer size:{len(self.position_buffer)}")

                package = 'offboard_py'
                launch_file = 'kalman_filter.launch'
                cmd = f"roslaunch {package} {launch_file}"
                rospy.loginfo(f"executing command: {cmd}")
                p = sp.Popen(cmd.split())#, shell=True)
                p.wait() # don't continue executing
                print('\n'*20, "EXITED", '\n'*20)

            self.rate.sleep()


    def compute_distance_matrix(self):

        points_array = [[p.x_pos, p.y_pos] for p in self.position_buffer]
        points_array = np.array(points_array)
        differences = points_array[:, np.newaxis, :] - points_array[np.newaxis, :, :]
    
        # Compute the squared Euclidean distances
        squared_distances = np.sum(differences**2, axis=2)
        
        # Take the square root to obtain the actual Euclidean distances
        distance_matrix = np.sqrt(squared_distances)
        
        return distance_matrix


    def check_distance_constraint(self, distance_matrix):
        # Check if the maximum distance in the distance matrix is below the threshold
        return np.max(distance_matrix) <= self.distance_threshold


if __name__ == '__main__':
    try:
        node = PositionBufferNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
