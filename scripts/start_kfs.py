#! /usr/bin/env python3
import rospy
from MARS_msgs.msg import TargetTelemetry
import roslaunch
import threading
import time
import subprocess as sp


class PoseListener:
    def __init__(self):
        rospy.init_node('pose_listener', anonymous=True)
        rospy.Subscriber("/uav0/target_position_geolocation", TargetTelemetry, self.callback)
        self.prev_time = None
        self.prev_x_pos = None
        self.prev_y_pos = None
        self.pose_change_threshold = 100  # meters
        self.frequency_threshold = 2  # Hz
        self.lock = threading.Lock()

    def callback(self, data):
        current_time = time.time()
        if self.prev_time is not None and self.prev_x_pos is not None and self.prev_y_pos is not None:
            time_diff = current_time - self.prev_time
            if time_diff > 0:
                frequency = 1 / time_diff
                if frequency > self.frequency_threshold:
                    x_pos_diff = abs(self.prev_x_pos - data.x_pos)
                    y_pos_diff = abs(self.prev_y_pos - data.y_pos)
                    print(f'x_pos_diff------{x_pos_diff}----y_pos_diff------{y_pos_diff}')
                    if x_pos_diff < self.pose_change_threshold and y_pos_diff < self.pose_change_threshold:
                        with self.lock:
                            threading.Thread(target=self.launch_file).start()
        self.prev_time = current_time
        self.prev_x_pos = data.x_pos
        self.prev_y_pos = data.y_pos

    def launch_file(self):
        package = 'offboard_py'
        launch_file = 'kalman_filter.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch.rlutil.resolve_launch_arguments([package, launch_file])[0])])
        launch.start()
        rospy.loginfo("started")

        # Keep the script alive while the launch file is running
        launch.spin()

if __name__ == '__main__':
    listener = PoseListener()
    rospy.spin()