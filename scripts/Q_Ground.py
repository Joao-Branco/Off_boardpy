#! /usr/bin/env python3

import rospy
import subprocess
import time

def run_qgroundcontrol():
    try:
        # Start QGroundControl.appimage using subprocess
        p = subprocess.Popen(['/home/jbranco/QGroundControl.AppImage'], shell= True)

        # Wait for 1 minute (60 seconds)
        time.sleep(40)

        # Kill the QGroundControl process
        #p.kill()
        subprocess.Popen('killall QGroundControl'.split(' '))

    except Exception as e:
        rospy.logerr("An error occurred: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('qgroundcontrol_launcher', anonymous=True)
    run_qgroundcontrol()
    rospy.spin()