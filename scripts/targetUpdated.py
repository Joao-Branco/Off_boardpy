#! /usr/bin/env python2

import rospy
import math
import pickle
import os.path

from MARS_msgs.msg import TargetTelemetry

def target_node():

    # Creates the publisher to publish the target position on the topic "target_position"
    pub = rospy.Publisher('target_position', TargetTelemetry, queue_size=10)
    rospy.init_node('target_node', anonymous=True)

    # Publishing rate
    rate = rospy.Rate(10)

    msg = TargetTelemetry()

    # Time related variables initialization
    start_time = rospy.get_time()
    curr_time = rospy.get_time()
    timer = 0.0
    dt = 0.0
 
    # Target variables initialization (All changeble)
    target_accel = 0.0
    position_x = 0.0
    position_y = 0.0
    target_psi = 0 #math.pi/4
    target_velocity = 0

    while not rospy.is_shutdown():

        # Time variation since last call on loop
        dt = rospy.get_time() - curr_time

        # Total time counter
        timer = rospy.get_time() - start_time

        curr_time = rospy.get_time()

        # Target movement calculations
        target_omega = 0.02*math.cos(0.03*dt) # Changeble
        target_accel = 0.2*math.sin(0.07*dt) # Changeble

        target_psi = target_psi + target_omega * dt
        target_velocity = target_velocity + (target_accel * dt) 
        target_vx = target_velocity * math.cos(target_psi)
        target_vy = target_velocity * math.sin(target_psi)
        position_x = position_x + target_vx * dt
        position_y = position_y + target_vy * dt

        # Defines message to be sent with target movement variables
        msg.x_pos = position_x
        msg.y_pos = position_y
        msg.vx = target_vx
        msg.vy = target_vy
        msg.omega = target_omega
        msg.accel = target_accel
        msg.vel = target_velocity
        msg.psi = target_psi

        # Publishes the target data
        pub.publish(msg)
	#print(msg)

        # Keeps the rate fixed
        rate.sleep()

if __name__ == '__main__':
    try:
        target_node()
    except rospy.ROSInterruptException:
        pass
