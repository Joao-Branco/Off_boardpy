#! /usr/bin/env python3

from __future__ import division
import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, VFR_HUD
from mavros_msgs.srv import StreamRateRequest, StreamRate
from tf.transformations import euler_from_quaternion
from MARS_msgs.msg import TargetTelemetry
from MARS_msgs.msg import target_info
from std_msgs.msg import Float64
import time
import csv
import os
from numpy.linalg import inv
import matplotlib.pyplot as plt

filename = 'last_target_pose.pkl'


def image2mountFunct():
    image2mountMat = np.array([[0, 0, 1],  # transformation from the camera to gimbal/mount frame
                               [1, 0, 0],
                               [0, 1, 0]])
    return image2mountMat


# Body to Mount Matrix
def body2mountFunct(tilt, pan):
    body2mountMat = np.array([[math.cos(tilt) * math.cos(pan), math.cos(tilt) * math.sin(pan), -math.sin(tilt)],
                              [-math.sin(pan), math.cos(pan), 0],
                              [math.sin(tilt) * math.cos(pan), math.sin(tilt) * math.sin(pan), math.cos(tilt)]])
    return body2mountMat


# Mount to Body Matrix
def mount2bodyFunct(tilt, pan):
    b2mMat = body2mountFunct(tilt, pan)
    mount2bodyMat = inv(b2mMat)  # matriz inversa ou elevado a -1

    return mount2bodyMat


# Body to Inertial Matrix
# def inertial2bodyFunct(pitch, yaw, roll):
#     inertial2bodyMat = np.array([[math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)],
#                                  [math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw),
#                                   math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw),
#                                   math.sin(roll) * math.cos(pitch)],
#                                  [math.cos(roll) * math.sin(yaw) * math.cos(yaw) + math.sin(roll) * math.sin(yaw),
#                                   math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw),
#                                   math.cos(roll) * math.cos(pitch)]])
#     return inertial2bodyMat
#
# # Inertial to Body Matrix
# def body2inertialFunct(pitch, yaw, roll):
#     i2bMat = inertial2bodyFunct(pitch, yaw, roll)
#     body2inertialMat = inv(i2bMat)  # matriz inversa ou elevado a -1
#     return body2inertialMat

def body2inertialFunct(pitch, yaw, roll):
    body2inertialMat = np.array([[math.cos(yaw) * math.cos(pitch),
                                  (-math.sin(yaw) * math.cos(roll)) + (
                                              math.cos(yaw) * math.sin(pitch) * math.sin(roll)),
                                  (math.sin(yaw) * math.sin(roll)) + (
                                              math.cos(yaw) * math.sin(pitch) * math.cos(roll))],
                                 [math.sin(yaw) * math.cos(pitch),
                                  (math.cos(yaw) * math.cos(roll)) + (math.sin(yaw) * math.sin(pitch) * math.sin(roll)),
                                  (-math.cos(yaw) * math.sin(roll)) + (
                                              math.sin(yaw) * math.sin(pitch) * math.cos(roll))],
                                 [-math.sin(pitch), math.cos(pitch) * math.sin(roll),
                                  math.cos(pitch) * math.cos(roll)]])

    return body2inertialMat


def pixel2WorldCoordinateFunct(image_coordx, image_coordy, roll, pitch, yaw, uavAlt, pan, tilt, sensor_width,
                               sensor_height, sensorFocalDistance, x_UAV, y_UAV):
    Image2MountM = image2mountFunct()
    Mount2BodyM = mount2bodyFunct(tilt, pan)
    Body2InertialM = body2inertialFunct(pitch, yaw, roll)

    ImageCoordFocalTemp = np.array(
        [[image_coordx - sensor_height / 2], [image_coordy - sensor_width / 2], [sensorFocalDistance]])

    ImageCoordFocalTemp_norm = math.sqrt(
        (image_coordx - sensor_height / 2) ** 2 + (image_coordy - sensor_width / 2) ** 2 + sensorFocalDistance ** 2)
    # ImageCoordFocalTemp_norm = np.linalg.norm(ImageCoordFocalTemp)

    imageCoordFocal = ImageCoordFocalTemp * (1 / ImageCoordFocalTemp_norm)  # l_c

    # print("imageCoordFocal", imageCoordFocal)

    MountVector = np.matmul(Image2MountM, imageCoordFocal)
    # print("MountVector", MountVector)
    BodyVector = np.matmul(Mount2BodyM, MountVector)

    NED_unitary = np.matmul(Body2InertialM, BodyVector)

    vector_norm = uavAlt / NED_unitary[2]  # representa o L

    NED_vector = NED_unitary * vector_norm
    # print('relativ_position', NED_vector)

    target_3d_coordinate = NED_vector + np.array([[y_UAV], [x_UAV], [-uavAlt]])

    return target_3d_coordinate


# def pixel2WorldCoordinateFunct_relative(image_coordx, image_coordy, roll, pitch, yaw, uavAlt, pan, tilt, sensor_width,
#                                         sensor_height, sensorFocalDistance, x_UAV, y_UAV):
#     Image2MountM = image2mountFunct()
#     Mount2BodyM = mount2bodyFunct(tilt, pan)
#     Body2InertialM = body2inertialFunct(pitch, yaw, roll)
#
#     ImageCoordFocalTemp = np.array(
#         [[image_coordx - sensor_height / 2], [image_coordy - sensor_width / 2], [sensorFocalDistance]])
#
#     ImageCoordFocalTemp_norm = np.linalg.norm(ImageCoordFocalTemp)
#
#     imageCoordFocal = ImageCoordFocalTemp * (1 / ImageCoordFocalTemp_norm)
#
#     MountVector = np.matmul(Image2MountM, imageCoordFocal)
#
#     BodyVector = np.matmul(Mount2BodyM, MountVector)
#
#     NED_unitary = np.matmul(Body2InertialM, BodyVector)
#
#     vector_norm = uavAlt / NED_unitary[2]  # representa o L
#
#     NED_vector = NED_unitary * vector_norm
#
#     return NED_vector


class Inicializacao(object):

    def __init__(self):
        self.state = State()
        self.local_position = PoseStamped()  # fcu local position
        self.rate = rospy.Rate(20)
        self.service_timeout = 30
        self.setup_pubsub()
        self.setup_services()
        self.target_2d_position = None
        self.erro_posicao = 0
        self.tempo_exato = 0
        self.vfr_hud = VFR_HUD()

        # self.animated_box_position = gazebo_republish()

        rospy.loginfo("init finished")

    '''ros subscribers/publisher'''

    def setup_pubsub(self):
        rospy.loginfo("-------Setting pub - sub-----")
        # subscriber
        self.state_sub = rospy.Subscriber(uav_id + "/mavros/state", State, self.state_cb)
        self.local_position_sub = rospy.Subscriber(uav_id + "/mavros/local_position/pose", PoseStamped, self.local_position_cb)
        # subscriber para target
        self.target2d_sub = rospy.Subscriber(uav_id + "/car_2d_position", target_info, self.image_target_cb)
        self.VFR_HUD_sub = rospy.Subscriber(uav_id + "/mavros/vfr_hud", VFR_HUD, self.vfr_hud_cb)

        # d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/target_position_geo.csv", "a")

        # with d:
        #     writer = csv.writer(d)
        #     writer.writerow(['target.x_pos',
        #                      'target.y_pos',
        #                      ])
    # ros services
    def setup_services(self):
        rospy.loginfo('----Waiting for services to connect----')
        try:
            rospy.wait_for_service('/mavros/set_stream_rate', self.service_timeout)
            rospy.loginfo('Services for video processing - georeferentiation - are connected and ready')
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service for video processing - georeferentiation')

        # set services
        self.set_stream_rate_srv = rospy.ServiceProxy(uav_id + '/mavros/set_stream_rate', StreamRate)

        '''set mavros stream rate to get mavros messages faster. 
        mavros publishes state/setpoint messages at 1 hz by default
        '''
        rospy.loginfo('Services setup finished')

    def set_mavros_stream_rate(self):
        stream_rate = StreamRateRequest()
        stream_rate.request.stream_id = 3
        stream_rate.request.message_rate = 10
        stream_rate.request.on_off = 1
        try:
            self.set_stream_rate_srv(stream_rate)
        except rospy.ServiceException as exp:
            rospy.logerr('Stream rate service failed')
        rospy.loginfo('setup mavros stream rate finished')

    def start(self):
        # wait to get heartbeat from fcu
        while not self.state.connected:
            self.rate.sleep()
        rospy.loginfo('--Got heartbeat from FCU----')

        pub = rospy.Publisher(uav_id + "/target_position_geolocation", TargetTelemetry, queue_size=1)
        pub_lat = rospy.Publisher(uav_id + "/target_position_geolocation_lat", Float64, queue_size=1)

        

        while not rospy.is_shutdown():
            if self.target_2d_position is not None:
                start_time = rospy.Time.now().to_nsec() * 1e-9

                targets = self.target_2d_position.targets

                self.target_2d_position = None

                # Get time in every loop iteration
                yaw = euler_from_quaternion([self.local_position.pose.orientation.x, self.local_position.pose.orientation.y,
                                            self.local_position.pose.orientation.z,
                                            self.local_position.pose.orientation.w])[2]
                yaw_transform1 = yaw - np.pi * 0.5
                yaw_transform2 = - yaw_transform1

                # yaw wraptopi
                while yaw_transform2 < -math.pi:
                    yaw_transform2 = yaw_transform2 + 2 * math.pi

                while yaw_transform2 > math.pi:
                    yaw_transform2 = yaw_transform2 - 2 * math.pi

                roll = \
                    euler_from_quaternion([self.local_position.pose.orientation.x, self.local_position.pose.orientation.y,
                                        self.local_position.pose.orientation.z,
                                        self.local_position.pose.orientation.w])[0]
                # roll wraptopi
                while roll < -math.pi:
                    roll = roll + 2 * math.pi

                while roll > math.pi:
                    roll = roll - 2 * math.pi

                pitch = \
                    euler_from_quaternion([self.local_position.pose.orientation.x, self.local_position.pose.orientation.y,
                                        self.local_position.pose.orientation.z,
                                        self.local_position.pose.orientation.w])[1]
                # pitch wraptopi
                while pitch < -math.pi:
                    pitch = pitch + 2 * math.pi

                while pitch > math.pi:
                    pitch = pitch - 2 * math.pi

                # definicao de variaveis

                uav_alt = np.abs(self.local_position.pose.position.z)
                x_UAV = self.local_position.pose.position.x
                y_UAV = self.local_position.pose.position.y
                pan_t = 1.2  # -1.2  # angles must be radians
                tilt_t = -0.633668  # angles must be radians
                hfov = 1.57
                # tilt_t = 0  # angles must be radians
                sensor_width_t = 640  # aumentamos
                sensor_height_t = 640
                sensorFocalDistance_t = (sensor_width_t/2)/(math.tan(hfov/2))

                length = len(targets)
                # rospy.loginfo("length %s", length)
                if length > 0:

                    # sort detections by confidence, decreasing confidence
                    confidence_sorted_targets = sorted(targets, key=lambda x: x.confianca, reverse=True)
                    max_target = confidence_sorted_targets[0]


                    target_2d_vertical_coordinate = max_target.x_min + ((max_target.x_max - max_target.x_min) / 2)
                    target_2d_horizontal_coordinate = max_target.y_min + ((max_target.y_max - max_target.y_min) / 2)

                    #executar a geolocalizacao
                    raw_target_3d_coordinate = pixel2WorldCoordinateFunct(target_2d_vertical_coordinate,
                                                                    target_2d_horizontal_coordinate, roll, pitch,
                                                                    yaw_transform2, uav_alt, pan_t, tilt_t,
                                                                    sensor_width_t,
                                                                    sensor_height_t, sensorFocalDistance_t, x_UAV,
                                                                    y_UAV)

                    measurement_x = raw_target_3d_coordinate[1]  # valor east
                    measurement_y = raw_target_3d_coordinate[0]  # valor north

                    # print('GEO position x ', measurement_x[0])
                    # print('GEO position y ', measurement_y[0])
                
                    target_position_geolocation_msg = TargetTelemetry()

                    target_position_geolocation_msg.x_pos = measurement_x[0]
                    target_position_geolocation_msg.y_pos = measurement_y[0]
                    target_position_geolocation_msg.timestamp = rospy.Time.now()

                    lat = rospy.Time.now().to_nsec() * 1e-9 - start_time
                    print(lat)
                    lat_msg = Float64()
                    lat_msg.data = lat
                    pub_lat.publish(lat_msg)


                    pub.publish(target_position_geolocation_msg)

                    

            # d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/target_position_geo.csv", "a")

            # with d:
            #     writer = csv.writer(d)
            #     writer.writerow([target_position_geolocation_msg.x_pos,
            #                      target_position_geolocation_msg.y_pos,
            #                      ])
            self.rate.sleep()

    # callback functions
    def altitude_cb(self, data):
        self.altitude = data

        # pass

    def attitude_cb(self, data):
        self.att = data

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def extended_state_cb(self, data):
        pass

    def setpoint_raw_cb(self, data):
        pass

    def local_position_cb(self, data):
        self.local_position = data
        # posicao = data

    def local_position_odom_cb(self, data):
        self.local_position_odom = data

    def imu_cb(self, data):
        self.imu = data

    def vfr_hud_cb(self, data):
        self.vfr_hud = data

    def image_target_cb(self, data):
        # rospy.loginfo(f'got targets:{data.targets}')
        self.target_2d_position = data

if __name__ == '__main__':
    rospy.init_node('georeferenciacao', anonymous=True)

    global uav_id_number, uav_id
    uav_id = rospy.get_param("~uav_id")
    uav_id_number = uav_id[-1]

    rospy.sleep(0)

    try:
        inicializacao = Inicializacao()
        inicializacao.start()

        rospy.spin()

    except rospy.ROSInterruptException as exception:
        pass
