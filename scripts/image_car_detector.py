#!/mnt/hdd_1_500gb/mgfelix/venv/bin/python

'''#! /usr/bin/env python3'''

import rospy
from sensor_msgs.msg import Image as RosImage
from MARS_msgs.msg import image_target
from MARS_msgs.msg import target_info
from PIL import Image
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tensorflow.python.keras.backend as K # Illegal core dumped.
import csv
from std_msgs.msg import Float64



print('Importing keras...')
from keras_yolo3.yolo import YOLO

print('Keras imported.')


class YoloWrapper:
    def __init__(self):
        self.yolo = YOLO(
            **{
                "model_path": '/home/jbranco/catkin_ws/src/offboard_py/scripts/TrainYourOwnYOLO/Data/Model_Weights/trained_weights_final.h5',
                "anchors_path": '/home/jbranco/catkin_ws/src/offboard_py/scripts/TrainYourOwnYOLO/3_Inference/keras_yolo3/model_data/yolo_anchors.txt',
                "classes_path": '/home/jbranco/catkin_ws/src/offboard_py/scripts/TrainYourOwnYOLO/Data/Model_Weights/data_classes.txt',
                "score": 0.25,
                "gpu_num": 1,
                "model_image_size": (416, 416),
            }
        )
        self.new_image = False
        self.ros_image = RosImage()
        self.n = 0
        self.length_anterior = 0
        self.n_detecao = 0
        self.no_detection = 0
        """d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/detecoes_data.csv", "a")

        with d:
            writer = csv.writer(d)
            writer.writerow(['time',
                             'time entre frame analizado',
                             'n de frames analizados', #contabiliza o numero de frames analisados (com ou sem detecao)
                             'n de frames que tiveram detecao', #contabiliza apenas os frames que tiveram detecao
                             'n de frames sem detecao',
                             'n detecoes em toda a simulacao', #numero total de detecoes feitas em toda a simulacao
                             'n detecoes por frame', #quantas detecoes tive em cada frame em particular
                             'ordem de detecao no frame', #dentro do frame, diz quantas detecoes teve
                             'x min',
                             'x max',
                             'y min',
                             'y max',
                             'confianca',
                             ])"""

    def start(self):
        pub = rospy.Publisher(uav_id + "/car_2d_position", target_info, queue_size=10)
        pub_latencia = rospy.Publisher(uav_id + "/detector_lantency", Float64, queue_size=10)
        self.image_sub = rospy.Subscriber(uav_id + "/original_image_topic", RosImage, self.image_cb)
        #self.image_sub = rospy.Subscriber(uav_id + "/mavros/plane_video/camera/image_raw", RosImage, self.image_cb)

        rospy.loginfo('Car detector script activated!')

        self.det_image = np.zeros((640,640,3))
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            time = rospy.get_time()
            time_plot = time - start_time
            if self.new_image:
                self.n += 1
                latency = rospy.get_time()
                self.dt_antes = rospy.get_time()
                self.callback()
                self.dt_entre_frame = rospy.get_time() - self.dt_antes
                info = target_info()
                info.img = self.ros_image
                info.targets = self.target_list
                info.n_detecao = self.n
                pub.publish(info)
                pub_latencia.publish(rospy.get_time() - latency)
                length = len(self.target_list)
                self.length = length + self.length_anterior
                self.length_anterior = self.length
                if len(self.target_list):
                    print('entrei no len')
                    self.n_detecao +=1
                    # for i in range(length):
                        # d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/detecoes_data.csv", "a")
                        # with d:
                        #     writer = csv.writer(d)
                        #     writer.writerow([time_plot,
                        #                      self.dt_entre_frame,
                        #                      self.n,
                        #                      self.n_detecao,
                        #                      [],
                        #                      self.length,
                        #                      length,
                        #                      i,
                        #                      self.target_list[i].x_min,
                        #                      self.target_list[i].x_max,
                        #                      self.target_list[i].y_min,
                        #                      self.target_list[i].y_max,
                        #                      self.target_list[i].confianca,
                        #                      ])
                else:
                    self.no_detection += 1
                    # d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/detecoes_data.csv", "a")
                    # with d:
                    #     writer = csv.writer(d)
                    #     writer.writerow([time_plot,
                    #                      self.dt_entre_frame,
                    #                      self.n,
                    #                      [],
                    #                      self.no_detection,
                    #                      ])

            rospy.sleep(0)

    def convert_to_target(self, prevision):
        msg = image_target()
        msg.x_min = prevision[0]
        msg.y_min = prevision[1]
        msg.x_max = prevision[2]
        msg.y_max = prevision[3]
        msg.confianca = prevision[5]
        return msg


    def callback(self):
        rospy.loginfo("running yolo...\n\n\n\n")
        self.out_pred, self.det_image = self.yolo.detect_image(self.image, show_stats=False)
        rospy.loginfo(type(self.det_image))
        im =np.array(self.det_image)
        im_2 = cv2.resize(im, (640, 400))
        cv2.imshow("window",np.array(self.det_image))
        cv2.imshow("window",im_2)
        cv2.waitKey(1)
        self.target_list = []
        self.new_image = False
        y_size, x_size, _ = np.array(self.det_image).shape #resolucao da imagem
        print(y_size)
        print(x_size)
        n = 0
        for single_prediction in self.out_pred:
            msgf = self.convert_to_target(single_prediction)
            if msgf.confianca > 0.6:
                self.target_list.append(msgf)
            else:
                n +=1
                rospy.logfatal('%s target under 0.6 of confidence',n)
                pass

        print('full target', self.target_list)
        self.num = n
        self.new_detection = True

    def image_cb(self, msg):
        self.ros_image = msg #guardar a imagem
        car_image = msg #converter para imagem PIL rgb
        w, h = car_image.width, car_image.height
        depth = car_image.step / car_image.width
        self.cv_image = np.ndarray((w, h, int(depth)), dtype=np.uint8, buffer=car_image.data)
        self.image = Image.fromarray(self.cv_image)
        self.new_image = True
        #rospy.loginfo('Got Image')


if __name__ == '__main__':
    rospy.init_node('image_car_node', anonymous=True, log_level=rospy.DEBUG)

    global uav_id_number, uav_id
    uav_id = rospy.get_param("~uav_id")
    uav_id_number = uav_id[-1]
    
    rospy.sleep(0)

    try:
        YoloWrapper = YoloWrapper()
        YoloWrapper.start()

    except rospy.ROSInterruptException as exception:
        pass
