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


from cv_bridge import CvBridge

print('Importing keras...')
from keras_yolo3.yolo import YOLO

print('Keras imported.')







class YoloWrapper:
    def __init__(self):

        # setup subscriber and publishers
        n_uavs = 3

        self.uav_pubs = {}
        self.img_buffer = []

        def cb_maker(uav_id):
            def cb(msg):
                rospy.loginfo(f"got image from uav {uav_id}")
                self.img_buffer.append((uav_id, msg))

            return cb

        for uav_id in range(n_uavs):
            self.uav_pubs[uav_id] = {
                'detector': rospy.Publisher(f"/uav{uav_id}/car_2d_position", target_info, queue_size=10),
                'latency': rospy.Publisher(f"/uav{uav_id}/detector_lantency", Float64, queue_size=10),
                'sub': rospy.Subscriber(f"/uav{uav_id}/original_image_topic", RosImage, cb_maker(uav_id))
            }
            
            
        for uav_id, d in self.uav_pubs.items():
            print(uav_id)
            print(d["detector"].name)


        # setup datector
        self.bridge = CvBridge()
        
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
                         rospy.loginfo(self.)    'n detecoes por frame', #quantas detecoes tive em cada frame em particular
                             'ordem de detecao no frame', #dentro do frame, diz quantas detecoes teve
                             'x min',
                             'x max',
                             'y min',
                             'y max',
                             'confianca',
                             ])"""

    def start(self):

        rospy.loginfo('Car detector script activated!')

        
        while not rospy.is_shutdown():
            # check is msgs in buffer
            while self.img_buffer:
                uav_id, msg = self.img_buffer.pop(0)

                det_pub = self.uav_pubs[uav_id]['detector']
                lat_pub = self.uav_pubs[uav_id]['latency']

                latency = rospy.get_time()
                w, h = msg.width, msg.height
                depth = msg.step / msg.width
                cv_image = np.ndarray((w, h, int(depth)), dtype=np.uint8, buffer=msg.data)
                pil_image = Image.fromarray(cv_image)
                det_image, target_list, n_discarded = self.yolo_detector(pil_image)

                info = target_info()

                # det_img_msg = self.bridge.cv2_to_imgmsg(det_image, encoding="passthrough")
                # info.img = det_img_msg

                info.targets = target_list
                info.n_detecao = n_discarded
                det_pub.publish(info)
                lat_pub.publish(rospy.get_time() - latency)

            rospy.sleep(0)

    def detect_objects(self, pub_detector, pub_latency):
        self.n += 1
        latency = rospy.get_time()
        self.dt_antes = rospy.get_time()
        
        self.callback()
        self.dt_entre_frame = rospy.get_time() - self.dt_antes

        info = target_info()
        detect_img_msg = self.bridge.cv2_to_imgmsg(self.det_image, encoding="passthrough")
        info.img = detect_img_msg
        info.targets = self.target_list
        info.n_detecao = self.n

        pub_detector.publish(info)

        pub_latency.publish(rospy.get_time() - latency)
        
        length = len(self.target_list)
        self.length = length + self.length_anterior
        self.length_anterior = self.length
        if len(self.target_list):
            print('entrei no len')
            self.n_detecao +=1
        else:
            self.no_detection += 1

    def yolo_detector(self, img):
        rospy.loginfo("running yolo...\n\n\n\n")
        rospy.loginfo(img.width)
        out_pred, det_image = self.yolo.detect_image(img, show_stats=False)

        target_list = []
        #y_size, x_size, _ = np.array(det_image).shape #resolucao da imagem
        n_discarded = 0
        for single_prediction in out_pred:
            msgf = self.convert_to_target(single_prediction)
            if msgf.confianca > 0.6:
                target_list.append(msgf)
            else:
                n_discarded +=1
                rospy.loginfo('%s target under 0.6 of confidence',n_discarded)
                pass

        print('full target', target_list)
        return det_image, target_list, n_discarded

    def convert_to_target(self, prevision):
        msg = image_target()
        msg.x_min = prevision[0]
        msg.y_min = prevision[1]
        msg.x_max = prevision[2]
        msg.y_max = prevision[3]
        msg.confianca = prevision[5]
        return msg



if __name__ == '__main__':
    rospy.init_node('car_detectors', log_level=rospy.DEBUG)
   
    rospy.sleep(0)

    try:
        YoloWrapper = YoloWrapper()
        YoloWrapper.start()

    except rospy.ROSInterruptException as exception:
        pass
