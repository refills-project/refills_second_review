import os
import random

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
from refills_msgs.msg import Barcode
from sensor_msgs.msg import Image
from tf.transformations import quaternion_from_matrix

import giskardpy.tfwrapper as tf
from giskardpy.python_interface import GiskardWrapper
from giskardpy.utils import create_path

class Muh(object):
    def __init__(self):
        self.giskard = GiskardWrapper()
        self.barcode_sub = rospy.Subscriber('barcode/pose', Barcode, self.barcode_cb, queue_size=10)
        self.barcodes = []
        self.barcodes_realsense = []
        self.bridge = CvBridge()
        self.left = True

    def make_cam_horizontal(self, left=True):
        self.left = left
        base_link_T_cam = tf.lookup_pose('base_footprint', 'camera_link')
        if not self.left:
            base_link_T_cam.pose.orientation = Quaternion(*quaternion_from_matrix(
                [[-1, 0, 0, 0],
                 [0, 0, -1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]]
            ))
        else:
            base_link_T_cam.pose.orientation = Quaternion(*quaternion_from_matrix(
                [[1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]]
            ))
        self.giskard.set_cart_goal(base_link_T_cam, 'camera_link', 'base_footprint')
        self.giskard.plan_and_execute()

    def move_down_vertical(self, distance):
        base_link_T_cam = tf.lookup_pose('base_footprint', 'camera_link')
        if not self.left:
            base_link_T_cam.pose.orientation = Quaternion(*quaternion_from_matrix(
                [[-1, 0, 0, 0],
                 [0, 0, -1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]]
            ))
        else:
            base_link_T_cam.pose.orientation = Quaternion(*quaternion_from_matrix(
                [[1, 0, 0, 0],
                 [0, 0, 1, 0],
                 [0, -1, 0, 0],
                 [0, 0, 0, 1]]
            ))
        base_link_T_cam.pose.position.z += distance
        self.giskard.set_cart_goal(base_link_T_cam, 'camera_link', 'base_footprint')
        self.giskard.plan_and_execute()

    def take_picture(self, layer_type):
        self.barcodes = []
        # self.barcodes_realsense = []
        print('waiting for barcodes')
        rospy.sleep(2)
        print('waiting for image')
        img_gray = rospy.wait_for_message('/refills_wrist_camera/image_raw', Image, timeout=2)
        # img_rs = rospy.wait_for_message('/rs_camera/color/image_raw', Image, timeout=2)
        cv_img_gray = self.bridge.imgmsg_to_cv2(img_gray, "bgr8")
        # cv_img_rs = self.bridge.imgmsg_to_cv2(img_rs, "bgr8")
        if len(self.barcodes) == 0:
            rospy.logerr('no barcodes detected, check if the barcode finder is running!!')
            return
        barcodes = np.array(self.barcodes)
        # barcodes_rs = np.array(self.barcodes_realsense)
        barcode_ys = barcodes[:, 1]
        # barcode_rs_ys = barcodes_rs[:, 1]
        lines = []
        lines_rs = []
        if max(barcode_ys) - min(barcode_ys) > 0.1:
            avg = np.average(barcode_ys)
            # avg_rs = np.average(barcode_rs_ys)
            lines.append(np.average(barcodes[barcode_ys < avg], axis=0))
            lines.append(np.average(barcodes[barcode_ys > avg], axis=0))
            # lines_rs.append(np.average(barcodes_rs[barcode_rs_ys < avg_rs], axis=0))
            # lines_rs.append(np.average(barcodes_rs[barcode_rs_ys > avg_rs], axis=0))
        else:
            lines.append(np.average(self.barcodes, axis=0))
            # lines_rs.append(np.average(self.barcodes_realsense, axis=0))
        print('got lines at {}'.format(lines))
        if layer_type == 'b':
            layer_type = 'bucket'
        elif layer_type == 'h':
            layer_type = 'hanging'
        elif layer_type == 's':
            layer_type = 'standing'
        else:
            print('unknown key')
            return
        for line in lines:
            rnd_hash = int(random.random() * 100000)
            img_name_gray = '{}_x_{}_y_{}_z_{}_rndnumber_{}_color.png'.format(layer_type, line[0], line[1], line[2],
                                                                              rnd_hash)
            # img_name_rs = '{}_x_{}_y_{}_z_{}_rndnumber_{}_realsense.png'.format(layer_type, line_rs[0], line_rs[1], line_rs[2],
            #                                                                     rnd_hash)
            img_path_gray = '~/{}/{}'.format(layer_type, img_name_gray)
            # img_path_rs = '/tmp/{}/{}'.format(layer_type, img_name_rs)
            cv2.imwrite(img_path_gray, cv_img_gray)
            # cv2.imwrite(img_path_rs, cv_img_rs)
            print('saved image at {}'.format(img_path_gray))
            # print('saved image at {}'.format(img_path_rs))
        self.get_number_of_images()

    def take_picture_while_moving(self, layer_type):
        self.barcodes = []
        # self.barcodes_realsense = []
        print('waiting for barcodes')
        rospy.sleep(2)
        print('waiting for image')
        img_gray = rospy.wait_for_message('/refills_wrist_camera/image_raw', Image, timeout=2)
        # img_rs = rospy.wait_for_message('/rs_camera/color/image_raw', Image, timeout=2)
        cv_img_gray = self.bridge.imgmsg_to_cv2(img_gray, "bgr8")
        # cv_img_rs = self.bridge.imgmsg_to_cv2(img_rs, "bgr8")
        if len(self.barcodes) == 0:
            rospy.logerr('no barcodes detected, check if the barcode finder is running!!')
            return
        barcodes = np.array(self.barcodes)
        # barcodes_rs = np.array(self.barcodes_realsense)
        barcode_ys = barcodes[:, 1]
        # barcode_rs_ys = barcodes_rs[:, 1]
        lines = []
        lines_rs = []
        if max(barcode_ys) - min(barcode_ys) > 0.1:
            avg = np.average(barcode_ys)
            # avg_rs = np.average(barcode_rs_ys)
            lines.append(np.average(barcodes[barcode_ys < avg], axis=0))
            lines.append(np.average(barcodes[barcode_ys > avg], axis=0))
            # lines_rs.append(np.average(barcodes_rs[barcode_rs_ys < avg_rs], axis=0))
            # lines_rs.append(np.average(barcodes_rs[barcode_rs_ys > avg_rs], axis=0))
        else:
            lines.append(np.average(self.barcodes, axis=0))
            # lines_rs.append(np.average(self.barcodes_realsense, axis=0))
        print('got lines at {}'.format(lines))
        if layer_type == 'b':
            layer_type = 'bucket'
        elif layer_type == 'h':
            layer_type = 'hanging'
        elif layer_type == 's':
            layer_type = 'standing'
        else:
            print('unknown key')
            return
        for line in lines:
            rnd_hash = int(random.random() * 100000)
            img_name_gray = '{}_x_{}_y_{}_z_{}_rndnumber_{}_color.png'.format(layer_type, line[0], line[1], line[2],
                                                                              rnd_hash)
            # img_name_rs = '{}_x_{}_y_{}_z_{}_rndnumber_{}_realsense.png'.format(layer_type, line_rs[0], line_rs[1], line_rs[2],
            #                                                                     rnd_hash)
            img_path_gray = '~/{}/{}'.format(layer_type, img_name_gray)
            # img_path_rs = '/tmp/{}/{}'.format(layer_type, img_name_rs)
            cv2.imwrite(img_path_gray, cv_img_gray)
            # cv2.imwrite(img_path_rs, cv_img_rs)
            print('saved image at {}'.format(img_path_gray))
            # print('saved image at {}'.format(img_path_rs))
        self.get_number_of_images()



    def get_number_of_images(self):
        DIR = '~/standing'
        standing = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
        DIR = '~/bucket'
        bucket = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
        DIR = '~/hanging'
        hanging = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
        print('{} standing items'.format(standing))
        print('{} hanging items'.format(hanging))
        print('{} bucket items'.format(bucket))

    def barcode_cb(self, data):
        """
        :type data: Barcode
        :return:
        """
        cam_T_barcode = tf.transform_pose('camera_link', data.barcode_pose)
        # cam2_T_barcode = tf.transform_pose('rs_camera_color_optical_frame', data.barcode_pose)
        self.barcodes.append([cam_T_barcode.pose.position.x,
                              cam_T_barcode.pose.position.y,
                              cam_T_barcode.pose.position.z])
        # self.barcodes_realsense.append([cam2_T_barcode.pose.position.x,
        #                                 cam2_T_barcode.pose.position.y,
        #                                 cam2_T_barcode.pose.position.z])


rospy.init_node('stefan')
create_path('~/standing/')
create_path('~/hanging/')
create_path('~/bucket/')
tf.init(2)
muh = Muh()
muh.get_number_of_images()
while not rospy.is_shutdown():
    key = raw_input('press button\n')
    if key == 'l':
        muh.make_cam_horizontal(True)
    elif key == 'r':
        muh.make_cam_horizontal(False)
    elif key == 'u':
        muh.move_down_vertical(0.05)
    elif key == 'd':
        muh.move_down_vertical(-0.05)
    else:
        while True:
                muh.take_picture_while_moving(key)





