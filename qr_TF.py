#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
from threading import Thread
import rospy
import tf
from detect_qr.msg import qrmsg
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
import math

class qr_tftransform(object):
    def __init__(self):
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.qr_width = 0.067
        self.qr_tr_pos = []
        self.qr_bl_pos = []
        self.pl_array_width = []
        self.pl_array_height = []
        self.pl_width_mean = None
        self.pl_height_mean = None
        self.pl_ratio = None # pixel to length ratio
        self.count = 0
        self.cam_to_qr = []
        self.bolt_x = None
        self.bolt_y = None
        # tr1 = Thread(target = self.set_qr_frame)
        # tr2 = Thread(target = self.get_qr_frame)
        tr3 = Thread(target = self.get_camera_to_qr_frame)
        tr4 = Thread(target = self.qr_img_pos_subscriber)
        tr5 = Thread(target = self.get_bolt_pos_subscriber)
        # self.qr_img_pos_subscriber()
        # tr1.start()
        # tr2.start()
        tr3.start()
        tr4.start()
        tr5.start()
    def set_qr_frame(self):
        while not rospy.is_shutdown():
            self.br.sendTransform((0.06, 0, 0.0),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "qr_tr",
                            "ar_marker_7")
            # print("in set")
            self.rate.sleep()
    def get_qr_frame(self):
        while not rospy.is_shutdown():
            # print("in get ")
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/camera_link', '/qr_tr', rospy.Time(0))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print( " no tf")
                continue
            self.rate.sleep()

    def bolt_img_callback(self,data):
        # print("in bolt img callback")
        boxes = data.bounding_boxes
        bolt = boxes[0]
        # print(self.bolt[0])
        self.bolt_x = (bolt.xmin + bolt.xmax) * 0.5
        self.bolt_y = (bolt.ymin + bolt.ymax) * 0.5
        

    def get_bolt_pos_subscriber(self):
        print("hello@")
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bolt_img_callback, queue_size =1, buff_size= 2**24)
        rospy.spin()

    def bolt_cal(self):
        # print("in bolt_cal")
        # print(self.qr_tr_pos)
        qr_tr_x = self.qr_tr_pos[0] + 0.5 * self.qr_tr_pos[2]
        qr_tr_y = self.qr_tr_pos[1] + 0.5 * self.qr_tr_pos[3]
        bolt_x = 630 + 93*0.5
        bolt_y = 276 + 103 *0.5

        v_ab_x = self.pl_ratio * (bolt_x - qr_tr_x)
        v_ab_y = self.pl_ratio * (bolt_y - qr_tr_y)
        v_ab = [v_ab_x, v_ab_y, 0,1]


        T = [[ -0.9944552, -0.0674844,  0.0806518, 0.188],
        [-0.0643657,  0.9970975,  0.0406652, 0.207],
        [-0.0831619,  0.0352485, -0.9959125, 0  ],
        [0, 0, 0, 1]] # from qr_tf to camera_color_optical_frame  Inverse

# [  0.0807208,  0.0386689, -0.9959864;
#    0.9944550,  0.0644501,  0.0830990;
#    0.0674048, -0.9971715, -0.0332520 ]
        # T = [[ 1, 0, 0, 0],
        # [0,  1,  0, 0],
        # [0,  0, 1, 0  ],
        # [0, 0, 0, 1]] # from qr_tf to camera_color_optical_frame 

#         [ -0.9941204, -0.0641015, -0.0872678;
#   -0.0677158,  0.9969381,  0.0391021;
#    0.0844941,  0.0447816, -0.9954172 ]
        v_ab_qr_frame = np.dot(T, v_ab)
        # print(v_ab_qr_frame)
        # while not rospy.is_shutdown():
            # x = v_ab_qr_frame[0,3]
            # y = v_ab_qr_frame[1,3]
            # z = v_ab_qr_frame[2,3]
        x = v_ab_qr_frame[0]
        y = v_ab_qr_frame[1]
        z = v_ab_qr_frame[2]    

        self.br.sendTransform((x, y, z),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "bolt1",
                        "qr_tr")
            # print("in set")
            # self.rate.sleep()

#         T = [0.]
    def get_camera_to_qr_frame(self):
        # print(" in get camera to qr")
        while not rospy.is_shutdown():
            # print("in get ")
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('qr_tr', '/camera_color_optical_frame' , rospy.Time(0))
                # print(trans)
                # print(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print( " no tf")
                continue
            # marker_trans = geometry_msgs.msg.Transform()
            x= trans[0]
            y= trans[1]
            z= trans[2]
            rx=rot[0]
            ry=rot[1]
            rz=rot[2]
            rw=rot[3]
            # print(rot)
            q0 = rw
            q1 = rx
            q2 = ry
            q3 = rz

            # Calculate the translation matrix
            R = np.array([
                        [1-2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
                        [2*(q1*q2 + q0*q3), 1-2*(q1**2+q3**2),2*(q2*q3 - q0*q1)],
                        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1- 2*(q1**2 + q2 **2)]
            ])
            # T = np.column_stack((R, np.transpose(trans)))
            T = np.column_stack((R, [0,0,0]))
            T = np.row_stack((T, [0.0, 0.0, 0.0, 1.0]))
            # print(T)
            self.cam_to_qr = T
            if self.pl_ratio is not None:
                qr_tr_x = self.qr_tr_pos[0] + 0.5 * self.qr_tr_pos[2]
                qr_tr_y = self.qr_tr_pos[1] + 0.5 * self.qr_tr_pos[3]
                # print(qr_tr_x)
                # print(qr_tr_y)
                # bolt_x = 719 63 272 128
                # bolt_x = 630 + 93*0.5
                # bolt_y = 276 + 103 *0.5
                bolt_x = self.bolt_x
                bolt_y = self.bolt_y

                v_ab_x = self.pl_ratio * (bolt_x - qr_tr_x)
                v_ab_y = self.pl_ratio * (bolt_y - qr_tr_y)
                v_ab = [v_ab_x, v_ab_y, 0,1]
                V = [[0.707, -0.707, 0, v_ab_x], [0.707, 0.707, 0, v_ab_y], [0, 0 , 1, 0], [0,0,0,1]]
                v_ab_qr_frame = np.dot(T, V)# for matrix
                R = v_ab_qr_frame[0:3,0:3]
                ww = 0.5 * math.sqrt(1 + R[0][0] + R[1][1]+R[2][2])
                xx = (R[2][1] - R[1][2])/(4 * ww)
                yy = (R[0][2] - R[2][0])/(4 * ww)
                zz = (R[1][0] - R[0][1])/(4 * ww)
                x = v_ab_qr_frame[0][3]
                y = v_ab_qr_frame[1][3]
                z = v_ab_qr_frame[2][3]




                # v_ab_qr_frame = np.dot(T, v_ab) for vector
                # x = v_ab_qr_frame[0]
                # y = v_ab_qr_frame[1]
                # z = v_ab_qr_frame[2]    

                self.br.sendTransform((x, y, z),
                                (xx, yy, zz, ww),
                                rospy.Time.now(),
                                "bolt1",
                                "qr_tr")
            # arm_final = np. matmul(T,l_trans)
            # final_rot = arm_final[0:3][0:3]


            # trans_pub = geometry_msgs.msg.Transform()
            # trans_pub.translation.x = arm_final[0][3]
            # trans_pub.translation.y = arm_final[1][3]
            # trans_pub.translation.z = arm_final[2][3]
            # lw = 0.5*math.sqrt(1 + final_rot[0][0] + final_rot[1][1] + final_rot[2][2])
            # trans_pub.rotation.x=(final_rot[2][1] -final_rot[1][2])/(4 * lw)
            # trans_pub.rotation.y=(final_rot[0][2] -final_rot[2][0])/(4 * lw)
            # trans_pub.rotation.z=(final_rot[1][0] -final_rot[0][1])/(4 * lw)
            # trans_pub.rotation.w=lw
            # ee_gear.publish(trans_pub)
            self.rate.sleep()

    def get_qr_to_base(self):
        while not rospy.is_shutdown():
            try:
                # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
                (trans,rot) = self.listener.lookupTransform('/base_link', '/qr_tr', rospy.Time(0))
                # print(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # print( " no tf")
                continue
            self.rate.sleep()
        
    def qr_img_callback(self,data):
        self.qr_bl_pos = data.qrpos[4:8]
        self.qr_tr_pos = data.qrpos[0:4]
        if self.count <300:
            print(self.count)
            self.pl_array_width.append(data.qrpos[6])
            self.pl_array_height.append(data.qrpos[7])
            self.count +=1
        else:
            self.pl_width_mean = np.mean(self.pl_array_width)
            self.pl_height_mean = np.mean(self.pl_array_height)
            self.pl_ratio = self.qr_width / np.minimum(self.pl_height_mean, self.pl_width_mean)
            # print(self.ratio)
            # self.bolt_cal()


    def qr_img_pos_subscriber(self):
        rospy.Subscriber("/qr_pos", qrmsg, self.qr_img_callback,queue_size =1, buff_size= 2**24)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('qr_TF')
    qr = qr_tftransform()
    # br = tf.TransformBroadcaster()
    # listener = tf.TransformListener()
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         # listener.waitForTransform('/camera_link', '/qr_1', rospy.Time.now(), rospy.Duration(4.0))
    #         (trans,rot) = listener.lookupTransform('/camera_link', '/qr_1', rospy.Time(0))
    #         print(trans)
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print( " no tf")
    #         continue
    #     rate.sleep()
