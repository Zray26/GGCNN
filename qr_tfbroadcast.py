#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.06, 0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "qr_tr",
                         "ar_marker_7")
        rate.sleep()
