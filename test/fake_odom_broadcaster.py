#!/usr/bin/env python

import rospy
import tf

import numpy

class FakeOdomBroadcaster:
    def __init__(self):
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self._br = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.1), self._broadcast_base_to_odom_tf)

    def _broadcast_base_to_odom_tf(self, event):
        self._br.sendTransform((self._x, self._y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self._yaw),
            rospy.Time.now(),
            'base_link', 'odom')

        self._x += 1.0
        self._y += 1.0
        self._yaw += numpy.deg2rad(1.0)


if __name__ == '__main__':
    rospy.init_node('fake_odom_broadcaster', anonymous=True)

    fob = FakeOdomBroadcaster()

    rospy.spin()
