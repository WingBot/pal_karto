#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Empty

import unittest

import numpy

class TestLaserReadings(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        srv = rospy.ServiceProxy('slam_karto/start', Empty)
        try:
            srv()
        except rospy.ServiceException as e:
            assert False, 'Failed to start slam_karto: %s' % e

        try:
            cls._val = rospy.get_param('val')
        except KeyError as e:
            assert False, 'val param not set: %s' % e

        # Scan configuration
        cls._frame_id = 'laser'
        cls._range_min = 0.01
        cls._range_max = 9.00
        cls._angle_max = 120.0
        cls._angle_min = -cls._angle_max
        fov = cls._angle_max - cls._angle_min
        cls._angle_increment = 1.0
        cls._num_readings = numpy.round(fov/cls._angle_increment) + 1
        cls._scan_time = 0.1
        cls._time_increment = cls._scan_time / cls._num_readings
        cls._r = (cls._range_max + cls._range_min)/2.0
        cls._idx = numpy.int(cls._num_readings/2)

        cls._pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        rospy.sleep(1.0) # wait for subscribers

    def tearDown(self):
        self._pub.unregister()

    def _create_scan(self):
        scan = LaserScan()
        scan.header.frame_id = self._frame_id
        scan.header.stamp = rospy.get_rostime()
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.angle_min = numpy.deg2rad(self._angle_min)
        scan.angle_max = numpy.deg2rad(self._angle_max)
        scan.angle_increment = numpy.deg2rad(self._angle_increment)
        scan.scan_time = self._scan_time
        scan.time_increment = self._time_increment
        scan.ranges = [self._r] * self._num_readings
        scan.intensities = [1.0] * self._num_readings

        return scan

    def _publish(self, scan):
        self._pub.publish(scan)
        rospy.sleep(1.0)

    def _is_karto_running(self):
        """
        Wait and check that karto is still running.
        """
        try:
            rospy.wait_for_message('slam_trajectory', PoseArray, 5.0)
            return True
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False

    def test_reading(self):
        rospy.loginfo('Test with bad reading = %f' % self._val)

        # Publish sane scan (needed to force scan matching)
        scan = self._create_scan()
        self._publish(scan)

        # Publish corrupted scan
        scan.ranges[self._idx] = self._val
        scan.header.stamp = rospy.get_rostime()
        self._publish(scan)
        if not self._is_karto_running():
            rospy.logerr('Karto is dead after sending the second corrupted scan with the bad reading val = %f' % self._val)
            self.fail()


if __name__ == '__main__':
    import rostest

    PKG_NAME = 'pal_karto'
    TEST_NAME= '%s_laser_readings' % PKG_NAME

    rospy.init_node(TEST_NAME)
    rostest.rosrun(PKG_NAME, TEST_NAME, TestLaserReadings)
