#!/usr/bin/env python
"""Node level characterisation tests for the defects corrected on this branch.

Covers the three defects that only show themselves through a running node:

  D4  the pose estimate topic fallback assigns to the wrong field, which leaves
      the pose estimate publisher advertising an empty topic name. In ROS that
      is fatal at construction, so the node never comes up at all.
  D5  the frame id fallbacks carry a leading slash.
  D7  output is stamped with the clock reading taken inside the callback rather
      than with the stamp of the scan that produced it.

The node is launched with no parameter file, so every parameter takes its
hardcoded fallback and all three paths are exercised.
"""

import math
import sys
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

PKG = 'fsm_lo'
NAME = 'node_defects'

SCAN_SIZE = 360
SETTLE = 0.5
TIMEOUT = 20.0

FIRST_STAMP = rospy.Time(1700000000, 0)
SECOND_STAMP = rospy.Time(1700000000, 100000000)


def square_room_scan(pose_x, pose_y, pose_t, width=8.0, height=5.0):
    """Ranges from a pose inside an axis aligned rectangle.

    Kept deliberately simple and independent of the generator: these tests care
    about stamps and frame names, not about the accuracy of the match.
    """
    ranges = []
    for i in range(SCAN_SIZE):
        angle = i * 2.0 * math.pi / SCAN_SIZE + pose_t - math.pi
        best = float('inf')
        for distance, normal in (
                (pose_x, (-1.0, 0.0)),
                (width - pose_x, (1.0, 0.0)),
                (pose_y, (0.0, -1.0)),
                (height - pose_y, (0.0, 1.0))):
            projection = (math.cos(angle) * normal[0] +
                          math.sin(angle) * normal[1])
            if projection > 1e-9:
                best = min(best, distance / projection)
        ranges.append(best if best != float('inf') else 0.0)
    return ranges


def build_scan(stamp, pose):
    scan = LaserScan()
    scan.header.stamp = stamp
    scan.header.frame_id = 'base_laser_link'
    scan.angle_min = -math.pi
    scan.angle_max = math.pi - 2.0 * math.pi / SCAN_SIZE
    scan.angle_increment = 2.0 * math.pi / SCAN_SIZE
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.0
    scan.range_max = 100.0
    scan.ranges = square_room_scan(*pose)
    scan.intensities = []
    return scan


class NodeDefects(unittest.TestCase):

    def setUp(self):
        self.odometry = []
        self.poses = []
        self.paths = []

        rospy.Subscriber('/fsm_lo/lo', Odometry, self.odometry.append)
        rospy.Subscriber('/fsm_lo/pose_estimate', PoseStamped,
                         self.poses.append)
        rospy.Subscriber('/fsm_lo/path_estimate', Path, self.paths.append)

        self.scan_publisher = rospy.Publisher('/base_scan', LaserScan,
                                              queue_size=1)

    def drive_two_scans(self):
        """Bring the node up, start it, and feed it two scans.

        Reaching this point at all is the D4 assertion: with the published
        fallback the pose estimate publisher is constructed with an empty topic
        name, the node dies during construction, and the service below never
        appears.
        """
        rospy.wait_for_service('/fsm_lo/start', timeout=TIMEOUT)
        start = rospy.ServiceProxy('/fsm_lo/start', Empty)
        start()

        deadline = rospy.Time.now() + rospy.Duration(TIMEOUT)
        while (self.scan_publisher.get_num_connections() == 0 and
               rospy.Time.now() < deadline):
            rospy.sleep(0.05)
        self.assertGreater(self.scan_publisher.get_num_connections(), 0,
                           'node did not subscribe to the scan topic')

        self.scan_publisher.publish(build_scan(FIRST_STAMP, (3.0, 2.5, 0.0)))
        rospy.sleep(SETTLE)
        self.scan_publisher.publish(build_scan(SECOND_STAMP, (3.02, 2.5, 0.0)))

        deadline = rospy.Time.now() + rospy.Duration(TIMEOUT)
        while not self.odometry and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
        self.assertTrue(self.odometry, 'node published no odometry')

    def test_d4_node_starts_without_a_pose_estimate_topic_parameter(self):
        self.drive_two_scans()

        self.assertTrue(self.poses, 'nothing was published on the pose '
                                    'estimate topic default')
        self.assertTrue(self.paths, 'nothing was published on the path '
                                    'estimate topic default')

    def test_d5_frame_ids_carry_no_leading_slash(self):
        self.drive_two_scans()

        odometry = self.odometry[0]
        self.assertFalse(odometry.header.frame_id.startswith('/'),
                         'odometry frame id has a leading slash: %r'
                         % odometry.header.frame_id)
        self.assertFalse(odometry.child_frame_id.startswith('/'),
                         'odometry child frame id has a leading slash: %r'
                         % odometry.child_frame_id)
        self.assertFalse(self.poses[0].header.frame_id.startswith('/'),
                         'pose estimate frame id has a leading slash: %r'
                         % self.poses[0].header.frame_id)

    def test_d7_output_is_stamped_from_the_scan(self):
        self.drive_two_scans()

        self.assertEqual(self.odometry[0].header.stamp, SECOND_STAMP,
                         'odometry was not stamped from the scan')
        self.assertEqual(self.poses[0].header.stamp, SECOND_STAMP,
                         'pose estimate was not stamped from the scan')

    def test_d7_twist_uses_the_interval_between_scan_stamps(self):
        self.drive_two_scans()

        interval = (SECOND_STAMP - FIRST_STAMP).to_sec()
        odometry = self.odometry[0]
        expected_x = odometry.pose.pose.position.x / interval

        self.assertAlmostEqual(odometry.twist.twist.linear.x, expected_x,
                               places=9,
                               msg='twist was not derived from the scan stamps')


if __name__ == '__main__':
    rospy.init_node('node_defects_tester', anonymous=True)
    rostest.rosrun(PKG, NAME, NodeDefects, sys.argv)
