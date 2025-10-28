#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist


class HitMe:
    """
    A class to subscribe to a lidar scan, find and publish the closest point to an obstacle and drive our Husky towards the closest point.
    """

    def callback(self, scanData):
        """
        The lidar callback function.
        scanData is the lidar scan data, cf. sensor_msgs/LaserScan
        """

        # TODO 1: Add code here for finding the closest point and setting the output message
        valid = False
        # fill me!
        ranges= np.array(scanData.ranges)
        min_index= np.argmin(ranges)
        min_range = ranges[min_index]
        angle = scanData.angle_min + min_index * scanData.angle_increment
        valid = True if np.isfinite(min_range)else False
        




        # TODO 2: Add code below to output the closest point as a marker in rviz
        m = Marker()
        m.header = scanData.header
        m.type = m.CUBE
        # set positions here
        m.pose.position.x = 0  # fill me!
        m.pose.position.y = 0  # fill me!
        m.pose.position.z = 0  # fill me!
        m.pose.orientation.x = 0
        m.pose.orientation.x = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        m.color.r = 1
        m.color.g = 0
        m.color.b = 0
        m.color.a = 1
        m.scale.x = 0.1
        m.scale.y = 0.1
        m.scale.z = 0.1

        self.closestPtMarkerPublisher.publish(m)

        # TODO 3: Send steering command
        if valid:
            t = Twist()
            # fill me!
            t.linear.x = 0.1
            t.angular.z = -angle
            self.cmdVelPublisher.publish(t)

    def run(self) -> None:
        """
        The main function: prepare publish/subscribe and run main callback functions.
        """
        # Initialize node
        rospy.init_node("hit_me")

        # initialize published topics
        self.closestPtMarkerPublisher = rospy.Publisher(
            "closest_pt_marker", Marker, queue_size=2
        )
        self.cmdVelPublisher = rospy.Publisher("cmd_vel", Twist, queue_size=2)

        # subscribe to Lidar scan data
        rospy.Subscriber("/front/scan", LaserScan, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        print(rospy.get_caller_id() + " initialized. Starting main loop.")
        rospy.spin()


if __name__ == "__main__":
    hit_me = HitMe()
    hit_me.run()
