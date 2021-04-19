#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class MeasurementSubscriber:

    def callback(self, data):
        rospy.loginfo(data)

    def listener(self):
        rospy.init_node('measurement_subscriber', anonymous=True)
        rospy.Subscriber('beacon_distances', Float64MultiArray, self.callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

if __name__ == 'main':
    sub = MeasurementSubscriber()
    sub.listener()