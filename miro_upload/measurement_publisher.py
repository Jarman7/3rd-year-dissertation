#!/usr/bin/env python
import rospy
from bluetoother_scanner import BluetootherScanner
from std_msgs.msg import Float64MultiArray

class MeasurementPublisher:

    RATE = 10

    def __init__(self):
        self.scanner = BluetootherScanner()
        self.publisher = rospy.Publisher('beacon_distances', Float64MultiArray, queue_size=10)
        rospy.init_node('measurement_publisher', anonymous=True)
        self.rate = rospy.Rate(self.RATE)

    
    def publish(self):
        while not rospy.is_shutdown():
            measurements = self.scanner.compute_average()
            measurements_msg = Float64MultiArray()
            measurements_msg.data = measurements
            rospy.loginfo(measurements_msg)
            self.publisher.publish(measurements_msg)
            self.rate.sleep

if __name__ == '__main__':
    try:
        pub = MeasurementPublisher()
        pub.publish()
    except rospy.ROSInterruptException:
        pass