#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from smbus import SMBus
import struct
import math


class IMUPublisher(object):
    def __init__(self):
        # --- Parameters ---
        self.I2C_BUS = rospy.get_param('~i2c_bus', 7)
        self.ADDRESS = rospy.get_param('~address', 0x6B)
        self.frame_id = rospy.get_param('~frame_id', 'imu_link')
        self.rate_hz = rospy.get_param('~rate', 800.0)  

        # --- ROS Publisher ---
        self.pub = rospy.Publisher('imu/data', Imu, queue_size=20)
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("IMU publisher started at %.1f Hz" % self.rate_hz)

    def spin(self):
        """Main loop."""
        while not rospy.is_shutdown():
            try:

                # Fill message
                msg = Imu()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id

                msg.linear_acceleration.x = 0.0
                msg.linear_acceleration.y = 0.0
                msg.linear_acceleration.z = 9.81

                msg.angular_velocity.x = 0.0
                msg.angular_velocity.y = 0.0
                msg.angular_velocity.z = 0.0

                # Publish
                self.pub.publish(msg)

                self.rate.sleep()

            except Exception as e:
                rospy.logerr_throttle(5.0, "Unexpected error: %s" % e)
                self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('imu_publisher', anonymous=True)
    try:
        node = IMUPublisher()
        node.spin()
    except rospy.ROSInterruptException:
        pass

