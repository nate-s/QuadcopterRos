#!/usr/bin/env python

import rospy
from sensor_msgs.img import Imu
from std_msgs.msg import Float64MultiArray

class Server:
    def __init__(self):

        self.accelerationx = None
        self.accelerationy = None
        self.accelerationz = None
        self.statevector = []

    def imu_callback(self, data):
        # "Store" message received.

        self.accelerationx = data.linear_acceleration.x
        self.accelerationy = data.linear_acceleration.y
        self.accelerationz = data.linear_acceleration.z

        self.statevector = [self.accelerationx, self.accelerationy, self.accelerationz]
        rospy.loginfo(rospy.get_caller_id() + "\nState Vector:\n [{}]".format(self.statevector))

    def publishedActionVector(self):
        return self.statevector


pub = rospy.Publisher('ActionMessage', Float64MultiArray, queue_size=10)

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/mavros/imu/data', Imu, server.imu_callback)

    # Gets the action vector and publishes it as a message

    while not rospy.is_shutdown():
        publishedActionMsg = server.publishedActionVector()
        pub.publish(publishedActionMsg)
        rospy.sleep(1)  # sleep for one second


if __name__ == '__main__':

    rospy.init_node('listener')
    server = Server()

    listener()
