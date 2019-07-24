#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

# Collects data retrieved by the imu
def imu_callback(self, data):
    # "Store" message received.
    stateVector = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.y]
    rospy.loginfo(rospy.get_caller_id() + "\nState:\n [{}]".format(stateVector))


    message = Float64MultiArray
    message.data = stateVector
    rospy.loginfo(rospy.get_caller_id() + "\nAction:\n [{}]".format(message))
    pub.publish(message)

# Creates a topic named Action that will contain an array of 64-bit Float type values
pub = rospy.Publisher('ActionMessage', Float64MultiArray, queue_size=10)


# listener function will subscribe to all the required topics.
# Additionally, since this node will also publish simultaneously,
# it will publish a message that contains the Action vector
def listener():
    rospy.init_node('DataCollection', anonymous=True)
    rospy.Subscriber('/mavros/imu/data', Imu, imu_callback)
    rospy.spin()



if __name__ == '__main__':
    listener()
