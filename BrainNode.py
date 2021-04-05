#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

import tensorflow as tf
from tensorflow.python.platform import gfile
import numpy as np
import time


# RL Brain Functions
# Obtain the learned model graph
def load_graph(frozen_graph_filename):
    # We load the protobuf file from the disk and parse it to retrieve the
    # unserialized graph_def
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Then, we import the graph_def into a new Graph and returns it
    with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
        tf.import_graph_def(graph_def, name="prefix")
    return graph

# Using the learned model graph, output correct action vectors based on state vector
# State vector is obtained from callbacks
# Action vector is a 4 x 1 vector with ESC pulse values
def get_ESC(state):
    graph = load_graph('./small_quad_sq_3-0.bytes')
    x = graph.get_tensor_by_name('prefix/vector_observation:0')
    y = graph.get_tensor_by_name('prefix/action:0')

    with tf.Session(graph=graph) as sess:
        # x_feed = np.random.rand(26)
        # x_feed = [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,-1,0,1,0,-1]
        x_feed = state + [0,1,0,-1,0,1,0,-1]
        y_out = sess.run(y, feed_dict={x: [x_feed]})

    y_ESC = []
    for y in y_out:
        y_ESC.append(np.clip(1500 + 800 * y,1000,2000))

    y_ESC = y_ESC[0:3]
    return y_ESC

# Collects data retrieved by the imu
def imu_callback(self, data):
    # "Store" message received.
    # Note that the 1s are just filler placeholders for actual data value
    stateVector = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, data.angular_velocity.x, data.angular_velocity.y,
                   data.angular_velocity.z, data.linear_acceleration.x, data.linear_acceleration.y,
                   data.linear_acceleration.y]
    rospy.loginfo(rospy.get_caller_id() + "\nState:\n [{}]".format(stateVector))


    # Calls brain function to get action vector of ESCs
    message = Float64MultiArray
    message.data = get_ESC(stateVector)
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
