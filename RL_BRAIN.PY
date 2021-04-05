#!/usr/bin/env python

#Does not start the NN 
#Clearly self.velocities are not set up

import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64

import tensorflow as tf
from tensorflow.python.platform import gfile
import numpy as np
import time


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

def get_ESC(state):
	graph = load_graph('./small_quad_sq_3-0.bytes')
	x = graph.get_tensor_by_name('prefix/vector_observation:0')
	y = graph.get_tensor_by_name('prefix/action:0')

	with tf.Session(graph=graph) as sess:
	        #x_feed = np.random.rand(26)
	        #x_feed = [0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,-1,0,1,0,-1]
	        x_feed = state + [0,1,0,-1,0,1,0,-1]
	        y_out = sess.run(y, feed_dict={
	        	x: [x_feed]
	        })

    y_ESC = []
    for y in y_out:
        y_ESC.append(np.clip(1500 + 800 * y,1000,2000))

    y_ESC = y_ESC[0:3]
	return y_ESC


pub = rospy.Publisher('Action', Float64MultiArray, queue_size=10)

class Server:
    def __init__(self):

        self.altitude = None
        self.roll = None
        self.pitch = None
        self.yawn = None
    	self.velocity = None
    	self.velocity = None
    	self.velocity = None
    	self.angularvelocityx = None
    	self.angularvelocityy = None
    	self.angularvelocityz = None
    	self.acceleration = None
    	self.acceleration = None
    	self.acceleration = None
    	self.statevector = []


    def imu_callback(self, msg):
        # "Store" message received.
        self.roll = msg.angular_velocityx.x
        self.pitch = msg.angular_velocityy.y
        self.yawn = msg.angular_velocityz.z
    	self.velocityx = msg
    	self.velocityy = msg
    	self.velocityz = msg
    	self.angularvelocityx = msg
    	self.angularvelocityy = msg
    	self.angularvelocityz = msg
    	self.accelerationx = msg.linear_acceleration.x
    	self.accelerationy = msg.linear_acceleration.y
    	self.accelerationz = msg.linear_acceleration.z

    	self.statevector = [0,2,0,0,0,0,self.roll,self.pitch,self.yawn,self.velocityx,self.velocityy,self.velocityz,self.angularvelocityx,self.angularvelocityy,self.angularvelocityz,self.accelerationx,self.accelerationy,self.accelerationz,0,1,0,-1,0,1,0,-1]
    #maybe call run_NN

    def alt_callback(self, msg):
        # "Store" the message received.
        self.altitude = msg


if __name__ == '__main__':
    rospy.init_node('listener')

    server = Server()

    rospy.Subscriber('/mavros/imu/data', Imu, server.imu_callback)
    rospy.Subscriber('/mavros/global_position/rel_alt', Float64, server.alt_callback)
    rospy.Subscriber('/mavros/global_position/gp_vel', Float64, server.alt_callback)

    #Maybe while true
    #Maybe create new thread to run the NN infinitely and pass the server by reference 

    rospy.spin()