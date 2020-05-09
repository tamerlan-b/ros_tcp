#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import threading
import socket
import json
import random
from messages import MsgType, MsgDataType, MsgPacker

# Return ip address of current machine
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]

# Function for creating socket
def createSocket(port=9090, ip='', verbose=True):
    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.bind((ip,port))
    if verbose:
        print('Server started at %s:%d'%(get_ip_address(), port))
    return s

# Run ROS in a loop
def rosLoop(rate, pub):
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


def callback(data):
    rospy.loginfo(data.data)


#########################################################
########## Global variables for PSO #####################
#########################################################

# Best point found by swarm
x_best, y_best, f_best = None, None, None

# Objective function - in real-world conditions
# robots should measure its value by themselves
def objectiveFunction(x,y):
	return x*x + y*y

# Global navigation (plan to do it with camera)
def getRobotCoordinates(robot_name):
	x = random.randint(0, 20)
	y = random.randint(0, 20)
	return x,y

#########################################################
#########################################################
#########################################################


def handleMsg(msg, addr):

	data = MsgPacker.unpack(msg)
	response = {}

	if data['type'] == MsgType.GET:
		if data['data_type'] == MsgDataType.POS:
			response['type'] = MsgType.SEND
			response['data_type'] = MsgDataType.POS
			x,y = getRobotCoordinates(addr)
			f = objectiveFunction(x,y)
			response['x'] = x
			response['y'] = y
			response['f'] = f
        elif data['data_type'] == MsgDataType.SWARM_BEST_POS:
            response['x'] = x_best
            response['y'] = y_best
            response['f'] = f_best
	elif data['type'] == MsgType.SEND:
		if data['data_type'] == MsgDataType.POS:
			# If we don't have best swarm point
			# or if our point is greater than the robot's one
			if f_best == None or f_best > data['f']:
				f_best = data['f']
				x_best = data['x']
				y_best = data['y']
		elif data['data_type'] == MsgDataType.COMMENT:
			print(addr, ': ', data['comment'])
	return response


if __name__ == '__main__':
    try:
        # Create socket
        s = createSocket()

        # List of clients
        clients = []

        # pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('server', anonymous=True)

        # Subscribe to image from camera
        rospy.Subscriber("chatter", String, callback)
        rate = rospy.Rate(10) # 10hz

        # Run ROS in a different thread
        # rT = threading.Thread(target = rosLoop, args=(rate, pub))
        # rT.start()

        while not rospy.is_shutdown():
            try:
                # Receive data
            	data, addr = s.recvfrom(1024)
            	# Append all addressers to client list
            	if addr not in clients:
            		clients.append(addr)
            	# Handle message and generate response
            	response = handleMsg(data, addr)
            	if response:
            		response_json = MsgPacker.pack(response)
            		# Send message back to addresser
            		s.sendto(response_json, addr)
                rate.sleep()
            except:
                print("\nServer stopped")
                break
        # Close socket
        s.close()

        # rT.join(1.0)

    except rospy.ROSInterruptException:
        pass
