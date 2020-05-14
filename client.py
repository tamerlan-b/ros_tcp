#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import socket, threading, json
from messages import MsgType, MsgDataType, MsgPacker

# Flag that indicates if we received request for coordinates
request_flag = False
# Lock to have an access to flag from different threads
lock = threading.Lock()

# Robot's marker id
robot_id = 4

# Send comment message to server
def sendComment(sock, server, comment):
    global robot_id
    msg = {'type':MsgType.SEND, 'data_type': MsgDataType.COMMENT, 'comment': comment}
    msg['id'] = robot_id
    msg_json = MsgPacker.pack(msg)
    sock.sendto(msg_json, server)

# Create socket and connect to server
def createSocket(port=0, nonblocking=False):
    # Get address
    host = socket.gethostbyname(socket.gethostname())
    # Connect to server
    s = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.bind((host,port))
    if nonblocking:
        s.setblocking(0)
    return s

# Get request from robot and send it to server
def coordinatesRequestCb(data):
    global request_flag
    if data.data == '1':
        # Create request message for server
        msg = {'type':MsgType.GET, 'data_type': MsgDataType.POS}
        msg_json = MsgPacker.pack(msg)
        with lock:
            request_flag = True
        # Send request
        s.sendto(msg_json, server)

# Receive messages from server
def receiveMsg(sock, pub, verbose=True):
    global lock, request_flag
    # Get data from server
    data, addr = sock.recvfrom(1024)
    # If we've been waiting for coordinates
    if request_flag:
        pub.publish(data.decode("utf-8"))
        if verbose:
            print(data.decode("utf-8"))
        with lock:
            request_flag = False

# Get server port and IP-address
def getServerAddress():
    s_ip = "192.168.1.56"
    s_port = 9090
    if rospy.has_param("client/server_ip"):
        s_ip = rospy.get_param("client/server_ip")
        s_port = rospy.get_param("client/server_port")
    server = (s_ip,s_port)
    return server

def noneBlockingClient():
    # # Non-blocking code
    # Don't forget to add for socket: s.setblocking(0)
    # while True:
    #     try:
    #         data, addr = s.recvfrom(1024)
    #         if request_flag:
    #             pub.publish(data.decode("utf-8"))
    #             print(data.decode("utf-8"))
    #             with lock:
    #                 request_flag = False
    #             rate.sleep()
    #     except KeyboardInterrupt:
    #         raise Exception('Shutdown the client')
    #         break
    #     except:
    #         pass
    pass

if __name__ == '__main__':
    try:
        print('Client connected')
        s = createSocket()
        # Get address of the server
        server = getServerAddress()
        # Init ros subsciber, pulisher and node
        rospy.Subscriber("coordinates_request", String, coordinatesRequestCb)
        pub = rospy.Publisher('coordinates_response', String, queue_size=10)
        rospy.init_node('client', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        # Join chat message
        sendComment(s, server, 'I joined the chat')

        while not rospy.is_shutdown():
            try:
                receiveMsg(s, pub)
                rate.sleep()
            except:
                # Leave chat message
                sendComment(s, server, 'I left chat!')
                print('Client disconnected')
                break
        s.close()
    except rospy.ROSInterruptException:
        pass
