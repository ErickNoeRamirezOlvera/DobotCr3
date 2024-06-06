#!/usr/bin/env python3

import rospy
import socket

def enable_robot():
    rospy.init_node('enable_robot', anonymous=True)
    ip = '192.168.0.101'  #The robot's IP address
    port = 30003 #Port number for command communication 29999 30003

    try:
        # Establish a connection to the robot
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((ip, port))
            rospy.loginfo("Connected to robot at {}:{}".format(ip, port))
            
            #Send the EnableRobot command
            command = "YourCommandHere()\n"  #Ensure the command ends with a newline character
            sock.sendall(command.encode('utf-8'))
            rospy.loginfo("Sent command: {}".format(command.strip()))
            
            #Wait for a response from the robot
            response = sock.recv(1024).decode('utf-8')
            rospy.loginfo("Received response: {}".format(response))
    except Exception as e:
        rospy.logerr("Failed to send command: {}".format(e))

if __name__ == '__main__':
    try:
        enable_robot()
    except rospy.ROSInterruptException:
        pass
