#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, String
import socket
import time

previous_action = None
action = None

robot_ip = '192.168.0.101'  #The robot's IP address
port = 29999 #Dashboard port, only for simple commands (DO NOT CHANGE)

#Define callback functions
def action_callback(data):
    global action
    action = data.data
    rospy.loginfo(f"Action updated to: {action}")

def send_command(command):
    try:
        #Establish a connection to the robot, send a command and close connection
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((robot_ip, port))
            rospy.loginfo("Connected to robot at {}:{}".format(robot_ip, port))
            
            #Send the command
            rospy.loginfo("Sending command: {}".format(command.strip()))
            sock.sendall(command.encode('utf-8'))

            #Wait for a response from the robot
            response = sock.recv(1024).decode('utf-8')
            rospy.loginfo("Received response: {}".format(response))

    except Exception as e:
        rospy.logerr("Failed to send command: {}".format(e))
        
def main_loop():
    global previous_action

    #Initialize rospy and set the node
    rospy.init_node("gripperControl")

    #Set subscribers
    rospy.Subscriber("/gripper_action", String, action_callback)

    gripperStatus_pub = rospy.Publisher('/gripperExecuting', Bool, queue_size=10)

    rospy.loginfo("The gripperControl node is running")

    while not rospy.is_shutdown():
        if action != previous_action and action is not None:
            if action == "open":
                #Send command
                gripperStatus_pub.publish(True)
                send_command("runScript(t3003bOpenGripper) \n")
                rospy.loginfo("Gripper opened")

                time.sleep(4) #Wait for command to be executed

                #Reset robot
                send_command("ResetRobot() \n")
                rospy.loginfo("Robot reseted")

                time.sleep(1.5) #Wait for command to be executed

                #Set tool TCP
                send_command("Tool(6) \n")
                rospy.loginfo("Tool TCP set")

                time.sleep(1) #Wait for command to be executed

            elif action == "close":
                #Send command
                gripperStatus_pub.publish(True)
                send_command("runScript(t3003bCloseGripper) \n")
                rospy.loginfo("Gripper closed")

                time.sleep(4) #Wait for command to be executed

                #Reset robot
                send_command("ResetRobot() \n")
                rospy.loginfo("Robot reseted")

                time.sleep(1.5) #Wait for command to be executed

                #Set tool TCP
                send_command("Tool(6) \n")
                rospy.loginfo("Tool TCP set")

                time.sleep(1) #Wait for command to be executed

            previous_action = action #Update previous action
            gripperStatus_pub.publish(False) #Publish that the gripper is done executing an action

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass