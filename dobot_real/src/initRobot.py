#!/usr/bin/env python3

import rospy
import socket
import re 
import time
from std_msgs.msg import Bool

robot_ip = '192.168.0.101'  #The robot's IP address
port = 29999 #Dashboard port, only for simple commands (DO NOT CHANGE)

def send_command(sock, command, extract=False):
    """Send a command to the robot and wait for the response."""
    rospy.loginfo("Sending command: {}".format(command.strip()))
    sock.sendall(command.encode('utf-8'))
    
    # Wait for a response from the robot
    response = sock.recv(1024).decode('utf-8')
    rospy.loginfo("Received response: {}".format(response))
    
    # Extract the number inside curly braces
    match = re.search(r'\{(\d+)\}', response)

    if extract:
        if match:
            number = int(match.group(1))  # Convert the matched string to an integer
            return number
        
        else:
            return None

def main():
    rospy.init_node('enable_robot', anonymous=True)

    enabled_pub = rospy.Publisher('/isEnabled', Bool, queue_size=10)

    try:
        #Establish a connection to the robot
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((robot_ip, port))
            rospy.loginfo("Connected to robot at {}:{}".format(robot_ip, port))

            #Send the RobotMode command
            robot_mode_number = send_command(sock, "RobotMode() \n", extract=True)
            
            #Print the extracted number
            if robot_mode_number is not None:

                #Check robot mode to take action
                if robot_mode_number == 4:  #Disabled
                    #Send the EnableRobot command
                    send_command(sock, "EnableRobot() \n")

                    time.sleep(5) #Wait for command to be executed

                    #Send init gripper command
                    send_command(sock, "runScript(t3003bInitGripper) \n", extract=True)

                    time.sleep(8) #Wait for command to be executed

                    #Reset robot
                    send_command(sock, "resetRobot() \n", extract=True)

                    time.sleep(1.5) #Wait for command to be executed

                    #Set tool TCP
                    send_command(sock, "Tool(6) \n", extract=True)
                    rospy.loginfo("Tool TCP set")

                    time.sleep(1) #Wait for command to be executed

                    isEnabled = True
                    enabled_pub.publish(isEnabled)
                    print("Stopping node")
                else:
                    rospy.logwarn("Robot not in correct mode to enable.")

            else:
                rospy.logwarn("Failed to extract mode number from response.")

    except Exception as e:
        rospy.logerr("Failed to send command: {}".format(e))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
