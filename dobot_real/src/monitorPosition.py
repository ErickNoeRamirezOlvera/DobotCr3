#!/usr/bin/env python3

import rospy
import socket
import time
from std_msgs.msg import String, Bool

startMonitoring = False

#Define callback functions
def startMonitoring_callback(data): #Check the gripper status
    global startMonitoring
    startMonitoring = data.data
    rospy.loginfo_once(f"Monitoring is: {startMonitoring}")

#Send commands to the robot
def send_command():
    ip = '192.168.0.101'  #The robot's IP address
    port = 29999 #Dashboard port, only for simple commands (DO NOT CHANGE)

    try:
        # Establish a connection to the robot
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.connect((ip, port))
            rospy.loginfo("Connected to robot at {}:{}".format(ip, port))

            #Send the EnableRobot command
            command = "getPose()\n"  #Ensure the command ends with a newline character
            sock.sendall(command.encode('utf-8'))
            rospy.loginfo("Sent command: {}".format(command.strip()))
            
            #Wait for a response from the robot
            response = sock.recv(1024).decode('utf-8')
            rospy.loginfo("Received response: {}".format(response))

            #Extract values and round up to two decimals
            response = response.strip() #Strip any whitespaces
            start = response.find("{") +1
            end = response.find("}")

            if start > 0 and end > 0:
                pose_data = response[start:end]
                values = pose_data.split(",")
                if len(values) == 6:
                    values[2] = float(values[2]) - 200.00
                    values[3] = abs(float(values[3]))
                    formatted_pose = ",".join(format(0.00 if abs(float(v)) < 0.005 else float(v),
                                                     ".2f") for v in values)
                    formatted_pose = str(formatted_pose)
                    return formatted_pose
                
    except Exception as e:
        rospy.logerr("Failed to send command: {}".format(e))

#Main loop
def main_loop():

    #Initialize rospy and set the node
    rospy.init_node("monitorCurrentPosition")

    #Set subscribers
    rospy.Subscriber("/startMonitoring", Bool, startMonitoring_callback)

    pub_currPos = rospy.Publisher('/currentPosition', String, queue_size=10)
    
    rospy.loginfo("The current pose monitoring node is running")

    while not rospy.is_shutdown():

        if startMonitoring:
            coordinates = send_command()

            #Publish coordinates
            rospy.loginfo("Formatted Position: {}".format(coordinates))
            pub_currPos.publish(coordinates)

            #Wait
            time.sleep(2)
        
        else:
            pass


if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass