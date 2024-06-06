#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float32, String, Bool
from nav_msgs.msg import Odometry
import socket
import time

#Counter variables
redCnt = 0
blueCnt = 0
greenCnt = 0

#State variables
currentState = ""
idle = "idle"
waitForMat = "waitForMat"
waitForObject = "waitForObject"

#Set flags
gripperExecuting = False
isEnabled = True
moveToMat = False
goToHome = True

#Define variables
actualPosition = None
formatted_coords = None
control_coords = None
control_matCoords = None
goDown_matCoords = None
prev_formatted_coords = None
red_matCoords = None
green_matCoords = None
blue_matCoords = None
home_position = "ServoJ(180.00,0.00,40.00,45.00,-90.00,0.00) \n"
clearTable_position = "ServoJ(180.00,0.00,0.00,90.00,-90.00,90.00) \n"

#Communication variables
robot_ip = '192.168.0.101'  #The robot's IP address
feedback_port = 30003

#Define callback functions
def isEnabled_callback(data): #Check if robot is enabled
    global isEnabled
    isEnabled = data.data
    rospy.loginfo(f"Robot is: {isEnabled}")

    if isEnabled:
        send_command(clearTable_position, feedback_port)

def rawObjectCoords_callback(data): #Fetch object coordinates
    global rawObjectCoords, formatted_coords, control_coords, goDown_coords, color, currentState, redCnt, blueCnt, greenCnt

    if currentState == waitForObject:
        #Check if data is not None and not empty
        if data.data and len(data.data.strip()) > 0:
            rawObjectCoords = data.data

            #Go back to home position
            send_command(home_position, feedback_port)

            time.sleep(1.5) #Wait for action to be completed
            
            #Format coords
            control_coords, formatted_coords, goDown_coords, color = formatCoords(rawObjectCoords)
            if formatted_coords and control_coords:
                rospy.loginfo(f"Formatted coords are: {formatted_coords}")
                rospy.loginfo(f"Control coords are: {control_coords}")
                rospy.loginfo(f"Color is: {color}")

                #Update color counter
                if color == "red":
                    redCnt += 1
                elif color == "blue":
                    blueCnt += 1
                elif color == "green":
                    greenCnt += 1
                
                #Change state to idle only if coords were formatted successfully
                currentState = idle
            else:
                rospy.logwarn("Formatting failed, skipping movement.")
        else:
            rospy.logwarn("No valid object coordinates received, remaining in current state.")
    else:
        rospy.loginfo_once("Callback received data while not in waitForObject state, ignoring.")

def rawMatCoords_callback(data): #Fetch mat coordinates rawMatCoords
    global currentState, rawMatCoords, matColor1, matColor2, matColor3,red_matCoords, blue_matCoords, green_matCoords,red_controlCoords, blue_controlCoords, green_controlCoords,red_goDownCoords, blue_goDownCoords, green_goDownCoords
    
    if currentState == waitForMat:
        #Check if data is not None and not empty
        if data.data and len(data.data.strip()) > 0:
            rawMatCoords = data.data
            
            #Format mat coords
            matColor1, matColor2, matColor3,red_matCoords, blue_matCoords, green_matCoords,red_controlCoords, blue_controlCoords, green_controlCoords,red_goDownCoords, blue_goDownCoords, green_goDownCoords = formatMatCoords(rawMatCoords)

            #Print the data
            rospy.loginfo(f"Red mat coords are: {red_matCoords}")
            rospy.loginfo(f"Blue mat coords are: {blue_matCoords}")
            rospy.loginfo(f"Green mat coords are: {green_matCoords}")

            #Change state
            currentState = idle
        else:
            rospy.logwarn("No valid mat coordinates received, remaining in current state.")
    else:
        rospy.loginfo_once("Callback received data while not in waitForMat state, ignoring.")

def gripperExecuting_callback(data): #Check if the gripper is executing
    global gripperExecuting
    gripperExecuting = data.data
    rospy.loginfo(f"Gripper is executing: {gripperExecuting}")

def actualPosition_callback(data): #Fetch current position
    global actualPosition
    actualPosition = data.data
    rospy.loginfo(f"Actual robot position is: {actualPosition}")

#Format coords so that robot controller can take them in
def formatCoords(rawObjectCoords):
    global z_float

    try:
        #Separate the color from the string
        rawObjectCoords = rawObjectCoords.strip()
        color_end_indx = rawObjectCoords.find("(")
        color = rawObjectCoords[:color_end_indx]
        coords_str = rawObjectCoords[color_end_indx+1:] #Remove color and parenthesis

        #Split the values by commas
        coords = coords_str.split(",")
        x = "{:.2f}".format(float(coords[0]))
        y = "{:.2f}".format(float(coords[1]))
        z = "{:.2f}".format(float(coords[2]))
        xz = "{:.2f}".format(float(coords[3]))
        
        #Cast z as an integer
        z_float = float(z)
        z_goDown = z_float - 60.00

        #Format the message
        formatted_coords = f"MovJ({x},{y},{z},180.00,0.00,{xz}) \n"
        control_coords = f"{x},{y},{z},180.00,0.00,{xz}"
        goDown_coords = f"MovJ({x},{y},{z_goDown},180.00,0.00,{xz}) \n"
        return control_coords, formatted_coords, goDown_coords, color
    
    except Exception as e:
        rospy.logerr(f"Error formatting coordinates: {e}")
        return None, None

def formatMatCoords(rawObjectCoords):

    #Initialize variables for colors and coordinates
    matColor1, matColor2, matColor3 = None, None, None
    red_matCoords, blue_matCoords, green_matCoords = None, None, None
    red_controlCoords, blue_controlCoords, green_controlCoords = None, None, None
    red_goDownCoords, blue_goDownCoords, green_goDownCoords = None, None, None

    #Parse the string to create a list from the message
    entries = rawObjectCoords.strip("[]' ").split("', '") #Remove brackets and split by ', ' to get individual entries

    #Dictionary to hold coordinates for each color
    color_coords = {}

    #Extract color and coordinates from each item
    for item in entries:
        parts = item.split('(') #Split the incomming message at the parenthesis
        if len(parts) == 2: #Parts should be split into two, left color and right coords
            color, coords_part = parts
            coords = coords_part.rstrip(',').split(',') #Strip the last character of the string and extract x and y
            if len(coords) >= 2: #Coords should contain x and y
                x = "{:.2f}".format(float(coords[0]))
                y = "{:.2f}".format(float(coords[1]))

                z_goDown = 70.00 if z_float >= 100.00 else 40.00

                formatted_coords = f"MovJ({x},{y},130.00,180.00,0.00,0.00)" #Format the coords
                control_coords = f"{x},{y},130.00,180.00,0.00,0.00"
                goDown_coords = f"MovJ({x},{y},{z_goDown},180.00,0.00,0.00)"
                color_coords[color] = {
                    'formatted_coords': formatted_coords,
                    'control_coords': control_coords,
                    'goDown_coords': goDown_coords
                } #Store in a dictionary with color as key

    #Assign to specific variables based on color
    if 'red' in color_coords:
        red_matCoords = color_coords['red']['formatted_coords']
        red_controlCoords = color_coords['red']['control_coords']
        red_goDownCoords = color_coords['red']['goDown_coords']
        matColor1 = 'red'
    if 'blue' in color_coords:
        blue_matCoords = color_coords['blue']['formatted_coords']
        blue_controlCoords = color_coords['blue']['control_coords']
        blue_goDownCoords = color_coords['blue']['goDown_coords']
        if matColor1 is None:
            matColor1 = 'blue'
        else:
            matColor2 = 'blue'
    if 'green' in color_coords:
        green_matCoords = color_coords['green']['formatted_coords']
        green_controlCoords = color_coords['green']['control_coords']
        green_goDownCoords = color_coords['green']['goDown_coords']
        if matColor1 is None:
            matColor1 = 'green'
        elif matColor2 is None:
            matColor2 = 'green'
        else:
            matColor3 = 'green'

    return (
        matColor1, matColor2, matColor3,
        red_matCoords, blue_matCoords, green_matCoords,
        red_controlCoords, blue_controlCoords, green_controlCoords,
        red_goDownCoords, blue_goDownCoords, green_goDownCoords
    )

def handleMatSequence(control):
    global pub_startMonitoring, moveToMat, currentState, red_matCoords, green_matCoords, blue_matCoords, goToHome

    mat_coords = None
    control_coords = None
    goDown_coords = None

    #Check if the matching mat is available
    while not rospy.is_shutdown():
        if color == 'red' and red_matCoords:
            mat_coords = red_matCoords
            control_coords = red_controlCoords
            goDown_coords = red_goDownCoords
            break
        elif color == 'blue' and blue_matCoords:
            mat_coords = blue_matCoords
            control_coords = blue_controlCoords
            goDown_coords = blue_goDownCoords
            break
        elif color == 'green' and green_matCoords:
            mat_coords = green_matCoords
            control_coords = green_controlCoords
            goDown_coords = green_goDownCoords
            break
        rospy.loginfo(f"Waiting for {color} mat coordinates...")
        currentState = waitForMat
        rospy.sleep(1)  #Wait before checking again

    if mat_coords:

        if goToHome:
            #Go back to starting pose
            send_command(home_position, feedback_port)

            time.sleep(2) #Wait for movement to be completed

            #Reset Flag
            goToHome = False
            
        #Move the robot to desired mat
        send_command(mat_coords, feedback_port)
        rospy.loginfo(f"Robot moving to {color} mat...")
        pub_startMonitoring.publish(True) #Start monitoring robot position
        
        time.sleep(1) #Wait for monitoring to start

        if control_coords == actualPosition:
            pub_startMonitoring.publish(False)
            rospy.loginfo(f"Robot reached {color} mat successfully!")
            time.sleep(1)

            #Move robot down to release object
            send_command(goDown_coords, feedback_port)
            rospy.loginfo(f"Robot moving down to release object...")
            time.sleep(2)

            #Ask gripper to open
            pub_action.publish("open")
            time.sleep(1.5)

            #Wait for the gripper to stop executing
            while gripperExecuting:
                rospy.loginfo_once("Gripper is executing...")
                time.sleep(0.2)

            #Move the robot up after releasing object
            send_command(mat_coords, feedback_port)
            rospy.loginfo("Robot going up after releasing object")
            time.sleep(2)  # Wait for action to be completed

            #Go back to starting pose
            send_command(home_position, feedback_port)
            moveToMat = False  #Set GoToMat to false

            time.sleep(2) #Wait for action to be completed

            #Clear table to see objects underneath robot
            send_command(clearTable_position, feedback_port)

            time.sleep(3) #Wait for action to be completed

            #Change state
            currentState = waitForObject

            #Set flag to true
            goToHome = True


#Send copmmands to the robot
def send_command(command, port):
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

#Main loop
def main_loop():
    global prev_formatted_coords, control_coords, moveToMat, pub_startMonitoring, pub_action, currentState

    #Initialize rospy and set the node
    rospy.init_node("moveRobot")

    #Set subscribers
    rospy.Subscriber("/isEnabled", Bool, isEnabled_callback)
    rospy.Subscriber("/gripperExecuting", Bool, gripperExecuting_callback)
    rospy.Subscriber("/objectCoords", String, rawObjectCoords_callback)
    rospy.Subscriber("/matCoords", String, rawMatCoords_callback)
    rospy.Subscriber('/currentPosition', String, actualPosition_callback)

    pub_action = rospy.Publisher('/gripper_action', String, queue_size=10)
    pub_startMonitoring = rospy.Publisher('/startMonitoring', Bool, queue_size=10)
    
    rospy.loginfo("The move robot node is running")

    #Change state
    currentState = waitForObject

    time.sleep(0.5) #Wait for coordinates to arrive

    while not rospy.is_shutdown():

        if formatted_coords != prev_formatted_coords and formatted_coords is not None and isEnabled:
            #Send command to move robot to object location
            send_command(formatted_coords, feedback_port)
            rospy.loginfo("Moving robot to pick object")

            time.sleep(1.5) #Wait for robot to start moving

            pub_startMonitoring.publish(True) #Start monitoring robot position

            '''print("Actual" + str(actualPosition))
            print("Control" + str(control_coords))'''

        if actualPosition == control_coords and control_coords != None:
            pub_startMonitoring.publish(False) #Stop monitoring robot position
            rospy.loginfo("Robot is at object")

            #Ask gripper to open
            pub_action.publish("open")

            time.sleep(2) #Wait for response

            #Wait for the gripper to stop executing
            while gripperExecuting:
                rospy.loginfo_once("Gripper is executing...")
                time.sleep(0.2)

            #Go down to grab object
            send_command(goDown_coords, feedback_port)
            rospy.loginfo("Robot going down to grab object")

            time.sleep(2) #Wait for action to be executed

            #Ask gripper to close
            pub_action.publish("close")

            time.sleep(1.5) #Wait for response

            #Wait for the gripper to stop executing
            while gripperExecuting:
                rospy.loginfo_once("Gripper is executing...")
                time.sleep(0.2)

            #Go up to previous coords
            send_command(formatted_coords, feedback_port)
            rospy.loginfo("Robot going up with object")

            time.sleep(2) #Wait for action to be completed

            control_coords = None #Set control coords to None to break condition

            #Go back to starting pose
            send_command(home_position, feedback_port)

            time.sleep(2) #Wait for movement to be completed

            #Clear table to see objects underneath robot
            send_command(clearTable_position, feedback_port)

            #Update previous action
            prev_formatted_coords = formatted_coords

            #GoToMat
            moveToMat = True
            
            #Change state
            currentState = waitForMat

            time.sleep(3) #Wait for action to be completed

        #Move robot to mat after picking up object
        if moveToMat and color:
            handleMatSequence(color)

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass