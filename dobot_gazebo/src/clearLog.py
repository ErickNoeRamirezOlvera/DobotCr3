#!/usr/bin/env python3
import rospy
import os

def clear_log():
    log_file = str(os.path.dirname(__file__)) + "/trash.log"
    open(log_file, 'w').close()

def main():

    # Initialize rospy and set the node name 
    rospy.init_node('clearLog')

    # Set the rate of the main loop
    rate = rospy.Rate(1)
	
    rospy.loginfo("Log cleaning node is running")	
    while not rospy.is_shutdown():
        clear_log()
        # Sleep for the remaining time to maintain the desired rate
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
