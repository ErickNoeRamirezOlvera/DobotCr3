#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from control_msgs.msg import GripperCommandActionGoal

#Define variables
action = "Close"

#Define callback functions
def action_callback(data): #For action string
    global action
    action = data.data
    rospy.loginfo(f"I received: {action}")

def main_loop():
    global action

    # Initialize rospy and set the node name 
    rospy.init_node('gripper_action')

    # Create subscribers and publishers
    rospy.Subscriber('action', String, action_callback)

    pub_action = rospy.Publisher('/gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=10)

    # Set the rate of the main loop
    rate = rospy.Rate(50)

    rospy.loginfo("The gripper action node is Running")

    while not rospy.is_shutdown():
        # Create the GripperCommandActionGoal message
        gripper_goal = GripperCommandActionGoal()

        # Set header
        gripper_goal.header.stamp = rospy.Time.now()
        gripper_goal.header.frame_id = "base_link"

        if action == "Open":
            # Set position and effort to open
            gripper_goal.goal.command.position = 0.0  # 0 cm
            gripper_goal.goal.command.max_effort = 1.0  # Minimum effort

            pub_action.publish(gripper_goal)

        elif action == "Close":
            # Set position and effort to close
            gripper_goal.goal.command.position = 0.9  # 0.8 = 5 cm
            gripper_goal.goal.command.max_effort = 5.0  # Maximum effort

            pub_action.publish(gripper_goal)
 
        rate.sleep()

        # Sleep for the remaining time to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main_loop()
    except rospy.ROSInterruptException:
        pass