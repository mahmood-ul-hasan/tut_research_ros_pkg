#!/usr/bin/env python3

# Task
# Develop a control program using ROS that connects to the TurtleBot3 Gazebo simulation environment 
# and navigates the TurtleBot3 continuously in the standard world without colliding with obstacles using randomly generated goals.

# Approach (Random Goal Navigation)
# The TurtleBot3 navigates to randomly generated pose goals.
# After reaching a goal, a new random goal is generated, and the TurtleBot3 starts moving towards it.
# This process repeats continuously, ensuring the robot keeps moving around the environment without stopping.




from actionlib_msgs.msg import *  # Import all messages from the actionlib_msgs package.
from geometry_msgs.msg import Pose, Point, Quaternion  # Import Pose, Point, and Quaternion message types.
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # Import MoveBase action and goal messages.
import actionlib  # Import the actionlib library for action clients.
import random  # Import the random library to generate random numbers.
import rospy  # Import the rospy library for ROS (Robot Operating System) Python client.

# Define a class for sending random navigation goals to the TurtleBot using move_base.
class keep_MoveBase_moving():

# Method: __init__()
# This is the initialization method for the keep_MoveBase_moving class. 
# It initializes the ROS node, sets up the move_base action client, and waits for the server to become available. 
# If the server is not accessible, it logs an error and shuts down the node. 
# This method also registers a shutdown hook to ensure proper cleanup.

    def __init__(self):
        # Initialize the ROS node with the name 'move_bot_by_random_goal'.
        rospy.init_node('move_bot_by_random_goal')
        
        # Initialize a flag to track if a goal has been sent.
        self.goal_sent = False
        
        # Register a shutdown hook to be called when the node is shutting down.
        rospy.on_shutdown(self.shutdown)
        
        # Create an action client for the 'move_base' action server.
        self.MoveBase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        
        # Wait for the move_base action server to be ready. Timeout after 5 seconds.
        wait_for_action_server = self.MoveBase_client.wait_for_server()
        if not wait_for_action_server:
            # Log an error and shut down the node if the server is not accessible.
            rospy.logerr("There is no move_base action server accessible!")
            rospy.signal_shutdown("There is no move_base action server accessible!")
            return
        rospy.loginfo("move_base server is now connected")
        rospy.loginfo("Begin achieving goals.")


# Method: go_to_target()
# This method sends a navigation goal to the robot using the move_base action server. 
# It sets the specified position and orientation for the goal and then sends it to the server. 
# The method waits for up to 60 seconds for the robot to reach the goal. 
# If the goal is not reached within this time, it is canceled. The method returns whether the goal was successfully achieved.
  
    def go_to_target(self, pos, quat):
        # Set the flag to indicate that a goal has been sent.
        self.goal_sent = True
        
        # Create a MoveBaseGoal message and populate its fields.
        goal = MoveBaseGoal()
        # Set the frame of reference to 'map', which is a global coordinate frame.
        goal.target_pose.header.frame_id = 'map'
        # Set the timestamp to the current time.
        goal.target_pose.header.stamp = rospy.Time.now()
        # Set the target position and orientation for the goal.
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Send the goal to the move_base action server.
        self.MoveBase_client.send_goal(goal)

        # Wait for the robot to reach the goal or until the timeout (60 seconds).
        success = self.MoveBase_client.wait_for_result(rospy.Duration(60))
        status = self.MoveBase_client.get_state()  # Get the current state of the goal.
        result = False

        # Check if the goal was achieved successfully.
        if success and status == GoalStatus.SUCCEEDED:
            result = True  # Goal reached successfully.
        else:
            # If the goal was not achieved, cancel the goal.
            self.MoveBase_client.cancel_goal()

        # Reset the goal_sent flag as the goal is no longer active.
        self.goal_sent = False
        return result  # Return whether the goal was successfully achieved.


# Method: shutdown()
# This method handles the shutdown procedure for the node. 
# It ensures that any active goals are canceled when the node is shutting down. 
# It logs the shutdown event and safely stops the robot. This method is automatically called when the node is terminated.
    def shutdown(self):
        # If a goal was sent and is still active, cancel it.
        if self.goal_sent:
            self.MoveBase_client.cancel_goal()
        # Log that the robot is stopping and shut down the move_base.
        rospy.loginfo("Stop the TurtleBot and shutdown the MoveBase")
        rospy.sleep(1)  # Allow time for log messages to be transmitted.

if __name__ == '__main__':
    try:
        # Continuously send random goals to the robot.
        while True:
            # Create an instance of the keep_MoveBase_moving class.
            TurtleBot_navigator = keep_MoveBase_moving()
        
            # Random Goal Generation and Execution:
            # this section generates random target positions within a specified range for the robot. 
            # The robot attempts to navigate to these randomly generated positions sequentially. 
            # If the robot successfully reaches a goal, it logs a success message. 
            # If the robot fails to reach the goal, it logs a failure message and sends a new random goal.


            # Generate random values for x and y coordinates within a specified range.
            xx = random.randrange(-18, 18) / 10
            yy = random.randrange(-18, 18) / 10
     
            # Adjust the x-coordinate to avoid setting goals too close to obstacles or invalid areas.
            if xx > -0.15 and xx < 0.15:
                if xx > 0: xx = xx + 0.15 
                else: xx = xx - 0.15 
            elif xx > -1.2 and xx < -0.8:
                if xx > -1: xx = xx + 0.15 
                else: xx = xx - 0.15 
            elif xx > 0.8 and xx < 1.2:
                if xx > 1: xx = xx + 0.15 
                else: xx = xx - 0.15 

            # Adjust the y-coordinate similarly to avoid invalid areas.
            if yy > -0.15 and yy < 0.15:
                if yy > 0: yy = yy + 0.15 
                else: yy = yy - 0.15 
            elif yy > -1.2 and yy < -0.8:
                if yy > -1: yy = yy + 0.15 
                else: yy = yy - 0.15 
            elif yy > 0.8 and yy < 1.2:
                if yy > 1: yy = yy + 0.15 
                else: yy = yy - 0.15 

            # Create a dictionary to store the target position.
            position = {'x': xx, 'y': yy}
            # Create a dictionary to store the target orientation (no rotation).
            quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': 0.000, 'r4': 1.000}

            # Log the target goal coordinates.
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            # Send the goal to the robot and check if it was successful.
            success = TurtleBot_navigator.go_to_target(position, quaternion)

            # Log the result of the goal.
            if success:
                rospy.loginfo("TurtleBot has achieved its goal! Congratulations")
            else:
                rospy.loginfo("TurtleBot failed to achieve its goal")

            # Sleep for 1 second to allow log messages to be transmitted.
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        # Log that the program is stopping due to a Ctrl-C (ROSInterruptException).
        rospy.loginfo("Stopping (Ctrl-C caught)")
