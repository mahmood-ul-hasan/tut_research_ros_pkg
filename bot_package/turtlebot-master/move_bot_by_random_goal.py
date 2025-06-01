#!/usr/bin/env python3

from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import random
import rospy

class keep_MoveBase_moving():
    def __init__(self):

        rospy.init_node('move_turtlebot3')
        self.goal_sent = False
        # To shut down 
        rospy.on_shutdown(self.shutdown)
        
        # Create action client
        self.MoveBase_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        # wait for the action server to come up
        wait_for_action_server = self.MoveBase_client.wait_for_server()
        if not wait_for_action_server:
            rospy.logerr("There is no action server accessible!")
            rospy.signal_shutdown("There is no action server accessible!")
            return
        rospy.loginfo("move_base server is now connected")
        rospy.loginfo("Begin achieving goals.")

    def go_to_target(self, pos, quat):
        # Sending goal pose
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                    Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Start moving
        self.MoveBase_client.send_goal(goal)

        # Wait for TurtleBot to finish the work.
        success = self.MoveBase_client.wait_for_result(rospy.Duration(60)) 
        status = self.MoveBase_client.get_state()
        result = False

        if success and status == GoalStatus.SUCCEEDED:
            result = True
        else:
            self.MoveBase_client.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.MoveBase_client.cancel_goal()
        rospy.loginfo("Stop the TurtleBot and shutdown the MoveBase")
        rospy.sleep(1)
    
        

if __name__ == '__main__':

    try:
        while True:

            TurtleBot_navigator = keep_MoveBase_moving()
            # get random values of x and y location for goal
            xx = random.randrange(-18, 18)/10
            yy = random.randrange(-18, 18)/10

            if xx > -0.15 and xx < 0.15:
                if xx > 0: xx = xx + 0.15; 
                else: xx = xx - 0.15; 
            elif xx > -1.2 and xx < -0.8:
                if xx > -1: xx = xx + 0.15; 
                else: xx = xx - 0.15; 
            elif xx > 0.8 and xx < 1.2:
                if xx > 1: xx = xx + 0.15; 
                else: xx = xx - 0.15; 

            if yy > -0.15 and yy < 0.15:
                if yy > 0: yy = yy + 0.15; 
                else: yy = yy - 0.15; 
            elif yy > -1.2 and yy < -0.8:
                if yy > -1: yy = yy + 0.15; 
                else: yy = yy - 0.15; 
            elif yy > 0.8 and yy < 1.2:
                if yy > 1: yy = yy + 0.15; 
                else: yy = yy - 0.15; 

            position = {'x': xx, 'y' : yy}
            quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = TurtleBot_navigator.go_to_target(position, quaternion)

            if success:
                rospy.loginfo("TurtleBot has achieved its goal! Congratulations")
            else:
                rospy.loginfo("TurtleBot failed to achieve its goal")

            # Sleep so that the final log messages can be transmitted.
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Stopping (Ctrl-C caught)")
