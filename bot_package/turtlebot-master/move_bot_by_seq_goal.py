#!/usr/bin/env python3
# license removed for brevity
__author__ = 'fiorellasibona'
import rospy
import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


class MoveBaseSeq():

    def __init__(self):

        rospy.init_node('move_turtlebot3')
        # points_seq = rospy.get_param('move_base_seq/p_seq')
        points_seq = [0.3,0.5,0,2.2,0.5,0,1.5,-0.5,0, 0.5,1.3,0,1.5,-1.5,0,-1.5,1.5,0, 0.5,-0.5, 0]
        # Only yaw angle required (no ratotions around x and y axes) in deg:
        # yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
        yaweulerangles_seq = [90,0,180, 45,-45,90, 0]
        #List of goal quaternions:
        quat_seq = list()
        #List of goal poses:
        self.pose_seq = list()
        self.goal_cnt = 0
        for yawangle in yaweulerangles_seq:
            #Unpacking the quaternion tuple and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        rospy.loginfo(str(points))
        for point in points:
            #Exploit n variable to cycle in quat_seq
            self.pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        #Create action client
        self.MoveBase_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server")
        # wait for the action server to come up
        wait_for_action_server = self.MoveBase_client.wait_for_server()
        if not wait_for_action_server:
            rospy.logerr("There is no action server accessible!")
            rospy.signal_shutdown("There is no action server accessible!")
            return
        rospy.loginfo("move_base server is now connected")
        rospy.loginfo("Begin achieving goals.")
        self.movebase_client()

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
    # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        # if status == 2:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
                self.MoveBase_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb) 
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        # if status == 4:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
        #     rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
        #     return

        # if status == 5:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
        #     rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
        #     return

        # if status == 8:
        #     rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    #for pose in pose_seq:   
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.MoveBase_client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()

if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
