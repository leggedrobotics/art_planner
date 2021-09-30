#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
import art_planner_msgs.msg



class PathToGoalClient:

    def goalCallback(self, msg):
        goal = art_planner_msgs.msg.PlanToGoalGoal()
        goal.goal = msg

        self.client.send_goal(goal)

    def __init__(self):
        self.sub = rospy.Subscriber('/goal', PoseStamped, self.goalCallback)

        self.client = actionlib.SimpleActionClient('/art_planner/plan_to_goal', art_planner_msgs.msg.PlanToGoalAction)

        print('Waiting for plan_to_goal server to appear...')
        self.client.wait_for_server()
        print('Found server.')



if __name__ == '__main__':
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('path_to_goal_client')
    client = PathToGoalClient()
    rospy.spin()
