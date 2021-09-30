#!/usr/bin/env python 
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
import roslib
import rospy
import math
import tf
import numpy as np



ROBOT_FRAME = 'base'
GOAL_THRES_POS = 0.2
GOAL_THRES_ANG = 0.2
FACE_GOAL_DIST = 1.0



def getYaw(quat):
    _, _, yaw = tf.transformations.euler_from_quaternion(quat)
    return yaw

def getConstrainedYaw(yaw):
    while yaw > math.pi:
        yaw -= 2*math.pi
    while yaw < -math.pi:
        yaw += 2*math.pi
    return yaw


def getAngleError(target, current):
    dyaw = target - current
    dyaw = getConstrainedYaw(dyaw)
    return dyaw

def getQuatFromYaw(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)



class PathFollower:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.pub_twist = rospy.Publisher('/path_planning_and_following/twist', TwistStamped, queue_size=1)
        self.pub_path = rospy.Publisher('/art_planner/followed_path', Path, queue_size=1, latch=True)
        self.sub = rospy.Subscriber('/art_planner/path', Path, self.pathCallback)
        self.current_pose = None
        self.goal_pose = None
        self.fixed_frame = None
        self.path = None
        self.path_ros = None
        self.gain_pid_pos = [2, 0.0, 0.0]
        self.gain_pid_ang = [5, 0.0, 0.0]
        self.i = [0, 0, 0]



    def pathCallback(self, path_msg):
        self.fixed_frame = path_msg.header.frame_id

        self.path = []
        self.path_ros = path_msg
        self.goal_pose = None

        if len(path_msg.poses) > 1:

            for ros_pose in path_msg.poses:
                pos = ros_pose.pose.position
                rot = ros_pose.pose.orientation
                yaw = getYaw([rot.x, rot.y, rot.z, rot.w])
                self.path.append([pos.x, pos.y, yaw])

            self.removePathNodesBeforeIndex(1)
            rospy.loginfo('Got path: ' + str(self.path))
            self.i = [0, 0, 0]    # Reset integrators.

        else:
            rospy.logwarn('Path message is too short')



    def updateCurrentPose(self):
        if self.fixed_frame is not None:
            try:
                (trans,rot) = self.listener.lookupTransform(self.fixed_frame, ROBOT_FRAME, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn_throttle('TF lookup of pose failed')
                return

            self.current_pose = [trans[0], trans[1], getYaw(rot)]
        else:
            rospy.logwarn_throttle(1, 'Fixed frame not set.')



    def publishPath(self):
        if self.fixed_frame is not None:
            msg = Path()
            msg.header.frame_id = self.fixed_frame
            if self.path_ros is not None:
                msg.poses = self.path_ros.poses

            self.pub_path.publish(msg)




    def removePathNodesBeforeIndex(self, index):
        self.path = self.path[index:]
        self.path_ros.poses = self.path_ros.poses[index:]



    def updateCurrentGoalPose(self):
        if self.goal_pose is not None:
            dx = self.goal_pose[0] - self.current_pose[0]
            dy = self.goal_pose[1] - self.current_pose[1]
            dist = (dx**2 + dy**2)**0.5
            dyaw = getAngleError(self.goal_pose[2], self.current_pose[2])
            if dist < GOAL_THRES_POS and abs(dyaw) < GOAL_THRES_ANG:
                if len(self.path) > 1:
                    self.removePathNodesBeforeIndex(1)
                else:
                    self.path = None
                    self.path_ros = None
                    self.publishPath()
                self.goal_pose = None

        # Only get new goal pose if we don't have one.
        if self.goal_pose is None:
            if self.current_pose is not None and self.path is not None:
                # Set goal to final path segment in case we have a weird path
                # and all checks fail.
                largest_valid_index = 0
                for i in range(len(self.path)-1):
                    path_segment = np.array([self.path[i+1][0] - self.path[i][0],
                                             self.path[i+1][1] - self.path[i][1]])
                    robot_from_node = np.array([self.current_pose[0] - self.path[i][0],
                                                self.current_pose[1] - self.path[i][1]])
                    dist_along_path = robot_from_node.dot(path_segment)
                    if (dist_along_path > 0):
                        # Robot is "in front of" the current node.
                        if i+1 > largest_valid_index:
                            largest_valid_index = i+1
                    else:
                        break

                self.removePathNodesBeforeIndex(largest_valid_index)
                self.goal_pose = self.path[0]
                self.publishPath()



    def getYawTarget(self):
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        dist = (dx**2 + dy**2)**0.5

        if dist < FACE_GOAL_DIST:
            return self.goal_pose[2]
        else:
            # Face towards goal.
            yaw_target = math.atan2(dy, dx)
            error = getAngleError(yaw_target, self.current_pose[2])
            if abs(error) > math.pi*0.5:
                # Face dat booty towards the goal.
                yaw_target = getConstrainedYaw(yaw_target + math.pi)
            return yaw_target



    def computeAndPublishTwist(self):
        self.updateCurrentPose()
        if self.path is not None and self.current_pose is not None:
            self.updateCurrentGoalPose()

            if self.goal_pose is None:
                return

            msg = TwistStamped()
            msg.header.frame_id = ROBOT_FRAME
            msg.header.stamp = rospy.Time.now()

            yaw = self.current_pose[2]
            yaw_target = self.getYawTarget()

            dx = self.goal_pose[0] - self.current_pose[0]
            dy = self.goal_pose[1] - self.current_pose[1]
            dyaw = getAngleError(yaw_target, self.current_pose[2])

            dlon = math.cos(yaw)*dx + math.sin(yaw)*dy
            dlat = -math.sin(yaw)*dx + math.cos(yaw)*dy

            # Update integrator.
            self.i[0] += dlon
            self.i[1] += dlat
            self.i[2] += dyaw

            lon_rate = dlon * self.gain_pid_pos[0] + self.i[0] * self.gain_pid_pos[1]
            lat_rate = dlat * self.gain_pid_pos[0] + self.i[1] * self.gain_pid_pos[1]
            yaw_rate = dyaw * self.gain_pid_ang[0] + self.i[2] * self.gain_pid_ang[1]

            msg.twist.linear.x = lon_rate
            msg.twist.linear.y = lat_rate
            msg.twist.angular.z = yaw_rate

            self.pub_twist.publish(msg)






if __name__ == '__main__':
    rospy.init_node('path_follower_pid')

    follower = PathFollower()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        follower.computeAndPublishTwist()
        rate.sleep()
        




#!/usr/bin/env python
# license removed for brevity
# import rospy
# from std_msgs.msg import String

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         talker()
#     except rospy.ROSInterruptException:
#         pass




#!/usr/bin/env python
# import rospy
# from std_msgs.msg import String

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
# def listener():

#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber("chatter", String, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()