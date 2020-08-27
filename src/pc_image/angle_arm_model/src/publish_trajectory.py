#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import RoboticArmClass as RoboticArm
from math import fabs
from TrajectoryFromArmClass import TrajectoryFromArm

keyOfFirstMsg = True
lastJointState = JointState()
roboticArm = RoboticArm.RoboticArm()
trajectoryArm = TrajectoryFromArm()
def read_pose_callback(msg):
    global roboticArm, trajectoryArm, keyOfFirstMsg, lastJointState

    if (keyOfFirstMsg):
        lastJointState = msg
        trajectoryArm.timeLastTrajectoryPub = rospy.get_time()
        keyOfFirstMsg = False

    elif (fabs(msg.position[0]-lastJointState.position[0]) < 0.03 and
        fabs(msg.position[1]-lastJointState.position[1]) < 0.03 and
        fabs(msg.position[2]-lastJointState.position[2]) < 0.03 and
        fabs(msg.position[3]-lastJointState.position[3]) < 0.03 and
        fabs(msg.position[4]-lastJointState.position[4]) < 0.03):
        if(rospy.get_time()-trajectoryArm.timeLastTrajectoryPub > 1):
            trajectoryArm.marker.points = []

    else:
        lastJointState = msg
        pointEndEffector = roboticArm.DirectProblem(lastJointState.position[0],lastJointState.position[1],lastJointState.position[2],lastJointState.position[3],lastJointState.position[4])
        trajectoryArm.marker.points.append(Point(pointEndEffector[0]*0.01,pointEndEffector[1]*0.01,pointEndEffector[2]*0.01))
        trajectoryArm.pub.publish(trajectoryArm.marker)
        trajectoryArm.timeLastTrajectoryPub = rospy.get_time()


if __name__=='__main__':
    rospy.init_node('publish_marker')
    rospy.Subscriber('/angle_robot/joint_states', JointState, read_pose_callback)
    rospy.spin()
