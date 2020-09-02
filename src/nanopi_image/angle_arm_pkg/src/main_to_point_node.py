#!/usr/bin/env python
import rospy  
import math
from RoboticArmClass import RoboticArm
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from inverse_problem_srv.srv import publish_cmd,publish_cmdResponse

rospy.wait_for_service('/angle_robot/cmd_joint_state_in_manip_coord')

nameList = ['ang_joint_1','ang_joint_2','ang_joint_3','ang_joint_4','ang_joint_5','gripper']
jointStateSrv = rospy.ServiceProxy('/angle_robot/cmd_joint_state_in_manip_coord', publish_cmd)
gripperPose = '0'

def ParseMsg(msg):
    try:
        coord_list = msg.point.split()
        x = float(coord_list[0])
        y = float(coord_list[1])
        z = float(coord_list[2])
        pith = float(coord_list[3])
        roll = float(coord_list[4])
        return x,y,z,pith,roll
    except ValueError:
        rospy.logerr('Input Error')

def MoveToPointCallback(msg):
    global lastGoalJointState
    x,y,z,pitch,roll = ParseMsg(msg)
    roboticArm = RoboticArm()
    availJointState,goalJointState = roboticArm.InversProblem(x,y,z,pitch,roll)

    if (not availJointState):
        rospy.loginfo('Point cannot be reached')
        return point_cmdResponse(False)
    else:
        rospy.loginfo('Wait...')
        goalJointState = [str(el) for el in goalJointState]
        lastGoalJointState = goalJointState
        strName = ' '.join(nameList)
        strJS = ' '.join(goalJointState) + ' ' + gripperPose
        strCmd = strName + ' ' + strJS
        jointStateSrv(strCmd)
        rospy.loginfo('Well Done!!!')
        return point_cmdResponse(True)

def GripperCmdCallback(msg):
    global gripperPose
    gripperPose = msg.point
    strName = ' '.join(nameList)
    strJS = ' '.join(lastGoalJointState) + ' ' + gripperPose
    strCmd = strName + ' ' + strJS
    jointStateSrv(strCmd)
    rospy.loginfo('Well Done!!!')
    return point_cmdResponse(True)


if __name__=='__main__':
    global lastGoalJointState
    rospy.init_node('angle_main_to_point_node')
    rospy.loginfo('Main node for move to point was started')
    strName = ' '.join(nameList)
    strJS = '0 0 0 0 0' + ' ' + gripperPose
    lastGoalJointState = '0 0 0 0 0'
    strCmd = strName + ' ' + strJS
    result = jointStateSrv(strCmd)
    rospy.Service('/angle_robot/cmd_point', point_cmd, MoveToPointCallback)
    rospy.Service('/angle_robot/gripper_cmd', point_cmd, GripperCmdCallback)
    rospy.spin()
