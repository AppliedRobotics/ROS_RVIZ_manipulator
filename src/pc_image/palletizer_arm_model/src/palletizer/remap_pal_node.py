#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

pubJointState = rospy.Publisher("/palletizer_robot/joint_states",JointState,queue_size=10)

nameList = ['pal_joint0', 'pal_joint1', 'pal_joint2', 'pal_joint3', 'prisos_joint', 'pal_joint4', 'pal_joint10', 'pal_joint5', 'pal_joint6', 'pal_joint7', 'pal_joint8']

jointList = [0,0,0,0,0,0,0,0,0,0,0]

def main_remap(jointState):
    newMsg = JointState()
    newMsg.name = nameList
    for i in range(5):
        jointList[i]=float(jointState.position[i])
    jointList[5] = jointList[2]
    jointList[6] = -jointList[1]
    jointList[7] = jointList[1]
    jointList[8] = -jointList[2]
    jointList[9] = -jointState.position[2]-jointState.position[1]
    jointList[10] = (jointState.position[2]+jointState.position[1])
    newMsg.position = jointList
    newMsg.header.stamp = rospy.Time.now()
    pubJointState.publish(newMsg)

if __name__=="__main__":
    rospy.init_node('remap_joint_states')
    rospy.Subscriber("/palletizer_robot/joint_states_remap", JointState, main_remap)
    rospy.spin()
