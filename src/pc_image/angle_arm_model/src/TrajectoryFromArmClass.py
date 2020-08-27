
import rospy
from visualization_msgs.msg import Marker

class TrajectoryFromArm:
    def __init__(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.ns = 'trajectory'

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        self.marker = marker
        self.pub = rospy.Publisher('/visualization_marker', Marker, queue_size = 100)
        self.timeLastTrajectoryPub = 0
