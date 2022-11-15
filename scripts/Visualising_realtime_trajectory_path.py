#! /usr/bin/env python3.8

import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TwistStamped, Pose, Point, Vector3, Quaternion
from std_msgs.msg import Header, ColorRGBA, String

class TrajectoryInteractiveMarkers:
    def __init__(self):
        self.count = 0 
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.listener = tf.TransformListener()
        rospy.sleep(2)

        self.marker = Marker()
        self.show_path_in_rviz()

    def show_path_in_rviz(self):
        rate = rospy.Rate(7)
        while not rospy.is_shutdown():
            (trans,rot)=self.listener.lookupTransform('/map','base_link',rospy.Time(0))
            self.marker.type = Marker.SPHERE
            self.marker.scale.x = 0.05
            self.marker.scale.y = 0.05
            self.marker.scale.z = 0.05
            self.marker.pose=Pose(Point(trans[0],trans[1],0), Quaternion(0, 0, 0, 1))
            self.marker.color=ColorRGBA(0.0, 2.0, 0.0, 0.8)
            self.marker.header.frame_id = 'map'
            self.count+=1
            self.marker.id = self.count
            self.marker_publisher.publish(self.marker)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
    trajectory_interactive_markers = TrajectoryInteractiveMarkers()
    rospy.sleep(0.5)
    rospy.spin()