#!/usr/bin/env python3
import rospy 
import tf
from aruco_msgs.msg import MarkerArray, Marker
from threading import Thread

class TfBroadcaster():
    def __init__(self):
        self.origin_name = "map"
        self.origin_marker_name = "marker_0"
        self.list_child_names = list()
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        rospy.Subscriber("/aruco_simple/markers", MarkerArray, self.tf_parser_cb)


    def tf_parser_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            if marker.id != 0 and "marker_" + str(marker.id) not in self.list_child_names:
                self.list_child_names.append("marker_" + str(marker.id))

            self.br.sendTransform((marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z), 
                                    (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w), 
                                    rospy.Time.now(), 
                                    "/marker_" + str(marker.id), 
                                    "/main_camera")
        if self.list_child_names:
            for marker_name in self.list_child_names:
                try:
                    (pos, rot) = self.listener.lookupTransform(marker_name, self.origin_marker_name, rospy.Time(0))
                    self.br.sendTransform(pos, rot, rospy.Time.now(), "transform_" + marker_name, self.origin_name)
                except:
                    pass
if __name__ == "__main__":
    rospy.init_node("tf_broadcaster_node")
    TfBroadcaster()
    rospy.spin()
