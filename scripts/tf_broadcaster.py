#!/usr/bin/env python3
import rospy 
import tf
from aruco_msgs.msg import MarkerArray, Marker

class TfBroadcaster():
    def __init__(self):
        self.origin_name = "/map"               # название глобальной системы координат, относительно которой мы будем публиковать все фреймы
        self.origin_marker_name = "/marker_0"   # название фрейма маркера, который примем за начало координат для всех маркеров
        self.camera_name = "/main_camera"       # имя фрейма камеры
        self.list_child_names = list()          # список имен фреймов всех маркеров кроме id 0
        
        self.use_z = False                      # использовать ось Z?

        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.use_z_axis = False
        
        rospy.Subscriber("/aruco_simple/markers", MarkerArray, self.tf_parser_cb)


    def tf_parser_cb(self, msg: MarkerArray):
        for marker in msg.markers:
            # перебираем все найденые маркеры и заносим каждый не нулевой маркер в список
            if marker.id != 0 and "marker_" + str(marker.id) not in self.list_child_names:
                self.list_child_names.append("marker_" + str(marker.id))
            # публикуем фреймы всех маркеров относительно фрейма камеры в tf
            self.br.sendTransform((marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z), 
                                    (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w), 
                                    rospy.Time.now(), 
                                    "/marker_" + str(marker.id), 
                                    self.camera_name)
        if self.list_child_names:
            for marker_name in self.list_child_names:
                try:
                    # получаем координаты всех маркеров относительно маркера начала координат, задуманно, что это маркер с id 0
                    (pos, rot) = self.listener.lookupTransform(self.origin_marker_name, marker_name, rospy.Time(0))
                    if not self.use_z_axis:
                    # если не хотим учитывать ось Z просто зануляем ее значение
                        pos[-1] = 0.0
                    # публикуем трансформ всех маркеров относительно нового начала координат
                    self.br.sendTransform(pos, rot, rospy.Time.now(), "transform_" + marker_name, self.origin_name)
                except:
                    pass

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster_node")
    TfBroadcaster()
    rospy.spin()
