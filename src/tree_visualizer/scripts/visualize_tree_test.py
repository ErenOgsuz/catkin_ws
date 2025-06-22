#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import random
import math

class Node:
    def __init__(self, x, y, z=0.0, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.parent = parent

class TreeVisualizer:
    def __init__(self):
        rospy.init_node("tree_visualizer", anonymous=True)
        self.tree_pub = rospy.Publisher("tree_markers", MarkerArray, queue_size=1)
        self.nodes = []
        self.create_sample_tree()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.visualize_tree)

    def create_sample_tree(self):
        root = Node(0, 0, 0)
        self.nodes.append(root)
        for i in range(1, 20):
            angle = random.uniform(0, 2*math.pi)
            distance = random.uniform(0.1, 1.0)
            parent = random.choice(self.nodes)
            x = parent.x + distance * math.cos(angle)
            y = parent.y + distance * math.sin(angle)
            self.nodes.append(Node(x, y, 0.0, parent))

    def visualize_tree(self, event=None):
        if not self.nodes or rospy.is_shutdown():
            return
        marker_array = MarkerArray()
        marker_id = 0
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for node in self.nodes:
            if node.parent is not None:
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime = rospy.Duration(rospy.get_param('~marker_lifetime', 10.0))
                p1 = Point(node.parent.x, node.parent.y, node.parent.z)
                p2 = Point(node.x, node.y, node.z)
                marker.points = [p1, p2]
                marker_array.markers.append(marker)

        self.tree_pub.publish(marker_array)

if __name__ == "__main__":
    try:
        TreeVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
