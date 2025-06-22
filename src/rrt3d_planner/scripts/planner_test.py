#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, PointStamped #PoseStamped is for start end and positions
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2
#from nav_core import BaseGlobalPlanner
from octomap_msgs.msg import Octomap
from octomap import OcTree
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
#First but nice try insallah no error for k-dimensional tree
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R

class Node:
    __slots__ = ('x', 'y', 'z', 'yaw', 'parent', 'cost', 'children')

    def __init__(self, x, y, z=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.parent = None
        self.cost = 0.0
        self.children = []

    def get_position(self):
        return [self.x, self.y, self.z]

    def get_pose(self):
        return [self.x, self.y, self.z, self.yaw]

class InformedRRT3D():
    def __init__(self):
        super(InformedRRT3D, self).__init__()

        # Parameters with validation for the first time
        self.max_iter = rospy.get_param('~max_iterations', 5000)
        self.step_size = rospy.get_param('~step_size', 0.8)
        if self.step_size <= 0:
            rospy.logerr("step_size must be positive; setting to default 0.8")
            self.step_size = 0.8
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)
        if self.goal_tolerance <= 0:
            rospy.logerr("goal_tolerance must be positive; setting to default 0.5")
            self.goal_tolerance = 0.5
        self.rewire_radius = rospy.get_param('~rewire_radius', 1.5)
        if self.rewire_radius <= 0:
            rospy.logerr("rewire_radius must be positive; setting to default 1.5")
            self.rewire_radius = 1.5
        self.informed_sampling = rospy.get_param('~informed_sampling', True)
        self.path_smoothing = rospy.get_param('~path_smoothing', True)
        self.adaptive_radius = rospy.get_param('~adaptive_radius', True)
        self.max_altitude = rospy.get_param('~max_altitude', 5.0)
        self.min_altitude = rospy.get_param('~min_altitude', 0.2)
        if self.min_altitude >= self.max_altitude:
            rospy.logerr("min_altitude must be less than max_altitude; setting defaults")
            self.min_altitude = 0.2
            self.max_altitude = 5.0
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', 1.0)
        if self.max_yaw_rate <= 0:
            rospy.logerr("max_yaw_rate must be positive; setting to default 1.0")
            self.max_yaw_rate = 1.0
        self.yaw_weight = rospy.get_param('~yaw_weight', 0.3)
        self.consider_orientation = rospy.get_param('~consider_orientation', True)
        self.safety_radius = rospy.get_param('~safety_radius', 0.3)
        if self.safety_radius <= 0:
            rospy.logerr("safety_radius must be positive; setting to default 0.3")
            self.safety_radius = 0.3
        self.octomap_resolution = rospy.get_param('~octomap_resolution', 0.1)
        self.nominal_speed = rospy.get_param('~nominal_speed', 1.0)
        self.kd_tree_rebuild_interval = rospy.get_param('~kd_tree_rebuild_interval', 100)
        self.collision_check_resolution = rospy.get_param('~collision_check_resolution', 0.1)
        self.treat_unknown_as_free = rospy.get_param('~treat_unknown_as_free', True)
        self.cost_propagation_threshold = rospy.get_param('~cost_propagation_threshold', 1e-6)

        # DS to use in the program
        self.nodes = []
        self.node_positions = []
        self.kd_tree = None
        self.last_tree_build_size = 0
        self.costmap = None
        self.octree = None
        self.octomap_frame = "map"
        self.map_resolution = 0.05
        self.map_origin = None

        # Publishers and Subscribers
        self.tree_pub = rospy.Publisher('rrt_tree', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('rrt_path_3d', Path, queue_size=1, latch=True)
        self.octomap_sub = rospy.Subscriber('/rtabmap/octomap_binary', Octomap, self.octomap_callback)
        self.rtabmap_cloud_sub = rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, self.pointcloud_callback, queue_size=1)

        # TF setup
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Optimized 3D Informed RRT* Planner initialized")

    def octomap_callback(self, msg):
        try:
            self.octree = OcTree(msg.resolution)
            self.octree.readBinary(msg.data)
            self.octomap_resolution = msg.resolution
            self.octomap_frame = msg.header.frame_id
            rospy.loginfo("Octomap updated: %.2f resolution, frame: %s", msg.resolution, self.octomap_frame)
        except Exception as e:
            rospy.logwarn("Octomap processing failed: %s", str(e))

    def pointcloud_callback(self, msg):
        rospy.logdebug("Received point cloud with %d points", msg.width * msg.height)

    def initialize(self, name, costmap_ros):
        self.name = name
        self.costmap_ros = costmap_ros
        self.update_costmap()
        return True

    def update_costmap(self):
        self.costmap = self.costmap_ros.getCostmap()
        self.map_resolution = self.costmap.getResolution()
        self.map_origin = (self.costmap.getOriginX(), self.costmap.getOriginY())
        rospy.logdebug("Costmap updated: %.3f resolution", self.map_resolution)

    def build_kd_tree(self):
        if len(self.nodes) > self.last_tree_build_size + self.kd_tree_rebuild_interval:
            self.node_positions = [node.get_position() for node in self.nodes]
            if self.node_positions:
                self.kd_tree = KDTree(self.node_positions)
                self.last_tree_build_size = len(self.nodes)
                rospy.logdebug("Rebuilt KDTree with %d nodes", len(self.nodes))

    def makePlan(self, start, goal):
        if self.costmap is None or self.octree is None:
            rospy.logerr("Costmap or OctoMap not initialized. Cannot plan.")
            return []

        rospy.loginfo("Starting 3D global planning")
        self.update_costmap()

        start_node = self.pose_to_node(start.pose)
        goal_node = self.pose_to_node(goal.pose)

        if not self.is_valid_position(start_node):
            rospy.logwarn("Start position invalid: (%.2f, %.2f, %.2f)", start_node.x, start_node.y, start_node.z)
            return []

        if not self.is_valid_position(goal_node):
            rospy.logwarn("Goal position invalid: (%.2f, %.2f, %.2f)", goal_node.x, goal_node.y, goal_node.z)
            return []

        self.nodes = [start_node]
        self.node_positions = [start_node.get_position()]
        self.kd_tree = KDTree(self.node_positions)
        self.last_tree_build_size = 1
        start_node.cost = 0.0

        best_cost = float('inf')
        best_path = None

        for iteration in range(self.max_iter):
            if iteration % 500 == 0:
                rospy.loginfo("Iteration %d/%d, nodes: %d", iteration, self.max_iter, len(self.nodes))
                if iteration % 1000 == 0:
                    self.visualize_tree()

            if self.informed_sampling and best_path is not None:
                rand_node = self.sample_informed(start_node, goal_node, best_cost)
            else:
                rand_node = self.sample_free_3d()

            if rand_node is None:
                continue

            _, nearest_idx = self.kd_tree.query(rand_node.get_position())
            nearest_node = self.nodes[nearest_idx]

            new_node = self.steer_3d(nearest_node, rand_node)
            if not self.collision_free_3d(nearest_node, new_node):
                continue

            radius = self.get_rewire_radius()
            near_idxs = self.kd_tree.query_ball_point(new_node.get_position(), radius)
            near_nodes = [self.nodes[i] for i in near_idxs]

            min_cost_node = nearest_node
            min_cost = nearest_node.cost + self.position_distance(nearest_node, new_node)

            for near_node in near_nodes:
                cost = near_node.cost + self.position_distance(near_node, new_node)
                if cost < min_cost and self.collision_free_3d(near_node, new_node):
                    min_cost_node = near_node
                    min_cost = cost

            new_node.parent = min_cost_node
            new_node.cost = min_cost
            min_cost_node.children.append(new_node)
            self.nodes.append(new_node)
            self.node_positions.append(new_node.get_position())
            self.rewire_tree(new_node, near_nodes)

            if self.position_distance(new_node, goal_node) <= self.goal_tolerance:
                if goal_node not in self.nodes:
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + self.position_distance(new_node, goal_node)
                    self.nodes.append(goal_node)
                    self.node_positions.append(goal_node.get_position())
                else:
                    if new_node.cost + self.position_distance(new_node, goal_node) < goal_node.cost:
                        goal_node.parent = new_node
                        goal_node.cost = new_node.cost + self.position_distance(new_node, goal_node)

                if goal_node.cost < best_cost:
                    best_cost = goal_node.cost
                    best_path = self.extract_path(goal_node)
                    rospy.loginfo("New best path: cost=%.2f, nodes=%d", best_cost, len(best_path))

            if iteration % self.kd_tree_rebuild_interval == 0:
                self.build_kd_tree()

        if best_path:
            rospy.loginfo("Found path with %d waypoints (cost=%.2f)", len(best_path), best_cost)
            ros_path = self.convert_to_ros_path(best_path, start.header)

            if self.path_smoothing:
                ros_path = self.smooth_path_3d(ros_path)

            self.visualize_tree()
            self.path_pub.publish(ros_path)
            return ros_path.poses
        else:
            rospy.logwarn("No path found after %d iterations", self.max_iter)
            return []

    def pose_to_node(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return Node(x, y, z, yaw)

    def sample_free_3d(self):
        if self.costmap is None:
            return None

        for _ in range(100):
            x = np.random.uniform(self.map_origin[0], self.map_origin[0] + self.costmap.getSizeInMetersX())
            y = np.random.uniform(self.map_origin[1], self.map_origin[1] + self.costmap.getSizeInMetersY())
            z = np.random.uniform(self.min_altitude, self.max_altitude)
            yaw = np.random.uniform(-math.pi, math.pi)
            node = Node(x, y, z, yaw)
            if self.is_valid_position(node):
                return node
        return None

    def sample_informed(self, start, goal, c_best):
        c_min = self.position_distance(start, goal)
        if c_best < float('inf') and c_best > c_min:
            a = c_best / 2
            c = c_min / 2
            if a <= c:
                rospy.logwarn("Invalid ellipsoid parameters, falling back to uniform sampling")
                return self.sample_free_3d()
            b = math.sqrt(a ** 2 - c ** 2)
            phi = np.random.uniform(0, 2 * math.pi)
            costheta = np.random.uniform(-1, 1)
            u = np.random.uniform(0, 1)
            theta = math.acos(costheta)
            r = a * (u ** (1 / 3))
            x_ell = r * math.sin(theta) * math.cos(phi)
            y_ell = r * math.sin(theta) * math.sin(phi)
            z_ell = b * math.cos(theta)
            direction = np.array([goal.x - start.x, goal.y - start.y, goal.z - start.z])
            direction_norm = np.linalg.norm(direction)
            if direction_norm < 1e-6:
                return self.sample_free_3d()
            direction /= direction_norm
            rotation = R.align_vectors([[0, 0, 1]], [direction])[0]
            point = rotation.apply([x_ell, y_ell, z_ell])
            center = np.array([(start.x + goal.x) / 2, (start.y + goal.y) / 2, (start.z + goal.z) / 2])
            x, y, z = center + point
            yaw = np.random.uniform(-math.pi, math.pi)
            return Node(x, y, z, yaw)
        return self.sample_free_3d()

    def steer_3d(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        if dist <= self.step_size:
            new_x, new_y, new_z = to_node.x, to_node.y, to_node.z
        else:
            scale = self.step_size / dist
            new_x = from_node.x + dx * scale
            new_y = from_node.y + dy * scale
            new_z = from_node.z + dz * scale
        if self.consider_orientation:
            time_step = self.step_size / max(self.nominal_speed, 0.1)
            max_dyaw = self.max_yaw_rate * time_step
            dyaw = self.angle_diff(to_node.yaw, from_node.yaw)
            if abs(dyaw) > max_dyaw:
                new_yaw = from_node.yaw + math.copysign(max_dyaw, dyaw)
            else:
                new_yaw = to_node.yaw
            new_yaw = self.normalize_angle(new_yaw)
        else:
            new_yaw = to_node.yaw
        return Node(new_x, new_y, new_z, new_yaw)

    def collision_free_3d(self, node1, node2):
        if not self.collision_free_2d(node1, node2):
            return False
        if node2.z < self.min_altitude or node2.z > self.max_altitude:
            return False
        if self.octree is not None:
            return self.collision_free_octomap(node1, node2)
        return True

    def collision_free_2d(self, node1, node2):
        if self.costmap is None:
            return True
        dist = math.sqrt((node2.x - node1.x) ** 2 + (node2.y - node1.y) ** 2)
        steps = max(int(dist / self.collision_check_resolution), 1)
        dx = (node2.x - node1.x) / steps
        dy = (node2.y - node1.y) / steps
        for i in range(steps + 1):
            x = node1.x + i * dx
            y = node1.y + i * dy
            if not self.is_free_2d(x, y):
                return False
        return True

    def collision_free_octomap(self, node1, node2):
        dist = self.position_distance(node1, node2)
        steps = max(int(dist / (self.octomap_resolution * 0.5)), 1)
        dx = (node2.x - node1.x) / steps
        dy = (node2.y - node1.y) / steps
        dz = (node2.z - node1.z) / steps
        for i in range(steps + 1):
            x = node1.x + i * dx
            y = node1.y + i * dy
            z = node1.z + i * dz
            if not self.is_free_3d(x, y, z):
                return False
        return True

    def is_valid_position(self, node):
        if not self.is_free_2d(node.x, node.y):
            return False
        if node.z < self.min_altitude or node.z > self.max_altitude:
            return False
        if self.octree is not None:
            if not self.is_free_3d(node.x, node.y, node.z):
                return False
        return True

    def is_free_2d(self, x, y):
        if self.costmap is None:
            return True
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        if (mx < 0 or mx >= self.costmap.getSizeInCellsX() or
                my < 0 or my >= self.costmap.getSizeInCellsY()):
            return False
        cost = self.costmap.getCost(mx, my)
        return cost < 50

    def is_free_3d(self, x, y, z):
        if self.octree is None:
            return True
        try:
            point_map = PointStamped()
            point_map.header.frame_id = "map"
            point_map.header.stamp = rospy.Time(0)
            point_map.point = Point(x, y, z)
            point_octomap = self.tf_buffer.transform(point_map, self.octomap_frame, rospy.Duration(0.1))
            x_tf, y_tf, z_tf = point_octomap.point.x, point_octomap.point.y, point_octomap.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error: %s", str(e))
            return False

        safety_radius = max(self.safety_radius, self.octomap_resolution)
        safety_points = [
            (x_tf, y_tf, z_tf),
            (x_tf + safety_radius, y_tf, z_tf),
            (x_tf - safety_radius, y_tf, z_tf),
            (x_tf, y_tf + safety_radius, z_tf),
            (x_tf, y_tf - safety_radius, z_tf),
            (x_tf, y_tf, z_tf + safety_radius),
            (x_tf, y_tf, z_tf - safety_radius)
        ]
        for px, py, pz in safety_points:
            node = self.octree.search(px, py, pz)
            if node is None:
                if not self.treat_unknown_as_free:
                    return False
            elif self.octree.isNodeOccupied(node):
                return False
        return True

    def position_distance(self, node1, node2):
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2 + (node1.z - node2.z) ** 2)

    def distance_3d(self, node1, node2):
        spatial_dist = self.position_distance(node1, node2)
        if self.consider_orientation:
            angular_dist = abs(self.angle_diff(node1.yaw, node2.yaw))
            return spatial_dist + self.yaw_weight * angular_dist
        return spatial_dist

    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_rewire_radius(self):
        if self.adaptive_radius and len(self.nodes) > 1:
            gamma = 2 * (1 + 1 / 3) ** (1 / 3) * ((self.max_altitude - self.min_altitude) / math.pi) ** (1 / 3)
            radius = min(gamma * (math.log(len(self.nodes)) / len(self.nodes)) ** (1 / 3), self.rewire_radius)
            return max(radius, self.step_size)
        return self.rewire_radius

    def rewire_tree(self, new_node, near_nodes):
        for near_node in near_nodes:
            if near_node == new_node.parent:
                continue
            cost = new_node.cost + self.position_distance(new_node, near_node)
            if cost < near_node.cost and self.collision_free_3d(new_node, near_node):
                if near_node.parent and near_node in near_node.parent.children:
                    near_node.parent.children.remove(near_node)
                near_node.parent = new_node
                near_node.cost = cost
                new_node.children.append(near_node)
                self.propagate_cost_to_children(near_node)

    def propagate_cost_to_children(self, node):
        for child in node.children:
            old_cost = child.cost
            child.cost = node.cost + self.position_distance(node, child)
            if abs(child.cost - old_cost) > self.cost_propagation_threshold:
                self.propagate_cost_to_children(child)

    def extract_path(self, goal_node):
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = current.parent
        return path[::-1]

    def convert_to_ros_path(self, node_path, header):
        path = Path()
        path.header = header
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"
        for node in node_path:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            pose.pose.position.z = node.z
            quaternion = quaternion_from_euler(0, 0, node.yaw)
            pose.pose.orientation = Quaternion(*quaternion)
            path.poses.append(pose)
        return path

    def smooth_path_3d(self, path):
        if len(path.poses) < 3:
            return path
        smoothed = Path()
        smoothed.header = path.header
        smoothed.poses.append(path.poses[0])
        i = 0
        while i < len(path.poses) - 1:
            found_shortcut = False
            for j in range(len(path.poses) - 1, i, -1):
                if self.line_of_sight_3d(path.poses[i], path.poses[j]):
                    smoothed.poses.append(path.poses[j])
                    i = j
                    found_shortcut = True
                    break
            if not found_shortcut:
                i += 1
                if i < len(path.poses):
                    smoothed.poses.append(path.poses[i])
        rospy.loginfo("Path smoothed: %d → %d waypoints", len(path.poses), len(smoothed.poses))
        return smoothed

    def line_of_sight_3d(self, pose1, pose2):
        node1 = self.pose_to_node(pose1.pose)
        node2 = self.pose_to_node(pose2.pose)
        return self.collision_free_3d(node1, node2)

    def visualize_tree(self):
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
                marker.color.a = 0.5
                marker.lifetime = rospy.Duration(rospy.get_param('~marker_lifetime', 10.0))
                p1 = Point(node.parent.x, node.parent.y, node.parent.z)
                p2 = Point(node.x, node.y, node.z)
                marker.points = [p1, p2]
                marker_array.markers.append(marker)
        self.tree_pub.publish(marker_array)

def create_planner():
    return InformedRRT3D()

if __name__ == '__main__':
    rospy.init_node('informed_rrt_3d_planner_test')

    planner = InformedRRT3D()

    rospy.sleep(1.0)  # Give time for publishers to register

    # Fake start and goal
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.position.z = 1.0
    start.pose.orientation.w = 1.0

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 5.0
    goal.pose.position.y = 5.0
    goal.pose.position.z = 1.0
    goal.pose.orientation.w = 1.0

    planner.makePlan(start, goal)

    rospy.spin()
