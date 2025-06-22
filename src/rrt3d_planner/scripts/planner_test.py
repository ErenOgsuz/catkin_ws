#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, PointStamped  # ROS message types for poses and points
from nav_msgs.msg import OccupancyGrid, Path  # For map and path messages
from sensor_msgs.msg import PointCloud2  # For point cloud data
# from nav_core import BaseGlobalPlanner  # Commented out - possibly for plugin base class
from octomap_msgs.msg import Octomap  # Octomap message type
from octomap import OcTree  # Octomap data structure for 3D occupancy
from tf.transformations import euler_from_quaternion, quaternion_from_euler  # For quaternion-euler conversions
from visualization_msgs.msg import Marker, MarkerArray  # For visualizing tree and path in RViz
import tf2_ros  # For ROS TF2 transform handling
# First but nice try insallah no error for k-dimensional tree
from scipy.spatial import KDTree  # KD-Tree spatial index for nearest neighbor searches
from scipy.spatial.transform import Rotation as R  # For 3D rotations and aligning vectors

class Node:
    """
    Represents a node/state in the RRT tree.
    Each node stores its 3D position (x,y,z), yaw orientation,
    a reference to its parent node, cost to reach it, and child nodes.
    """

    __slots__ = ('x', 'y', 'z', 'yaw', 'parent', 'cost', 'children')

    def __init__(self, x, y, z=0.0, yaw=0.0):
        self.x = x  # X coordinate
        self.y = y  # Y coordinate
        self.z = z  # Z coordinate (altitude)
        self.yaw = yaw  # Orientation around Z axis
        self.parent = None  # Parent node in tree
        self.cost = 0.0  # Cost to reach this node from root
        self.children = []  # Child nodes

    def get_position(self):
        """Returns position as list [x, y, z]."""
        return [self.x, self.y, self.z]

    def get_pose(self):
        """Returns full pose including yaw [x, y, z, yaw]."""
        return [self.x, self.y, self.z, self.yaw]

class InformedRRT3D():
    """
    3D Informed RRT* planner class implementing a sampling-based path planning
    algorithm optimized for 3D environments with orientation consideration.
    """

    def __init__(self):
        super(InformedRRT3D, self).__init__()

        # Load planner parameters from ROS parameter server with defaults
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

        # Initialize data structures
        self.nodes = []  # List of all nodes
        self.node_positions = []  # Positions for KDTree indexing
        self.kd_tree = None  # KDTree for nearest neighbor queries
        self.last_tree_build_size = 0  # Track when to rebuild KDTree

        self.costmap = None  # 2D costmap for obstacles
        self.octree = None  # Octomap (3D occupancy map)

        self.octomap_frame = "map"  # Frame of the octomap
        self.map_resolution = 0.05  # Map resolution (meters per cell)
        self.map_origin = None  # Map origin coordinates

        # ROS publishers for visualization and path output
        self.tree_pub = rospy.Publisher('rrt_tree', MarkerArray, queue_size=1, latch=True)
        self.path_pub = rospy.Publisher('rrt_path_3d', Path, queue_size=1, latch=True)

        # Subscribers for octomap and point cloud data
        self.octomap_sub = rospy.Subscriber('/rtabmap/octomap_binary', Octomap, self.octomap_callback)
        self.rtabmap_cloud_sub = rospy.Subscriber('/rtabmap/cloud_map', PointCloud2, self.pointcloud_callback, queue_size=1)

        # TF2 buffer and listener for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Optimized 3D Informed RRT* Planner initialized")

    def octomap_callback(self, msg):
        """
        Octomap binary message callback.
        Converts incoming octomap data into OcTree structure for collision checking.
        """
        try:
            self.octree = OcTree(msg.resolution)
            self.octree.readBinary(msg.data)
            self.octomap_resolution = msg.resolution
            self.octomap_frame = msg.header.frame_id
            rospy.loginfo("Octomap updated: %.2f resolution, frame: %s", msg.resolution, self.octomap_frame)
        except Exception as e:
            rospy.logwarn("Octomap processing failed: %s", str(e))

    def pointcloud_callback(self, msg):
        """
        PointCloud2 callback - currently only logs the number of points received.
        Can be expanded for more processing if needed.
        """
        rospy.logdebug("Received point cloud with %d points", msg.width * msg.height)

    def initialize(self, name, costmap_ros):
        """
        Initialization method for use with ROS move_base plugin interface.
        Stores the costmap_ros instance and updates internal costmap data.
        """
        self.name = name
        self.costmap_ros = costmap_ros
        self.update_costmap()
        return True

    def update_costmap(self):
        """
        Updates internal costmap, map resolution, and origin from costmap_ros.
        """
        self.costmap = self.costmap_ros.getCostmap()
        self.map_resolution = self.costmap.getResolution()
        self.map_origin = (self.costmap.getOriginX(), self.costmap.getOriginY())
        rospy.logdebug("Costmap updated: %.3f resolution", self.map_resolution)

    def build_kd_tree(self):
        """
        Rebuilds the KDTree for nearest neighbor search if the number of nodes
        has increased sufficiently since last build.
        """
        if len(self.nodes) > self.last_tree_build_size + self.kd_tree_rebuild_interval:
            self.node_positions = [node.get_position() for node in self.nodes]
            if self.node_positions:
                self.kd_tree = KDTree(self.node_positions)
                self.last_tree_build_size = len(self.nodes)
                rospy.logdebug("Rebuilt KDTree with %d nodes", len(self.nodes))

    def makePlan(self, start, goal):
        """
        Main path planning method.
        Given start and goal poses, attempts to find a collision-free path using
        Informed RRT* in 3D space considering orientation.
        Returns a list of PoseStamped waypoints or empty list if no path found.
        """
        # Ensure costmap and octomap are initialized
        if self.costmap is None or self.octree is None:
            rospy.logerr("Costmap or OctoMap not initialized. Cannot plan.")
            return []

        rospy.loginfo("Starting 3D global planning")
        self.update_costmap()

        # Convert start and goal Pose to Node format
        start_node = self.pose_to_node(start.pose)
        goal_node = self.pose_to_node(goal.pose)

        # Validate start and goal positions
        if not self.is_valid_position(start_node):
            rospy.logwarn("Start position invalid: (%.2f, %.2f, %.2f)", start_node.x, start_node.y, start_node.z)
            return []

        if not self.is_valid_position(goal_node):
            rospy.logwarn("Goal position invalid: (%.2f, %.2f, %.2f)", goal_node.x, goal_node.y, goal_node.z)
            return []

        # Initialize RRT tree with start node
        self.nodes = [start_node]
        self.node_positions = [start_node.get_position()]
        self.kd_tree = KDTree(self.node_positions)
        self.last_tree_build_size = 1
        start_node.cost = 0.0

        best_cost = float('inf')
        best_path = None

        for iteration in range(self.max_iter):
            # Periodically log progress and visualize tree
            if iteration % 500 == 0:
                rospy.loginfo("Iteration %d/%d, nodes: %d", iteration, self.max_iter, len(self.nodes))
                if iteration % 1000 == 0:
                    self.visualize_tree()

            # Sample a random node either informed (inside ellipsoid) or uniform in free space
            if self.informed_sampling and best_path is not None:
                rand_node = self.sample_informed(start_node, goal_node, best_cost)
            else:
                rand_node = self.sample_free_3d()

            if rand_node is None:
                continue

            # Find nearest node in the tree to the sampled node
            _, nearest_idx = self.kd_tree.query(rand_node.get_position())
            nearest_node = self.nodes[nearest_idx]

            # Attempt to move from nearest node towards sampled node with step size
            new_node = self.steer_3d(nearest_node, rand_node)
            if not self.collision_free_3d(nearest_node, new_node):
                continue  # Skip if path segment collides

            # Find nearby nodes for possible rewiring to optimize path cost
            radius = self.get_rewire_radius()
            near_idxs = self.kd_tree.query_ball_point(new_node.get_position(), radius)
            near_nodes = [self.nodes[i] for i in near_idxs]

            # Select parent node minimizing cost to new node
            min_cost_node = nearest_node
            min_cost = nearest_node.cost + self.position_distance(nearest_node, new_node)

            for near_node in near_nodes:
                cost = near_node.cost + self.position_distance(near_node, new_node)
                if cost < min_cost and self.collision_free_3d(near_node, new_node):
                    min_cost_node = near_node
                    min_cost = cost

            # Attach new node to tree
            new_node.parent = min_cost_node
            new_node.cost = min_cost
            min_cost_node.children.append(new_node)

            self.nodes.append(new_node)
            self.node_positions.append(new_node.get_position())

            # Rewire nearby nodes if beneficial
            self.rewire_tree(new_node, near_nodes)

            # Check if new node is close enough to goal to connect
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

                # Update best path if improved
                if goal_node.cost < best_cost:
                    best_cost = goal_node.cost
                    best_path = self.extract_path(goal_node)
                    rospy.loginfo("New best path: cost=%.2f, nodes=%d", best_cost, len(best_path))

            # Rebuild KDTree periodically for efficient NN queries
            if iteration % self.kd_tree_rebuild_interval == 0:
                self.build_kd_tree()

        # After iterations, return the best path found (if any)
        if best_path:
            rospy.loginfo("Found path with %d waypoints (cost=%.2f)", len(best_path), best_cost)
            ros_path = self.convert_to_ros_path(best_path, start.header)

            # Optionally smooth the path
            if self.path_smoothing:
                ros_path = self.smooth_path_3d(ros_path)

            self.visualize_tree()
            self.path_pub.publish(ros_path)
            return ros_path.poses
        else:
            rospy.logwarn("No path found after %d iterations", self.max_iter)
            return []

    def pose_to_node(self, pose):
        """
        Converts a ROS Pose message to a Node object with x, y, z and yaw.
        """
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return Node(x, y, z, yaw)

    def sample_free_3d(self):
        """
        Samples a random free node in 3D space within map and altitude bounds.
        Attempts up to 100 times to find a valid free sample.
        """
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
        """
        Samples a node inside an ellipsoid (informed sampling) based on current best path cost.
        The ellipsoid is defined to bias samples toward promising regions.
        Falls back to uniform sampling if ellipsoid parameters invalid.
        """
        c_min = self.position_distance(start, goal)

        if c_best < float('inf') and c_best > c_min:
            a = c_best / 2
            c = c_min / 2

            if a <= c:
                rospy.logwarn("Invalid ellipsoid parameters, falling back to uniform sampling")
                return self.sample_free_3d()

            b = math.sqrt(a ** 2 - c ** 2)

            # Sample random point in unit ball and scale to ellipsoid
            phi = np.random.uniform(0, 2 * math.pi)
            costheta = np.random.uniform(-1, 1)
            u = np.random.uniform(0, 1)

            theta = math.acos(costheta)
            r = a * (u ** (1 / 3))  # Cube root for uniform volume sampling

            x_ell = r * math.sin(theta) * math.cos(phi)
            y_ell = r * math.sin(theta) * math.sin(phi)
            z_ell = b * math.cos(theta)

            # Align ellipsoid axis to vector from start to goal
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
        """
        Generate a new node by moving from 'from_node' towards 'to_node' by at most step_size.
        Takes into account orientation (yaw) changes constrained by max_yaw_rate if enabled.
        Returns the new Node in 3D space with updated position and yaw.
        """
        # Calculate difference in x, y, z coordinates
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        
        # Euclidean distance between from_node and to_node
        dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        
        # If within step size, directly use to_node position
        if dist <= self.step_size:
            new_x, new_y, new_z = to_node.x, to_node.y, to_node.z
        else:
            # Otherwise, scale the step to max step_size in direction towards to_node
            scale = self.step_size / dist
            new_x = from_node.x + dx * scale
            new_y = from_node.y + dy * scale
            new_z = from_node.z + dz * scale
        
        # Handle orientation (yaw) if consideration is enabled
        if self.consider_orientation:
            # Calculate time required to make this step
            time_step = self.step_size / max(self.nominal_speed, 0.1)
            max_dyaw = self.max_yaw_rate * time_step
            
            # Calculate shortest angular difference between target and current yaw
            dyaw = self.angle_diff(to_node.yaw, from_node.yaw)
            
            # Limit yaw change to max yaw rate
            if abs(dyaw) > max_dyaw:
                new_yaw = from_node.yaw + math.copysign(max_dyaw, dyaw)
            else:
                new_yaw = to_node.yaw
            
            # Normalize yaw angle to [-pi, pi]
            new_yaw = self.normalize_angle(new_yaw)
        else:
            # If orientation not considered, use target yaw directly
            new_yaw = to_node.yaw
        
        # Return new node at computed pose
        return Node(new_x, new_y, new_z, new_yaw)

    def collision_free_3d(self, node1, node2):
        """
        Checks collision-free status for a 3D edge between node1 and node2.
        Validates 2D collision first, altitude bounds, and Octomap if available.
        Returns True if edge is collision free, False otherwise.
        """
        # Check 2D collision between node projections
        if not self.collision_free_2d(node1, node2):
            return False
        
        # Check altitude limits
        if node2.z < self.min_altitude or node2.z > self.max_altitude:
            return False
        
        # If Octomap available, check 3D collision along edge
        if self.octree is not None:
            return self.collision_free_octomap(node1, node2)
        
        # No obstacles detected
        return True

    def collision_free_2d(self, node1, node2):
        """
        Check collision in 2D plane between node1 and node2 based on costmap.
        Samples points along the line at intervals defined by collision_check_resolution.
        Returns True if all points are free, False if any point is obstructed.
        """
        # If no costmap, assume free space
        if self.costmap is None:
            return True
        
        # Compute distance and steps along the line
        dist = math.sqrt((node2.x - node1.x) ** 2 + (node2.y - node1.y) ** 2)
        steps = max(int(dist / self.collision_check_resolution), 1)
        
        # Calculate increments for each step
        dx = (node2.x - node1.x) / steps
        dy = (node2.y - node1.y) / steps
        
        # Sample each point and check if free
        for i in range(steps + 1):
            x = node1.x + i * dx
            y = node1.y + i * dy
            if not self.is_free_2d(x, y):
                return False
        
        return True

    def collision_free_octomap(self, node1, node2):
        """
        Checks collision along the edge between node1 and node2 using the Octomap occupancy.
        Samples points with finer resolution (half the octomap resolution) and checks occupancy.
        Returns True if all sampled points are free or unknown (based on setting), False if any occupied.
        """
        # Calculate distance and number of steps based on octomap resolution
        dist = self.position_distance(node1, node2)
        steps = max(int(dist / (self.octomap_resolution * 0.5)), 1)
        
        # Compute increments in each dimension
        dx = (node2.x - node1.x) / steps
        dy = (node2.y - node1.y) / steps
        dz = (node2.z - node1.z) / steps
        
        # Sample points along edge and check if free in octomap
        for i in range(steps + 1):
            x = node1.x + i * dx
            y = node1.y + i * dy
            z = node1.z + i * dz
            if not self.is_free_3d(x, y, z):
                return False
        
        return True

    def is_valid_position(self, node):
        """
        Check if a given node's position is valid and free of obstacles.
        Checks 2D costmap, altitude bounds, and 3D octomap occupancy.
        Returns True if valid and free, False otherwise.
        """
        # Check 2D costmap free space
        if not self.is_free_2d(node.x, node.y):
            return False
        
        # Check altitude constraints
        if node.z < self.min_altitude or node.z > self.max_altitude:
            return False
        
        # Check 3D occupancy if octomap available
        if self.octree is not None:
            if not self.is_free_3d(node.x, node.y, node.z):
                return False
        
        return True

    def is_free_2d(self, x, y):
        """
        Checks if the 2D position (x,y) is free based on costmap.
        Returns True if cost is below threshold; False otherwise or if outside map bounds.
        """
        if self.costmap is None:
            return True
        
        # Convert world coordinates to map grid indices
        mx = int((x - self.map_origin[0]) / self.map_resolution)
        my = int((y - self.map_origin[1]) / self.map_resolution)
        
        # Check map bounds
        if (mx < 0 or mx >= self.costmap.getSizeInCellsX() or
                my < 0 or my >= self.costmap.getSizeInCellsY()):
            return False
        
        # Check cost value against threshold for obstacles
        cost = self.costmap.getCost(mx, my)
        return cost < 50

    def is_free_3d(self, x, y, z):
        """
        Checks if 3D position (x,y,z) is free according to octomap occupancy.
        Transforms point from map frame to octomap frame.
        Checks occupancy of the point and safety radius neighbors.
        Returns True if free or unknown (based on setting), False if occupied or TF error.
        """
        # If no octomap, assume free
        if self.octree is None:
            return True
        
        try:
            # Prepare point stamped in map frame
            point_map = PointStamped()
            point_map.header.frame_id = "map"
            point_map.header.stamp = rospy.Time(0)
            point_map.point = Point(x, y, z)
            
            # Transform point to octomap frame
            point_octomap = self.tf_buffer.transform(point_map, self.octomap_frame, rospy.Duration(0.1))
            x_tf, y_tf, z_tf = point_octomap.point.x, point_octomap.point.y, point_octomap.point.z
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF error: %s", str(e))
            return False
        
        # Define safety radius and points around main point to check
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
        
        # Check each point's occupancy status in octomap
        for px, py, pz in safety_points:
            node = self.octree.search(px, py, pz)
            if node is None:
                # If unknown and configured to treat unknown as occupied, reject
                if not self.treat_unknown_as_free:
                    return False
            elif self.octree.isNodeOccupied(node):
                return False
        
        # All checked points are free or unknown allowed
        return True

    def position_distance(self, node1, node2):
        """
        Computes Euclidean distance between node1 and node2 in 3D space.
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2 + (node1.z - node2.z) ** 2)

    def distance_3d(self, node1, node2):
        """
        Computes combined distance metric between two nodes.
        Spatial distance plus weighted angular difference if orientation is considered.
        """
        spatial_dist = self.position_distance(node1, node2)
        if self.consider_orientation:
            angular_dist = abs(self.angle_diff(node1.yaw, node2.yaw))
            return spatial_dist + self.yaw_weight * angular_dist
        return spatial_dist

    def angle_diff(self, angle1, angle2):
        """
        Computes the minimal signed difference between two angles in radians.
        Result is normalized to [-pi, pi].
        """
        diff = angle2 - angle1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def normalize_angle(self, angle):
        """
        Normalize angle to be within [-pi, pi].
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def get_rewire_radius(self):
        """
        Calculates the rewiring radius for RRT* based on number of nodes and environment size.
        Uses an adaptive radius formula if enabled, bounded by a maximum radius and minimum step size.
        """
        if self.adaptive_radius and len(self.nodes) > 1:
            gamma = 2 * (1 + 1 / 3) ** (1 / 3) * ((self.max_altitude - self.min_altitude) / math.pi) ** (1 / 3)
            radius = min(gamma * (math.log(len(self.nodes)) / len(self.nodes)) ** (1 / 3), self.rewire_radius)
            return max(radius, self.step_size)
        return self.rewire_radius

    def rewire_tree(self, new_node, near_nodes):
        """
        Attempts to rewire nearby nodes to the new_node if it reduces their cost and path is collision free.
        Updates parent-child relationships and propagates cost changes downstream.
        """
        for near_node in near_nodes:
            # Skip the parent node
            if near_node == new_node.parent:
                continue
            
            # Compute potential new cost via new_node
            cost = new_node.cost + self.position_distance(new_node, near_node)
            
            # If cost improved and path collision free, rewire
            if cost < near_node.cost and self.collision_free_3d(new_node, near_node):
                # Remove near_node from old parent's children
                if near_node.parent and near_node in near_node.parent.children:
                    near_node.parent.children.remove(near_node)
                
                # Assign new parent and cost
                near_node.parent = new_node
                near_node.cost = cost
                new_node.children.append(near_node)
                
                # Propagate cost changes down the subtree
                self.propagate_cost_to_children(near_node)

    def propagate_cost_to_children(self, node):
        """
        Recursively update costs of children nodes based on parent's updated cost.
        Stops propagating when changes are negligible.
        """
        for child in node.children:
            old_cost = child.cost
            child.cost = node.cost + self.position_distance(node, child)
            if abs(child.cost - old_cost) > self.cost_propagation_threshold:
                self.propagate_cost_to_children(child)

    def extract_path(self, goal_node):
        """
        Extracts the path from start to goal by traversing parent links from goal_node.
        Returns the path as a list of nodes in start-to-goal order.
        """
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = current.parent
        return path[::-1]

    def convert_to_ros_path(self, node_path, header):
        """
        Converts a list of Node objects to a ROS nav_msgs/Path message.
        Sets header info and converts node poses including orientation.
        """
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
        """
        Performs path smoothing by shortcutting between waypoints if line of sight is clear.
        Reduces number of waypoints while preserving collision-free trajectory.
        """
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
        """
        Checks if straight-line path between two poses is collision free in 3D.
        Converts poses to nodes and uses collision checking.
        """
        node1 = self.pose_to_node(pose1.pose)
        node2 = self.pose_to_node(pose2.pose)
        return self.collision_free_3d(node1, node2)

    def visualize_tree(self):
        """
        Publishes a visualization of the current RRT tree as line markers in RViz.
        Clears previous markers before publishing new ones.
        """
        # If no nodes or ROS shutdown, do nothing
        if not self.nodes or rospy.is_shutdown():
            return
        
        marker_array = MarkerArray()
        marker_id = 0
        
        # Add clear all marker to remove old markers in RViz
        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        # For each node (except root), publish a line from parent to node
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
        
        # Publish all markers at once
        self.tree_pub.publish(marker_array)

def create_planner():
    """
    Factory function to create and return an instance of the InformedRRT3D planner.
    """
    return InformedRRT3D()

if __name__ == '__main__':
    # Initialize ROS node for testing the InformedRRT3D planner
    rospy.init_node('informed_rrt_3d_planner_test')

    # Create planner instance
    planner = InformedRRT3D()

    # Wait briefly to allow ROS publishers to initialize
    rospy.sleep(1.0)

    # Create a fake start pose
    start = PoseStamped()
    start.header.frame_id = "map"
    start.pose.position.x = 0.0
    start.pose.position.y = 0.0
    start.pose.position.z = 1.0
    start.pose.orientation.w = 1.0

    # Create a fake goal pose
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = 5.0
    goal.pose.position.y = 5.0
    goal.pose.position.z = 1.0
    goal.pose.orientation.w = 1.0

    # Run planning between start and goal
    planner.makePlan(start, goal)

    # Keep node alive until shutdown
    rospy.spin()
