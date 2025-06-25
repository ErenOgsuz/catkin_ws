// informed_rrt3d.cpp

#include "rrt3d_planner/informed_rrt3d.h"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <random>

PLUGINLIB_EXPORT_CLASS(rrt3d_planner::InformedRRT3D, nav_core::BaseGlobalPlanner)

namespace rrt3d_planner
{

  InformedRRT3D::InformedRRT3D() {}

  InformedRRT3D::InformedRRT3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void InformedRRT3D::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    nh_ = ros::NodeHandle("~" + name);
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    map_resolution_ = costmap_->getResolution();
    origin_x_ = costmap_->getOriginX();
    origin_y_ = costmap_->getOriginY();

    nh_.param("step_size", step_size_, 0.8);
    nh_.param("goal_tolerance", goal_tolerance_, 0.5);
    nh_.param("rewire_radius", rewire_radius_, 1.5);
    nh_.param("min_altitude", min_altitude_, 0.2);
    nh_.param("max_altitude", max_altitude_, 5.0);
    nh_.param("consider_orientation", consider_orientation_, true);
    nh_.param("max_yaw_rate", max_yaw_rate_, 1.0);   // rad/s
    nh_.param("nominal_speed", nominal_speed_, 1.0); // m/s
    nh_.param("yaw_weight", yaw_weight_, 0.3);       // cost function
    nh_.param("informed_sampling", informed_sampling_, true);

    tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrt_tree", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path_3d", 1);
    octomap_sub_ = nh_.subscribe("/rtabmap/octomap_binary", 1, &InformedRRT3D::octomapCallback, this);

    ROS_INFO("InformedRRT3D planner initialized");
  }

  void InformedRRT3D::octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
  {
    if (!msg)
      return;
    octomap::AbstractOcTree *tree = octomap_msgs::binaryMsgToMap(*msg);
    if (tree)
    {
      octree_.reset(dynamic_cast<octomap::OcTree *>(tree));
      if (octree_)
      {
        octomap_resolution_ = octree_->getResolution();
        octomap_frame_ = msg->header.frame_id;
        ROS_INFO("Octomap updated: resolution = %.2f", octomap_resolution_);
      }
    }
  }

  bool InformedRRT3D::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!costmap_ || !octree_)
    {
      ROS_WARN("Costmap or Octomap not initialized");
      return false;
    }

    nodes_.clear();

    // Get yaw from quaternion
    tf::Quaternion q;
    tf::quaternionMsgToTF(start.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Node *start_node = new Node(start.pose.position.x, start.pose.position.y, start.pose.position.z, yaw);

    // Repeat for goal
    tf::quaternionMsgToTF(goal.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Node *goal_node = new Node(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, yaw);
    nodes_.emplace_back(start_node);

    for (int i = 0; i < 1000; ++i)
    {
      Node *rand = sampleRandomNode();
      if (!rand)
        continue;

      Node *nearest = start_node;
      double min_dist = distance(rand, nearest);
      for (const auto &node : nodes_)
      {
        double d = distance(rand, node.get());
        if (d < min_dist)
        {
          nearest = node.get();
          min_dist = d;
        }
      }

      Node *new_node = steer(nearest, rand);
      if (isValid(new_node) && collisionFree(nearest, new_node))
      {
        new_node->parent = nearest;
        new_node->cost = nearest->cost + distance(nearest, new_node);
        nearest->children.push_back(new_node);
        nodes_.emplace_back(new_node);

        if (distance(new_node, goal_node) < goal_tolerance_)
        {
          goal_node->parent = new_node;
          goal_node->cost = new_node->cost + distance(new_node, goal_node);
          nodes_.emplace_back(goal_node);
          break;
        }
      }
    }

    std::vector<Node *> path = extractPath(goal_node);
    if (path.empty())
      return false;

    publishPath(path, start.header.frame_id);
    visualizeTree();
    for (const auto &n : path)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = start.header;
      pose.pose.position.x = n->x;
      pose.pose.position.y = n->y;
      pose.pose.position.z = n->z;
      plan.push_back(pose);
    }
    return true;
  }

  Node *InformedRRT3D::sampleRandomNode()
  {
    double x = origin_x_ + ((double)rand() / RAND_MAX) * costmap_->getSizeInMetersX();
    double y = origin_y_ + ((double)rand() / RAND_MAX) * costmap_->getSizeInMetersY();
    double z = min_altitude_ + ((double)rand() / RAND_MAX) * (max_altitude_ - min_altitude_);
    double yaw = ((double)rand() / RAND_MAX) * 2 * M_PI - M_PI;
    return new Node(x, y, z, yaw);
  }

  Node *InformedRRT3D::sampleInformedNode(Node *start, Node *goal, double c_best)
  {
    double c_min = distance(start, goal);
    if (c_best == std::numeric_limits<double>::infinity() || c_best <= c_min)
      return sampleRandomNode(); // fallback to uniform sampling

    // Ellipse parameters
    double a = c_best / 2.0; // semi-major axis
    double c = c_min / 2.0;
    double b = std::sqrt(a * a - c * c); // semi-minor axis

    // Sample random point in unit ball
    double phi = ((double)rand() / RAND_MAX) * 2 * M_PI;
    double costheta = ((double)rand() / RAND_MAX) * 2.0 - 1.0;
    double u = ((double)rand() / RAND_MAX);

    double theta = std::acos(costheta);
    double r = std::cbrt(u); // uniform volume

    double x_ball = r * std::sin(theta) * std::cos(phi);
    double y_ball = r * std::sin(theta) * std::sin(phi);
    double z_ball = r * std::cos(theta);

    // Transform unit ball to ellipsoid

    Eigen::Vector3d start_pt(start->x, start->y, start->z);
    Eigen::Vector3d goal_pt(goal->x, goal->y, goal->z);
    Eigen::Vector3d center = (start_pt + goal_pt) / 2.0;
    Eigen::Vector3d dir = (goal_pt - start_pt).normalized();

    // Create basis using direction vector
    Eigen::Matrix3d M;
    Eigen::Vector3d z = dir;
    Eigen::Vector3d x = z.unitOrthogonal(); // any vector orthogonal to z
    Eigen::Vector3d y = z.cross(x);
    M.col(0) = x;
    M.col(1) = y;
    M.col(2) = z;

    Eigen::Vector3d D(b, b, a); // scale
    Eigen::Vector3d sample = M * (D.asDiagonal() * Eigen::Vector3d(x_ball, y_ball, z_ball)) + center;

    double yaw = ((double)rand() / RAND_MAX) * 2 * M_PI - M_PI;

    return new Node(sample.x(), sample.y(), sample.z(), yaw);
  }

  double InformedRRT3D::normalizeAngle(double angle)
  {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  Node *InformedRRT3D::steer(Node *from, Node *to)
  {
    double dx = to->x - from->x;
    double dy = to->y - from->y;
    double dz = to->z - from->z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    double new_x, new_y, new_z;
    if (dist <= step_size_)
    {
      new_x = to->x;
      new_y = to->y;
      new_z = to->z;
    }
    else
    {
      double scale = step_size_ / dist;
      new_x = from->x + dx * scale;
      new_y = from->y + dy * scale;
      new_z = from->z + dz * scale;
    }

    // Interpolate yaw (handling wrap-around)
    double dyaw = normalizeAngle(to->yaw - from->yaw);
    double yaw_step = std::min(dyaw, max_yaw_rate_ * (step_size_ / nominal_speed_));
    double new_yaw = normalizeAngle(from->yaw + yaw_step);

    return new Node(new_x, new_y, new_z, new_yaw);
  }

  double InformedRRT3D::distance(Node *a, Node *b)
  {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    double dyaw = normalizeAngle(a->yaw - b->yaw);
    return std::sqrt(dx * dx + dy * dy + dz * dz) + yaw_weight_ * std::abs(dyaw);
  }

  bool InformedRRT3D::isFree(double x, double y, double z)
  {
    if (!octree_)
      return true;
    octomap::point3d pt(x, y, z);
    octomap::OcTreeNode *node = octree_->search(pt);
    if (!node)
      return true;
    return !octree_->isNodeOccupied(node);
  }

  bool InformedRRT3D::isValid(Node *node)
  {
    return isFree(node->x, node->y, node->z);
  }

  bool InformedRRT3D::collisionFree(Node *from, Node *to)
  {
    int steps = static_cast<int>(distance(from, to) / (octomap_resolution_ * 0.5));
    for (int i = 0; i <= steps; ++i)
    {
      double x = from->x + (to->x - from->x) * i / steps;
      double y = from->y + (to->y - from->y) * i / steps;
      double z = from->z + (to->z - from->z) * i / steps;
      if (!isFree(x, y, z))
        return false;
    }
    return true;
  }

  std::vector<Node *> InformedRRT3D::extractPath(Node *goal)
  {
    std::vector<Node *> path;
    for (Node *n = goal; n != nullptr; n = n->parent)
    {
      path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  void InformedRRT3D::publishPath(const std::vector<Node *> &path, const std::string &frame_id)
  {
    nav_msgs::Path ros_path;
    ros_path.header.stamp = ros::Time::now();
    ros_path.header.frame_id = frame_id;
    for (const auto &n : path)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = ros_path.header;
      pose.pose.position.x = n->x;
      pose.pose.position.y = n->y;
      pose.pose.position.z = n->z;

      // Convert yaw to quaternion for pose.orientation
      tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, n->yaw);
      tf::quaternionTFToMsg(q, pose.pose.orientation);

      ros_path.poses.push_back(pose);
    }

    path_pub_.publish(ros_path);
  }

  void InformedRRT3D::visualizeTree()
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);
    int id = 0;
    for (const auto &node : nodes_)
    {
      if (!node->parent)
        continue;
      visualization_msgs::Marker line;
      line.header.frame_id = "map";
      line.header.stamp = ros::Time::now();
      line.ns = "rrt_tree";
      line.id = id++;
      line.type = visualization_msgs::Marker::LINE_STRIP;
      line.scale.x = 0.02;
      line.color.r = 0.0;
      line.color.g = 1.0;
      line.color.b = 0.0;
      line.color.a = 0.6;
      geometry_msgs::Point p1, p2;
      p1.x = node->parent->x;
      p1.y = node->parent->y;
      p1.z = node->parent->z;
      p2.x = node->x;
      p2.y = node->y;
      p2.z = node->z;
      line.points.push_back(p1);
      line.points.push_back(p2);
      array.markers.push_back(line);
    }
    tree_pub_.publish(array);
  }

} // namespace rrt3d_planner
