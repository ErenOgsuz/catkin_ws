// informed_rrt3d.cpp

#include "rrt3d_planner/informed_rrt3d.h"
#include "rrt3d_planner/nanoflann.hpp"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <octomap_msgs/conversions.h>
#include "nanoflann.hpp"
#include <random>

PLUGINLIB_EXPORT_CLASS(rrt3d_planner::InformedRRT3D, nav_core::BaseGlobalPlanner)

namespace rrt3d_planner
{

  InformedRRT3D::InformedRRT3D() : tf_listener_(tf_buffer_) 
  {
    best_cost_ = std::numeric_limits<double>::infinity();
  }

  InformedRRT3D::InformedRRT3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros) : tf_listener_(tf_buffer_)
  {
    best_cost_ = std::numeric_limits<double>::infinity();
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
    nh_.param("adaptive_radius", adaptive_radius_, true); // default = true
    nh_.param("safety_radius", safety_radius_, 0.3);
    nh_.param("lethal_cost_threshold", lethal_cost_threshold_, 50);
    // nh_.param("kd_tree_rebuild_interval", kd_tree_rebuild_interval_, 50);
    nh_.param("octomap_resolution", octomap_resolution_, 0.1);
    nh_.param("treat_unknown_as_free", treat_unknown_as_free_, true);
    nh_.param("cost_propagation_threshold", cost_propagation_threshold_, 1e-6);
    nh_.param("path_smoothing", path_smoothing_, true);
    nh_.param("collision_check_resolution", collision_check_resolution_, 0.05);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/rtabmap/cloud_map", 1, &InformedRRT3D::cloudCallback, this);

    tree_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("rrt_tree", 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>("rrt_path_3d", 1);
    octomap_sub_ = nh_.subscribe("/rtabmap/octomap_binary", 1, &InformedRRT3D::octomapCallback, this);

    ROS_INFO("InformedRRT3D planner initialized");
  }

  void InformedRRT3D::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    size_t num_points = msg->width * msg->height;
    ROS_DEBUG_STREAM_THROTTLE(2.0, "Received PointCloud2 with " << num_points << " points.");

    // You can later convert and use these points
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *cloud);
  }

  void InformedRRT3D::octomapCallback(const octomap_msgs::OctomapConstPtr &msg)
  {
    if (!msg)
      return;

    // Store target frame
    std::string new_frame = msg->header.frame_id;

    // Check TF availability before accepting frame
    const int max_retries = 3;
    bool tf_available = false;

    for (int i = 0; i < max_retries; ++i)
    {
      if (tf_buffer_.canTransform(costmap_ros_->getGlobalFrameID(), new_frame, ros::Time(0), ros::Duration(0.2)))
      {
        tf_available = true;
        break;
      }
      else
      {
        ROS_WARN_THROTTLE(2.0, "TF not yet available from %s to %s (attempt %d/%d)",
                          new_frame.c_str(),
                          costmap_ros_->getGlobalFrameID().c_str(),
                          i + 1,
                          max_retries);
        ros::Duration(0.1).sleep();
      }
    }

    if (!tf_available)
    {
      ROS_ERROR_THROTTLE(5.0, "Octomap frame '%s' not TF-connected to global frame '%s'. Skipping update.",
                         new_frame.c_str(), costmap_ros_->getGlobalFrameID().c_str());
      return;
    }

    // Load octree
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
    if (tree)
    {
      std::shared_ptr<octomap::OcTree> converted(dynamic_cast<octomap::OcTree *>(tree));
      if (converted)
      {
        octree_ = converted;
        octomap_resolution_ = octree_->getResolution();
        octomap_frame_ = new_frame;
        ROS_INFO("Octomap updated: resolution = %.2f, frame = %s", octomap_resolution_, octomap_frame_.c_str());
      }
      else
      {
        ROS_ERROR("Failed to cast AbstractOcTree to OcTree.");
      }
    }
    else
    {
      ROS_ERROR("Failed to deserialize Octomap message.");
    }
    return;
  }

  bool InformedRRT3D::makePlan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &plan)
  {
    best_cost_ = std::numeric_limits<double>::infinity();

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

    auto start_node = std::make_shared<Node>(start.pose.position.x, start.pose.position.y, start.pose.position.z, yaw);

    // Repeat for goal
    tf::quaternionMsgToTF(goal.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    auto goal_node = std::make_shared<Node>(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, yaw);
    nodes_.emplace_back(start_node);
    buildKDTree();

    for (int i = 0; i < 1000; ++i)
    {
      std::shared_ptr<Node> rand;
      if (informed_sampling_ && best_cost_ < std::numeric_limits<double>::infinity())
        rand = sampleInformedNode(start_node, goal_node, best_cost_);
      else
        rand = sampleRandomNode();

      if (!rand)
        continue;

      // In logarithmic time this will find the nearest/closest node
      if (!kd_tree_)
      {
        ROS_WARN("KD-tree not yet built; skipping sample");
        continue;
      }

      std::array<double, 3> query_pt = {rand->x, rand->y, rand->z};

      size_t nearest_idx;
      double out_dist_sqr;
      nanoflann::KNNResultSet<double> resultSet(1);
      resultSet.init(&nearest_idx, &out_dist_sqr);
      kd_tree_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParameters());

      std::shared_ptr<Node> nearest = nodes_[nearest_idx];

      // Add new node
      std::shared_ptr<Node> new_node = steer(nearest, rand);

      if (isValid(new_node) && collisionFree(nearest, new_node))
      {
        new_node->parent = nearest;
        new_node->cost = nearest->cost + distance(nearest, new_node);
        nearest->children.push_back(new_node);
        nodes_.push_back(new_node); // standard C++ vector push
        buildKDTree();              // rebuild tree if needed

        // Get nearby nodes for rewiring
        double radius = getRewireRadius();
        std::vector<std::shared_ptr<Node>> near_nodes;

        // Use KD-tree to find neighbors
        nanoflann::SearchParameters params;
        std::vector<nanoflann::ResultItem<uint32_t, double>> ret_matches;
        kd_tree_->radiusSearch(query_pt.data(), radius, ret_matches, params);

        for (const auto &match : ret_matches)
        {
          near_nodes.push_back(nodes_[match.first]);
        }

        // Now rewire them
        rewireTree(new_node, near_nodes);

        if (distance(new_node, goal_node) < goal_tolerance_)
        {
          goal_node->parent = new_node;
          goal_node->cost = new_node->cost + distance(new_node, goal_node);

          if (goal_node->cost < best_cost_)
            best_cost_ = goal_node->cost;

          nodes_.push_back(goal_node);
          buildKDTree(); // in case more planning continues

          break;
        }
      }
    }

    std::vector<std::shared_ptr<Node>> path = extractPath(goal_node);
    if (path.empty())
      return false;

    // Smooth the path
    std::vector<std::shared_ptr<Node>> smoothed_path = smoothPath(path);

    // Publish and convert
    publishPath(smoothed_path, start.header.frame_id);

    visualizeTree();
    for (const auto &n : smoothed_path)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = start.header;
      pose.pose.position.x = n->x;
      pose.pose.position.y = n->y;
      pose.pose.position.z = n->z;

      // Convert yaw to quaternion
      tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, n->yaw);
      tf::quaternionTFToMsg(q, pose.pose.orientation);

      plan.push_back(pose);
    }
    return true;
  }

  std::shared_ptr<Node> InformedRRT3D::sampleRandomNode()
  {
    double x = origin_x_ + ((double)rand() / RAND_MAX) * costmap_->getSizeInMetersX();
    double y = origin_y_ + ((double)rand() / RAND_MAX) * costmap_->getSizeInMetersY();
    double z = min_altitude_ + ((double)rand() / RAND_MAX) * (max_altitude_ - min_altitude_);
    double yaw = ((double)rand() / RAND_MAX) * 2 * M_PI - M_PI;
    return std::make_shared<Node>(x, y, z, yaw);
  }

  std::shared_ptr<Node> InformedRRT3D::sampleInformedNode(const std::shared_ptr<Node> &start, const std::shared_ptr<Node> &goal, double c_best)
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

    return std::make_shared<Node>(sample.x(), sample.y(), sample.z(), yaw);
  }

  void InformedRRT3D::buildKDTree()
  {
    if (nodes_.size() < 2 || (nodes_.size() - last_kdtree_build_size_) < kd_tree_rebuild_interval_)
      return;

    point_cloud_.nodes = &nodes_;
    kd_tree_ = std::make_unique<KDTree>(3, point_cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kd_tree_->buildIndex();
    last_kdtree_build_size_ = nodes_.size();
    ROS_DEBUG("KD-Tree rebuilt with %zu nodes", nodes_.size());
  }

  double InformedRRT3D::getRewireRadius() const
  {
    if (!adaptive_radius_ || nodes_.size() <= 1)
      return rewire_radius_; // Use fixed radius if disabled or too small

    double n = static_cast<double>(nodes_.size());
    double gamma = 2.0 * std::pow(1.0 + 1.0 / 3.0, 1.0 / 3.0); // 3D
    double radius = gamma * std::pow(std::log(n) / n, 1.0 / 3.0);

    return std::min(rewire_radius_, std::max(radius, step_size_)); // Clamp
  }

  void InformedRRT3D::rewireTree(const std::shared_ptr<Node> &new_node,
                                 const std::vector<std::shared_ptr<Node>> &near_nodes)
  {
    for (auto &near_node : near_nodes)
    {
      if (near_node == new_node->parent)
        continue;

      double new_cost = new_node->cost + distance(new_node, near_node);
      if (new_cost < near_node->cost && collisionFree(new_node, near_node))
      {
        // Remove from old parent's children
        if (near_node->parent)
        {
          auto &siblings = near_node->parent->children;
          siblings.erase(std::remove(siblings.begin(), siblings.end(), near_node), siblings.end());
        }

        // Update parent and cost
        near_node->parent = new_node;
        near_node->cost = new_cost;
        new_node->children.push_back(near_node);

        propagateCostToChildren(near_node);
      }
    }
  }

  std::vector<std::shared_ptr<Node>> InformedRRT3D::smoothPath(const std::vector<std::shared_ptr<Node>> &path)
  {
    if (path.size() < 3)
      return path;

    std::vector<std::shared_ptr<Node>> smoothed;
    smoothed.push_back(path.front());

    size_t i = 0;
    while (i < path.size() - 1)
    {
      size_t j = path.size() - 1;
      bool shortcut_found = false;

      while (j > i + 1)
      {
        if (collisionFree(path[i], path[j]))
        {
          smoothed.push_back(path[j]);
          i = j;
          shortcut_found = true;
          break;
        }
        --j;
      }

      if (!shortcut_found)
      {
        ++i;
        smoothed.push_back(path[i]);
      }
    }

    return smoothed;
  }

  void InformedRRT3D::propagateCostToChildren(const std::shared_ptr<Node> &node)
  {
    for (auto &child : node->children)
    {
      double old_cost = child->cost;
      child->cost = node->cost + distance(node, child);

      if (std::abs(child->cost - old_cost) > 1e-6)
      {
        propagateCostToChildren(child); // recursive
      }
    }
  }

  double InformedRRT3D::normalizeAngle(double angle)
  {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  std::shared_ptr<Node> InformedRRT3D::steer(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to)
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

    // Interpolate yaw
    double dyaw = normalizeAngle(to->yaw - from->yaw);
    double yaw_step = std::min(dyaw, max_yaw_rate_ * (step_size_ / nominal_speed_));
    double new_yaw = normalizeAngle(from->yaw + yaw_step);

    return std::make_shared<Node>(new_x, new_y, new_z, new_yaw);
  }

  double InformedRRT3D::distance(const std::shared_ptr<Node> &a, const std::shared_ptr<Node> &b)
  {
    double dx = a->x - b->x;
    double dy = a->y - b->y;
    double dz = a->z - b->z;
    double dyaw = normalizeAngle(a->yaw - b->yaw);
    return std::sqrt(dx * dx + dy * dy + dz * dz) + yaw_weight_ * std::abs(dyaw);
  }

  bool InformedRRT3D::isFree2D(double x, double y)
  {
    if (!costmap_)
      return true;

    unsigned int mx, my;

    if (!costmap_->worldToMap(x, y, mx, my))
      return false; // Out of bounds

    unsigned char cost = costmap_->getCost(mx, my);

    // Threshold: 50 = moderately inflated; change if needed
    return cost < lethal_cost_threshold_;
  }

  bool InformedRRT3D::isFree3D(double x, double y, double z)
  {
    if (!octree_)
      return true;

    geometry_msgs::PointStamped input, transformed;
    input.header.stamp = ros::Time(0);                        // latest
    input.header.frame_id = costmap_ros_->getGlobalFrameID(); // usually "map"
    input.point.x = x;
    input.point.y = y;
    input.point.z = z;

    // Try transforming with retries
    bool success = false;
    const int max_retries = 3;
    const ros::Duration retry_delay(0.05);

    for (int attempt = 0; attempt < max_retries; ++attempt)
    {
      try
      {
        transformed = tf_buffer_.transform(input, octomap_frame_, ros::Duration(0.2));
        success = true;
        break;
      }
      catch (tf2::LookupException &ex)
      {
        ROS_WARN_THROTTLE(2.0, "TF LookupException (attempt %d): %s", attempt + 1, ex.what());
      }
      catch (tf2::ConnectivityException &ex)
      {
        ROS_WARN_THROTTLE(2.0, "TF ConnectivityException (attempt %d): %s", attempt + 1, ex.what());
      }
      catch (tf2::ExtrapolationException &ex)
      {
        ROS_WARN_THROTTLE(2.0, "TF ExtrapolationException (attempt %d): %s", attempt + 1, ex.what());
      }
      ros::Duration(retry_delay).sleep();
    }

    if (!success)
    {
      ROS_ERROR_THROTTLE(2.0, "TF transform to frame '%s' failed after %d attempts",
                         octomap_frame_.c_str(), max_retries);
      return false; // treat as occupied
    }

    std::vector<std::tuple<double, double, double>> offsets = {
        {0, 0, 0},
        {safety_radius_, 0, 0},
        {-safety_radius_, 0, 0},
        {0, safety_radius_, 0},
        {0, -safety_radius_, 0},
        {0, 0, safety_radius_},
        {0, 0, -safety_radius_}};

    for (const auto &[dx, dy, dz] : offsets)
    {
      octomap::point3d pt(transformed.point.x + dx,
                          transformed.point.y + dy,
                          transformed.point.z + dz);

      octomap::OcTreeNode *node = octree_->search(pt);

      if (node && octree_->isNodeOccupied(node))
        return false;
    }

    return true;
  }

  bool InformedRRT3D::isValid(const std::shared_ptr<Node> &node)
  {
    if (!isFree2D(node->x, node->y))
      return false;

    if (node->z < min_altitude_ || node->z > max_altitude_)
      return false;

    return isFree3D(node->x, node->y, node->z);
  }

  bool InformedRRT3D::collisionFree(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to)
  {
    int steps = static_cast<int>(distance(from, to) / (collision_check_resolution_));
    for (int i = 0; i <= steps; ++i)
    {
      double x = from->x + (to->x - from->x) * i / steps;
      double y = from->y + (to->y - from->y) * i / steps;
      double z = from->z + (to->z - from->z) * i / steps;
      if (!isFree3D(x, y, z))
        return false;
    }
    return true;
  }

  std::vector<std::shared_ptr<Node>> InformedRRT3D::extractPath(const std::shared_ptr<Node> &goal)
  {
    std::vector<std::shared_ptr<Node>> path;
    for (auto n = goal; n != nullptr; n = n->parent)
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
