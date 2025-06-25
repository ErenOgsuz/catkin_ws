// File: include/rrt3d_planner/informed_rrt3d.h
#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <memory>
#include <random>

namespace rrt3d_planner
{

    struct Node
    {
        double x, y, z, yaw;
        double yaw;
        double cost;
        std::shared_ptr<Node> parent;
        std::vector<std::shared_ptr<Node>> children;

        Node(double x_, double y_, double z_ = 0.0, double yaw_ = 0.0)
            : x(x_), y(y_), z(z_), yaw(yaw_), cost(0.0), parent(nullptr) {}
    };

    class InformedRRT3D : public nav_core::BaseGlobalPlanner
    {
    public:
        InformedRRT3D();
        InformedRRT3D(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) override;
        bool makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan) override;

    private:
        ros::NodeHandle nh_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;

        std::shared_ptr<octomap::OcTree> octree_;
        ros::Subscriber octomap_sub_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        ros::Publisher path_pub_;
        ros::Publisher tree_pub_;

        std::vector<std::shared_ptr<Node>> nodes_;

        double step_size_;
        double goal_tolerance_;
        double rewire_radius_;
        bool informed_sampling_;
        double max_altitude_;
        double min_altitude_;
        double map_resolution_;
        double safety_radius_;

        std::default_random_engine rng_;

        void octomapCallback(const octomap_msgs::Octomap &msg);
        bool isFree3D(double x, double y, double z);
        std::shared_ptr<Node> steer(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to);
        bool collisionFree(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to);
        double distance(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2);
        void visualizeTree();
        nav_msgs::Path createPathMsg(const std::vector<std::shared_ptr<Node>> &path, const std::string &frame_id);
        std::vector<std::shared_ptr<Node>> extractPath(const std::shared_ptr<Node> &goal);
    };

} // namespace rrt3d_planner