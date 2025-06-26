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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <vector>
#include <memory>
#include "nanoflann.hpp"
#include <random>

namespace rrt3d_planner
{
    struct PointCloud
    {
        const std::vector<std::shared_ptr<Node>> *nodes;

        inline size_t kdtree_get_point_count() const
        {
            return nodes->size();
        }

        inline double kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            const auto &n = (*nodes)[idx];
            if (dim == 0)
                return n->x;
            else if (dim == 1)
                return n->y;
            else
                return n->z;
        }

        // Not implementing bounding box
        template <class BBOX>
        bool kdtree_get_bbox(BBOX &) const { return false; }
    };

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
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, PointCloud>,
            PointCloud,
            3 // dimensionality: x, y, z
            >;

        PointCloud point_cloud_;          // Wrapper around nodes_
        std::unique_ptr<KDTree> kd_tree_; // The actual KD-tree

        size_t last_kdtree_build_size_ = 0;    // Track when to rebuild
        size_t kd_tree_rebuild_interval_ = 50; // Rebuild every N new nodes

        ros::NodeHandle nh_;
        costmap_2d::Costmap2DROS *costmap_ros_;
        costmap_2d::Costmap2D *costmap_;

        std::shared_ptr<octomap::OcTree> octree_;
        ros::Subscriber octomap_sub_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        ros::Subscriber cloud_sub_;

        ros::Publisher path_pub_;
        ros::Publisher tree_pub_;

        std::vector<std::shared_ptr<Node>> nodes_;

        std::string octomap_frame_;

        double step_size_;
        double goal_tolerance_;
        double rewire_radius_;
        bool informed_sampling_;
        double max_altitude_;
        double min_altitude_;
        double map_resolution_;
        double safety_radius_;
        double best_cost_;
        bool adaptive_radius_;
        double safety_radius_;
        int lethal_cost_threshold_;

        std::default_random_engine rng_;

        void octomapCallback(const octomap_msgs::Octomap &msg);
        bool isFree3D(double x, double y, double z);
        std::shared_ptr<Node> steer(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to);
        bool collisionFree(const std::shared_ptr<Node> &from, const std::shared_ptr<Node> &to);
        double distance(const std::shared_ptr<Node> &n1, const std::shared_ptr<Node> &n2);
        void visualizeTree();
        double getRewireRadius() const;
        void rewireTree(const std::shared_ptr<Node> &new_node,
                        const std::vector<std::shared_ptr<Node>> &near_nodes);
        void propagateCostToChildren(const std::shared_ptr<Node> &node);
        void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

        std::vector<std::shared_ptr<Node>> smoothPath(const std::vector<std::shared_ptr<Node>> &path);
        nav_msgs::Path createPathMsg(const std::vector<std::shared_ptr<Node>> &path, const std::string &frame_id);
        std::vector<std::shared_ptr<Node>> extractPath(const std::shared_ptr<Node> &goal);
    };

} // namespace rrt3d_planner