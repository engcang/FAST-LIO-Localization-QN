#ifndef FAST_LIO_LOCALIZATION_QN_MAIN_H
#define FAST_LIO_LOCALIZATION_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <vector>
#include <mutex>
#include <string>
#include <memory>
#include <utility> // pair, make_pair
///// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>               // load map
#include <rosbag/view.h>              // load map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// PCL
#include <pcl/point_types.h>                 //pt
#include <pcl/point_cloud.h>                 //cloud
#include <pcl/common/transforms.h>           //transformPointCloud
#include <pcl/conversions.h>                 //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/filters/voxel_grid.h>          //voxelgrid
///// Nano-GICP
#include <nano_gicp/point_type_nano_gicp.hpp>
#include <nano_gicp/nano_gicp.hpp>
///// Quatro
#include <quatro/quatro_module.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// coded headers
#include "utilities.hpp"
#include "pose_pcd.hpp"
#include "map_matcher.h"

using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioLocalizationQn
{
private:
    ///// basic params
    std::string map_frame_;
    ///// shared data - odom and pcd
    std::mutex keyframes_mutex_, vis_mutex_;
    bool is_initialized_ = false;
    int current_keyframe_idx_ = 0;
    PosePcd current_frame_, last_keyframe_, not_processed_latest_keyframe_;
    std::vector<PosePcdReduced> saved_map_from_bag_;
    Eigen::Matrix4d last_corrected_TF_ = Eigen::Matrix4d::Identity();
    ///// map match
    double keyframe_dist_thr_;
    double voxel_res_;
    ///// visualize
    bool saved_map_vis_switch_ = true;
    tf::TransformBroadcaster broadcaster_;
    nav_msgs::Path raw_odom_path_, corrected_odom_path_;
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> matched_pairs_xyz_; // for vis
    pcl::PointCloud<pcl::PointXYZ> raw_odoms_, corrected_odoms_;
    pcl::PointCloud<PointType> saved_map_pcd_; // for vis
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, realtime_pose_pub_, map_match_pub_;
    ros::Publisher saved_map_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    ros::Timer match_timer_;
    // odom, pcd sync subscriber
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    ///// Map match
    std::shared_ptr<MapMatcher> map_matcher_;

public:
    explicit FastLioLocalizationQn(const ros::NodeHandle &n_private);
    ~FastLioLocalizationQn() {};

private:
    // methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> &match_xyz_pairs);
    void loadMap(const std::string &saved_map_path);
    // cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void matchingTimerFunc(const ros::TimerEvent &event);
};

#endif
