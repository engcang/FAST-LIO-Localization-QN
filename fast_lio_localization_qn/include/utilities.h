#ifndef FAST_LIO_LOCALIZATION_QN_UTILITY_H
#define FAST_LIO_LOCALIZATION_QN_UTILITY_H

///// common headers
#include <string>
///// ROS
#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler
#include <tf/transform_datatypes.h> // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h> // tf <-> eigen
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
///// PCL
#include <pcl/point_types.h> //pt
#include <pcl/point_cloud.h> //cloud
#include <pcl/conversions.h> //ros<->pcl
#include <pcl_conversions/pcl_conversions.h> //ros<->pcl
#include <pcl/common/transforms.h>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library: Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)


//////////////////////////////////////////////////////////////////////
///// conversions
geometry_msgs::PoseStamped poseEigToPoseStamped(const Eigen::Matrix4d& pose_eig_in, std::string frame_id="map")
{
	double r_, p_, y_;
	tf::Matrix3x3 mat_;
	tf::matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat_);
	mat_.getRPY(r_, p_, y_);
	tf::Quaternion quat_ = tf::createQuaternionFromRPY(r_, p_, y_);
	geometry_msgs::PoseStamped pose_;
	pose_.header.frame_id = frame_id;
	pose_.pose.position.x = pose_eig_in(0, 3);
	pose_.pose.position.y = pose_eig_in(1, 3);
	pose_.pose.position.z = pose_eig_in(2, 3);
	pose_.pose.orientation.w = quat_.getW();
	pose_.pose.orientation.x = quat_.getX();
	pose_.pose.orientation.y = quat_.getY();
	pose_.pose.orientation.z = quat_.getZ();
	return pose_;
}
template <typename T>
sensor_msgs::PointCloud2 pclToPclRos(pcl::PointCloud<T> cloud, std::string frame_id="map")
{
	sensor_msgs::PointCloud2 cloud_ROS_;
	pcl::toROSMsg(cloud, cloud_ROS_);
	cloud_ROS_.header.frame_id = frame_id;
	return cloud_ROS_;
}
///// transformation
template <typename T>
pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T>& cloud_in, const Eigen::Matrix4d &pose_tf)
{
	if (cloud_in.size() == 0) return cloud_in;
	pcl::PointCloud<T> pcl_out_ = cloud_in;
	pcl::transformPointCloud(cloud_in, pcl_out_, pose_tf);
	return pcl_out_;
}



#endif