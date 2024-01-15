#include "main.h"


void FastLioLocalizationQnClass::updateVisVars(const PosePcd& pose_pcd_in)
{
  m_odoms.points.emplace_back(pose_pcd_in.pose_eig(0, 3), pose_pcd_in.pose_eig(1, 3), pose_pcd_in.pose_eig(2, 3));
  m_corrected_odoms.points.emplace_back(pose_pcd_in.pose_corrected_eig(0, 3), pose_pcd_in.pose_corrected_eig(1, 3), pose_pcd_in.pose_corrected_eig(2, 3));
  m_odom_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_eig, m_map_frame));
  m_corrected_path.poses.push_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig, m_map_frame));
  return;
}

visualization_msgs::Marker FastLioLocalizationQnClass::getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>& match_xyz_pairs)
{
  visualization_msgs::Marker edges_; edges_.type = 5u;
  edges_.scale.x = 0.2f; edges_.header.frame_id = m_map_frame; edges_.pose.orientation.w = 1.0f;
  edges_.color.r = 1.0f; edges_.color.g = 1.0f; edges_.color.b = 1.0f; edges_.color.a = 1.0f;
  {
    for (int i = 0; i < match_xyz_pairs.size(); ++i)
    {
      geometry_msgs::Point p_, p2_;
      p_.x = match_xyz_pairs[i].first.x; p_.y = match_xyz_pairs[i].first.y; p_.z = match_xyz_pairs[i].first.z;
      p2_.x = match_xyz_pairs[i].second.x; p2_.y = match_xyz_pairs[i].second.y; p2_.z = match_xyz_pairs[i].second.z;
      edges_.points.push_back(p_);
      edges_.points.push_back(p2_);
    }
  }
  return edges_;
}

void FastLioLocalizationQnClass::voxelizePcd(pcl::VoxelGrid<PointType>& voxelgrid, pcl::PointCloud<PointType>& pcd_in)
{
  pcl::PointCloud<PointType>::Ptr before_(new pcl::PointCloud<PointType>);
  *before_ = pcd_in;
  voxelgrid.setInputCloud(before_);
  voxelgrid.filter(pcd_in);
  return;
}

bool FastLioLocalizationQnClass::checkIfKeyframe(const PosePcd& pose_pcd_in, const PosePcd& latest_pose_pcd)
{
  return m_keyframe_thr < (latest_pose_pcd.pose_corrected_eig.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig.block<3, 1>(0, 3)).norm();
}

int FastLioLocalizationQnClass::getClosestKeyframeIdx(const PosePcd& current_keyframe, const std::vector<PosePcdReduced>& saved_map)
{
  double shortest_distance_ = m_match_det_radi*3.0;
  int closest_idx_ = -1;
  for (int idx = 0; idx < saved_map.size()-1; ++idx)
  {
    //check if potential matchee: close enough in distance, then get the closest one
    double tmp_dist_ = (saved_map[idx].pose_eig.block<3, 1>(0, 3) - current_keyframe.pose_corrected_eig.block<3, 1>(0, 3)).norm();
    if (m_match_det_radi > tmp_dist_)
    {
      if (tmp_dist_ < shortest_distance_)
      {
        shortest_distance_ = tmp_dist_;
        closest_idx_ = saved_map[idx].idx;
      }
    }
  }
  return closest_idx_;
}

Eigen::Matrix4d FastLioLocalizationQnClass::icpKeyToSubkeys(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // merge subkeyframes before ICP
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  for (int i = closest_idx-m_sub_key_num; i < closest_idx+m_sub_key_num+1; ++i)
  {
    if (i>=0 && i < keyframes.size()-1) //if exists
    {
      dst_raw_ += transformPcd(keyframes[i].pcd, keyframes[i].pose_eig);
    }
  }
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then match with Nano-GICP
  pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
  *dst_ = dst_raw_;
  *src_ = src_raw_;
  pcl::PointCloud<PointType> aligned_;
  m_nano_gicp.setInputSource(src_);
  m_nano_gicp.calculateSourceCovariances();
  m_nano_gicp.setInputTarget(dst_);
  m_nano_gicp.calculateTargetCovariances();
  m_nano_gicp.align(aligned_);
  // vis for debug
  m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
  m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
  m_debug_fine_aligned_pub.publish(pclToPclRos(aligned_, m_map_frame));
  // handle results
  score = m_nano_gicp.getFitnessScore();
  if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
  {
    if_converged = true;
    output_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
  }
  return output_tf_;
}

Eigen::Matrix4d FastLioLocalizationQnClass::coarseToFineKeyToKey(const PosePcd& current_keyframe, const int& closest_idx, const std::vector<PosePcdReduced>& keyframes, bool& if_converged, double& score)
{
  Eigen::Matrix4d output_tf_ = Eigen::Matrix4d::Identity();
  if_converged = false;
  // Prepare the keyframes
  pcl::PointCloud<PointType> dst_raw_, src_raw_;
  src_raw_ = transformPcd(current_keyframe.pcd, current_keyframe.pose_corrected_eig);
  dst_raw_ = transformPcd(keyframes[closest_idx].pcd, keyframes[closest_idx].pose_eig); //Note: Quatro should work on scan-to-scan (keyframe-to-keyframe), not keyframe-to-merged-many-keyframes
  // voxlize pcd
  voxelizePcd(m_voxelgrid, dst_raw_);
  voxelizePcd(m_voxelgrid, src_raw_);
  // then perform Quatro
  Eigen::Matrix4d quatro_tf_ = m_quatro_handler->align(src_raw_, dst_raw_, if_converged);
  if (!if_converged) return quatro_tf_;
  else //if valid,
  {
    // coarse align with the result of Quatro
    pcl::PointCloud<PointType> src_coarse_aligned_ = transformPcd(src_raw_, quatro_tf_);
    // then match with Nano-GICP
    pcl::PointCloud<PointType> fine_aligned_;
    pcl::PointCloud<PointType>::Ptr src_(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr dst_(new pcl::PointCloud<PointType>);
    *dst_ = dst_raw_;
    *src_ = src_coarse_aligned_;
    m_nano_gicp.setInputSource(src_);
    m_nano_gicp.calculateSourceCovariances();
    m_nano_gicp.setInputTarget(dst_);
    m_nano_gicp.calculateTargetCovariances();
    m_nano_gicp.align(fine_aligned_);
    // handle results
    score = m_nano_gicp.getFitnessScore();
    if(m_nano_gicp.hasConverged() && score < m_icp_score_thr) // if matchness score is lower than threshold, (lower is better)
    {
      if_converged = true;
      Eigen::Matrix4d icp_tf_ = m_nano_gicp.getFinalTransformation().cast<double>();
      output_tf_ = icp_tf_ * quatro_tf_; // IMPORTANT: take care of the order
    }
    else if_converged = false;
    // vis for debug
    m_debug_src_pub.publish(pclToPclRos(src_raw_, m_map_frame));
    m_debug_dst_pub.publish(pclToPclRos(dst_raw_, m_map_frame));
    m_debug_coarse_aligned_pub.publish(pclToPclRos(src_coarse_aligned_, m_map_frame));
    m_debug_fine_aligned_pub.publish(pclToPclRos(fine_aligned_, m_map_frame));
  }

  return output_tf_;
}

void FastLioLocalizationQnClass::loadMap(const std::string& saved_map_path)
{
  rosbag::Bag bag_;
  bag_.open(saved_map_path, rosbag::bagmode::Read);
  rosbag::View view1_(bag_, rosbag::TopicQuery("/keyframe_pcd"));
  rosbag::View view2_(bag_, rosbag::TopicQuery("/keyframe_pose"));
  std::vector<sensor_msgs::PointCloud2> load_pcd_vec_;
  std::vector<geometry_msgs::PoseStamped> load_pose_vec_;
  for (const rosbag::MessageInstance& pcd_msg_ : view1_)
  {
    sensor_msgs::PointCloud2::ConstPtr pcd_msg_ptr_ = pcd_msg_.instantiate<sensor_msgs::PointCloud2>();
    if (pcd_msg_ptr_ != nullptr)
    {
      load_pcd_vec_.push_back(*pcd_msg_ptr_);
    }
  }
  for (const rosbag::MessageInstance& pose_msg_ : view2_)
  {
    geometry_msgs::PoseStamped::ConstPtr pose_msg_ptr_ = pose_msg_.instantiate<geometry_msgs::PoseStamped>();
    if (pose_msg_ptr_ != nullptr)
    {
      load_pose_vec_.push_back(*pose_msg_ptr_);
    }
  }
  if (load_pcd_vec_.size() != load_pose_vec_.size()) ROS_ERROR("WRONG BAG FILE!!!!!");
  for (int i = 0; i < load_pose_vec_.size(); ++i)
  {
    m_saved_map.push_back(PosePcdReduced(load_pose_vec_[i], load_pcd_vec_[i], i));
    m_saved_map_pcd += transformPcd(m_saved_map[i].pcd, m_saved_map[i].pose_eig);
  }
  voxelizePcd(m_voxelgrid, m_saved_map_pcd);
  bag_.close();
  return;
}