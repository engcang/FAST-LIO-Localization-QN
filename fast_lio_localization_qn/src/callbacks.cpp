#include "main.h"


void FastLioLocalizationQnClass::odomPcdCallback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2ConstPtr& pcd_msg)
{
  m_current_frame = PosePcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    //// 1. realtime pose = last TF * odom
    geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
    m_realtime_pose_pub.publish(current_pose_stamped_);
    tf::Transform transform_;
    transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
    transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
    m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub.publish(pclToPclRos(transformPcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));
    // 2. save first keyframe
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      m_last_keyframe = m_current_frame;
      m_not_processed_keyframe = m_current_frame; //to check match in another thread
    }
    m_current_keyframe_idx++;
    //// 3. vis
    {
      lock_guard<mutex> lock(m_vis_mutex);
      updateVisVars(m_current_frame);
    }
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last TF * odom
    m_current_frame.pose_corrected_eig = m_last_TF * m_current_frame.pose_eig;
    geometry_msgs::PoseStamped current_pose_stamped_ = poseEigToPoseStamped(m_current_frame.pose_corrected_eig, m_map_frame);
    m_realtime_pose_pub.publish(current_pose_stamped_);
    tf::Transform transform_;
    transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
    transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
    m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub.publish(pclToPclRos(transformPcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));

    //// 2. check if keyframe
    if (checkIfKeyframe(m_current_frame, m_last_keyframe))
    {
      // 2-2. if so, save
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_last_keyframe = m_current_frame;
        m_not_processed_keyframe = m_current_frame; //to check match in another thread
      }
      m_current_keyframe_idx++;
      //// 3. vis
      {
        lock_guard<mutex> lock(m_vis_mutex);
        updateVisVars(m_current_frame);
      }
    }
  }
  return;
}

void FastLioLocalizationQnClass::matchingTimerFunc(const ros::TimerEvent& event)
{
  if (!m_init) return;

  //// 1. copy not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  PosePcd not_proc_key_copy_;
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    not_proc_key_copy_ = m_not_processed_keyframe;
    m_not_processed_keyframe.processed = true;
  }
  if (not_proc_key_copy_.idx==0 || not_proc_key_copy_.processed) return; //already processed or initial keyframe

  //// 2. detect match and calculate TF
  // from not_proc_key_copy_ keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
  int closest_keyframe_idx_ = getClosestKeyframeIdx(not_proc_key_copy_, m_saved_map);
  if (closest_keyframe_idx_ >= 0) //if exists
  {
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    bool converged_well_ = false;
    double score_;
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
    if (m_enable_quatro) pose_between_eig_ = coarseToFineKeyToKey(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);
    else pose_between_eig_ = icpKeyToSubkeys(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);

    if(converged_well_) // TF the pose with the result of match
    {
      //// 3. handle corrected results
      m_last_TF = pose_between_eig_ * m_last_TF; // update TF
      Eigen::Matrix4d TFed_pose_ = pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig;
      // correct poses in vis data
      {
        lock_guard<mutex> lock(m_vis_mutex);
        m_corrected_odoms.points[not_proc_key_copy_.idx] = pcl::PointXYZ(TFed_pose_(0, 3), TFed_pose_(1, 3), TFed_pose_(2, 3));
        m_corrected_path.poses[not_proc_key_copy_.idx] = poseEigToPoseStamped(TFed_pose_, m_map_frame);
      }
      // map matches
      m_match_xyz_pairs.push_back({m_corrected_odoms.points[not_proc_key_copy_.idx], m_odoms.points[not_proc_key_copy_.idx]}); //for vis
      m_map_match_pub.publish(getMatchMarker(m_match_xyz_pairs));
    }
  }
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();

  // publish corrected odoms, paths
  {
    lock_guard<mutex> lock(m_vis_mutex);
    m_corrected_odom_pub.publish(pclToPclRos(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
  }
  // publish others
  m_odom_pub.publish(pclToPclRos(m_odoms, m_map_frame));
  m_path_pub.publish(m_odom_path);
  if (m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() > 0)
  {
    m_saved_map_pub.publish(pclToPclRos(m_saved_map_pcd, m_map_frame));
    m_saved_map_vis_switch = false;
  }
  if (!m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() == 0)
  {
    m_saved_map_vis_switch = true;
  }
  high_resolution_clock::time_point t3_ = high_resolution_clock::now();
  ROS_INFO("Matching: %.1fms, vis: %.1fms", duration_cast<microseconds>(t2_-t1_).count()/1e3, 
                                            duration_cast<microseconds>(t3_-t2_).count()/1e3);
  return;
}