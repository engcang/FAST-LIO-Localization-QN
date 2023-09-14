#include "main.h"


void FAST_LIO_LOCALIZATION_QN_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2ConstPtr& pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_;
  last_odom_tf_ = m_current_frame.pose_eig; //to calculate delta
  m_current_frame = pose_pcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::PoseStamped current_pose_stamped_ = pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
      m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    }
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(tf_pcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));
    // 2. save first keyframe
    {
      lock_guard<mutex> lock(m_keyframes_mutex);
      m_keyframes.push_back(m_current_frame);
      m_not_processed_keyframe = m_current_frame; //to check match in another thread
    }
    m_current_keyframe_idx++;
    //// 3. vis
    {
      lock_guard<mutex> lock(m_vis_mutex);
      update_vis_vars(m_current_frame);
    }
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      geometry_msgs::PoseStamped current_pose_stamped_ = pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame);
      m_realtime_pose_pub.publish(current_pose_stamped_);
      tf::Transform transform_;
      transform_.setOrigin(tf::Vector3(current_pose_stamped_.pose.position.x, current_pose_stamped_.pose.position.y, current_pose_stamped_.pose.position.z));
      transform_.setRotation(tf::Quaternion(current_pose_stamped_.pose.orientation.x, current_pose_stamped_.pose.orientation.y, current_pose_stamped_.pose.orientation.z, current_pose_stamped_.pose.orientation.w));
      m_broadcaster.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), m_map_frame, "robot"));
    }
    // pub current scan in corrected pose frame
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(tf_pcd(m_current_frame.pcd, m_current_frame.pose_corrected_eig), m_map_frame));

    //// 2. check if keyframe
    if (check_if_keyframe(m_current_frame, m_keyframes.back()))
    {
      // 2-2. if so, save
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_keyframes.push_back(m_current_frame);
        m_not_processed_keyframe = m_current_frame; //to check match in another thread
      }
      m_current_keyframe_idx++;

      //// 3. vis
      {
        lock_guard<mutex> lock(m_vis_mutex);
        update_vis_vars(m_current_frame);
      }
    }
  }
  return;
}

void FAST_LIO_LOCALIZATION_QN_CLASS::matching_timer_func(const ros::TimerEvent& event)
{
  if (!m_init) return;

  //// 1. copy not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  pose_pcd not_proc_key_copy_;
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    not_proc_key_copy_ = m_not_processed_keyframe;
    m_not_processed_keyframe.processed = true;
  }
  if (m_not_processed_keyframe.idx==0 || not_proc_key_copy_.processed) return; //already processed or initial keyframe

  //// 2. detect match and calculate TF
  // from not_proc_key_copy_ keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
  int closest_keyframe_idx_ = get_closest_keyframe_idx(not_proc_key_copy_, m_saved_map);
  if (closest_keyframe_idx_ >= 0) //if exists
  {
    // Quatro + NANO-GICP to check match (from current_keyframe to closest keyframe in saved map)
    bool converged_well_ = false;
    double score_;
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
    if (m_enable_quatro) pose_between_eig_ = coarse_to_fine_key_to_key(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);
    else pose_between_eig_ = icp_key_to_subkeys(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);

    if(converged_well_) // TF the pose with the result of match
    {
      //// 3. handle corrected results
      Eigen::Matrix4d TFed_pose_ = pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig;
      // get TFed pose and reset odom delta (for realtime pose pub)
      {
        lock_guard<mutex> lock(m_realtime_pose_mutex);
        m_last_corrected_pose = TFed_pose_;
        m_odom_delta = Eigen::Matrix4d::Identity();
      }
      // correct poses in keyframes
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        m_keyframes[not_proc_key_copy_.idx].pose_corrected_eig = TFed_pose_;
      }
      // correct poses in vis data
      {
        lock_guard<mutex> lock(m_vis_mutex);
        m_corrected_odoms.points[not_proc_key_copy_.idx] = pcl::PointXYZ(TFed_pose_(0, 3), TFed_pose_(1, 3), TFed_pose_(2, 3));
        m_corrected_path.poses[not_proc_key_copy_.idx] = pose_eig_to_pose_stamped(TFed_pose_, m_map_frame);
        m_match_idx_pairs.push_back({not_proc_key_copy_.idx, closest_keyframe_idx_}); //for vis
      }
      m_match_flag_vis = true;
    }
  }
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  ROS_INFO("Matching: %.1fms", duration_cast<microseconds>(t2_-t1_).count()/1e3);
  return;
}

void FAST_LIO_LOCALIZATION_QN_CLASS::vis_timer_func(const ros::TimerEvent& event)
{
  if (!m_init) return;

  high_resolution_clock::time_point tv1_ = high_resolution_clock::now();
  // publish corrected odoms, paths
  {
    lock_guard<mutex> lock(m_vis_mutex);
    m_corrected_odom_pub.publish(pcl_to_pcl_ros(m_corrected_odoms, m_map_frame));
    m_corrected_path_pub.publish(m_corrected_path);
  }
  // publish others
  m_odom_pub.publish(pcl_to_pcl_ros(m_odoms, m_map_frame));
  m_path_pub.publish(m_odom_path);
  // map matches
  if (m_match_flag_vis)
  {
    if (!m_match_idx_pairs.empty())
    {
      m_map_match_pub.publish(get_match_markers());
    }
    m_match_flag_vis = false;      
  }
  if (m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() > 0)
  {
    m_saved_map_pub.publish(pcl_to_pcl_ros(m_saved_map_pcd, m_map_frame));
    m_saved_map_vis_switch = false;
  }
  if (!m_saved_map_vis_switch && m_saved_map_pub.getNumSubscribers() == 0)
  {
    m_saved_map_vis_switch = true;
  }
  high_resolution_clock::time_point tv2_ = high_resolution_clock::now();
  ROS_INFO("vis: %.1fms", duration_cast<microseconds>(tv2_-tv1_).count()/1e3);
  return;
}
