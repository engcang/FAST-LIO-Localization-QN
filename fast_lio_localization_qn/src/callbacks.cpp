#include "main.h"


void FAST_LIO_LOCALIZATION_QN_CLASS::odom_pcd_cb(const nav_msgs::OdometryConstPtr &odom_msg, const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
  Eigen::Matrix4d last_odom_tf_;
  last_odom_tf_ = m_current_frame.pose_eig; //to calculate delta
  m_current_frame = pose_pcd(*odom_msg, *pcd_msg, m_current_keyframe_idx); //to be checked if keyframe or not

  if (!m_init) //// init only once
  {
    m_keyframes.push_back(m_current_frame);
    update_vis_vars(m_current_frame);
    m_corrected_current_pcd_pub.publish(pcl_to_pcl_ros(m_current_frame.pcd, m_map_frame));
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      m_realtime_pose_pub.publish(pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame));
    }
    m_current_keyframe_idx++;
    m_init = true;
  }
  else
  {
    //// 1. realtime pose = last corrected odom * delta (last -> current)
    {
      lock_guard<mutex> lock(m_realtime_pose_mutex);
      m_odom_delta = m_odom_delta * last_odom_tf_.inverse() * m_current_frame.pose_eig;
      m_current_frame.pose_corrected_eig = m_last_corrected_pose * m_odom_delta;
      m_realtime_pose_pub.publish(pose_eig_to_pose_stamped(m_current_frame.pose_corrected_eig, m_map_frame));
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

  //// 1. copy keyframes and not processed keyframes
  high_resolution_clock::time_point t1_ = high_resolution_clock::now();
  pose_pcd not_proc_key_copy_;
  {
    lock_guard<mutex> lock(m_keyframes_mutex);
    not_proc_key_copy_ = m_not_processed_keyframe;
    m_not_processed_keyframe.processed = true;
  }
  if (not_proc_key_copy_.processed) return; //already processed

  //// 2. detect loop and add to graph
  bool if_matched_ = false;
  // from not_proc_key_copy_ keyframe to map (saved keyframes) in threshold radius, get the closest keyframe
  int closest_keyframe_idx_ = get_closest_keyframe_idx(not_proc_key_copy_, m_saved_map);
  if (closest_keyframe_idx_ >= 0) //if exists
  {
    // Quatro + NANO-GICP to check loop (from front_keyframe to closest keyframe's neighbor)
    bool converged_well_ = false;
    double score_;
    Eigen::Matrix4d pose_between_eig_ = Eigen::Matrix4d::Identity();
    if (m_enable_quatro) pose_between_eig_ = coarse_to_fine_key_to_subkeys(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);
    else pose_between_eig_ = icp_key_to_subkeys(not_proc_key_copy_, closest_keyframe_idx_, m_saved_map, converged_well_, score_);

    if(converged_well_) // correct TF
    {
      pose_between_eig_ * not_proc_key_copy_.pose_corrected_eig // from
      m_saved_map[closest_keyframe_idx_].pose_corrected_eig // to
        m_gtsam_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(not_proc_key_copy_.idx, closest_keyframe_idx_, pose_from_.between(pose_to_), loop_noise_));
      m_loop_idx_pairs.push_back({not_proc_key_copy_.idx, closest_keyframe_idx_}); //for vis

      //// 3. handle corrected results
      // get corrected poses and reset odom delta (for realtime pose pub)
      {
        lock_guard<mutex> lock(m_realtime_pose_mutex);
        m_last_corrected_pose = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(m_corrected_esti.size()-1));
        m_odom_delta = Eigen::Matrix4d::Identity();
      }
      // correct poses in keyframes
      {
        lock_guard<mutex> lock(m_keyframes_mutex);
        for (int i = 0; i < m_corrected_esti.size(); ++i)
        {
          m_keyframes[i].pose_corrected_eig = gtsam_pose_to_pose_eig(m_corrected_esti.at<gtsam::Pose3>(i));
        }
        if_matched_ = false;
      }
      // correct poses in vis data
      {
        lock_guard<mutex> lock(m_vis_mutex);
        m_corrected_odoms = corrected_odoms_;
        m_corrected_path.poses = corrected_path_.poses;
      }
      m_match_flag_vis = true;
    }
  }
  high_resolution_clock::time_point t2_ = high_resolution_clock::now();
  ROS_INFO("Matching: %.1f", duration_cast<microseconds>(t2_-t1_).count()/1e3);
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
    if (!m_loop_idx_pairs.empty())
    {
      m_map_match_pub.publish(get_loop_markers(corrected_esti_copy_));
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
