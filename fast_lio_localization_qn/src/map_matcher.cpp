#include "map_matcher.h"

MapMatcher::MapMatcher(const MapMatcherConfig &config)
{
    config_ = config;
    const auto &gc = config_.gicp_config_;
    const auto &qc = config_.quatro_config_;
    ////// nano_gicp init
    nano_gicp_.setNumThreads(gc.nano_thread_number_);
    nano_gicp_.setCorrespondenceRandomness(gc.nano_correspondences_number_);
    nano_gicp_.setMaximumIterations(gc.nano_max_iter_);
    nano_gicp_.setRANSACIterations(gc.nano_ransac_max_iter_);
    nano_gicp_.setMaxCorrespondenceDistance(gc.max_corr_dist_);
    nano_gicp_.setTransformationEpsilon(gc.transformation_epsilon_);
    nano_gicp_.setEuclideanFitnessEpsilon(gc.euclidean_fitness_epsilon_);
    nano_gicp_.setRANSACOutlierRejectionThreshold(gc.ransac_outlier_rejection_threshold_);
    ////// quatro init
    quatro_handler_ = std::make_shared<quatro<PointType>>(qc.fpfh_normal_radius_,
                                                          qc.fpfh_radius_,
                                                          qc.noise_bound_,
                                                          qc.rot_gnc_factor_,
                                                          qc.rot_cost_diff_thr_,
                                                          qc.quatro_max_iter_,
                                                          qc.estimat_scale_,
                                                          qc.use_optimized_matching_,
                                                          qc.quatro_distance_threshold_,
                                                          qc.quatro_max_num_corres_);
    src_cloud_.reset(new pcl::PointCloud<PointType>);
    dst_cloud_.reset(new pcl::PointCloud<PointType>);
}

MapMatcher::~MapMatcher() {}

int MapMatcher::fetchClosestKeyframeIdx(const PosePcd &front_keyframe,
                                        const std::vector<PosePcdReduced> &saved_map)
{
    const auto &loop_det_radi = config_.loop_detection_radius_;
    double shortest_distance = loop_det_radi * 3.0;
    int closest_idx = -1;
    for (size_t idx = 0; idx < saved_map.size() - 1; ++idx)
    {
        // check if potential loop: close enough in distance, far enough in time
        double tmp_dist = (saved_map[idx].pose_eig_.block<3, 1>(0, 3) - front_keyframe.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
        if (loop_det_radi > tmp_dist)
        {
            if (tmp_dist < shortest_distance)
            {
                shortest_distance = tmp_dist;
                closest_idx = saved_map[idx].idx_;
            }
        }
    }
    return closest_idx;
}

PcdPair MapMatcher::setSrcAndDstCloud(const std::vector<PosePcdReduced> &saved_keyframes,
                                      const int src_idx,
                                      const int dst_idx,
                                      const int submap_range,
                                      const double voxel_res,
                                      const bool enable_quatro,
                                      const bool enable_submap_matching)
{
    pcl::PointCloud<PointType> dst_accum, src_accum;
    int num_approx = saved_keyframes[src_idx].pcd_.size() * 2 * submap_range;
    src_accum.reserve(num_approx);
    dst_accum.reserve(num_approx);
    if (enable_submap_matching)
    {
        for (int i = src_idx - submap_range; i < src_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(saved_keyframes.size() - 1))
            {
                src_accum += transformPcd(saved_keyframes[i].pcd_, saved_keyframes[i].pose_eig_);
            }
        }
        for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
        {
            if (i >= 0 && i < static_cast<int>(saved_keyframes.size() - 1))
            {
                dst_accum += transformPcd(saved_keyframes[i].pcd_, saved_keyframes[i].pose_eig_);
            }
        }
    }
    else
    {
        src_accum = transformPcd(saved_keyframes[src_idx].pcd_, saved_keyframes[src_idx].pose_eig_);
        if (enable_quatro)
        {
            dst_accum = transformPcd(saved_keyframes[dst_idx].pcd_, saved_keyframes[dst_idx].pose_eig_);
        }
        else
        {
            // For ICP matching,
            // empirically scan-to-submap matching works better
            for (int i = dst_idx - submap_range; i < dst_idx + submap_range + 1; ++i)
            {
                if (i >= 0 && i < static_cast<int>(saved_keyframes.size() - 1))
                {
                    dst_accum += transformPcd(saved_keyframes[i].pcd_, saved_keyframes[i].pose_eig_);
                }
            }
        }
    }
    return {*voxelizePcd(src_accum, voxel_res), *voxelizePcd(dst_accum, voxel_res)};
}

RegistrationOutput
MapMatcher::icpAlignment(const pcl::PointCloud<PointType> &src,
                         const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    aligned_.clear();
    // merge subkeyframes before ICP
    pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr dst_cloud(new pcl::PointCloud<PointType>());
    *src_cloud = src;
    *dst_cloud = dst;
    nano_gicp_.setInputSource(src_cloud);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_cloud);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    // handle results
    reg_output.score_ = nano_gicp_.getFitnessScore();
    // if matchness score is lower than threshold, (lower is better)
    if (nano_gicp_.hasConverged() && reg_output.score_ < config_.gicp_config_.icp_score_thr_)
    {
        reg_output.is_valid_ = true;
        reg_output.is_converged_ = true;
        reg_output.pose_between_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
    }
    return reg_output;
}

RegistrationOutput
MapMatcher::coarseToFineAlignment(const pcl::PointCloud<PointType> &src,
                                  const pcl::PointCloud<PointType> &dst)
{
    RegistrationOutput reg_output;
    coarse_aligned_.clear();

    reg_output.pose_between_eig_ = (quatro_handler_->align(src, dst, reg_output.is_converged_));
    if (!reg_output.is_converged_)
    {
        return reg_output;
    }
    else // if valid,
    {
        // coarse align with the result of Quatro
        coarse_aligned_ = transformPcd(src, reg_output.pose_between_eig_);
        const auto &fine_output = icpAlignment(coarse_aligned_, dst);
        const auto quatro_tf_ = reg_output.pose_between_eig_;
        reg_output = fine_output;
        reg_output.pose_between_eig_ = fine_output.pose_between_eig_ * quatro_tf_;
    }
    return reg_output;
}

RegistrationOutput MapMatcher::performMapMatcher(const PosePcd &query_keyframe,
                                                 const std::vector<PosePcdReduced> &saved_keyframes,
                                                 const int closest_keyframe_idx)
{
    RegistrationOutput reg_output;
    closest_keyframe_idx_ = closest_keyframe_idx;
    if (closest_keyframe_idx_ >= 0)
    {
        // Quatro + NANO-GICP to check loop (from front_keyframe to closest
        // keyframe's neighbor)
        const auto &[src_cloud, dst_cloud] = setSrcAndDstCloud(saved_keyframes,
                                                               query_keyframe.idx_,
                                                               closest_keyframe_idx_,
                                                               config_.num_submap_keyframes_,
                                                               config_.voxel_res_,
                                                               config_.enable_quatro_,
                                                               config_.enable_submap_matching_);
        // Only for visualization
        *src_cloud_ = src_cloud;
        *dst_cloud_ = dst_cloud;

        if (config_.enable_quatro_)
        {
            std::cout << "\033[1;35mExecute coarse-to-fine alignment: "
                      << src_cloud.size() << " vs " << dst_cloud.size()
                      << "\033[0m\n";
            return coarseToFineAlignment(src_cloud, dst_cloud);
        }
        else
        {
            std::cout << "\033[1;35mExecute GICP: " << src_cloud.size() << " vs "
                      << dst_cloud.size() << "\033[0m\n";
            return icpAlignment(src_cloud, dst_cloud);
        }
    }
    else
    {
        return reg_output; // dummy output whose `is_valid` is false
    }
}

pcl::PointCloud<PointType> MapMatcher::getSourceCloud()
{
    return *src_cloud_;
}

pcl::PointCloud<PointType> MapMatcher::getTargetCloud()
{
    return *dst_cloud_;
}

pcl::PointCloud<PointType> MapMatcher::getCoarseAlignedCloud()
{
    return coarse_aligned_;
}

// NOTE(hlim): To cover ICP-only mode, I just set `Final`, not `Fine`
pcl::PointCloud<PointType> MapMatcher::getFinalAlignedCloud()
{
    return aligned_;
}

int MapMatcher::getClosestKeyframeidx()
{
    return closest_keyframe_idx_;
}
