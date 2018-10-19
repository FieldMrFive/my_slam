//
// Created by Kuangye Chen on 9/27/18.
//

#pragma once

#include <memory>
#include <Map.h>
#include <Frame.h>
#include <Config.h>
#include <opencv2/features2d.hpp>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

namespace my_slam
{

class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;

    enum State
    {
        INITIALIZING = -1,
        OK = 1,
        LOST = 0,
    };

    explicit VisualOdometry(const Config& config);

    bool AddFrame(Frame::Ptr frame);

private:
    void AddKeyFrame();
    void ExtractFeaturePoints();
    void ComputeDescriptors();
    void FeatureMatching();
    void PoseEstimationPnP();
    void SetRef3DPoints();
    bool CheckEstimatedPose();
    bool CheckKeyFrame();

    State state_;
    Map::Ptr map_;
    Frame::Ptr ref_, curr_;
    int32_t num_lost_, num_inliers_, num_of_features_, level_pyramid_, min_inliers_, max_num_lost_;
    float scale_factor_;
    double match_ratio_, keyframe_min_rot_, keyframe_min_trans_;
    cv::Ptr<cv::ORB> orb_;

    std::vector<cv::Point3d> ref_3D_points_;
    std::vector<cv::KeyPoint> curr_feature_points_;
    std::vector<cv::DMatch> feature_matches_;
    cv::Mat curr_descriptors_, ref_descriptors_;
    Sophus::SE3d transform_estimate_;
};

} // namespace my_slam

