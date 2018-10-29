//
// Created by Kuangye Chen on 9/27/18.
//


#include "VisualOdometry.h"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <g2oTypes.h>

namespace my_slam
{

VisualOdometry::VisualOdometry(const Config& config) :
    state_{INITIALIZING}, ref_{nullptr}, curr_{nullptr}, map_{new Map}, num_lost_{0}, num_inliers_{0},
    num_of_features_{config.GetValueByKey<int32_t>("number_of_features")},
    level_pyramid_{config.GetValueByKey<int32_t>("level_pyramid")},
    min_inliers_{config.GetValueByKey<int32_t>("min_inliers")},
    max_num_lost_{config.GetValueByKey<int32_t>("max_num_lost")},
    scale_factor_{config.GetValueByKey<float>("scale_factor")},
    match_ratio_{config.GetValueByKey<double>("match_ratio")},
    keyframe_min_rot_{config.GetValueByKey<double>("keyframe_min_rotation")},
    keyframe_min_trans_{config.GetValueByKey<double>("keyframe_min_translation")},
    orb_{cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_)},
    ref_3D_points_{}, curr_feature_points_{}, feature_matches_{},
    curr_descriptors_{}, ref_descriptors_{},
    transform_estimate_{} {}

bool VisualOdometry::AddFrame(Frame::Ptr frame)
{
    switch (state_)
    {
        case INITIALIZING:
        {
            state_ = OK;
            curr_ = frame;
            ref_ = frame;
            map_ -> InsertKeyFrame(frame);
            ExtractFeaturePoints();
            ComputeDescriptors();
            SetRef3DPoints();
            break;
        }
        case OK:
        {
            curr_ = frame;
            ExtractFeaturePoints();
            ComputeDescriptors();
            FeatureMatching();
            PoseEstimationPnP();
            if (CheckEstimatedPose()) // a good estimation
            {
                curr_ -> GetCamera() -> SetExtrinsic(
                        Eigen::Isometry3d(
                                transform_estimate_.matrix() * ref_ -> GetCamera() -> GetExtrinsic().matrix())
                );
                ref_ = curr_;
                SetRef3DPoints();
                num_lost_ = 0;
                if (CheckKeyFrame())
                {
                    AddKeyFrame();
                }
            }
            else // bad estimation due to various reasons
            {
                num_lost_++;
                if ( num_lost_ > max_num_lost_ )
                {
                    state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:
        {
            return false;
        }

    }
    return true;
}

void VisualOdometry::AddKeyFrame()
{
    std::cout << "Add key frame" << std::endl;
    map_ -> InsertKeyFrame(curr_);
}

void VisualOdometry::ExtractFeaturePoints()
{
    orb_ -> detect(curr_ -> GetColorImage(), curr_feature_points_);
    std::cout << "Num features " << curr_feature_points_.size() << std::endl;
}

void VisualOdometry::ComputeDescriptors()
{
    orb_ -> compute(curr_ -> GetColorImage(), curr_feature_points_, curr_descriptors_);
}

void VisualOdometry::FeatureMatching()
{
    std::vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(ref_descriptors_, curr_descriptors_, matches);
    double min_dist = std::min_element(
            matches.begin(), matches.end(),
            [](const cv::DMatch& m1, const cv::DMatch& m2)
            {
                return m1.distance < m2.distance;
            }) -> distance;
    feature_matches_.clear();
    for (auto const& match : matches)
    {
        if (match.distance > std::max<double>(min_dist * match_ratio_, 30.0)) { continue; }
        feature_matches_.emplace_back(match);
    }

    std::cout << "Good matches: " << feature_matches_.size() << std::endl;
}

void VisualOdometry::PoseEstimationPnP()
{
    std::vector<cv::Point3d> points_3d;
    std::vector<cv::Point2d> points_2d;

    for (const auto& match : feature_matches_)
    {
        points_3d.emplace_back(ref_3D_points_[match.queryIdx]);
        points_2d.emplace_back(curr_feature_points_[match.trainIdx].pt);
    }

    cv::Mat intrinsic, rvec, tvec, inliers;
    Eigen::Vector3d rvec_eigen;
    cv::eigen2cv(curr_ -> GetCamera() -> GetIntrinsic(), intrinsic);
    cv::solvePnPRansac(points_3d, points_2d, intrinsic, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
    cv::cv2eigen(rvec, rvec_eigen);

    num_inliers_ = inliers.rows;
    std::cout << "PnP inliers: " << num_inliers_ << std::endl;
    transform_estimate_ = Sophus::SE3d(
            Eigen::Quaterniond(Eigen::AngleAxisd(rvec_eigen.norm(), rvec_eigen.normalized())),
            Eigen::Vector3d(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );

    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> BlockSolver;
    auto linearSolver = g2o::make_unique<g2o::LinearSolverDense<BlockSolver::PoseMatrixType>>();
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (
            g2o::make_unique<BlockSolver>(std::move(linearSolver))
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto pose = new g2o::VertexSE3Expmap();
    pose -> setId(0);
    pose -> setEstimate(g2o::SE3Quat(
            transform_estimate_.rotationMatrix(),
            transform_estimate_.translation()
    ));
    optimizer.addVertex(pose);

    for (int32_t i = 0; i < num_inliers_; i++)
    {
        int32_t index = inliers.at<int32_t>(i, 0);
        auto edge = new EdgeProjectXYZ2UVPoseOnly();
        edge -> setId(i);
        edge -> setVertex(0, pose);
        edge -> camera_ = curr_ -> GetCamera();
        edge -> point_ = Eigen::Vector3d(points_3d[index].x, points_3d[index].y, points_3d[index].z);
        edge -> setMeasurement(Eigen::Vector2d(points_2d[index].x, points_2d[index].y));
        edge -> setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    transform_estimate_ = Sophus::SE3d(
            Eigen::Quaterniond(pose -> estimate().rotation()),
            Eigen::Vector3d(pose -> estimate().translation())
    );
}

void VisualOdometry::SetRef3DPoints()
{
    ref_3D_points_.clear();
    ref_descriptors_ = cv::Mat();

    for (size_t i = 0; i < curr_feature_points_.size(); i++)
    {
        const cv::KeyPoint& point = curr_feature_points_[i];
        double depth = ref_ -> GetDepth(point);
        if (depth <= 0.) { continue; }

        Eigen::Vector3d cam_point = ref_ -> GetCamera() -> Pixel2Camera(
                Eigen::Vector2d(point.pt.x, point.pt.y), depth);
        ref_3D_points_.emplace_back(cam_point(0), cam_point(1), cam_point(2));
        ref_descriptors_.push_back(curr_descriptors_.row(i));
    }
}

bool VisualOdometry::CheckEstimatedPose()
{
    if (num_inliers_ < min_inliers_)
    {
        std::cout << "Reject: too few inliers, " << num_inliers_ << std::endl;
        return false;
    }

    Sophus::Vector6d d = transform_estimate_.log();
    if (d.norm() > 5.0)
    {
        std::cout << "Reject: too large motion, " << d.norm() << std::endl;
        return false;
    }
    return true;
}

bool VisualOdometry::CheckKeyFrame()
{
    Sophus::Vector6d d = transform_estimate_.log();
    Eigen::Vector3d trans = d.head<3>();
    Eigen::Vector3d rot = d.tail<3>();
    if ( rot.norm() > keyframe_min_rot_ || trans.norm() > keyframe_min_trans_ )
        return true;
    return false;
}

} // namespace my_slam
