//
// Created by Kuangye Chen on 9/26/18.
//

#pragma once

#include <memory>
#include <Eigen/Core>
#include <opencv2/core/mat.hpp>

namespace my_slam
{

class FeaturePoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<FeaturePoint> Ptr;

    FeaturePoint() = default;
    FeaturePoint(const Eigen::Vector3d &pos, uint32_t id);

    bool SetPos(const Eigen::Vector3d &pos);
    uint32_t GetId() const { return id_; }

private:
    Eigen::Vector3d pos_;
    uint32_t id_;
    cv::Mat descripter_;
};

} // namespace my_slam