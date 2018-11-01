//
// Created by Kuangye Chen on 2018/10/27.
//

#pragma once

#include <g2o/core/base_unary_edge.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <Eigen/Core>
#include <Camera.h>

namespace my_slam
{

class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void computeError() override;
    void linearizeOplus() override;

    bool read(std::istream& in) override { return true; }
    bool write(std::ostream& out) const override { return true; }

    Eigen::Vector3d point_;
    Camera::Ptr camera_;
};

}

