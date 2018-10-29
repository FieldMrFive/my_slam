//
// Created by Kuangye Chen on 2018/10/27.
//

#include "g2oTypes.h"

void my_slam::EdgeProjectXYZ2UVPoseOnly::computeError()
{
    auto pose = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    _error = _measurement - camera_ -> Camera2Pixel(pose -> estimate().map(point_));
}

void my_slam::EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
    auto pose = dynamic_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    Eigen::Vector3d xyz_trans = pose -> estimate().map(point_);
    const double x = xyz_trans[0];
    const double y = xyz_trans[1];
    const double z = xyz_trans[2];
    const double z_2 = z * z;
    const double fx = camera_ -> GetIntrinsic()(0, 0);
    const double fy = camera_ -> GetIntrinsic()(1, 1);

    _jacobianOplusXi(0, 0) = x * y * fx / z_2;
    _jacobianOplusXi(0, 1) = - fx - fx * x * x / z_2;
    _jacobianOplusXi(0, 2) = fx * y / z;
    _jacobianOplusXi(0, 3) = - fx / z;
    _jacobianOplusXi(0, 4) = 0.;
    _jacobianOplusXi(0, 5) = fx * x / z_2;

    _jacobianOplusXi(1, 0) = fy + fy * y * y / z_2;
    _jacobianOplusXi(1, 1) = - fy * x * y /z_2;
    _jacobianOplusXi(1, 2) = - fy * x / z;
    _jacobianOplusXi(1, 3) = 0.;
    _jacobianOplusXi(1, 4) = - fy / z;
    _jacobianOplusXi(1, 5) = fy * y / z_2;
}
