//
// Created by Kuangye Chen on 9/25/18.
//

#include "Camera.h"

namespace my_slam
{

bool Camera::SetExtrinsic(const Eigen::Isometry3d &extrinsics)
{
    extrinsic_ = extrinsics;
    return true;
}

bool Camera::SetIntrinsic(const Eigen::Matrix3d &intrisic)
{
    intrinsic_ = intrisic;
    return true;
}

bool Camera::SetDepthScale(const double depth_scale)
{
    depth_scale_ = depth_scale;
    return true;
}

double Camera::GetDepthScale() const
{
    return depth_scale_;
}

Eigen::Matrix3d Camera::MakeIntrinsic(const double fx, const double fy, const double cx, const double cy)
{
    Eigen::Matrix3d res;
    res << fx,  0, cx,
            0, fy, cy,
            0,  0,  1;
    return res;
}

Eigen::Isometry3d Camera::MakeExtrinsic(const Eigen::Matrix3d &rotation, const Eigen::Vector3d translation)
{
    Eigen::Isometry3d res = Eigen::Isometry3d::Identity();
    res.translate(translation);
    res.rotate(rotation);
    return res;
}

Eigen::Vector3d Camera::World2Camera(const Eigen::Vector3d &point)
{
    return extrinsic_ * point;
}

Eigen::Vector3d Camera::Camera2World(const Eigen::Vector3d &point)
{
    return extrinsic_.inverse() * point;
}

Eigen::Vector2d Camera::Camera2Pixel(const Eigen::Vector3d &point)
{
    Eigen::Vector3d homo_pixel = intrinsic_ * point;
    return Eigen::Vector2d(homo_pixel(0), homo_pixel(1)) / homo_pixel(2);
}

Eigen::Vector3d Camera::Pixel2Camera(const Eigen::Vector2d &pixel, const double depth)
{
    Eigen::Vector3d homo_pixel(pixel(0), pixel(1), 1.);
    return depth / depth_scale_ * intrinsic_.inverse() * homo_pixel;
}

Eigen::Vector3d Camera::Pixel2World(const Eigen::Vector2d &pixel, const double depth)
{
    return Camera2World(Pixel2Camera(pixel, depth));
}

Eigen::Vector2d Camera::World2Pixel(const Eigen::Vector3d &point)
{
    return Camera2Pixel(World2Camera(point));
}

}