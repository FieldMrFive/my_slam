//
// Created by Kuangye Chen on 9/25/18.
//

#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace my_slam
{

class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;

    Camera() = default;
    Camera(const Eigen::Matrix3d& intrinsic, const double depth_scale) :
        intrinsic_{intrinsic}, depth_scale_{depth_scale}, extrinsic_{Eigen::Isometry3d::Identity()} {};
    Camera(const Eigen::Matrix3d& intrinsic, const double depth_scale, const Eigen::Isometry3d& extrinsic) :
        intrinsic_{intrinsic}, depth_scale_{depth_scale}, extrinsic_{extrinsic} {};

    bool SetExtrinsic(const Eigen::Isometry3d& extrinsics);
    bool SetIntrinsic(const Eigen::Matrix3d& intrisic);
    bool SetDepthScale(const double depth_scale);

    double GetDepthScale() const;

    static Eigen::Matrix3d MakeIntrinsic(const double fx, const double fy, const double cx, const double cy);
    static Eigen::Isometry3d MakeExtrinsic(const Eigen::Matrix3d& rotation, const Eigen::Vector3d translation);

    Eigen::Vector3d World2Camera(const Eigen::Vector3d& point);
    Eigen::Vector3d Camera2World(const Eigen::Vector3d& point);
    Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d& point);
    Eigen::Vector3d Pixel2Camera(const Eigen::Vector2d& pixel, const double depth);
    Eigen::Vector3d Pixel2World(const Eigen::Vector2d& pixel, const double depth);
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d& point);

private:
    double depth_scale_;
    Eigen::Matrix3d intrinsic_;
    Eigen::Isometry3d extrinsic_;
};

}




