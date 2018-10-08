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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<Camera> Ptr;

    Camera() = default;
    Camera(const Eigen::Matrix3d &intrinsic, double depth_scale);
    Camera(const Eigen::Matrix3d &intrinsic, double depth_scale, const Eigen::Isometry3d &extrinsic);

    bool SetExtrinsic(const Eigen::Isometry3d& extrinsic);
    bool SetIntrinsic(const Eigen::Matrix3d& intrinsic);
    bool SetDepthScale(double depth_scale);

    double GetDepthScale() const { return depth_scale_; }

    static Eigen::Matrix3d MakeIntrinsic(double fx, double fy, double cx, double cy);
    static Eigen::Isometry3d MakeExtrinsic(const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation);

    Eigen::Vector3d World2Camera(const Eigen::Vector3d &point) const;
    Eigen::Vector3d Camera2World(const Eigen::Vector3d &point) const;
    Eigen::Vector2d Camera2Pixel(const Eigen::Vector3d &point) const;
    Eigen::Vector3d Pixel2Camera(const Eigen::Vector2d &pixel, double depth) const;
    Eigen::Vector3d Pixel2World(const Eigen::Vector2d &pixel, double depth) const;
    Eigen::Vector2d World2Pixel(const Eigen::Vector3d &point) const;

private:
    double depth_scale_;
    Eigen::Matrix3d intrinsic_;
    Eigen::Isometry3d extrinsic_;
};

} // namespace my_slam




