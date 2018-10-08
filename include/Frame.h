//
// Created by Kuangye Chen on 9/26/18.
//

#pragma once

#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <Camera.h>

namespace my_slam
{

class Frame
{
public:
    typedef std::shared_ptr<Frame> Ptr;

    Frame() = default;
    Frame(uint32_t id, double time_stamp = 0, Camera::Ptr camera = nullptr,
          const cv::Mat& color_img = cv::Mat(), const cv::Mat& depth_img = cv::Mat());
    double GetDepth(const cv::KeyPoint& key_point);
    bool HasPoint(const Eigen::Vector3d& point);
    Eigen::Vector3d GetCameraCenter();

private:
    uint32_t id_;
    double time_stamp_;
    Camera::Ptr camera_;
    cv::Mat color_img_, depth_img_;
};

}
