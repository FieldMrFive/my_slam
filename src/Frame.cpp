//
// Created by Kuangye Chen on 9/26/18.
//

#include <Frame.h>

#include "Frame.h"

namespace my_slam
{

Frame::Frame(uint32_t id, double time_stamp, Camera::Ptr camera, const cv::Mat& color_img, const cv::Mat& depth_img) :
    id_{id}, time_stamp_{time_stamp}, camera_{std::move(camera)}, color_img_{color_img}, depth_img_{depth_img} {}

double Frame::GetDepth(const cv::KeyPoint &key_point)
{
    int32_t x = cvRound(key_point.pt.x), y = cvRound(key_point.pt.y);
    return 0;
}

bool Frame::HasPoint(const Eigen::Vector3d &point)
{
    return false;
}

Eigen::Vector3d Frame::GetCameraCenter()
{
    return Eigen::Vector3d(0, 0, 0);
}

} // namespace my_slam