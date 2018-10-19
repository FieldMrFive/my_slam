//
// Created by Kuangye Chen on 9/26/18.
//

#include <Frame.h>

#include "Frame.h"

namespace my_slam
{

Frame::Frame(uint32_t id, double time_stamp, Camera::Ptr camera, const cv::Mat& color_img, const cv::Mat& depth_img) :
    id_{id}, time_stamp_{time_stamp}, camera_{std::move(camera)}, color_img_{color_img}, depth_img_{depth_img} {}

double Frame::GetDepth(const cv::KeyPoint &feature_point) const
{
    int32_t x = cvRound(feature_point.pt.x), y = cvRound(feature_point.pt.y);
    auto depth = static_cast<double>(depth_img_.ptr<uint16_t>(y)[x]);
    if ( depth != 0. )
    {
        return depth / camera_ -> GetDepthScale();
    }
    else
    {
        // check the nearby points
        int dx[4] = {-1, 0 , 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++)
        {
            depth = static_cast<double>(depth_img_.ptr<ushort>(y + dy[i])[x + dx[i]]);
            if ( depth != 0 )
            {
                return depth / camera_ -> GetDepthScale();
            }
        }
    }
    return -1.0;
}

bool Frame::HasPoint(const Eigen::Vector3d &point) const
{
    Eigen::Vector3d cam_point = camera_ -> World2Camera(point);
    if (cam_point(2) < 0.)
    {
        return false;
    }

    Eigen::Vector2d pixel = camera_ -> World2Pixel(point);
    return pixel(0) > 0. && pixel(1) > 0. && pixel(0) < color_img_.cols && pixel(1) < color_img_.rows;
}

Eigen::Vector3d Frame::GetCameraCenter() const
{
    return camera_ -> GetExtrinsic().inverse().translation();
}

} // namespace my_slam