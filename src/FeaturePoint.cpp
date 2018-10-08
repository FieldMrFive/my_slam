//
// Created by Kuangye Chen on 9/26/18.
//

#include "FeaturePoint.h"

namespace my_slam
{

FeaturePoint::FeaturePoint(const Eigen::Vector3d &pos, uint32_t id) :
    pos_{pos}, id_{id} {}

bool FeaturePoint::SetPos(const Eigen::Vector3d &pos)
{
    pos_ = pos;
    return true;
}

} // namespace my_slam
