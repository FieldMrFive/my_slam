//
// Created by Kuangye Chen on 9/26/18.
//

#include "FeaturePoint.h"

namespace my_slam
{

FeaturePoint::FeaturePoint(const Eigen::Vector3d &pos, uint32_t id) :
    pos_{pos}, id_{id} {}

} // namespace my_slam
