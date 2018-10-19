//
// Created by Kuangye Chen on 9/26/18.
//

#include <iostream>

#include "Map.h"

namespace my_slam
{

bool Map::InsertKeyFrame(Frame::Ptr frame)
{
    std::cout << "Num of key frames: " << key_frames_.size();
    if (key_frames_.find(frame -> GetID()) == key_frames_.end())
    {
        key_frames_.insert({frame -> GetID(), frame});
    }
    else
    {
        key_frames_[frame -> GetID()] = frame;
    }
    return true;
}

bool Map::InsertMapPoint(FeaturePoint::Ptr feature_point)
{
    if (map_points_.find(feature_point -> GetID()) == map_points_.end())
    {
        map_points_.insert({feature_point -> GetID(), feature_point});
    }
    else
    {
        map_points_[feature_point -> GetID()] = feature_point;
    }
    return true;
}

} // namespace my_slam