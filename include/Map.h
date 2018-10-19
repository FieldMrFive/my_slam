//
// Created by Kuangye Chen on 9/26/18.
//

#pragma once

#include <memory>
#include <unordered_map>

#include <Frame.h>
#include <FeaturePoint.h>

namespace my_slam
{

class Map
{
public:
    typedef std::shared_ptr<Map> Ptr;

    Map() = default;
    bool InsertKeyFrame(Frame::Ptr frame);
    bool InsertMapPoint(FeaturePoint::Ptr feature_point);

private:
    std::unordered_map<uint32_t, FeaturePoint::Ptr> map_points_;
    std::unordered_map<uint32_t, Frame::Ptr> key_frames_;
};

}


