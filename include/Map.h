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

private:
    std::unordered_map<uint32_t, FeaturePoint> map_points_;
    std::unordered_map<uint32_t, Frame> key_frames_;
};

}


