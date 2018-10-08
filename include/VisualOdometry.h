//
// Created by Kuangye Chen on 9/27/18.
//

#pragma once

#include <memory>
#include <Map.h>
#include <Frame.h>

namespace my_slam
{

class VisualOdometry
{
public:
    typedef std::shared_ptr<VisualOdometry> Ptr;

    enum State
    {
        INITIALIZING = -1;
        OK = 1;
        LOST = 0;
    };

private:
    State state_;
    Map::Ptr map_;
    Frame::Ptr ref_, curr_;
};

} // namespace my_slam

