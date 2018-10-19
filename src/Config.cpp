//
// Created by Kuangye Chen on 9/27/18.
//

#include "Config.h"

namespace my_slam
{

Config::Config(const std::string &filename) : file_(filename, cv::FileStorage::READ) {}

} // namespace my_slam
