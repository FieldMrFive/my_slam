//
// Created by Kuangye Chen on 9/27/18.
//

#include "Config.h"

namespace my_slam
{

Config::Config(const std::string &filename) : file_(filename, cv::FileStorage::READ) {}

template<typename T>
T Config::GetValueByKey(const std::string &key)
{
    return static_cast<T>(file_[key]);
}

} // namespace my_slam
