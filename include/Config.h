//
// Created by Kuangye Chen on 9/27/18.
//

#pragma once

#include <memory>
#include <string>
#include <opencv2/core.hpp>

namespace my_slam
{

class Config
{
public:
    typedef std::shared_ptr<Config> Ptr;

    explicit Config(const std::string &filename);

    template<typename T>
    T GetValueByKey(const std::string &key) const
    {
        return static_cast<T>(file_[key]);
    }

private:
    cv::FileStorage file_;
};

} // namespace my_slam
