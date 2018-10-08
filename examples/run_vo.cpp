//
// Created by Kuangye Chen on 9/27/18.
//

#include <iostream>
#include <fstream>
#include <tuple>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/viz.hpp>

#include <Config.h>
#include <VisualOdometry.h>
#include <Camera.h>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./run_vo config_file\n"
                  << "config_file: config for VO." << std::endl;
    }

    my_slam::Config config(argv[1]);
    my_slam::VisualOdometry::Ptr vo(new my_slam::VisualOdometry);

    auto dataset_dir = config.GetValueByKey<std::string>("dataset_dir");
    std::ifstream fin(dataset_dir + "/associate.txt");
    if (!fin)
    {
        std::cerr << "Found no associate file." << std::endl;
        return 1;
    }

    std::vector<std::tuple<std::string, double, std::string, double>> data_list;
    while(!fin.eof())
    {
        std::string rgb_file, depth_file;
        double rgb_ts, depth_ts;
        fin >> rgb_ts >> rgb_file >> depth_ts >> depth_file;
        data_list.emplace_back(std::make_tuple(rgb_file, rgb_ts, depth_file, depth_ts));
    }

    my_slam::Camera::Ptr camera;
}