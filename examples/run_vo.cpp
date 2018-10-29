//
// Created by Kuangye Chen on 9/27/18.
//

#include <iostream>
#include <fstream>
#include <tuple>
#include <vector>
#include <string>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>

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
    std::cout << "Config: " << argv[1] << std::endl;
    my_slam::Config config(argv[1]);
    my_slam::VisualOdometry::Ptr vo(new my_slam::VisualOdometry(config));

    auto dataset_dir = config.GetValueByKey<std::string>("dataset_dir") + "/";
    std::ifstream fin(dataset_dir + "associate.txt");
    if (!fin)
    {
        std::cerr << "Found no associate file." << std::endl;
        return 1;
    }

    std::vector<std::tuple<std::string, double, std::string, double>> data_list;
    std::cout << "Read data." << std::endl;
    while (!fin.eof())
    {
        std::string rgb_file, depth_file;
        double rgb_ts, depth_ts;
        fin >> rgb_ts >> rgb_file >> depth_ts >> depth_file;
        data_list.emplace_back(std::make_tuple(dataset_dir + rgb_file, rgb_ts,
                                               dataset_dir + depth_file, depth_ts));
    }

    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_frame(1.0), camera_frame(0.5);
    cv::Point3d cam_pos(0., -1., -1.), cam_focal_point(0., 0., 0.), cam_y_dir(0., 1., 0.);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);

    world_frame.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_frame.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("World", world_frame);
    vis.showWidget("Camera", camera_frame);

    std::cout << "Total number of data: " << data_list.size() << std::endl;
    uint32_t frame_seq = 0;
    for (const auto& data : data_list)
    {
        std::cout << "--------------- One Frame ----------------" << std::endl;
        cv::Mat color = cv::imread(std::get<0>(data), cv::IMREAD_COLOR);
        cv::Mat depth = cv::imread(std::get<2>(data), cv::IMREAD_UNCHANGED);
        if (color.data == nullptr || depth.data == nullptr)
        {
            std::cerr << "IMAGE WRONG." << std::endl;
            break;
        }

        my_slam::Camera::Ptr camera(new my_slam::Camera(my_slam::Camera::MakeIntrinsic(
                config.GetValueByKey<double>("fx"), config.GetValueByKey<double>("fy"),
                config.GetValueByKey<double>("cx"), config.GetValueByKey<double>("cy")),
                config.GetValueByKey<double>("depth_scale")));
        my_slam::Frame::Ptr pFrame(new my_slam::Frame(frame_seq, std::get<1>(data), camera, color, depth));
        frame_seq += 1;

        auto start = std::chrono::system_clock::now();
        vo -> AddFrame(pFrame);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "VO costs " << elapsed_seconds.count() << "s" << std::endl;

        if (!vo -> Good())
        {
            std::cout << "VO lost." << std::endl;
            break;
        }

        Eigen::Matrix3d cam_pose_R = pFrame -> GetCamera() -> GetExtrinsic().inverse().rotation().matrix();
        Eigen::Vector3d cam_pose_t = pFrame -> GetCamera() -> GetExtrinsic().inverse().translation().matrix();
        cv::Affine3d cam_pose(
                cv::Affine3d::Mat3(
                        cam_pose_R(0,0), cam_pose_R(0,1), cam_pose_R(0,2),
                        cam_pose_R(1,0), cam_pose_R(1,1), cam_pose_R(1,2),
                        cam_pose_R(2,0), cam_pose_R(2,1), cam_pose_R(2,2)
                ),
                cv::Affine3d::Vec3(
                        cam_pose_t(0), cam_pose_t(1), cam_pose_t(2)
                )
        );

        cv::Mat color_with_feature = color.clone();
        for (const auto& key_point : vo -> GetCurrentFeature())
        {
            cv::circle(color_with_feature, key_point.pt, 5, cv::Scalar(0, 255, 0), 1);
        }
        cv::imshow("image", color_with_feature);
        cv::waitKey(1);
        vis.setWidgetPose("Camera", cam_pose);
        vis.spinOnce(1, false);
    }
}