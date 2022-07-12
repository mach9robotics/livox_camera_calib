/**
 * Copyright (C) Mach9 Robotics, Inc - All Rights Reserved
 * Proprietary and confidential
 *
 * Tune parameters in calibrations
 *
 * Written by Jason Zheng <jason@mach9.io>, July 2022
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <livox_camera_calib/CalibTuneConfig.h>
#include "include/lidar_camera_calib.hpp"
#include "include/common.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>

using namespace std;

class CalibTune: public Calibration {
public:

    CalibTune(const std::string &image_file,
              const std::string &pcd_file,
              const std::string &calib_config_file) : 
              Calibration(image_file, pcd_file, calib_config_file){
                // setup dynamic reconfiguration
                auto f = boost::bind(&CalibTune::dyncfg_cb, this, _1, _2);
                this->m_server.setCallback(f);
              }
    void show_image();
    void extract_image_edges();
    void extract_pcd_edges();
    void init_voxel(unordered_map<VOXEL_LOC, Voxel *> &voxel_map);
    void publish_clouds();

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    bool m_prev_exec = false;
    bool m_curr_exec = false;

    ros::NodeHandle m_nh;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;

};


void CalibTune::dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %i %i %f %s",
              config.grey_threshold,
              config.len_threshold,
              config.voxel_size,
              config.execuate?"True":"False");
    this->rgb_canny_threshold_ = config.grey_threshold;
    this->rgb_edge_minLen_ = config.len_threshold;
    this->voxel_size_ = config.voxel_size;
    this->m_curr_exec = config.execuate;
    if (this->m_prev_exec != this->m_curr_exec){
        cv::Mat edge_image;
        this->edgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_, edge_image, rgb_egde_cloud_);
        cv::imshow("image edge result", edge_image);
        cv::waitKey(0);
        this->m_prev_exec = this->m_curr_exec;
    }
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_tune");
    string image_file = "/tmp/mach9/auto_mlcc/image/front/0.bmp";
    string pcd_file = "/tmp/mach9/auto_mlcc/pcd/front/0.pcd";
    string calib_config_file = "/home/jason/map_ws/src/livox_camera_calib/config/config_outdoor.yaml";
    CalibTune cb = CalibTune(image_file, pcd_file, calib_config_file);
    // cb.show_image();
    // cb.extract_pcd_edges();
    // cb.publish_clouds();

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
