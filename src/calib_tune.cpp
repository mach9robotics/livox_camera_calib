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

using namespace std;

class CalibTune {
public:

    CalibTune(string image_path, string pcd_path);
    void show_image();
    void extract_image_edges();
    void extract_pcd_edges();
    void execuate();

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    int m_canny_grey_thresh;
    int m_canny_len_thresh;
    ros::NodeHandle m_nh;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;
    string m_image_path, m_pcd_path;
    cv::Mat m_image;
    // cv::Mat* m_edge_image_ptr;
    int m_width, m_height;
    bool m_prev_exec = false;
    bool m_curr_exec = false;

};

CalibTune::CalibTune(string image_path, string pcd_path){
    // setup dynamic reconfiguration
    auto f = boost::bind(&CalibTune::dyncfg_cb, this, _1, _2);
    m_server.setCallback(f);
    // store the image and pcd
    m_image_path = image_path;
    this->m_pcd_path = pcd_path;
    const cv::Mat image;
    this->m_image =  cv::imread(m_image_path, cv::IMREAD_UNCHANGED);
    this->m_width = this->m_image.cols;
    this->m_height = this->m_image.rows;
}


void CalibTune::show_image(){
    // cv::namedWindow("Original Image",cv::WINDOW_AUTOSIZE);
    cv::imshow("Original Image",this->m_image);
    cv::waitKey(0);
    // cv::destroyWindow("Original Image");
}

void CalibTune::dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %i %i %f %s",
              config.grey_threshold,
              config.len_threshold,
              config.voxel_size,
              config.execuate?"True":"False");
    this->m_canny_grey_thresh = config.grey_threshold;
    this->m_canny_len_thresh = config.len_threshold;
    this->m_curr_exec = config.execuate;
    // if (this->m_prev_exec != this->m_curr_exec){
    //     // this->extract_image_edges();
    //     ROS_WARN("EXEC CHANGED!");
    //     this->m_prev_exec = this->m_curr_exec;
    // }
    // this->extract_image_edges();
    // ROS_INFO("THRESHOLD: %i",this->m_canny_grey_thresh);
}

void CalibTune::extract_image_edges(){
    int gaussian_size = 5;
    cv::Mat blur_image = cv::Mat::zeros(this->m_height, this->m_width, CV_8UC1);
    cv::GaussianBlur(m_image, blur_image, cv::Size(gaussian_size,gaussian_size), 0, 0);
    cv::Mat canny_result = cv::Mat::zeros(this->m_height, this->m_width, CV_8UC1);
    cv::Canny(blur_image, canny_result,m_canny_grey_thresh,m_canny_grey_thresh*3,
              3, true);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(canny_result, contours, hierarchy, cv::RETR_EXTERNAL,
                    cv::CHAIN_APPROX_NONE, cv::Point(0, 0));
    cv::Mat edge_img;
    edge_img = cv::Mat::zeros(this->m_height, this->m_width, CV_8UC1);

    for (size_t i = 0; i < contours.size(); i++) {
        if (contours[i].size() > this->m_canny_len_thresh) {
                for (size_t j = 0; j < contours[i].size(); j++) {
                    cv::Point p(contours[i][j].y, contours[i][j].x);
                    edge_img.at<uchar>(p) = 255;
                }
        }
    }
    cv::imshow("image edge result", edge_img);
    cv::waitKey(0);
}

void CalibTune::execuate(){
    ROS_ERROR("%i, %i",this->m_prev_exec,this->m_curr_exec);
    if (this->m_prev_exec != this->m_curr_exec){
        this->extract_image_edges();
        this->m_prev_exec = this->m_curr_exec;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_tune");
    string image_path = "/tmp/mach9/auto_mlcc/image/front/0.bmp";
    string pcd_path = "/tmp/mach9/auto_mlcc/pcd/front/0.pcd";
    CalibTune cb = CalibTune(image_path, pcd_path);
    cb.show_image();
    // cb.execuate();
    // ros::spin();
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        cb.execuate();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
