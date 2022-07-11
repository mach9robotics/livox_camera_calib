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

class CalibTune {
public:

    CalibTune(string image_path, string pcd_path);
    void show_image();
    void extract_image_edges();
    void extract_pcd_edges();
    void init_voxel(unordered_map<VOXEL_LOC, Voxel *> &voxel_map);
    void publish_cloud();

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    int m_canny_grey_thresh;
    int m_canny_len_thresh;
    float m_voxel_size;
    int m_width, m_height;
    bool m_prev_exec = false;
    bool m_curr_exec = false;
    sensor_msgs::PointCloud2 m_pub_cloud;

    ros::NodeHandle m_nh;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;
    const cv::Mat m_image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_raw_cloud;
    ros::Publisher m_rgb_cloud_pub = 
        m_nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);

};

CalibTune::CalibTune(string image_path, string pcd_path) :
    m_image(cv::imread(image_path, cv::IMREAD_UNCHANGED))
{
    // setup dynamic reconfiguration
    auto f = boost::bind(&CalibTune::dyncfg_cb, this, _1, _2);
    this->m_server.setCallback(f);
    this->m_width = this->m_image.cols;
    this->m_height = this->m_image.rows;
    // load point cloud
    this->m_raw_cloud = 
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    ROS_INFO_STREAM("Loading point cloud from pcd file.");
    auto load_result = pcl::io::loadPCDFile(pcd_path, *m_raw_cloud);
    ROS_INFO("Load result: %i", load_result);
    if(load_result == 0){
        string msg = "Successfully load pcd, pointcloud size: " + 
                    to_string(m_raw_cloud->size());
        ROS_INFO_STREAM(msg.c_str());
    }
    else{
        string msg = "Unable to load" + pcd_path;
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    }
    
}


void CalibTune::show_image(){
    cv::imshow("Original Image",this->m_image);
    cv::waitKey(0);
}

void CalibTune::dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level){
    ROS_INFO("Reconfigure Request: %i %i %f %s",
              config.grey_threshold,
              config.len_threshold,
              config.voxel_size,
              config.execuate?"True":"False");
    this->m_canny_grey_thresh = config.grey_threshold;
    this->m_canny_len_thresh = config.len_threshold;
    this->m_voxel_size = config.voxel_size;
    this->m_curr_exec = config.execuate;
    if (this->m_prev_exec != this->m_curr_exec){
        this->extract_image_edges();
        this->m_prev_exec = this->m_curr_exec;
    }
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
                pcl::PointXYZ p;
                p.x = contours[i][j].x;
                p.y = -contours[i][j].y;
                p.z = 0;
                edge_img.at<uchar>(-p.y, p.x) = 255;
            }
        }
    }
    cv::imshow("image edge result", edge_img);
    cv::waitKey(0);
}

void CalibTune::init_voxel(unordered_map<VOXEL_LOC, Voxel *> &voxel_map){
    ROS_INFO_STREAM("Building Voxel");
    // voxel test
    srand((unsigned)time(NULL));
    pcl::PointCloud<pcl::PointXYZRGB> test_cloud;
    for (size_t i = 0; i < m_raw_cloud->size(); i++) {
        const pcl::PointXYZI& p_c = m_raw_cloud->points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++){
            loc_xyz[j] = p_c.data[j] / m_voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                            (int64_t)loc_xyz[2]);
        auto iter = voxel_map.find(position);
        if (iter != voxel_map.end()) {
            voxel_map[position]->cloud->push_back(p_c);
            pcl::PointXYZRGB p_rgb;
            p_rgb.x = p_c.x;
            p_rgb.y = p_c.y;
            p_rgb.z = p_c.z;
            p_rgb.r = voxel_map[position]->voxel_color(0);
            p_rgb.g = voxel_map[position]->voxel_color(1);
            p_rgb.b = voxel_map[position]->voxel_color(2);
            test_cloud.push_back(p_rgb);
        }
        else{
            Voxel* voxel = new Voxel(m_voxel_size);
            voxel_map[position] = voxel;
            voxel_map[position]->voxel_origin[0] = position.x * m_voxel_size;
            voxel_map[position]->voxel_origin[1] = position.y * m_voxel_size;
            voxel_map[position]->voxel_origin[2] = position.z * m_voxel_size;
            voxel_map[position]->cloud->push_back(p_c);
            int r = rand() % 256;
            int g = rand() % 256;
            int b = rand() % 256;
            voxel_map[position]->voxel_color<< r, g, b;
        }
    }
    // test color cloud
    // sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(test_cloud, m_pub_cloud);
    m_pub_cloud.header.frame_id = "world";
    // m_rgb_cloud_pub.publish(pub_cloud);
}

void CalibTune::extract_pcd_edges(){
    Eigen::Vector3d lwh(50, 50, 30);
    Eigen::Vector3d origin(0, -25, -10);
    vector<VoxelGrid> voxel_list;
    unordered_map<VOXEL_LOC, Voxel*> voxel_map;
    init_voxel(voxel_map);
}

void CalibTune::publish_cloud() {
    m_rgb_cloud_pub.publish(m_pub_cloud);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_tune");
    string image_path = "/tmp/mach9/auto_mlcc/image/front/0.bmp";
    string pcd_path = "/tmp/mach9/auto_mlcc/pcd/front/0.pcd";
    CalibTune cb = CalibTune(image_path, pcd_path);
    // cb.show_image();
    cb.extract_pcd_edges();
    cb.publish_cloud();

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
