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
              const std::string &pcd_file) : 
              Calibration(image_file, pcd_file){
                // setup dynamic reconfiguration
                auto f = boost::bind(&CalibTune::dyncfg_cb, this, _1, _2);
                this->m_server.setCallback(f);
                // load image
                image_ = cv::imread(image_file, cv::IMREAD_UNCHANGED);
                this->m_width = this->image_.cols;
                this->m_height = this->image_.rows;
                // load point cloud
                this->raw_lidar_cloud_ = 
                    pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                ROS_INFO_STREAM("Loading point cloud from pcd file.");
                auto load_result = pcl::io::loadPCDFile(pcd_file, *raw_lidar_cloud_);
                if(load_result == 0){
                    string msg = "Successfully load pcd, pointcloud size: " + 
                                to_string(raw_lidar_cloud_->size());
                    ROS_INFO_STREAM(msg.c_str());
                }
                else{
                    string msg = "Unable to load" + pcd_file;
                    ROS_ERROR_STREAM(msg.c_str());
                    exit(-1);
                }
              }
    void show_image();
    void extract_image_edges();
    void extract_pcd_edges();
    void init_voxel(unordered_map<VOXEL_LOC, Voxel *> &voxel_map);
    void publish_clouds();

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    int m_canny_grey_thresh;
    int m_canny_len_thresh;
    float m_voxel_size;
    int m_width, m_height;
    bool m_prev_exec = false;
    bool m_curr_exec = false;
    pcl::PointCloud<pcl::PointXYZRGB> m_voxel_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_plane_line_cloud;
    std::vector<int> m_plane_line_number;
    int m_line_number = 0;

    ros::NodeHandle m_nh;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;
    // ros::Publisher m_rgb_cloud_pub = 
    //     m_nh.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 1);
    // ros::Publisher m_plane_cloud_pub = 
    //     m_nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 1);

};


void CalibTune::show_image(){
    cv::imshow("Original Image",this->image_);
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
    cv::GaussianBlur(image_, blur_image, cv::Size(gaussian_size,gaussian_size), 0, 0);
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
    // pcl::PointCloud<pcl::PointXYZRGB> m_voxel_cloud;
    for (size_t i = 0; i < raw_lidar_cloud_ ->size(); i++) {
        const pcl::PointXYZI& p_c = raw_lidar_cloud_->points[i];
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
            m_voxel_cloud.push_back(p_rgb);
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
    // sensor_msgs::PointCloud2 voxel_cloud;
    // pcl::toROSMsg(m_voxel_cloud, m_voxel_cloud);
    // m_voxel_cloud.header.frame_id = "world";
    // m_rgb_cloud_pub.publish(voxel_cloud);
    for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++){
        if (iter->second->cloud->size() > 20) {
            down_sampling_voxel(*(iter->second->cloud), 0.02);
        }
    }
}

void CalibTune::extract_pcd_edges(){
    Eigen::Vector3d lwh(50, 50, 30);
    Eigen::Vector3d origin(0, -25, -10);
    vector<VoxelGrid> voxel_list;
    unordered_map<VOXEL_LOC, Voxel*> voxel_map;
    init_voxel(voxel_map);
    const float ransac_dis_thre = 0.015;
    const int plane_size_threshold = 60;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_line_cloud_3d;
    ROS_INFO_STREAM("Extracting Lidar Edges");
    LiDAREdgeExtraction(voxel_map, ransac_dis_thre, plane_size_threshold, lidar_line_cloud_3d);


    // ros::Rate loop(5000);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_line_cloud_3d;
    // lidar_line_cloud_3d = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // for (auto iter = voxel_map.begin(); iter != voxel_map.end(); iter++) {
    //     if (iter->second->cloud->size() > 50) {
    //         std::vector<Plane> plane_list;
    //         // 创建一个体素滤波器
    //         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter(
    //             new pcl::PointCloud<pcl::PointXYZI>);
    //         pcl::copyPointCloud(*iter->second->cloud, *cloud_filter);
    //         //创建一个模型参数对象，用于记录结果
    //         pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //         // inliers表示误差能容忍的点，记录点云序号
    //         pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //         //创建一个分割器
    //         pcl::SACSegmentation<pcl::PointXYZI> seg;
    //         // Optional,设置结果平面展示的点是分割掉的点还是分割剩下的点
    //         seg.setOptimizeCoefficients(true);
    //         // Mandatory-设置目标几何形状
    //         seg.setModelType(pcl::SACMODEL_PLANE);
    //         //分割方法：随机采样法
    //         seg.setMethodType(pcl::SAC_RANSAC);
    //         //设置误差容忍范围，也就是阈值
    //         seg.setDistanceThreshold(ransac_dis_thre);
    //         pcl::PointCloud<pcl::PointXYZRGB> color_planner_cloud;
    //         int plane_index = 0;
    //         while (cloud_filter->points.size() > 10) {
    //             pcl::PointCloud<pcl::PointXYZI> planner_cloud;
    //             pcl::ExtractIndices<pcl::PointXYZI> extract;
    //             //输入点云
    //             seg.setInputCloud(cloud_filter);
    //             seg.setMaxIterations(500);
    //             //分割点云
    //             seg.segment(*inliers, *coefficients);
    //             if (inliers->indices.size() == 0) {
    //             ROS_INFO_STREAM(
    //                 "Could not estimate a planner model for the given dataset");
    //             break;
    //             }
    //             extract.setIndices(inliers);
    //             extract.setInputCloud(cloud_filter);
    //             extract.filter(planner_cloud);

    //             if (planner_cloud.size() > plane_size_threshold) {
    //                 pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    //                 std::vector<unsigned int> colors;
    //                 colors.push_back(static_cast<unsigned int>(rand() % 256));
    //                 colors.push_back(static_cast<unsigned int>(rand() % 256));
    //                 colors.push_back(static_cast<unsigned int>(rand() % 256));
    //                 pcl::PointXYZ p_center(0, 0, 0);
    //                 for (size_t i = 0; i < planner_cloud.points.size(); i++) {
    //                     pcl::PointXYZRGB p;
    //                     p.x = planner_cloud.points[i].x;
    //                     p.y = planner_cloud.points[i].y;
    //                     p.z = planner_cloud.points[i].z;
    //                     p_center.x += p.x;
    //                     p_center.y += p.y;
    //                     p_center.z += p.z;
    //                     p.r = colors[0];
    //                     p.g = colors[1];
    //                     p.b = colors[2];
    //                     color_cloud.push_back(p);
    //                     color_planner_cloud.push_back(p);
    //                 }
    //                 p_center.x = p_center.x / planner_cloud.size();
    //                 p_center.y = p_center.y / planner_cloud.size();
    //                 p_center.z = p_center.z / planner_cloud.size();
    //                 Plane single_plane;
    //                 single_plane.cloud = planner_cloud;
    //                 single_plane.p_center = p_center;
    //                 single_plane.normal << coefficients->values[0],
    //                     coefficients->values[1], coefficients->values[2];
    //                 single_plane.index = plane_index;
    //                 plane_list.push_back(single_plane);
    //                 plane_index++;
    //             }
    //             extract.setNegative(true);
    //             pcl::PointCloud<pcl::PointXYZI> cloud_f;
    //             extract.filter(cloud_f);
    //             *cloud_filter = cloud_f;
    //         }
    //         if (plane_list.size() >= 2) {
    //             sensor_msgs::PointCloud2 planner_cloud2;
    //             pcl::toROSMsg(color_planner_cloud, planner_cloud2);
    //             planner_cloud2.header.frame_id = "world";
    //             planner_cloud_pub_.publish(planner_cloud2);
    //             loop.sleep();
    //         }
    //         ROS_WARN("Plane list: %li", plane_list.size());
    //         std::vector<pcl::PointCloud<pcl::PointXYZI>> line_cloud_list;
    //         calcLine(plane_list, m_voxel_size, iter->second->voxel_origin,
    //                 line_cloud_list);
    //         // ROS_WARN("Line cloud: %li",line_cloud_list.size());
    //         // ouster 5,normal 3
    //         if (line_cloud_list.size() > 0 && line_cloud_list.size() <= 8) {

    //             for (size_t cloud_index = 0; cloud_index < line_cloud_list.size();
    //                 cloud_index++) {
    //             for (size_t i = 0; i < line_cloud_list[cloud_index].size(); i++) {
    //                 pcl::PointXYZI p = line_cloud_list[cloud_index].points[i];
    //                 m_plane_line_cloud->points.push_back(p);
    //                 // sensor_msgs::PointCloud2 voxel_cloud;
    //                 // pcl::toROSMsg(line_cloud_list[cloud_index], voxel_cloud);
    //                 // voxel_cloud.header.frame_id = "world";
    //                 // line_cloud_pub_.publish(voxel_cloud);
    //                 // loop.sleep();
    //                 m_plane_line_number.push_back(m_line_number);
    //                 ROS_INFO("Line number: %i", m_line_number);
    //             }
    //             m_line_number++;
    //             }
    //         }
    //     }
    // }
    // ROS_INFO_STREAM("Complete Lidar edge extraction");
}

void CalibTune::publish_clouds() {
    sensor_msgs::PointCloud2 voxel_cloud;
    pcl::toROSMsg(m_voxel_cloud, voxel_cloud);
    voxel_cloud.header.frame_id = "world";
    rgb_cloud_pub_.publish(voxel_cloud);
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_tune");
    string image_file = "/tmp/mach9/auto_mlcc/image/front/0.bmp";
    string pcd_file = "/tmp/mach9/auto_mlcc/pcd/front/0.pcd";
    string calib_config_file = "/home/jason/map_ws/src/livox_camera_calib/config/config_outdoor.yaml";
    CalibTune cb = CalibTune(image_file, pcd_file);
    // cb.show_image();
    cb.extract_pcd_edges();
    // cb.publish_clouds();

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
