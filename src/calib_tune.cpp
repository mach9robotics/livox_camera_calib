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
#include <bits/stdc++.h>
#include <filesystem>

using namespace std;
using namespace cv;

class CalibTune: public Calibration {
public:

    CalibTune(const std::string &image_file,
              const std::string &pcd_file,
              const std::string &calib_config_file) : 
              Calibration(image_file, pcd_file, calib_config_file)
        {
        // setup dynamic reconfiguration
        auto f = boost::bind(&CalibTune::dyncfg_cb, this, _1, _2);
        this->m_server.setCallback(f);
        // get parameters
        m_nh.param<vector<double>>("camera/camera_matrix", m_camera_matrix, vector<double>());
        m_nh.param<vector<double>>("camera/dist_coeffs", m_dist_coeffs, vector<double>());
        fx_ = m_camera_matrix[0];
        cx_ = m_camera_matrix[2];
        fy_ = m_camera_matrix[4];
        cy_ = m_camera_matrix[5];
        k1_ = m_dist_coeffs[0];
        k2_ = m_dist_coeffs[1];
        p1_ = m_dist_coeffs[2];
        p2_ = m_dist_coeffs[3];
        k3_ = m_dist_coeffs[4];
        // set config file
        m_config_file = calib_config_file;
        }
    void align_edges();
    void save_config_file(string& file_path);

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    string m_config_file;
    bool m_prev_exec = false;
    bool m_curr_exec = false;
    unordered_set<uint32_t> m_changes;
    Eigen::Vector3d m_rotation = init_rotation_matrix_.eulerAngles(2,1,0);
    Eigen::Vector3d m_translation = init_translation_vector_;

    ros::NodeHandle m_nh;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;

    vector<double> m_camera_matrix;
    vector<double> m_dist_coeffs;

};


void CalibTune::dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level){
    ROS_INFO("Level: %i", level);
    m_changes.insert(level);
    // set values
    this->m_curr_exec = config.execuate;
    if (level == 1){
        this->rgb_canny_threshold_ = config.grey_threshold;
        this->rgb_edge_minLen_ = config.len_threshold;
    }
    if (level == 2){
        this->voxel_size_ = config.voxel_size;
    }
    if (level == 3){
        this->m_translation[0] = config.translation_x;
        this->m_translation[1] = config.translation_y;
        this->m_translation[2] = config.translation_z;
        this->m_rotation[0] = config.rotation_z;
        this->m_rotation[1] = config.rotation_y;
        this->m_rotation[2] = config.rotation_x;
    }

    // execuate and show residual image
    if (this->m_prev_exec != this->m_curr_exec){
        if (m_changes.size() == 0) {}
        else{
            if (m_changes.find(1) != m_changes.end()){
                ROS_INFO_STREAM("start image edge extraction");
                cv::Mat edge_image;
                this->edgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_, edge_image, rgb_egde_cloud_);
                ROS_INFO_STREAM("complete image edge extraction");
                // cv::imshow("image edge result", edge_image);
                // cv::waitKey(0);
            }
            if (m_changes.find(2) != m_changes.end()){
                ROS_INFO_STREAM("start point cloud edge extraction");
                std::unordered_map<VOXEL_LOC, Voxel *> voxel_map;
                initVoxel(raw_lidar_cloud_, voxel_size_, voxel_map);
                LiDAREdgeExtraction(voxel_map, ransac_dis_threshold_, plane_size_threshold_, plane_line_cloud_);
                ROS_INFO_STREAM("complete point cloud edge extraction");
            }
            this->align_edges();
            m_changes.clear();
        }
        this->m_prev_exec = this->m_curr_exec;
    }
}

void CalibTune::align_edges(){
    Eigen::Vector3d init_euler_angle = this->m_rotation;
    Eigen::Vector3d init_translation = this->m_translation;

    ROS_WARN_STREAM(init_euler_angle.transpose()<<" "<<init_translation.transpose());

    Vector6d calib_params;
    calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
        init_translation(0), init_translation(1), init_translation(2);
    ROS_INFO_STREAM("Calibration 6D pose:" << calib_params.transpose());
    std::vector<std::vector<std::vector<pcl::PointXYZI>>> img_pts_container;
    for (int y = 0; y < height_; y++) {
        std::vector<std::vector<pcl::PointXYZI>> row_pts_container;
        for (int x = 0; x < width_; x++) {
        std::vector<pcl::PointXYZI> col_pts_container;
        row_pts_container.push_back(col_pts_container);
        }
        img_pts_container.push_back(row_pts_container);
    }
    std::vector<cv::Point3d> pts_3d;
    Eigen::AngleAxisd rotation_vector3;
    rotation_vector3 =
        Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitX());
    for (size_t i = 0; i < plane_line_cloud_->size(); i++) {
        pcl::PointXYZI point_3d = plane_line_cloud_->points[i];
        pts_3d.emplace_back(cv::Point3d(point_3d.x, point_3d.y, point_3d.z));
    }
    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff =
        (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    cv::Mat r_vec =
        (cv::Mat_<double>(3, 1)
            << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
        rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
        rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << calib_params[3],
                    calib_params[4], calib_params[5]);
    // project 3d-points into image view
    std::vector<cv::Point2d> pts_2d;
    cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                        pts_2d);
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_edge_cloud_2d(
        new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> line_edge_cloud_2d_number;
    for (size_t i = 0; i < pts_2d.size(); i++) {
        pcl::PointXYZ p;
        p.x = pts_2d[i].x;
        p.y = -pts_2d[i].y;
        p.z = 0;
        pcl::PointXYZI pi_3d;
        pi_3d.x = pts_3d[i].x;
        pi_3d.y = pts_3d[i].y;
        pi_3d.z = pts_3d[i].z;
        pi_3d.intensity = 1;
        if (p.x > 0 && p.x < width_ && pts_2d[i].y > 0 && pts_2d[i].y < height_) {
        if (img_pts_container[pts_2d[i].y][pts_2d[i].x].size() == 0) {
            line_edge_cloud_2d->points.push_back(p);
            line_edge_cloud_2d_number.push_back(plane_line_number_[i]);
            img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
        } else {
            img_pts_container[pts_2d[i].y][pts_2d[i].x].push_back(pi_3d);
        }
        }
    }
    int dis_threshold = 25;
    cv::Mat residual_img =
        getConnectImg(dis_threshold, rgb_egde_cloud_, line_edge_cloud_2d);
    cv::imshow("residual", residual_img);
    cv::waitKey(0);
}

void CalibTune::save_config_file(string& file_path) {
    // check directory
    string file_dir = file_path.substr(0, file_path.find_last_of("/"));
    if (!filesystem::exists(file_dir)){
        filesystem::create_directories(file_dir);
    }
    // check file
    if (filesystem::exists(file_path)){
        ROS_INFO_STREAM("File exists, may overwrite the file");
    }
    else{
        std::ofstream ofs(file_path);
        ofs<< "new file" << endl;
        ofs.close();
        ROS_INFO_STREAM("File doesn't exist, create the file");
    }
    // calculate updated rotation matrix 
    Eigen::AngleAxisd rollAngle(m_rotation[0], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(m_rotation[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(m_rotation[2], Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rot = q.matrix();

    // ROS_INFO_STREAM("Rotation:"<< endl << rotationMatrix);
    cv::Mat extrinsic = (cv::Mat_<double>(4,4) << rot(0,0), rot(0,1), rot(0,2), m_translation[0],
                                                  rot(1,0), rot(1,1), rot(1,2), m_translation[1],
                                                  rot(2,0), rot(2,1), rot(2,2), m_translation[2],
                                                  0.0, 0.0, 0.0, 1.0);
    ROS_INFO_STREAM("Extrinsic:"<< endl << extrinsic);
    // write config file
    cv::FileStorage fs;
    fs.open(file_path, cv::FileStorage::WRITE);
    fs << "PointCloudTopic" << "/livox/lidar";
    fs << "ImageTopic" << "/camera/color/image_raw";
    fs << "Data_custom_msg" << 1;
    fs << "ExtrinsicMat" << extrinsic;
    fs << "Canny_gray_threshold" << rgb_canny_threshold_;
    fs << "Canny_len_threshold" << rgb_edge_minLen_;
    fs << "Voxel_size" << voxel_size_;
    fs << "Voxel_down_sample_size" << 0.02;
    fs << "Plane_min_points_size" << 60;
    fs << "Plane_normal_theta_min" << 30;
    fs << "Plane_normal_theta_max" << 150;
    fs << "Plane_max_size" << 5;
    fs << "Ransac_dis_threshold" << 0.015;
    fs << "Ransac_iter_num" << 200;
    fs << "Edge_min_dis_threshold" << 0.03;
    fs << "Edge_max_dis_threshold" << 0.06;
    fs << "Color_dense" << 1;
    fs << "Color_intensity_threshold" << 10;


    fs.release();
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "calib_tune");
    string image_file = "/tmp/mach9/auto_mlcc/image/front/0.bmp";
    string pcd_file = "/tmp/mach9/auto_mlcc/pcd/front/0.pcd";
    string calib_config_file = "/home/jason/map_ws/src/livox_camera_calib/config/config_outdoor.yaml";
    string save_path = "/tmp/mach9/auto_mlcc/edge_cfg/config_update.yaml";
    CalibTune cb = CalibTune(image_file, pcd_file, calib_config_file);
    // ROS_INFO("Complete initial image and point cloud edge extractions!");
    // cb.align_edges();
    // ROS_INFO("Complete initial residual image!");
    cb.save_config_file(save_path);

    // ros::Rate loop_rate(30);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}
