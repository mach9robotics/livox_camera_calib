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
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <bits/stdc++.h>
#include <filesystem>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>


using namespace std;
using namespace cv;

class CalibTune: public Calibration 
{
public:

    CalibTune(const std::string &image_file,
              const std::string &pcd_file,
              const std::string &calib_config_file) : m_it(m_nh),
              Calibration(image_file, pcd_file, calib_config_file)
    {
        // define paths
        m_image_path = image_file;
        m_pcd_path = pcd_file;
        m_config_path = calib_config_file;

        // setup publisher
        m_image_pub = m_it.advertise("/edge_align_image",1);
        // init image message & setup image header
        m_image_header.seq = 0;
        m_image_header.frame_id = "livox";
        m_image_header.stamp = ros::Time::now();
        sensor_msgs::ImagePtr pub_image = cv_bridge::CvImage(
            m_image_header, "bgr8", image_).toImageMsg();
        m_image_pub.publish(pub_image);
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

        align_edges();
    }
    void align_edges();
    void save_config_file(string& file_path);
    // string find_pcd_path(string& image_path);
    void load_image(string& image_path);
    void load_cloud(string& pcd_path);

private:
    void dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level);

private:
    bool m_prev_exec = false;
    bool m_prev_save = false;
    bool m_prev_load = false;
    unordered_set<uint32_t> m_changes;
    Eigen::Vector3d m_rotation = init_rotation_matrix_.eulerAngles(0,1,2);
    Eigen::Vector3d m_translation = init_translation_vector_;
    image_transport::Publisher m_image_pub;
    std_msgs::Header m_image_header;

    ros::NodeHandle m_nh;
    image_transport::ImageTransport m_it;
    dynamic_reconfigure::Server<livox_camera_calib::CalibTuneConfig> m_server;

    vector<double> m_camera_matrix;
    vector<double> m_dist_coeffs;

    string m_image_path;
    string m_pcd_path;
    string m_config_path;
    string m_save_path;
};

string find_pcd_path(string& image_path);

void CalibTune::dyncfg_cb(livox_camera_calib::CalibTuneConfig &config, uint32_t level)
{
    // ROS_INFO("Level: %i", level);
    if (level == 2 || level == 5) 
    {
        m_changes.insert(level);
    }
    // set values
    this->m_save_path = config.save_path;
    if (level == 1)
    {
        this->rgb_canny_threshold_ = config.grey_threshold;
        this->rgb_edge_minLen_ = config.len_threshold;
        ROS_INFO_STREAM("start image edge extraction");
        cv::Mat edge_image;
        // cv::Mat grey_image = image_;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr rgb_edge_cloud = rgb_egde_cloud_;
        this->edgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_, edge_image, rgb_egde_cloud_);
        ROS_INFO_STREAM("complete image edge extraction");
        this->align_edges();
    }
    if (level == 2)
    {
        this->voxel_size_ = config.voxel_size;
    }
    if (level == 3)
    {
        this->m_translation[0] = config.translation_x;
        this->m_translation[1] = config.translation_y;
        this->m_translation[2] = config.translation_z;
        this->m_rotation[0] = config.rotation_x;
        this->m_rotation[1] = config.rotation_y;
        this->m_rotation[2] = config.rotation_z;
        this->align_edges();
    }
    if (level == 4)
    {
        this->m_save_path = config.save_path;
    }
    if (level == 5)
    {   
        if (config.config_path != "")
        {
            this->m_config_path = config.config_path;
        }
        if (config.image_path != "")
        {
            this->m_image_path = config.image_path;
            this->m_pcd_path = find_pcd_path(this->m_image_path);
        }
    } 

    // execuate and show residual image
    if (this->m_prev_exec != config.execuate)
    {
        if (m_changes.size()!=0) {
            if (m_changes.find(2) != m_changes.end()) {
                ROS_INFO_STREAM("start point cloud edge extraction");
                std::unordered_map<VOXEL_LOC, Voxel *> voxel_map;
                initVoxel(raw_lidar_cloud_, voxel_size_, voxel_map);
                LiDAREdgeExtraction(voxel_map, ransac_dis_threshold_, plane_size_threshold_, plane_line_cloud_);
                ROS_INFO_STREAM("complete point cloud edge extraction");
            }
            this->align_edges();
            m_changes.clear();
        }
        this->m_prev_exec = config.execuate;
    }
    // load new image and pcd
    if (this->m_prev_load != config.load_input && m_image_path != "" && m_config_path != "")
    {
        if (m_changes.size()!=0) {
            if (m_changes.find(5) != m_changes.end()) {
                this->loadCalibConfig(this->m_config_path);
                this->m_rotation = this->init_rotation_matrix_.eulerAngles(0,1,2);
                this->m_translation = this->init_translation_vector_;
                this->load_image(this->m_image_path);
                ROS_INFO_STREAM("start image edge extraction");
                cv::Mat edge_image;
                this->edgeDetector(rgb_canny_threshold_, rgb_edge_minLen_, grey_image_, edge_image, rgb_egde_cloud_);
                ROS_INFO_STREAM("complete image edge extraction");
                this->load_cloud(this->m_pcd_path);
                ROS_INFO_STREAM("start point cloud edge extraction");
                std::unordered_map<VOXEL_LOC, Voxel *> voxel_map;
                initVoxel(raw_lidar_cloud_, voxel_size_, voxel_map);
                LiDAREdgeExtraction(voxel_map, ransac_dis_threshold_, plane_size_threshold_, plane_line_cloud_);
                ROS_INFO_STREAM("complete point cloud edge extraction");
            }
            this->align_edges();
            m_changes.clear();
        }
        this->m_prev_load = config.load_input;
    }
    // save config file
    if (this->m_prev_save != config.save && m_save_path != "")
    {
        this->save_config_file(this->m_save_path);
        this->m_prev_save = config.save;
    }
}

void CalibTune::align_edges()
{
    // parse and store 6d pose to calib_params
    Eigen::Vector3d init_euler_angle = this->m_rotation;
    Eigen::Vector3d init_translation = this->m_translation;
    Vector6d calib_params;
    calib_params << init_euler_angle(0), init_euler_angle(1), init_euler_angle(2),
        init_translation(0), init_translation(1), init_translation(2);

    // initialize image point container
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
        Eigen::AngleAxisd(calib_params[0], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(calib_params[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(calib_params[2], Eigen::Vector3d::UnitZ());
    // ROS_WARN_STREAM("rotation vector: " <<rotation_vector3.matrix().eulerAngles(0,1,2).transpose());
    ROS_WARN_STREAM("rotation vector (Matrix): "<< endl <<rotation_vector3.matrix()<<endl);
    // ROS_WARN_STREAM("rotation vector (angle): " <<rotation_vector3.angle());
    // ROS_WARN_STREAM("rotation vector (axis): " <<rotation_vector3.axis().transpose());
    for (size_t i = 0; i < plane_line_cloud_->size(); i++) {
        pcl::PointXYZI point_3d = plane_line_cloud_->points[i];
        pts_3d.emplace_back(cv::Point3d(point_3d.x, point_3d.y, point_3d.z));
    }
    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
    cv::Mat distortion_coeff =
        (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
    // cv::Mat r_vec =
    //     (cv::Mat_<double>(3, 1)
    //         << rotation_vector3.angle() * rotation_vector3.axis().transpose()[2],
    //     rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
    //     rotation_vector3.angle() * rotation_vector3.axis().transpose()[0]);
    auto rm = rotation_vector3.matrix();
    cv::Mat r_vec = (cv::Mat_<double>(3,3) << rm(0,0), rm(0,1), rm(0,2),
                                              rm(1,0), rm(1,1), rm(1,2),
                                              rm(2,0), rm(2,1), rm(2,2));
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
    // cv::imshow("residual", residual_img);
    // cv::waitKey(0);
    m_image_header.seq += 1;
    m_image_header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr pub_image = cv_bridge::CvImage(
        m_image_header, "bgr8", residual_img).toImageMsg();
    m_image_pub.publish(pub_image);
}

void CalibTune::save_config_file(string& file_path) {
    // check directory
    string file_dir = file_path.substr(0, file_path.find_last_of("/"));
    if (!filesystem::exists(file_dir)){
        filesystem::create_directories(file_dir);
    }

    std::ofstream ofs(file_path);
    ofs<< "new file" << endl;
    ofs.close();

    // calculate updated rotation matrix 
    Eigen::AngleAxisd rollAngle(m_rotation[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd yawAngle(m_rotation[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(m_rotation[2], Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::Matrix3d rot = q.matrix();

    // ROS_INFO_STREAM("Rotation:"<< endl << rotationMatrix);
    cv::Mat extrinsic = (cv::Mat_<double>(4,4) << rot(0,0), rot(0,1), rot(0,2), m_translation[0],
                                                  rot(1,0), rot(1,1), rot(1,2), m_translation[1],
                                                  rot(2,0), rot(2,1), rot(2,2), m_translation[2],
                                                  0.0, 0.0, 0.0, 1.0);
    // ROS_INFO_STREAM("Extrinsic:"<< endl << extrinsic);
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
    fs << "match_dis" << 25.0;
    fs << "match_dis_threshold_min" << 8;
    fs << "match_dis_threshold_max" << 20;

    fs.release();
    ROS_INFO_STREAM("Saved config file to " << file_path);
}



string find_pcd_path(string& image_path) 
{
    vector<string> words_0;
    boost::split(words_0, image_path, boost::is_any_of("/"), boost::token_compress_on);
    words_0[words_0.size()-3] = "pcd";
    string pcd_path;
    for (auto w:words_0) {
        pcd_path += w;
        if (w!=words_0[words_0.size()-1]) pcd_path += "/";
    }
    pcd_path.replace(pcd_path.size()-3,3,"pcd");
    return pcd_path;
}

void CalibTune::load_image(string& image_path) 
{
    image_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
    if (!image_.data) {
        std::string msg = "Can not load image from " + image_path;
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    } else {
        std::string msg = "Sucessfully load image!";
        ROS_INFO_STREAM(msg.c_str());
    }
    width_ = image_.cols;
    height_ = image_.rows;
    // check rgb or gray
    if (image_.type() == CV_8UC1) {
        grey_image_ = image_;
    } else if (image_.type() == CV_8UC3) {
        cv::cvtColor(image_, grey_image_, cv::COLOR_BGR2GRAY);
    } else {
        std::string msg = "Unsupported image type, please use CV_8UC3 or CV_8UC1";
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    }
}

void CalibTune::load_cloud(string& pcd_path) 
{
    raw_lidar_cloud_ =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    ROS_INFO_STREAM("Loading point cloud from pcd file.");
    if (!pcl::io::loadPCDFile(pcd_path, *raw_lidar_cloud_)) {
        std::string msg = "Sucessfully load pcd, pointcloud size: " +
                            std::to_string(raw_lidar_cloud_->size());
        ROS_INFO_STREAM(msg.c_str());
    } else {
        std::string msg = "Unable to load " + pcd_path;
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    }
}


int main(int argc, char *argv[]) 
{
    ros::init(argc, argv, "calib_tune");
    string image_file, pcd_file, calib_config_file, save_path, multi_calib_file;
    // load from ros parameter server
    ros::NodeHandle nh;
    nh.getParam("multi_calib_path", multi_calib_file);
    nh.getParam("image_path", image_file);
    nh.getParam("calib_config_path", calib_config_file);
    nh.getParam("pcd_path", pcd_file);
    if (pcd_file == "")
    {
        pcd_file = find_pcd_path(image_file);
    }
    // check image and pcd file
    ROS_INFO_STREAM("multi calib file: " << multi_calib_file);
    if (image_file.empty() || calib_config_file.empty() || multi_calib_file.empty() || pcd_file.empty()) 
    {
        ROS_ERROR_STREAM("Insuficient arguments, please check your input.");
        exit(-1);
    }
    std::vector<std::string> input_files = {image_file, pcd_file, calib_config_file, multi_calib_file};
    for (auto file:input_files) {
        if (!boost::filesystem::exists(file)) {
            ROS_ERROR_STREAM("File " << file << " does not exist.");
            exit(-1);
        }
    }
    CalibTune cb = CalibTune(image_file, pcd_file, calib_config_file);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
