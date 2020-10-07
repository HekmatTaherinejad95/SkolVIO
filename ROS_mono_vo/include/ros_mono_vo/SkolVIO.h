//
// Created by hekmat on 10/7/20.
//

#ifndef ROS_MONO_VO_SKOLVIO_H
#define ROS_MONO_VO_SKOLVIO_H

#include "util.h"
#include <iostream>
#include <string>
#include <iterator>
#include <fstream>
#include <map>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>



class SkolVIO {
public:
    SkolVIO(int max_frame, int min_num_pts, std::string dataset_path);
    void FeatureTracking(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> &p1, std::vector<cv::Point2f> &p2);
    void FeatureDetection(cv::Mat img, std::vector<cv::Point2f> &p);
    void Initialization();
    void PoseTracking(int num_frame);
    void Visualize(int num_frame);
    void ReduceVector(std::vector<int> &v);
    void FetchIntrinsicParams();
    void ComputeAbsoluteScale(int num_frame);
    cv::Mat GetRotation() { return prev_R_; }
    cv::Mat GetTranslation() { return prev_t_; }
    cv::Mat GetResultImage() { return dst_; }

private:
    int max_frame_;
    int min_num_pts_;
    std::string dataset_path_;
    std::string data_calib_;
    std::string data_poses_;
    std::string data_images_;
    cv::Mat prev_img_;
    cv::Mat curr_img_;
    cv::Mat dst_;
    std::vector<cv::Point2f> prev_pts_;
    std::vector<cv::Point2f> curr_pts_;
    cv::Mat prev_R_;
    cv::Mat prev_t_;
    cv::Mat curr_R_;
    cv::Mat curr_t_;
    double scale_;
    double f_;
    cv::Point2f pp_;
    cv::Mat E_;
    std::vector<unsigned char> status_;
    cv::Mat mask_;
    cv::Mat img_traj_;
    std::vector<int> idx_;
    unsigned int id_;
    std::map<int, cv::Point2f> prev_pts_map_;
};

#endif //ROS_MONO_VO_SKOLVIO_H
