//
// Created by hekmat on 8/30/20.
//

#ifndef SKOLVIO_VISUAL_ODOMETRY_H
#define SKOLVIO_VISUAL_ODOMETRY_H

#include <vector>
#include <opencv2/opencv.hpp>

#include "pinhole_camera.h"

class VisualOdometry
{
public:

    enum FrameStage {
        STAGE_FIRST_FRAME,//��һ֡
        STAGE_SECOND_FRAME,//�ڶ�֡
        STAGE_DEFAULT_FRAME//Ĭ��֡
    };

    VisualOdometry(PinholeCamera* cam);
    virtual ~VisualOdometry();

    /// �ṩһ��ͼ��
    void addImage(const cv::Mat& img, int frame_id);


    cv::Mat getCurrentR() { return cur_R_; }

    cv::Mat getCurrentT() { return cur_t_; }

protected:

    virtual bool processFirstFrame();

    virtual bool processSecondFrame();

    virtual bool processFrame(int frame_id);

    double getAbsoluteScale(int frame_id);

    void featureDetection(cv::Mat image, std::vector<cv::Point2f> &px_vec);

    void featureTracking(cv::Mat image_ref, cv::Mat image_cur,
                         std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities);

protected:
    FrameStage frame_stage_;
    PinholeCamera *cam_;
    cv::Mat new_frame_;
    cv::Mat last_frame_;

    cv::Mat cur_R_;
    cv::Mat cur_t_;

    std::vector<cv::Point2f> px_ref_;
    std::vector<cv::Point2f> px_cur_;
    std::vector<double> disparities_;

    double focal_;
    cv::Point2d pp_;
};


#endif //SKOLVIO_VISUAL_ODOMETRY_H
