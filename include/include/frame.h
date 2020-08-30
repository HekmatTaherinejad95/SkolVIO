//
// Created by hekmat on 8/30/20.
//
#pragma once
#ifndef SKOLVIO_FRAME_H
#define SKOLVIO_FRAME_H

#include "pinhole_camera.h"
#include "common_include.h"

namespace SkolVIO {

    // forward declare
    struct MapPoint;
    struct Feature;

    struct Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;           // id of this frame
        unsigned long keyframe_id_ = 0;  // id of key frame
        bool is_keyframe_ = false;
        double time_stamp_;
        SE3 pose_;                       // Tcw 形式Pose
        std::mutex pose_mutex_;
        cv::Mat left_img_, right_img_;   // stereo images

        // extracted features in left image
        std::vector<std::shared_ptr<Feature>> features_left_;
        // corresponding features in right image, set to nullptr if no corresponding
        std::vector<std::shared_ptr<Feature>> features_right_;

    public:  // data members
        Frame() {}

        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
              const Mat &right);

        // set and get pose, thread safe
        SE3 Pose() {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            return pose_;
        }

        void SetPose(const SE3 &pose) {
            std::unique_lock<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }


        void SetKeyFrame();


        static std::shared_ptr<Frame> CreateFrame();
    };

}  // namespace SkolVIO

#endif //SKOLVIO_FRAME_H
