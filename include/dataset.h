//
// Created by hekmat on 8/30/20.
//

#ifndef SKOLVIO_DATASET_H
#define SKOLVIO_DATASET_H
#include "pinhole_camera.h"
#include "common_include.h"
#include "frame.h"

namespace SkolVIO {

    class Dataset {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string& dataset_path);


        bool Init();

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        /// get camera by id
        Camera::Ptr GetCamera(int camera_id) const {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };
}  // namespace SkolVIO

#endif //SKOLVIO_DATASET_H
