#ifndef DATASET_H
#define DATASET_H

#include <memory>
#include "frame.h"
#include "camera.h"

namespace vo {

class Dataset {

public:
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& path);

    bool init();

    Frame::Ptr next_frame();

    Camera::Ptr get_camera(int cam_id) const {
        return cameras_[cam_id];
    }

private:
    std::string path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
};

}

#endif // DATASET_H