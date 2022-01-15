#ifndef DATASET_H
#define DATASET_H

#include <memory>
#include "frame.h"
#include "camera.h"

namespace vo {

class DatasetBase {
public:
    typedef std::shared_ptr<DatasetBase> Ptr;

    virtual bool init() = 0;
    virtual Frame::Ptr next_frame() = 0;
    virtual Camera::Ptr get_camera(int cam_id) const = 0;
};


class Dataset7Scenes : public DatasetBase {
public:
    typedef std::shared_ptr<Dataset7Scenes> Ptr;
    Dataset7Scenes(const std::string& path);

    bool init();

    virtual Frame::Ptr next_frame();

    virtual Camera::Ptr get_camera(int cam_id) const {
        return cameras_[cam_id];
    }

private:
    std::string path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
};



class DatasetKITTI : public DatasetBase {

public:
    typedef std::shared_ptr<DatasetKITTI> Ptr;
    DatasetKITTI(const std::string& path);

    bool init();

    virtual Frame::Ptr next_frame();

    virtual Camera::Ptr get_camera(int cam_id) const {
        return cameras_[cam_id];
    }

private:
    std::string path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
};

}

#endif // DATASET_H