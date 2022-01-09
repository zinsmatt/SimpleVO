#ifndef FRONTEND_H
#define FRONTEND_H

#include <memory>

#include <opencv2/features2d.hpp>

#include "camera.h"
#include "common.h"
#include "frame.h"
#include "map.h"

namespace vo {

class Backend;
class Viewer;
    
enum class  FrontendStatus {INIT, TRACKING_GOOD, TRACKING_BAD, LOST};

class Frontend {
public:
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    bool add_frame(Frame::Ptr frame);

    void set_map(Map::Ptr map) { map_ = map; }

    void set_backend(std::shared_ptr<Backend> backend) { backend_ = backend; }
    void set_viewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus get_status() const { return status_; }

    void set_cameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    bool track();

    bool reset();

    int track_last_frame();

    int estimate_current_pose();

    bool insert_keyframe();

    bool stereo_init();

    int detect_new_features();

    int find_features_in_right();

    bool build_init_map();

    int triangulate_new_points();

    void set_observations_for_keyframe();



private:
    FrontendStatus status_ = FrontendStatus::INIT;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_;
    std::shared_ptr<Viewer> viewer_;

    SE3 relative_motion_;
    int tracking_inliers_ = 0;

    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    cv::Ptr<cv::GFTTDetector> gftt_;
};

}



#endif // FRONTEND_H