#include "visual_odometry.h"

#include <chrono>
#include "config.h"

namespace vo {

VisualOdometry::VisualOdometry(const std::string& config_path)
    : config_path_(config_path) {}

bool VisualOdometry::init() {
    if (!Config::SetParameterFile(config_path_)) {
        return false;
    }

    dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->init(), true);

    frontend_ = Frontend::Ptr(new Frontend);
    // backend_ = new Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    // frontend_->set_backend(backend_);
    frontend_->set_map(map_);
    frontend_->set_viewer(viewer_);
    frontend_->set_cameras(dataset_->get_camera(0), dataset_->get_camera(1));

    // backend_->set_map(map_);
    // backend_->set_cameras(dataset_->get_camera(0), dataset_->get_camera(1));

    viewer_->set_map(map_);

    return true;
}

void VisualOdometry::run() {
    while (1) {
        LOG(INFO) << "VO is running";
        if (step() == false)
            break;
    }
}

bool VisualOdometry::step() {
    Frame::Ptr new_frame = dataset_->next_frame();
    if (!new_frame) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->add_frame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO step time: " << time_used.count() << " s.";
    return success;
}

}