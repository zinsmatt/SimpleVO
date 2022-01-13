#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <memory>

#include "dataset.h"
#include "frontend.h"
#include "map.h"

namespace vo {

class VisualOdometry {

public:
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(const std::string& config_path);

    bool init();

    void run();

    bool step();

    FrontendStatus get_frontend_status() const { return frontend_->get_status(); }


private:
    bool initialized_ = false;
    std::string config_path_;

    Frontend::Ptr frontend_ = nullptr;
    Map::Ptr map_ = nullptr;
    // backend
    // viewer

    // dataset
    Dataset::Ptr dataset_ = nullptr;
};



}

#endif // VISUAL_ODOMETRY