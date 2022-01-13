#include <iostream>

#include "feature.h"
#include "frame.h"
#include "map.h"
#include "visual_odometry.h"


#include <gflags/gflags.h>

DEFINE_string(config_file, "../config/default.yaml", "config file path");

int main(int argc, char *argv[])
{

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "Visual Odometry\n";
    
    vo::VisualOdometry::Ptr vo(new vo::VisualOdometry(FLAGS_config_file));
    assert(vo->init() == true);

    vo->run();

    google::ShutdownGoogleLogging();

    return 0;
}