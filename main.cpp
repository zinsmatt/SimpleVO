#include <iostream>

#include "feature.h"
#include "frame.h"
#include "map.h"


int main(int argc, char *argv[])
{

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::cout << "Simple Visual Odometry\n";

    vo::Frame::Ptr frame = vo::Frame::CreateFrame();
    vo::Feature feat;
    
    vo::Map::Ptr map(new vo::Map);
    map->insert_keyframe(frame);
    map->insert_map_point(vo::MapPoint::CreateNewMapPoint());
    map->clean_map();

    google::ShutdownGoogleLogging();

    return 0;
}