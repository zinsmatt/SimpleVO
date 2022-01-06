#ifndef MAP_H
#define MAP_H

#include <memory>
#include <unordered_map>

#include "frame.h"
#include "map_point.h"

namespace vo {

class Map {

public:
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;


    Map() {}

    void insert_keyframe(Frame::Ptr frame);

    void insert_map_point(MapPoint::Ptr map_point);

    LandmarksType get_all_map_points() {
        return landmarks_;
    }

    KeyframesType get_all_keyframes() {
        return keyframes_;
    }

    LandmarksType get_active_map_points() {
        return active_landmarks_;
    }

    KeyframesType get_active_keyframes() {
        return active_keyframes_;
    }

    void clean_map();

private:
    void remove_old_keyframes();
    std::mutex data_mutex_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframesType keyframes_;
    KeyframesType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    unsigned int nb_active_keyframes_ = 7;
};


}

#endif // MAP_H