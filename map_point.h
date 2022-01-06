#ifndef MAP_POINT_H
#define MAP_POINT_H

#include <list>
#include <memory>
#include <mutex>

#include "common.h"

namespace vo {

struct Feature;

struct MapPoint {

public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero();
    std::mutex data_mutex_;
    int observed_times_ = 0;
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}
    MapPoint(long id, Vec3 position);

    Vec3 get_pos() const {
        return pos_;
    }

    void set_pos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        pos_ = pos;
    }

    void add_observation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void remove_observation(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> get_obs() {
        return observations_;
    }

    static MapPoint::Ptr CreateNewMapPoint();
};

}

#endif // MAP_POINT_H