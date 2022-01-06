#include "map_point.h"

#include "feature.h"

namespace vo {

    MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

    MapPoint::Ptr MapPoint::CreateNewMapPoint() {
        static long factory_id = 0;
        MapPoint::Ptr new_map_point(new MapPoint);
        new_map_point->id_ = factory_id++;
        return new_map_point;
    }

    void MapPoint::remove_observation(std::shared_ptr<Feature> feat)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        for (auto it = observations_.begin(); it != observations_.end(); ++it)
        {
            if (it->lock() == feat) {
                observations_.erase(it);
                feat->map_point_.reset();
                observed_times_--;
                break;
            }
        }
    }


}
