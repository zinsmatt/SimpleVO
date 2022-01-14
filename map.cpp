#include "map.h"

#include <iostream>

#include "common.h"
#include "feature.h"

namespace vo {

void Map::insert_keyframe(Frame::Ptr frame) {
    frame->set_is_keyframe();
    current_frame_ = frame;
    keyframes_[frame->keyframe_id_] = frame;
    active_keyframes_[frame->keyframe_id_] = frame;

    if (active_keyframes_.size() > nb_active_keyframes_) {
        remove_old_keyframes();
    }
}

void Map::insert_map_point(MapPoint::Ptr map_point) {
    landmarks_[map_point->id_] = map_point;
    active_landmarks_[map_point->id_] = map_point;
}

void Map::remove_old_keyframes() {
    if (current_frame_ == nullptr) return;

    double max_dist = 0.0, min_dist = std::numeric_limits<double>::infinity();
    double max_kf_id = 0, min_kf_id = 0;

    auto Twc = current_frame_->get_Rt().inverse();
    for (const auto& kf : active_keyframes_)
    {
        if (kf.second == current_frame_) 
            continue;

        auto dist = (kf.second->get_Rt() * Twc).log().norm();
        if (dist > max_dist) {
            max_dist = dist;
            max_kf_id = kf.first;
        }
        if (dist < min_dist) {
            min_dist = dist;
            min_kf_id = kf.first;
        }
    }

    const double min_dist_threshold = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dist < min_dist_threshold) { // if very close frames, remove them first
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {    // else remove the farthest ones
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_ << "\n";

    // Remove the observations corresponding to the features in frame_to_remove
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto& feat : frame_to_remove->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp)
            mp->remove_observation(feat);
    }
    int i=0;
    for (auto& feat : frame_to_remove->features_right_) {
        if (!feat) continue;
        auto mp = feat->map_point_.lock();
        if (mp)
            mp->remove_observation(feat);
        ++i;
    }
    clean_map();
}

void Map::clean_map() {
    int count_landmarks_removed = 0;
    for (auto it = active_landmarks_.begin(); it != active_landmarks_.end();) {
        if (it->second->observed_times_ == 0) {
            it = active_landmarks_.erase(it);
            count_landmarks_removed++;
        } else {
            ++it;
        }
    }
    LOG(INFO) << "removed " << count_landmarks_removed << " active landmarks";
}

}