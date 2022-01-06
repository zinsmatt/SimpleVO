#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>


namespace vo {

struct Frame;
struct MapPoint;

struct Feature
{
public:
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_;

    bool is_outlier_ = false;
    bool is_on_left_image_ = true;

    Feature() {}
    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}

};

}

#endif // FEATURE_H