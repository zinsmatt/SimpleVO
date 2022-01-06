#ifndef FRAME_H
#define FRAME_H

#include <memory>
#include <mutex>
// #include <thread>

#include "common.h"

#include <opencv2/core/core.hpp>

namespace vo {

struct Feature;

struct Frame {

public:
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;
    unsigned long keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double timestamp_;
    SE3 Rt_;
    std::mutex Rt_mutex;
    cv::Mat left_img_, right_img_;

    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;

    Frame() {}
    Frame(long id, double timestamp, const SE3 &Rt, 
          const cv::Mat &left, const cv::Mat& right);


    SE3 get_Rt() const {
        // std::unique_lock<std::mutex> lock(Rt_mutex);
        return Rt_;
    }

    void set_Rt(const SE3 &Rt)
    {
        std::unique_lock<std::mutex> lock(Rt_mutex);
        Rt_ = Rt;
    }

    void set_is_keyframe();

    static Frame::Ptr CreateFrame();

};


}




#endif // FRAME_H