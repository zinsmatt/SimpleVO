#include "frame.h"

namespace vo
{

    Frame::Frame(long id, double timestamp, 
                const SE3 &Rt, const cv::Mat &left,
                const cv::Mat &right) :
                id_(id), timestamp_(timestamp), Rt_(Rt),
                left_img_(left), right_img_(right)
                {}

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0;
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }

    void Frame::set_is_keyframe() {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }


}