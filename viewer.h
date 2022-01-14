#ifndef VIEWER_H
#define VIEWER_H

#include <memory>
#include <thread>
#include <unordered_map>

#include <pangolin/pangolin.h>

#include "frame.h"
#include "map.h"
#include "map_point.h"

namespace vo {

class Viewer {
public:
    typedef std::shared_ptr<Viewer> Ptr;
    
    Viewer();

    void set_map(Map::Ptr map) { map_ = map; }
    void close();

    void add_current_frame(Frame::Ptr frame);
    void update_map();

private:
    void thread_loop();
    void draw_frame(Frame::Ptr frame, const float* color);
    void draw_map();
    void follow_current_frame(pangolin::OpenGlRenderState& vis_camera);

    cv::Mat plot_frame_image();

    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread thread_;
    bool is_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_map_points_;
    std::unordered_map<unsigned long, MapPoint::Ptr> all_map_points_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex;

};


}


#endif // VIEWER_H