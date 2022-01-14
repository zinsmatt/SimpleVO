#include "viewer.h"

#include "common.h"
#include "feature.h"

namespace vo {


    Viewer::Viewer() {
        thread_ = std::thread(std::bind(&Viewer::thread_loop, this));
    }

    void Viewer::close() {
        is_running_ = false;
        thread_.join();
    }

    void Viewer::add_current_frame(Frame::Ptr frame) {
        std::unique_lock<std::mutex> lock(viewer_data_mutex);
        current_frame_ = frame;
    }

    void Viewer::update_map() {
        std::unique_lock<std::mutex> LOCK(viewer_data_mutex);
        assert(map_  != nullptr);
        active_keyframes_ = map_->get_active_keyframes();
        active_map_points_ = map_->get_active_map_points();
        all_map_points_ = map_->get_all_map_points();
        map_updated_ = true;
    }

    void Viewer::thread_loop() {
        pangolin::CreateWindowAndBind("Simple VO", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5.0, -10.0, 0.0, 0.0,0.0, 0.0, -1.0, 0.0)
        );

        pangolin::View& vis_display = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0 / 768.0).SetHandler(new pangolin::Handler3D(vis_camera));

        const float blue[3] = {0.0, 0.0, 1.0};
        const float green[3] = {0.0, 1.0, 0.0};

        while (!pangolin::ShouldQuit() && is_running_) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0, 1.0, 1.0, 1.0);
            vis_display.Activate(vis_camera);

            std::unique_lock<std::mutex> lock(viewer_data_mutex);
            if (current_frame_) {
                draw_frame(current_frame_, green);
                follow_current_frame(vis_camera);

                cv::Mat img = plot_frame_image();
                cv::imshow("left frame", img);
                cv::waitKey(1);
            }

            if (map_) {
                draw_map();
            }

            pangolin::FinishFrame();
            usleep(50000);
        }
        LOG(INFO) << "Stop viewer.";
    }

    cv::Mat Viewer::plot_frame_image() {
        cv::Mat img;
        cv::cvtColor(current_frame_->left_img_, img, CV_GRAY2BGR);
        for (const auto& feat : current_frame_->features_left_) {
            if (feat && feat->map_point_.lock()) {
                cv::circle(img, feat->position_.pt, 2, cv::Scalar(0, 255, 0), 2);
            }
        }
        return img;
    }

    void Viewer::follow_current_frame(pangolin::OpenGlRenderState &vis_camera) {
        SE3 cam_pose = current_frame_->get_Rt().inverse();
        pangolin::OpenGlMatrix m(cam_pose.matrix());
        vis_camera.Follow(m, true);
    }

    void Viewer::draw_frame(Frame::Ptr frame, const float* color) {
        SE3 cam_pose = frame->get_Rt().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;
        glPushMatrix();

        Sophus::Matrix4f m = cam_pose.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());
        if (color == nullptr) {
            glColor3f(1.0, 0.0, 0.0);
        } else {
            glColor3f(color[0], color[1], color[2]);
        }

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glEnd();
        glPopMatrix();
    }

    void Viewer::draw_map() {
        const float red[3] = {1.0, 0.0, 0.0};
        const float blue[3] = {0.0, 0.0, 1.0};

        // draw past keyframes
        for (auto& kf : active_keyframes_) {
            draw_frame(kf.second, red);
        }

        // draw map points
        glPointSize(2);
        glBegin(GL_POINTS);
        glColor3f(blue[0], blue[1], blue[2]);
        for (auto& p : all_map_points_) {
            if (active_map_points_.find(p.first) == active_map_points_.end()) {
                Vec3 pos = p.second->get_pos();
                glVertex3d(pos.x(), pos.y(), pos.z());
            }
        }

        glColor3f(red[0], red[1], red[2]);
        for (auto& p : active_map_points_) {
            Vec3 pos = p.second->get_pos();
            glVertex3d(pos.x(), pos.y(), pos.z());
        }
        glEnd();
    }

}