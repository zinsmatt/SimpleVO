#ifndef CAMERA_H
#define CAMERA_H

#include <memory>

#include "common.h"

namespace vo {

class Camera {
public:
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
    double baseline_ = 0.0;

    SE3 pose_;  // camera frame to left/right camera
    SE3 pose_inv_;

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
            pose_inv_ = pose_.inverse();
        }
    
    SE3 pose() const { return pose_; }
    Mat33 build_K() const {
        Mat33 K;
        K << fx_, 0.0, cx_, 
             0.0, fy_, cy_,
             0.0, 0.0, 1.0;
        return K;
    }



    Vec3 world2camera(const Vec3 &pw, const SE3 &Tcw);
    Vec3 camera2world(const Vec3 &pc, const SE3 &Tcw);

    Vec2 camera2pixel(const Vec3 &pc);
    Vec3 pixel2camera(const Vec2 &pp, double depth=1.0);

    Vec3 pixel2world(const Vec2 &pp, const SE3 &Tcw, double depth=1.0) {
        return camera2world(pixel2camera(pp, depth), Tcw);
    }

    Vec2 world2pixel(const Vec3 &pw, const SE3 &Tcw) {
        return camera2pixel(world2camera(pw, Tcw));
    }

};


    
}

#endif // CAMERA_H