#include "camera.h"


namespace vo {


    Vec3 Camera::world2camera(const Vec3 &pw, const SE3 &Tcw) {
        return pose_ * Tcw * pw;
    }

    Vec3 Camera::camera2world(const Vec3 &pc, const SE3 &Tcw) {
        return Tcw.inverse() * pose_inv_ * pc;
    }

    Vec2 Camera::camera2pixel(const Vec3 &pc) {
        Vec2 uv(fx_ * pc.x() / pc.z() + cx_, 
                fy_ * pc.y() / pc.z() + cy_);
        return uv;
    }

    Vec3 Camera::pixel2camera(const Vec2 &pp, double depth) {
        Vec3 pc((pp.x() - cx_) / fx_, 
                (pp.y() - cy_) / fy_,
                1.0);
        return depth * pc;
    }
    

}