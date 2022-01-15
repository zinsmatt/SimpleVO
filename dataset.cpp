#include "dataset.h"

#include "common.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace vo {

Dataset7Scenes::Dataset7Scenes(const std::string& path) : path_(path) {}

bool Dataset7Scenes::init() {
    Camera::Ptr rgb_cam(new Camera(532.57, 531.54, 320.0, 240.0, 0.0, SE3(SO3(), Vec3::Zero())));
    Vec3 t_depth_rgb(0.02344903, 0.00617666, 0.01052531);
    Camera::Ptr depth_cam(new Camera(598.836, 587.618669, 320.0, 240.0, 0.0, SE3(SO3(), t_depth_rgb)));
    cameras_.push_back(rgb_cam);
    cameras_.push_back(depth_cam);

    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset7Scenes::next_frame() {

    boost::format fmt_rgb("%s/frame-%06d.color.png");
    boost::format fmt_depth("%s/frame-%06d.depth.png");
    cv::Mat img_left, img_right;
    img_left = cv::imread((fmt_rgb % path_ % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    img_right = cv::imread((fmt_depth % path_ % current_image_index_).str(), cv::IMREAD_UNCHANGED);

    if (img_left.data == nullptr || img_right.data == nullptr) {
        LOG(WARNING) << "Cannot find images at index " << current_image_index_;
        return nullptr;
    }

    // Resize the images (optionnal)
    // cv::Mat img_left_resized, img_right_resized;
    // cv::resize(img_left, img_left, cv::Size(), 0.5, 0.5,
    //            cv::INTER_NEAREST);
    // cv::resize(img_right, img_right, cv::Size(), 0.5, 0.5,
    //            cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = img_left;
    new_frame->right_img_ = img_right;
    ++current_image_index_;
    return new_frame;
}





DatasetKITTI::DatasetKITTI(const std::string& path) : path_(path) {}

bool DatasetKITTI::init() {
    std::ifstream fin(path_ + "/calib.txt");
    if (!fin) {
        LOG(ERROR) << "Cannot find " << path_ << "/calib.txt";
        return false;
    }

    for (int i = 0; i < 4; ++i) {   // load the 4 cameras
        char cam_name[3];
        for (int j = 0; j < 3; ++j) {
            fin >> cam_name[j];
        }

        double proj_data[12];
        for (int k = 0; k < 12; ++k) {
            fin >> proj_data[k];
        }
        Eigen::Matrix<double, 3, 4, Eigen::RowMajor> temp(proj_data);
        Mat33 K = temp.block<3, 3>(0, 0);
        Vec3 t = temp.block<3, 1>(0, 3);
        t = K.inverse() * t; // suppose the 3x4 matrix is the full projection P but we known R is identity so we can recover t and K
        // K = K * 0.5; // probably because downscaling the images
        Camera::Ptr new_cam(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)));
        cameras_.push_back(new_cam);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
    }

    fin.close();
    current_image_index_ = 0;
    return true;
}

Frame::Ptr DatasetKITTI::next_frame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat img_left, img_right;
    img_left = cv::imread((fmt % path_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
    img_right = cv::imread((fmt % path_ % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);

    if (img_left.data == nullptr || img_right.data == nullptr) {
        LOG(WARNING) << "Cannot find images at index " << current_image_index_;
        return nullptr;
    }

    // Resize the images (optionnal)
    // cv::Mat img_left_resized, img_right_resized;
    // cv::resize(img_left, img_left, cv::Size(), 0.5, 0.5,
    //            cv::INTER_NEAREST);
    // cv::resize(img_right, img_right, cv::Size(), 0.5, 0.5,
    //            cv::INTER_NEAREST);

    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = img_left;
    new_frame->right_img_ = img_right;
    ++current_image_index_;
    return new_frame;
}

}
