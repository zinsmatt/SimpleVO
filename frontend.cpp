#include "frontend.h"

#include <opencv2/opencv.hpp>

#include "algorithm.h"
#include "camera.h"
#include "feature.h"
#include "g2o_types.h"

namespace vo {

    Frontend::Frontend() {
        gftt_ = cv::GFTTDetector::create(num_features_, 0.01, 20);        
    }

    bool Frontend::add_frame(Frame::Ptr frame) {
        current_frame_ = frame;
        LOG(INFO) << "Add new frame status";
        switch (status_) {
            case FrontendStatus::INIT:
                LOG(INFO) << "status init";
                stereo_init();
                break;
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                LOG(INFO) << "status tracking";
                track();
            case FrontendStatus::LOST:
                LOG(INFO) << "status lost";
                reset();
                break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::track() {
        if (last_frame_) {
            // if last_frame exists increment it with relative motion for prediction
            current_frame_->set_Rt(relative_motion_ * last_frame_->get_Rt());
        }

        int num_track_last = track_last_frame();
        tracking_inliers_ = estimate_current_pose();

        if (tracking_inliers_ > num_features_tracking_) {
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (tracking_inliers_ > num_features_tracking_bad_) {
            status_ = FrontendStatus::TRACKING_BAD;
        } else {
            status_ = FrontendStatus::LOST;
        }

        insert_keyframe();
        relative_motion_ = current_frame_->get_Rt() * last_frame_->get_Rt().inverse();


        // if (viewer_)
        //     viewer_->add_current_frame(current_frame_);
        
        return true;
    }

    int Frontend::track_last_frame() {
        // track features from last frame to current frame (always the left image)
        int n_features_left = last_frame_->features_left_.size();
        std::vector<cv::Point2f> kps_last(n_features_left), kps_current(n_features_left);
        int index = 0;
        for (auto& feat: last_frame_->features_left_) {
            if (feat->map_point_.lock()) {
                auto mp = feat->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->get_pos(), current_frame_->get_Rt());
                kps_last[index] = feat->position_.pt;
                kps_current[index] = cv::Point2f(px.x(), px.y());
            } else {
                kps_last[index] = feat->position_.pt;
                kps_current[index] = feat->position_.pt;
            }
            ++index;
        }

        std::vector<unsigned char> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_,
                                 kps_last, kps_current, status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.001),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int  num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr new_feat(new Feature(current_frame_, kp));
                new_feat->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(new_feat);
                ++num_good_pts;
            }
        }

        LOG(INFO) << "Found " << num_good_pts << " good track from last image.";
        return num_good_pts;
    }

    int Frontend::estimate_current_pose() {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        VertexPose6D *vertex_pose = new VertexPose6D();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->get_Rt());
        optimizer.addVertex(vertex_pose);

        Mat33 K = camera_left_->build_K();

        int index = 1;
        std::vector<EdgeProjectionPoseOnly*> edges;
        std::vector<Feature::Ptr> features_with_mp;
        for (auto& feat : current_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp) {
                // this feature has an associated map point
                features_with_mp.push_back(feat);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(
                    mp->get_pos(), K
                );
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                Vec2 meas(feat->position_.pt.x, feat->position_.pt.y);
                edge->setMeasurement(meas);
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                ++index;
            }
        }

        const double chi2_th = 5.991;
        int nb_outliers = 0;
        for (int iter = 0; iter < 4; ++iter) {
            // several iterations for outlier removal
            vertex_pose->setEstimate(current_frame_->get_Rt());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            nb_outliers = 0;

            for (size_t i = 0; i < edges.size(); ++i) {
                auto *e = edges[i];
                if (features_with_mp[i]->is_outlier_) {
                    e->computeError(); // compute error even for outliers which are during optimization
                }
                if (e->chi2() > chi2_th) {
                    features_with_mp[i]->is_outlier_ = true;
                    e->setLevel(1);
                    nb_outliers++;
                } else {
                    features_with_mp[i]->is_outlier_ = false;
                    e->setLevel(0);
                }
                if (iter == 2) {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outliers/Inliers in pose estimation : " << nb_outliers 
                  << " / " << features_with_mp.size();
        current_frame_->set_Rt(vertex_pose->estimate());

        LOG(INFO) << "Current pose = \n" << current_frame_->get_Rt().matrix();

        for (auto &feat : features_with_mp) {
            if (feat->is_outlier_) {
                feat->map_point_.reset();
                feat->is_outlier_ = false; // maybe we can still use them in future
            }
        }

        return features_with_mp.size() - nb_outliers;
    }


    bool Frontend::insert_keyframe() {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
            // still enough tracks to continue tracking
            return false;
        }

        current_frame_->set_is_keyframe();
        map_->insert_keyframe(current_frame_);
        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_ << "\n";
        set_observations_for_keyframe();

        detect_new_features();

        find_features_in_right();

        triangulate_new_points();

        // update backend with new mp

    }

    void Frontend::set_observations_for_keyframe() {
        for (auto& feat : current_frame_->features_left_) {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->add_observation(feat);
        }
    }


    int Frontend::detect_new_features() {
        // Detect new features in left image
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        cv::Point2f shift(10, 10);
        for (auto& feat : current_frame_->features_left_) {
            cv::rectangle(mask, feat->position_.pt - shift, feat->position_.pt + shift, 0, CV_FILLED);
        }

        std::vector<cv::KeyPoint> new_kps;
        gftt_->detect(current_frame_->left_img_, new_kps, mask);
        int nb_detected = new_kps.size();
        int index = current_frame_->features_left_.size();
        current_frame_->features_left_.resize(index + nb_detected);
        for (auto& kp : new_kps) {
            current_frame_->features_left_[index++] 
                = Feature::Ptr(new Feature(current_frame_, kp));
        }

        LOG(INFO) << "Detect " << nb_detected << " new features.";
        return nb_detected;
    }

    int Frontend::find_features_in_right() {
        // Use LK flow to find features in right image
        std::vector<cv::Point2f> kps_left(current_frame_->features_left_.size()),
                                 kps_right(current_frame_->features_left_.size());
        int index = 0;
        for (auto &kp : current_frame_->features_left_) {
            kps_left[index] = kp->position_.pt;
            auto mp = kp->map_point_.lock();
            if (mp) {
                auto px = camera_right_->world2pixel(mp->get_pos(), current_frame_->get_Rt());
                kps_right[index] = cv::Point2f(px.x(), px.y());
            } else {
                kps_right[index] = kp->position_.pt;
            }
            ++index;
        }
        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_,
                                 kps_left, kps_right, status, error, cv::Size(11, 11), 3,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        int nb_goods = 0;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                ++nb_goods;
            } else {
                current_frame_->features_right_.push_back(nullptr);
            }
        }

        LOG(INFO) << "Found " << nb_goods << " features in the right image\n";
        return nb_goods;
    }

    int Frontend::triangulate_new_points() {
        std::vector<SE3> poses({camera_left_->pose(), camera_right_->pose()});
        SE3 current_pose_Twc = current_frame_->get_Rt().inverse();
        int nb_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i] &&
                current_frame_->features_left_[i]->map_point_.expired()) {
                // only if a corresponding feature exist in the right image
                // and if the left feature is not associated to a map point 
                // anymore
                Vec2 px_left(current_frame_->features_left_[i]->position_.pt.x,
                                current_frame_->features_left_[i]->position_.pt.y);
                Vec2 px_right(current_frame_->features_right_[i]->position_.pt.x,
                                current_frame_->features_right_[i]->position_.pt.y);
                std::vector<Vec3> points({camera_left_->pixel2camera(px_left, 1.0),
                                            camera_right_->pixel2camera(px_right, 1.0)});
                Vec3 pw = Vec3::Zero();
                if (triangulate(poses, points, pw) && pw.z() > 0) {
                    MapPoint::Ptr new_mp = MapPoint::CreateNewMapPoint();
                    new_mp->set_pos(current_pose_Twc * pw);
                    new_mp->add_observation(current_frame_->features_left_[i]);
                    new_mp->add_observation(current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_mp;
                    current_frame_->features_right_[i]->map_point_ = new_mp;
                    map_->insert_map_point(new_mp);
                    ++nb_triangulated_pts;
                }
            }
        }
        LOG(INFO) << "New " << nb_triangulated_pts << " triangulated points";
        return nb_triangulated_pts;
    }

    bool Frontend::stereo_init() {
        int nb_features_left = detect_new_features();
        int nb_correspond_right = find_features_in_right();

        if (nb_correspond_right < num_features_init_) {
            return false;
        }

        bool build_map_success = build_init_map();
        if (build_map_success) {
            status_ = FrontendStatus::TRACKING_GOOD;
            return true;
        }
        return false;
    }


    bool Frontend::build_init_map() {
        std::vector<SE3> poses({camera_left_->pose(), camera_right_->pose()});
        int nb_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            if (current_frame_->features_right_[i]) {
                Vec2 px_left(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y);
                Vec2 px_right(current_frame_->features_right_[i]->position_.pt.x,
                              current_frame_->features_right_[i]->position_.pt.y);
                std::vector<Vec3> points({camera_left_->pixel2camera(px_left, 1.0),
                                          camera_right_->pixel2camera(px_right, 1.0)});
                Vec3 pw = Vec3::Zero();
                bool ret = triangulate(poses, points, pw);
                // std::cout << "ret triangulatation" << ret << "\n";
                // std::cout << pw.transpose() << "\n";
                if (ret && pw.z() > 0) {
                    MapPoint::Ptr new_mp = MapPoint::CreateNewMapPoint();
                    new_mp->set_pos(pw);
                    new_mp->add_observation(current_frame_->features_left_[i]);
                    new_mp->add_observation(current_frame_->features_right_[i]);
                    current_frame_->features_left_[i]->map_point_ = new_mp;
                    current_frame_->features_right_[i]->map_point_ = new_mp;
                    map_->insert_map_point(new_mp);
                    ++nb_triangulated_pts;
                }
            }
        }
        std::cout << "nb triangulated points " << nb_triangulated_pts << "\n";
        if (nb_triangulated_pts < min_num_points_init_map) {
            LOG(INFO) << "Not enough triangulated points " << nb_triangulated_pts;
            return false;
        }

        current_frame_->set_is_keyframe();
        map_->insert_keyframe(current_frame_);
        // backend->udpate_map();

        LOG(INFO) << "Initial map created with " << nb_triangulated_pts << " points.";
        return true;
    }

    bool Frontend::reset() {
        LOG(INFO) << "Reset is not implemented.";
        return false;
    }




}