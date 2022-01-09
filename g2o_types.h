#ifndef G2O_TYPES_H
#define G2O_TYPES_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "common.h"

namespace vo {

class VertexPose6D : public g2o::BaseVertex<6, SE3> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    virtual void setToOriginImpl() override {
        _estimate = SE3();
    }

    virtual void oplusImpl(const double *update) override {
        // tx ty tz rx ry rz
        // first translation update and then rotation udpate in a 6d vector
        Eigen::Map<const Sophus::Vector6d> update_eigen(update);
        _estimate = SE3::exp(update_eigen) * _estimate;
    }

    virtual bool read(std::istream &) override { return true; }
    virtual bool write(std::ostream &) const override { return true; }

};

class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2, Vec2, VertexPose6D> {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectionPoseOnly(const Vec3 &pos, const Mat33& K)
        : pos3d_(pos), K_(K) {}
    
    virtual void computeError() override {
        const VertexPose6D *v = static_cast<VertexPose6D*>(_vertices[0]);
        SE3 Rt = v->estimate();
        Vec3 p = K_ * (Rt * pos3d_);
        p /= p.z();
        _error = _measurement - p.head<2>();
    }

    virtual void linearizeOplus() override {
        const VertexPose6D *v = static_cast<VertexPose6D*>(_vertices[0]);
        Sophus::SE3d Rt = v->estimate();
        double fx = K_(0, 0);
        double fy = K_(1, 1);
        Eigen::Vector3d Xc = Rt * pos3d_;
        double Z2 = std::pow(Xc.z(), 2);
        _jacobianOplusXi = Eigen::Matrix<double, 2, 6>::Zero();
        _jacobianOplusXi(0, 0) = -fx / Xc.z();
        _jacobianOplusXi(0, 2) = fx * Xc.x() / Z2;
        _jacobianOplusXi(0, 3) = fx * Xc.x() * Xc.y() / Z2;
        _jacobianOplusXi(0, 4) = -fx-fx * std::pow(Xc.x(), 2) / Z2;
        _jacobianOplusXi(0, 5) = fx * Xc.y() / Xc.z();

        _jacobianOplusXi(1, 1) = -fy / Xc.z();
        _jacobianOplusXi(1, 2) = fy * Xc.y() / Z2;
        _jacobianOplusXi(1, 3) = fy + fy * std::pow(Xc.y(), 2) / Z2;
        _jacobianOplusXi(1, 4) = -fy * Xc.y() * Xc.x() / Z2;
        _jacobianOplusXi(1, 5) = -fy * Xc.x() / Xc.z();
    }

    virtual bool read(std::istream &) override { return true; }
    virtual bool write(std::ostream &) const override { return true; }


    private:
        Vec3 pos3d_;
        Mat33 K_;

};


}

#endif // G2O_TYPES_H