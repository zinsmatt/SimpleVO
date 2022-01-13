#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "common.h"

namespace vo {

inline bool triangulate(const std::vector<SE3> &poses,
                        const std::vector<Vec3> &points,
                        Vec3 &pw) 
{
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    for (size_t i = 0; i < poses.size(); ++i)
    {
        auto P = poses[i].matrix3x4();
        A.row(i * 2) = points[i].x() * P.row(2) - P.row(0);
        A.row(i * 2 + 1) = points[i].y() * P.row(2) - P.row(1);
    }

    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pw = svd.matrixV().col(3).head<3>();
    pw /= svd.matrixV()(3, 3);

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        return true;
    }
    return false;
}

}

#endif // ALGORITHM_H