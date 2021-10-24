#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "utility.h"

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        // std::cout << "plus!" << "\n";
        Eigen::Map<const Eigen::Matrix<double, 5, 1>> _p(x);
        Eigen::Map<const Eigen::Quaterniond> _q(x + 5);

        Eigen::Map<const Eigen::Matrix<double, 5, 1>> dp(delta);

        Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 5));

        Eigen::Map<Eigen::Matrix<double, 5, 1>> p(x_plus_delta);
        Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 5);

        p = _p + dp;
        q = (_q * dq).normalized();

        return true;
    }
    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        // std::cout << "jacobian!" << "\n";
        Eigen::Map<Eigen::Matrix<double, 9, 8, Eigen::RowMajor>> j(jacobian);
        j.topRows<9>().setIdentity();
        j.bottomRows<1>().setZero();

        return true;
    }
    virtual int GlobalSize() const { return 9; };
    virtual int LocalSize() const { return 8; };
};
