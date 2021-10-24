#pragma once
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#include "utility.h"
#include <ceres/ceres.h>

using namespace std;

class EdgeAlignmentFactor : public ceres::SizedCostFunction<1, 9>
{
public:
    EdgeAlignmentFactor() = delete ;
    EdgeAlignmentFactor( double x, double y, double z,
                         Eigen::MatrixXf& map, Eigen::MatrixXf& dx, Eigen::MatrixXf&dy,
                         int w, int h)
        :dGx(dx), dGy(dy), cost(map)
    {
        width = w - 3 ;
        height = h - 3 ;
        p << x, y, z ;
    }

    bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // cout << "parameter_block_sizes_ is " << parameter_block_sizes().size() << "\n";
        // cout << "num_residuals_ is " << num_residuals() << "\n";
        double fx = parameters[0][0];
        double fy = parameters[0][1];
        double cx = 960.0;
        double cy = 540.0;
        Eigen::Vector3d t(parameters[0][2], parameters[0][3], parameters[0][4]);
        Eigen::Quaterniond q(parameters[0][8], parameters[0][5], parameters[0][6], parameters[0][7]);
        Eigen::Matrix3d R = q.toRotationMatrix();
        //Eigen::Matrix3d Ri = Qi.toRotationMatrix() ;

        // cout << "t = " << t.transpose() << "\n" ;
        // cout << "R = \n" << R << "\n" ;
        // cout << "fx: " << fx << " fy: " << fy << " cx: " << cx << " cy: " << cy << "\n";


        ofstream outfile;
        outfile.open("result.txt", ios::app);
        outfile << fx << " " << fy << " " << cx << " " << t(0) << " " << t(1) << " " << t(2) << " "
                << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << R(1, 0) << " " << R(1, 1) << " "
                << R(1, 2) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n";
        // cout << "write down" << endl;
        sleep(0.5);
        

        Eigen::Vector3d p2 = R*p + t ;
        float X = (float)p2(0);
        float Y = (float)p2(1);
        float Z = (float)p2(2);
        bool out = false ;

        float u_ = fx * X / Z + cx ;
        float v_ = fy * Y / Z + cy ;
        // printf("u_%f v_%f\n", u_, v_) ;
        if (u_ < 2 || u_ > width || v_ < 2 || v_ > height || Z < 0.0001 ){
            residuals[0] = 0 ;
            out = true;
            // cout << "********************out!******************" << "\n";
            // cout << "u: " << u_ << "v: " << v_ << "\n";
        }
        else {
            // cout << "********************in!******************" << "\n";
            // cout << "u: " << u_ << "v: " << v_ << "\n";
            residuals[0] = (double)getInterpolatedElementEigen(cost, u_, v_) ;
        }
        //cout << "residuals[0] cost = " << residuals[0] << endl;
        //ROS_WARN("%lf\n", residuals[0] ) ;
        if ( jacobians )
        {
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 9, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();
                if ( out == false )
                {
                    float gx = getInterpolatedElementEigen(dGx, u_, v_) ;
                    float gy = getInterpolatedElementEigen(dGy, u_, v_) ;
                    // cout << "gx :" << gx << "gy: " << gy << "\n";

                    jacobian_pose_i(0, 0) = gx*X/Z;
                    jacobian_pose_i(0, 1) = gy*Y/Z;
                    // jacobian_pose_i(0, 2) = gx;
                    // jacobian_pose_i(0, 3) = gy;

                    Eigen::Matrix<double,1,3> oneByThree;
                    oneByThree(0, 0) = gx*fx/Z;
                    oneByThree(0, 1) = gy*fy/Z;
                    oneByThree(0, 2) = -gx*fx*X/(Z*Z) - gy*fy*Y/(Z*Z);

                    Eigen::Matrix3d threeByThree = -R * skewSymmetric(p) ;

                    jacobian_pose_i(0, 2) = oneByThree(0, 0) ;
                    jacobian_pose_i(0, 3) = oneByThree(0, 1) ;
                    jacobian_pose_i(0, 4) = oneByThree(0, 2) ;
                    jacobian_pose_i.block<1, 3>(0, 5) = oneByThree*threeByThree ;

                    // cout << "oneByThree :" << oneByThree << "\n" ;
                    // cout << "threeByThree :" << threeByThree << "\n" ;
                    // cout << "jacobian_pose_i " << jacobian_pose_i << "\n";

                    if (fabs(jacobian_pose_i.maxCoeff()) > 1e8 ||
                            fabs(jacobian_pose_i.minCoeff()) < -1e8)
                    {
                        std::cout << "oneByThree :" << oneByThree << "\n" ;
                        std::cout << "threeByThree :" << threeByThree << "\n" ;
                        std::cout << "jacobian_pose_i " << jacobian_pose_i << "\n";
                        //ROS_BREAK();
                        return true;
                    }
                }
            }
        }

        return true;
    }

    Eigen::Vector3d p ;
    double cx, cy;
    int width;
    int height;
    Eigen::MatrixXf& cost;
    Eigen::MatrixXf& dGx;
    Eigen::MatrixXf& dGy;
};
