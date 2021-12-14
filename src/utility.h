#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;

float getInterpolatedElementEigen(const Eigen::MatrixXf &mat, const float x, const float y)
{
    int ix = (int)x;
    int iy = (int)y;
    float dx = x - ix;
    float dy = y - iy;
    float dxdy = dx * dy;
    float res = dxdy * mat(iy + 1, ix + 1) + (dy - dxdy) * mat(iy + 1, ix) + (dx - dxdy) * mat(iy, ix + 1) + (1 - dx - dy + dxdy) * mat(iy, ix);

    return res;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

void get_distance_transform(const cv::Mat &input, Eigen::MatrixXf &out_distance_transform_map, Eigen::MatrixXf &out_distance_transform_gradx, Eigen::MatrixXf &out_distance_transform_grady)
{
    cout << input.rows << " " << input.cols << endl;
    cv::Mat channels[3];
    cv::split(input, channels);

    // Distance Transform
    cv::Mat kernX = (cv::Mat_<float>(3, 3) << 0, 0, 0,
                     -1.5, 0.0, 1.5,
                     0, 0, 0);
    cv::Mat kernY = (cv::Mat_<float>(3, 3) << 0, -1.5, 0,
                     0, 0.0, 0,
                     0, 1.5, 0);
    cv::Mat distmap, distgradx, distgrady;
    cv::distanceTransform(channels[0], distmap, CV_DIST_L2, 0);
    // cv::normalize(distmap, distmap, 0, 1., cv::NORM_MINMAX);
    // distmap = distmap * 255;
    cv::cv2eigen(distmap, out_distance_transform_map);
    //cv::filter2D(distmap, distgradx, CV_32F, kernX);
    cv::Sobel(distmap, distgradx, CV_32F, 1, 0, 1);
    cv::cv2eigen(distgradx, out_distance_transform_gradx);
    //cv::filter2D(distmap, distgrady, CV_32F, kernY);
    cv::Sobel(distmap, distgrady, CV_32F, 0, 1, 1);
    cv::cv2eigen(distgrady, out_distance_transform_grady);
}

void reproject(const Eigen::MatrixXd &a_X, const Eigen::Matrix4d &b_T_a, const Eigen::Matrix3d &K, Eigen::MatrixXd &b_u)
{
    Eigen::MatrixXd b_X = b_T_a * a_X;

    Eigen::MatrixXd unvn = Eigen::MatrixXd(3, b_X.cols());
    for (int i = 0; i < b_X.cols(); i++)
    {
        unvn(0, i) = b_X(0, i) / b_X(2, i);
        unvn(1, i) = b_X(1, i) / b_X(2, i);
        unvn(2, i) = 1.0;
    }

    b_u = K * unvn;
}

void s_overlay(cv::Mat &im, const Eigen::MatrixXd &uv, int width, int height)
{
    assert(uv.rows() == 3);
    assert(im.rows > 0 && im.cols > 0 && im.channels() == 3);

    for (int i = 0; i < uv.cols(); i++)
    {
        cv::Vec3b color = cv::Vec3d(0, 0, 255);
        if (uv(1, i) > 1 && uv(1, i) < height && uv(0, i) > 1 && uv(0, i) < width)
            im.at<cv::Vec3b>(uv(1, i), uv(0, i)) = color;
    }
}
