#include "edge_alignment_factor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "utility.h"
#include "pose_local_parameterization.h"
#include <vector>
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace cv;
using namespace ceres;
using namespace Eigen;

int main(int argc, char **argv)
{
    YAML::Node conf = YAML::LoadFile("../config/param.yaml");
    int feature_num = stoi(conf["feature_num"].as<string>());
    string image_path = conf["image_path"].as<string>();
    double fx = stod(conf["fx"].as<string>());
    double fy = stod(conf["fy"].as<string>());
    double cx = stod(conf["cx"].as<string>());
    double cy = stod(conf["cy"].as<string>());
    vector<string> extrinsic_string = conf["extrinsic"].as<vector<string>>();
    vector<double> extrinsic_double;
    for (int i = 0; i < extrinsic_string.size(); ++i)
    {
        extrinsic_double.push_back(stod(extrinsic_string[i]));
    }
    // intrinsic parameters
    Eigen::Matrix3d K;
    K << fx, 0., cx, 0., fy, cy, 0., 0., 1.;
    cout << "K\n"
         << K << "\n";
    // extrinsic parameters
    Eigen::Matrix4d extrinsic_T(extrinsic_double.data());
    Eigen::Matrix4d extrinsic = extrinsic_T.transpose();
    Eigen::Quaterniond q(extrinsic.topLeftCorner<3, 3>());
    cout << "extrinsic\n"
         << extrinsic << "\n";
    // initial parameters
    double pose[9] = {fx, fy, extrinsic(0, 3), extrinsic(1, 3), extrinsic(2, 3),
                      q.x(), q.y(), q.z(), q.w()};
    // 3d
    pcl::PointCloud<pcl::PointXYZI> feature_pcd[feature_num];
    vector<Eigen::MatrixXd> feature_3d_points;

    int backslashIndex = image_path.find_last_of('/');
    string path = image_path.substr(0, backslashIndex);

    for (int i = 0; i < feature_num; ++i)
    {
        string feature_3d_path = path + "/3d_feature/feature" + to_string(i + 1) + ".pcd";
        pcl::io::loadPCDFile(feature_3d_path, feature_pcd[i]);
        Eigen::MatrixXd feature_3d(4, feature_pcd[i].points.size());
        feature_3d_points.push_back(feature_3d);
        for (int j = 0; j < feature_pcd[i].points.size(); j++)
        {
            feature_3d_points[i].col(j) << feature_pcd[i].points[j].x, feature_pcd[i].points[j].y, feature_pcd[i].points[j].z, 1.0;
        }
    }

    // 2d
    Eigen::MatrixXf feature_distmap[feature_num];
    Eigen::MatrixXf feature_distgradx[feature_num];
    Eigen::MatrixXf feature_distgrady[feature_num];
    cv::Mat ori_img = cv::imread(image_path);

    for (int i = 0; i < feature_num; ++i)
    {
        string feature_2d_path = path + "/feature_bit/feature" + to_string(i + 1) + ".jpg";
        cv::Mat img = cv::imread(feature_2d_path);
        get_distance_transform(img, feature_distmap[i], feature_distgradx[i], feature_distgrady[i]);
    }

    // initial reproject
    Eigen::MatrixXd feature_init_uv[feature_num];
    cv::Mat init_image_repro = ori_img.clone();
    for (int i = 0; i < feature_num; ++i)
    {
        reproject(feature_3d_points[i], extrinsic, K, feature_init_uv[i]);
        s_overlay(init_image_repro, feature_init_uv[i]);
    }
    cv::namedWindow("initial", 0);
    cv::resizeWindow("initial", 960, 540);
    cv::imshow("initial", init_image_repro);

    //optimization
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(5.0);
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(pose, 9, local_parameterization);

    int width = ori_img.rows;
    int height = ori_img.cols;
    cout << width << " " << height << "\n";
    for (int i = 0; i < feature_num; i++)
    {
        for (int j = 0; j < feature_3d_points[i].cols(); j += 10)
        {
            EdgeAlignmentFactor *f = new EdgeAlignmentFactor(feature_3d_points[i](0, j),
                                                             feature_3d_points[i](1, j),
                                                             feature_3d_points[i](2, j),
                                                             feature_distmap[i],
                                                             feature_distgradx[i],
                                                             feature_distgrady[i],
                                                             width, height);

            problem.AddResidualBlock(f, NULL, pose);
        }
    }

    // setting the solver
    ceres::Solver::Options options;
    options.max_num_iterations = 3000;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.num_threads = 1;
    options.minimizer_type = ceres::TRUST_REGION;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.min_line_search_step_size = 1e-5;

    // print convergence process
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // print intrinsic and extrinsic results
    Quaterniond q_final = Quaterniond(pose[8], pose[5], pose[6], pose[7]);
    Eigen::Matrix4d dstT;
    dstT = Matrix4d::Zero();
    dstT.topLeftCorner<3, 3>() = q_final.toRotationMatrix();

    dstT(0, 3) = pose[2];
    dstT(1, 3) = pose[3];
    dstT(2, 3) = pose[4];
    dstT(3, 3) = 1.0;
    cout << dstT << "\n";

    K << pose[0], 0., cx, 0., pose[1], cy, 0., 0., 1.;
    cout << "final_K: " << K << "\n";

    // final reproject
    Eigen::MatrixXd feature_final_uv[feature_num];
    cv::Mat final_image_repro = ori_img.clone();
    for (int i = 0; i < feature_num; ++i)
    {
        reproject(feature_3d_points[i], dstT, K, feature_final_uv[i]);
        s_overlay(final_image_repro, feature_final_uv[i]);
    }
    cv::namedWindow("final", 0);
    cv::resizeWindow("final", 960, 540);
    cv::imshow("final", final_image_repro);
    cv::waitKey(0);
}