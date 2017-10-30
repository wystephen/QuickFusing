/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 17-10-29.
//

#include "CSVReader.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "EKF.hpp"
#include "MYEKF.h"
#include "Zero_Detecter.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

// GTSAM related includes.
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/ManifoldPreintegration.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/slam/RangeFactor.h>
//#indluce <gtsam


#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>


#include <thread>
#include <OwnFactor/MagConstraintFactor.h>
#include <ImuKeyPointInfo.h>

using namespace gtsam;
using namespace std;


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::M; // Bias of Magconstraint
using symbol_shorthand::C; // Central point of each moment

PreintegratedImuMeasurements *imu_preintegrated_l_;
PreintegratedImuMeasurements *imu_preintegrated_r_;


namespace plt = matplotlibcpp;


Eigen::Isometry3d tq2Transform(Eigen::Vector3d offset,
                               Eigen::Quaterniond q) {
    Eigen::Isometry3d T;
    T.setIdentity();
    T.rotate(q.toRotationMatrix());
    T(0, 3) = offset(0);
    T(1, 3) = offset(1);
    T(2, 3) = offset(2);
    return T;
}


int main(int argc, char *argv[]) {
    std::string dir_name = "/home/steve/Data/II/20/";


//    CppExtent::CSVReader imu_data_reader(dir_name + "ImuData.csv");
    CppExtent::CSVReader imu_data_reader_r(dir_name + "imu2.txt");
    Eigen::MatrixXd imudata_r;
    imudata_r.resize(imu_data_reader_r.GetMatrix().GetRows(),
                     imu_data_reader_r.GetMatrix().GetCols());
    imudata_r.setZero();
    auto imu_data_tmp_matrix = imu_data_reader_r.GetMatrix();

    Eigen::Vector3d central_r(-25, -128, 80);
    Eigen::Vector3d scale_axis_r(238, 263, 271);
    for (int i(0); i < imudata_r.rows(); ++i) {
        for (int j(0); j < imudata_r.cols(); ++j) {
            imudata_r(i, j) = *(imu_data_tmp_matrix(i, j));
            if (0 < j && j < 4) {
                imudata_r(i, j) *= 9.81;
            } else if (4 <= j && j < 7) {
                imudata_r(i, j) *= (M_PI / 180.0f);
            } else if (7 <= j && j < 10) {
                imudata_r(i, j) = (imudata_r(i, j) - central_r(j - 7));///scale_axis(j-7);
            }

        }
    }
    CppExtent::CSVReader imu_data_reader_l(dir_name + "imu.txt");
    Eigen::MatrixXd imudata_l;
    imudata_l.resize(imu_data_reader_l.GetMatrix().GetRows(),
                     imu_data_reader_l.GetMatrix().GetCols());
    imudata_l.setZero();
    auto imu_data_tmp_matrix_l = imu_data_reader_l.GetMatrix();

    Eigen::Vector3d central_l(-25, -128, 80);
    Eigen::Vector3d scale_axis_l(238, 263, 271);
    for (int i(0); i < imudata_l.rows(); ++i) {
        for (int j(0); j < imudata_l.cols(); ++j) {
            imudata_l(i, j) = *(imu_data_tmp_matrix_l(i, j));
            if (0 < j && j < 4) {
                imudata_l(i, j) *= 9.81;
            } else if (4 <= j && j < 7) {
                imudata_l(i, j) *= (M_PI / 180.0f);
            } else if (7 <= j && j < 10) {
                imudata_l(i, j) = (imudata_l(i, j) - central_l(j - 7));///scale_axis(j-7);
            }

        }
    }


    /**
     * Initial ZUPT para
     */

    double sa(0.01), sg(0.02 / 180.0 * M_PI), sv(0.000001);
    double gravity(9.6), smag_attitude(0.1), sgravity_attitude(-1.7);
    double initial_heading = 180.0 / 180.0 * M_PI;
    if (argc >= 4) {
        sv = std::stod(argv[1]);
        sa = std::stod(argv[2]);
        sg = std::stod(argv[3]) / 180.0 * M_PI;
    }

    if (argc >= 7) {
        gravity = std::stod(argv[4]);
        smag_attitude = std::stod(argv[5]);
        sgravity_attitude = std::stod(argv[6]);

    }

    if (argc >= 8) {
        initial_heading = std::stod(argv[7]) / 180.0 * M_PI;
    }

    SettingPara initial_para(true);
    initial_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    initial_para.init_heading1_ = initial_heading;// imudata.block(0, 8, 20, 1).mean() * M_PI;
    initial_para.Ts_ = 1.0f / 200.0f;


    initial_para.sigma_vel_ = Eigen::Vector3d(sv, sv, sv);
    initial_para.sigma_acc_ = Eigen::Vector3d(sa, sa, sa);
    initial_para.sigma_gyro_ = Eigen::Vector3d(sg, sg, sg);

//    initial_para.sigma_a_ = 1.1;//zupt detector parameter
//    initial_para.sigma_g_ = 2.0 / 180.0 * M_PI;
    initial_para.sigma_a_ = 1.1;//zupt detector parameter
    initial_para.sigma_g_ = 2.0 / 180.0 * M_PI;

    std::vector<double> ax, ay, az, zupt_v;

    initial_para.ZeroDetectorWindowSize_ = 5;

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata_l.block(0, 1, 20, 6));

    /**
     * Initial graph para
     */
    int left_offset = 1000000;
    int right_offset = 2000000;


    Rot3 prior_rotation = Rot3(myekf.getTransformation().matrix().block(0, 0, 3, 3));
    Point3 prior_point(0, 0, 0);
    Pose3 prior_pose(prior_rotation, prior_point);


    Vector3 prior_velocity(Vector3(0, 0, 0));
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    Values initial_values;
    int correction_count = 0;
    initial_values.insert(X(correction_count + left_offset), prior_pose);
    initial_values.insert(V(correction_count + left_offset), prior_velocity);
    initial_values.insert(B(correction_count + left_offset), prior_imu_bias);

    initial_values.insert(X(correction_count + right_offset), prior_pose);
    initial_values.insert(V(correction_count + right_offset), prior_velocity);
    initial_values.insert(B(correction_count + right_offset), prior_imu_bias);

    // Assemble prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 11100000.1, 11100010.1, 11100010.1, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    graph->add(PriorFactor<Pose3>(X(correction_count + left_offset), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count + left_offset), prior_velocity, velocity_noise_model));
    graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count + left_offset), prior_imu_bias, bias_noise_model));

    graph->add(PriorFactor<Pose3>(X(correction_count + right_offset), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count + right_offset), prior_velocity, velocity_noise_model));
    graph->add(
            PriorFactor<imuBias::ConstantBias>(B(correction_count + right_offset), prior_imu_bias, bias_noise_model));


    double accel_noise_sigma = initial_para.sigma_acc_(0);// 0.0003924;
    double gyro_noise_sigma = initial_para.sigma_gyro_(0);//0.000205689024915;
    double accel_bias_rw_sigma = 0.004905;
    double gyro_bias_rw_sigma = 0.000001454441043;
    Matrix33 measured_acc_cov = Matrix33::Identity(3, 3) * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov =
            Matrix33::Identity(3, 3) * 1e-8; // error committed in integrating position from velocities
    Matrix33 bias_acc_cov = Matrix33::Identity(3, 3) * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = Matrix33::Identity(3, 3) * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_int = Matrix::Identity(6, 6) * 1e-5; // error in the bias used for preintegration


    //error gravity...!!!
    boost::shared_ptr<PreintegratedImuMeasurements::Params> p =
            PreintegratedImuMeasurements::Params::MakeSharedU(gravity);
    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous


    NavState prev_state(prior_pose, prior_velocity);
//    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    imu_preintegrated_l_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
    imu_preintegrated_r_ = new PreintegratedImuMeasurements(p, prior_imu_bias);

    int trace_id_l(left_offset);
    int trace_id_r(right_offset);


    std::vector<ImuKeyPointInfo> left_info_vec, right_info_vec;

    int accumulate_preintegra_num = 0;


    /**
     * Start add left imu data
     */

    for (int index(0); index < imudata_l.rows(); ++index) {
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {
            if (GLRT_Detector(
                    imudata_l.block(index - initial_para.ZeroDetectorWindowSize_, 1,
                                    initial_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                    initial_para)) {
                zupt_flag = 1.0;
            }
        }

        /// Integrated part
        accumulate_preintegra_num++;
        if (accumulate_preintegra_num > 15) {
            accumulate_preintegra_num = 0;
            trace_id_l++;

            PreintegratedImuMeasurements *preint_imu =
                    dynamic_cast<PreintegratedImuMeasurements *> (imu_preintegrated_l_);
            try {

                graph->add(ImuFactor(X(trace_id_l - 1), V(trace_id_l - 1),
                                     X(trace_id_l), V(trace_id_l),
                                     B(trace_id_l), *preint_imu));

                preint_imu->resetIntegration();

                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0),
                                                Vector3(0, 0, 0));

                graph->add(BetweenFactor<imuBias::ConstantBias>(
                        B(trace_id_l - 1),
                        B(trace_id_l),
                        zero_bias, bias_noise_model
                ));


                if (zupt_flag > 0.5) {
                    noiseModel::Diagonal::shared_ptr velocity_noise =
                            noiseModel::Isotropic::Sigma(3, sv);


                    PriorFactor<Vector3> zero_velocity(V(trace_id_l),
                                                       Vector3(0.0, 0.0, 0.0),
                                                       velocity_noise);

                    graph->add(zero_velocity);


                    left_info_vec.push_back(ImuKeyPointInfo(
                            trace_id_l,
                            imudata_l.block(index, 0, 1, 10).transpose()
                    ));
                }


            } catch (...) {
                assert(true);
            }
            try {
                Pose3 pp;
//                    p.matrix() = myekf.getTransformation().matrix();
                initial_values.insert(X(trace_id_l), pp);
                initial_values.insert(V(trace_id_l), Vector3(0, 0, 0));
                initial_values.insert(B(trace_id_l), prev_bias);


            } catch (const std::exception &e) {
                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : " << e.what() << std::endl;
                std::cout << initial_values.at<Pose3>(X(trace_id_l)).matrix() << std::endl;
            } catch (...) {
                std::cout << "unexpected error " << std::endl;
            }


        }


        imu_preintegrated_l_->integrateMeasurement(
                imudata_l.block(index, 1, 1, 3).transpose(),
                imudata_l.block(index, 4, 1, 3).transpose(),
                initial_para.Ts_);


    }

    /**
     * start add right imu
     */

    accumulate_preintegra_num = 0;
    for (int index(0); index < imudata_r.rows(); ++index) {
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {
            if (GLRT_Detector(
                    imudata_r.block(index - initial_para.ZeroDetectorWindowSize_, 1,
                                    initial_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                    initial_para)) {
                zupt_flag = 1.0;
            }
        }

        /// Integrated part
        accumulate_preintegra_num++;
        if (accumulate_preintegra_num > 15) {
            accumulate_preintegra_num = 0;
            trace_id_l++;

            PreintegratedImuMeasurements *preint_imu =
                    dynamic_cast<PreintegratedImuMeasurements *> (imu_preintegrated_r_);
            try {

                graph->add(ImuFactor(X(trace_id_r - 1), V(trace_id_r - 1),
                                     X(trace_id_r), V(trace_id_r),
                                     B(trace_id_r), *preint_imu));

                preint_imu->resetIntegration();

                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0),
                                                Vector3(0, 0, 0));

                graph->add(BetweenFactor<imuBias::ConstantBias>(
                        B(trace_id_r - 1),
                        B(trace_id_r),
                        zero_bias, bias_noise_model
                ));

                if (zupt_flag > 0.5) {
                    noiseModel::Diagonal::shared_ptr velocity_noise =
                            noiseModel::Isotropic::Sigma(3, sv);


                    PriorFactor<Vector3> zero_velocity(V(trace_id_r),
                                                       Vector3(0.0, 0.0, 0.0),
                                                       velocity_noise);

                    graph->add(zero_velocity);


                    right_info_vec.push_back(ImuKeyPointInfo(
                            trace_id_r,
                            imudata_r.block(index, 0, 1, 10).transpose()
                    ));
                }


            } catch (...) {
                assert(true);
            }
            try {
                Pose3 pp;
//                    p.matrix() = myekf.getTransformation().matrix();
                initial_values.insert(X(trace_id_r), pp);
                initial_values.insert(V(trace_id_r), Vector3(0, 0, 0));
                initial_values.insert(B(trace_id_r), prev_bias);


            } catch (const std::exception &e) {
                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : " << e.what() << std::endl;
                std::cout << initial_values.at<Pose3>(X(trace_id_r)).matrix() << std::endl;
            } catch (...) {
                std::cout << "unexpected error " << std::endl;
            }

        }
        imu_preintegrated_r_->integrateMeasurement(
                imudata_r.block(index, 1, 1, 3).transpose(),
                imudata_r.block(index, 4, 1, 3).transpose(),
                initial_para.Ts_);


    }


    /**
     * Optimzation
     */

    std::cout << "begin optimizer" << std::endl;
//    graph.print("before optimize");
//    GaussNewtonOptimizer optimizer(*graph, initial_values);
    LevenbergMarquardtParams lm_para;
    lm_para.setMaxIterations(1000);
    LevenbergMarquardtOptimizer optimizer(*graph, initial_values, lm_para);


    /// Show itereation times ~
    std::thread thread1([&] {
        int last_index = 0;
        int counter = 0;
        while (true) {
            sleep(1);


            if (last_index >= optimizer.iterations()) {
                counter += 1;
            } else {
                std::cout << "i :" << optimizer.iterations() << std::endl;
                counter = 0;
            }

            if (counter > 10) {
                break;
            }
            last_index = int(optimizer.iterations());
        }
    });
    thread1.detach();

    auto result = initial_values;

    result = optimizer.optimize();

    /**
     * Output Result
     */
    std::vector<double> gx, gy;
    try {

        ofstream test_out_put("./ResultData/test.txt");

        for (int k(left_offset); k < trace_id_l; ++k) {
            double t_data[10] = {0};

            try {
                auto pose_result = result.at<Pose3>(X(k));
                t_data[0] = pose_result.matrix()(0, 3);
                t_data[1] = pose_result.matrix()(1, 3);
                t_data[2] = pose_result.matrix()(2, 3);

                gx.push_back(t_data[0]);
                gy.push_back(t_data[1]);

                test_out_put << t_data[0] << ","
                             << t_data[1] << ","
                             << t_data[2] << std::endl;

//                auto velocity_result = result.at<Vector3>(V(k));
//                std::cout << velocity_result.transpose() << std::endl;
            } catch (std::exception &e) {
                std::cout << "error when get value :" << e.what() << std::endl;
            }

        }


    } catch (std::exception &e) {
        std::cout << e.what() << " :" << __FILE__ << ":" << __LINE__ << std::endl;

    }

    /**
     * Plot Trace
     */
    plt::plot(gx, gy, "r-+");
    plt::title("img-sv:" + std::to_string(sv) + "sa:" + std::to_string(sa) + "-sg:" +
               std::to_string(sg)
               + "g:" + std::to_string(gravity) + "s_mag_att:" + std::to_string(smag_attitude) +
               "s_g_att:" + std::to_string(sgravity_attitude) + "initial_heading:" + std::to_string(initial_heading));
    plt::show();


}