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
// Created by steve on 17-10-6.
//

//#define _GLIBCXX_DEBUG

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

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearISAM.h>


#include <thread>

using namespace gtsam;
using namespace std;


using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

PreintegratedImuMeasurements *imu_preintegrated_;

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
    /**
     * Load Data
     */
    std::string dir_name = "/home/steve/Data/AttitudeIMU/";
//    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/91/";

    CppExtent::CSVReader imu_data_reader(dir_name + "ImuData.csv");
//    CppExtent::CSVReader imu_data_reader(dir_name + "imu.txt");
    Eigen::MatrixXd imudata;
    imudata.resize(imu_data_reader.GetMatrix().GetRows(),
                   imu_data_reader.GetMatrix().GetCols());
    imudata.setZero();
    auto imu_data_tmp_matrix = imu_data_reader.GetMatrix();

    for (int i(0); i < imudata.rows(); ++i) {
        for (int j(0); j < imudata.cols(); ++j) {
            imudata(i, j) = *(imu_data_tmp_matrix(i, j));
//            if (0 < j&& j < 4) {
//                imudata(i, j) *= 9.81;
//            } else if (4 <= j && j< 7) {
//                imudata(i,j) *= (M_PI/180.0f);
//            }

        }
    }



    /**
     * Initial ZUPT
     */
    std::cout << "start initial ZUPT" << std::endl;
    SettingPara initial_para(true);
    initial_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    initial_para.init_heading1_ = imudata.block(0, 8, 20, 1).mean() * M_PI;
    initial_para.Ts_ = 1.0f / 100.0f;


    std::cout << 'argc:' << argc << std::endl;
    double sv, sa, sg;
    if (argc == 4) {
        sv = std::stod(argv[1]);
        sa = std::stod(argv[2]);
        sg = std::stod(argv[3]);
    initial_para.sigma_vel_ = Eigen::Vector3d(sv,sv,sv);
    initial_para.sigma_acc_ = Eigen::Vector3d(sa,sa,sa);
    initial_para.sigma_gyro_ = Eigen::Vector3d(sg,sg,sg) / 180.0 * M_PI;

    }

//    initial_para.sigma_a_ = 1.1;//zupt detector parameter
//    initial_para.sigma_g_ = 2.0 / 180.0 * M_PI;
    initial_para.sigma_a_ = 1.1;//zupt detector parameter
    initial_para.sigma_g_ = 2.0 / 180.0 * M_PI;

    std::vector<double> ax, ay, az, zupt_v;

    initial_para.ZeroDetectorWindowSize_ = 5;

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata.block(0, 0, 20, 6));

    std::vector<double> ekfx, ekfy;

    /**
     * Initial Graph parameters.
     */

    std::cout << "start initial GRAPH" << std::endl;
    ISAM2GaussNewtonParams isam2GaussNewtonParams(0.001);
    ISAM2Params isam2Params(isam2GaussNewtonParams);
    isam2Params.relinearizeThreshold = 0.0001;
//    isam2Params.relinearizeSkip = 1;


    ISAM2 isam2(isam2Params);
//    NonlinearISAM isam2(3);

    Eigen::Matrix<double, 10, 1> initial_state = Eigen::Matrix<double, 10, 1>::Zero();
    initial_state(6) = 1.0;

    Rot3 prior_rotation = Rot3(myekf.getTransformation().matrix().block(0, 0, 3, 3));
    Point3 prior_point(initial_state.head<3>());
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(initial_state.tail<3>());
    imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

    Values initial_values;
    int correction_count = 0;
    initial_values.insert(X(correction_count), prior_pose);
    initial_values.insert(V(correction_count), prior_velocity);
    initial_values.insert(B(correction_count), prior_imu_bias);

    // Assemble prior noise model and add it the graph.
    noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 1110.1, 11110.1, 11110.1, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.01); // m/s
    noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    NonlinearFactorGraph *graph = new NonlinearFactorGraph();
    graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
    graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity, velocity_noise_model));
    graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias, bias_noise_model));

    // We use the sensor specs to build the noise model for the IMU factor.
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
            PreintegratedImuMeasurements::Params::MakeSharedD(9.6);

    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous



    NavState prev_state(prior_pose, prior_velocity);
//    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    ////Define the imu preintegration
    imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);





    /**
     * Start Location
     */

    std::vector<double> time_of_each_iteration; // optimization time of ISAM2

    int trace_id(0);
    double last_zupt_flag = 0.0;


    int accumulate_preintegra_num = 0;


    /*
     * Main loop for positioning.
     */
    std::cout << "start Main Loop" << std::endl;
    for (int index(0); index < imudata.rows(); ++index) {

        /// ZUPT DETECTOR
        double zupt_flag = 0.0;// probability of be zero-velocity.

        if (index <= initial_para.ZeroDetectorWindowSize_) {
            // first several data set as zero-velocity state.
            zupt_flag = 1.0;
        } else {
            if (GLRT_Detector(imudata.block(index - initial_para.ZeroDetectorWindowSize_, 1,
                                            initial_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                              initial_para)) {
                zupt_flag = 1.0;
            }
        }
//        std::cout << "zupt flag :" << zupt_flag << std::endl;
        ax.push_back(imudata(index, 1));
        ay.push_back(imudata(index, 2));
        az.push_back(imudata(index, 3));
        zupt_v.push_back(zupt_flag);


        /// ekf test
        auto result_x = myekf.GetPosition(imudata.block(index, 1, 1, 6).transpose(), zupt_flag);
        ekfx.push_back(result_x(0));
        ekfy.push_back(result_x(1));


        /// IntegratedImu
        accumulate_preintegra_num++;
        if (accumulate_preintegra_num > 15) {
            accumulate_preintegra_num = 0;
            trace_id++;

            PreintegratedImuMeasurements *preint_imu =
                    dynamic_cast<PreintegratedImuMeasurements *>(imu_preintegrated_);

            try {

                ///IMU preintegrate
                graph->add(ImuFactor(X(trace_id - 1), V(trace_id - 1),
                                     X(trace_id), V(trace_id),
                                     B(trace_id), *preint_imu));


                preint_imu->resetIntegration();




                ///Imu Bias
                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
//
                graph->add(BetweenFactor<imuBias::ConstantBias>(
                        B(trace_id - 1),
                        B(trace_id),
                        zero_bias, bias_noise_model
                ));

                // considering gravity constraint...



                ///Zero-velocity constraint
                if (zupt_flag > 0.5) {
                    noiseModel::Diagonal::shared_ptr velocity_noise =
                            noiseModel::Isotropic::Sigma(3, sv);


                    PriorFactor<Vector3> zero_velocity(V(trace_id),
                                                       Vector3(0.0, 0.0, 0.0),
                                                       velocity_noise);

                    graph->add(zero_velocity);

                    ///Mag constraint
//                    noiseModel::Diagonal::shared_ptr mag_noise =
//                            noiseModel::Diagonal::Sigmas(Vector3(M_PI * 1000000.0, M_PI * 1000000.0, M_PI * 1.5));

//                    graph->add(PoseRotationPrior<Pose3>(
//                            X(trace_id),
//                            Rot3::Yaw(( imudata(index,7)/180.0 * M_PI)),
//                            mag_noise
//
//                    ));


                    noiseModel::Diagonal::shared_ptr mag_all_noise =
                            noiseModel::Diagonal::Sigmas(Vector3(M_PI, M_PI, M_PI));
//                    graph->add(PoseRotationPrior<Pose3>(
//                            X(trace_id),
//                            Rot3::RzRyRx(Vector3(imudata(index, 9) / 180.0 * M_PI,
//                                                 imudata(index, 8) / 180.0 * M_PI,
//                                                 imudata(index, 7) / 180.0 * M_PI)),
//                            mag_all_noise
//                    ));
                    std::cout << imudata(index, 7) << std::endl;
                }


                if (trace_id == 1) {

                    noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 0.1);
                    graph->add(GPSFactor(X(trace_id),
                                         prior_pose.matrix().block(0, 3, 3, 1),
                                         correction_noise));

                }
                ///Set intial values
                try {
                    Pose3 p;
//                    p.matrix() = myekf.getTransformation().matrix();
                    initial_values.insert(X(trace_id), p);
                    initial_values.insert(V(trace_id), Vector3(0, 0, 0));
                    initial_values.insert(B(trace_id), prev_bias);


                } catch (const std::exception &e) {
                    std::cout << "error at :" << __FILE__
                              << " " << __LINE__ << " : " << e.what() << std::endl;
                    std::cout << initial_values.at<Pose3>(X(trace_id)).matrix() << std::endl;
                } catch (...) {
                    std::cout << "unexpected error " << std::endl;
                }


            } catch (const std::exception &e) {

                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : " << e.what() << std::endl;
                return 0;
            } catch (...) {
                std::cout << "unexpected error " << std::endl;
                return 0;
            }


            // only reset after optimization.


        }

        imu_preintegrated_->integrateMeasurement(imudata.block(index, 1, 1, 3).transpose(),
                                                 imudata.block(index, 4, 1, 3).transpose(),
                                                 initial_para.Ts_);

        last_zupt_flag = zupt_flag;


    };



    /**
     * Optimzation
     */


    std::cout << "begin optimizer" << std::endl;
//    graph.print("before optimize");
//    GaussNewtonOptimizer optimizer(*graph, initial_values);
    LevenbergMarquardtParams lm_para;
    lm_para.setMaxIterations(200);
    LevenbergMarquardtOptimizer optimizer(*graph, initial_values, lm_para);


    /// Show itereation times ~
    std::thread thread1([&] {
        int last_index = 0;
        int counter = 0;
        while (1) {
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

        for (int k(0); k < trace_id; ++k) {
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
    plt::plot(ekfx, ekfy, "b-");
    plt::title("img-sv:"+std::to_string(sv)+"sa:"+std::to_string(sa)+"-sg:"+
    std::to_string(sg));



//    plt::plot(ax);
//    plt::plot(ay);
//    plt::plot(az);
//    plt::plot(zupt_v);
    plt::show();
//    plt::save("img-sa:"+std::to_string(sa)+"-sg:"+
//    std::to_string(sg)+
//    ".png");


    return 0;
}

