//
// Created by steve on 17-9-11.
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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam_unstable/dynamics/VelocityConstraint3.h>

#include <gtsam/base/LieVector.h>
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
    std::string dir_name = "/home/steve/Data/XIMU&UWB/5/";

    /// Global parameters
    double first_info(10), second_info(10 * M_PI / 180.0);
    double ori_info(100);

    double turn_threshold = 1000.0;
    double corner_ratio = 10.0;

    //// Load data
    CppExtent::CSVReader imu_data_reader(dir_name + "ImuData.csv");

    Eigen::MatrixXd imudata;
    imudata.resize(imu_data_reader.GetMatrix().GetRows(),
                   imu_data_reader.GetMatrix().GetCols());
    imudata.setZero();
    auto imu_data_tmp_matrix = imu_data_reader.GetMatrix();

    for (int i(0); i < imudata.rows(); ++i) {
        for (int j(0); j < imudata.cols(); ++j) {
            imudata(i, j) = *(imu_data_tmp_matrix(i, j));
        }
    }
    std::cout << "imu data size: " << imudata.rows() << "x"
              << imudata.cols() << std::endl;


    std::vector<double> ix, iy; //ix iy
    std::vector<double> gx, gy;// graph x

    std::vector<double> ori_1, ori_2, ori_3;

    /**
     * Initial ZUPT parameters
     */
    SettingPara initial_para(true);
    initial_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    initial_para.init_heading1_ = M_PI / 2.0;
    initial_para.Ts_ = 1.0f / 128.0f;

    initial_para.sigma_a_ = 1.1;//zupt detector parameter
    initial_para.sigma_g_ = 2.0 / 180.0 * M_PI;

    initial_para.ZeroDetectorWindowSize_ = 10;

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata.block(0, 0, 20, 6));

    /**
     * Create and prepare for graph
     *
     */

    Eigen::Matrix<double, 10, 1> initial_state = Eigen::Matrix<double, 10, 1>::Zero();
    initial_state(6) = 1.0;
    // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
//    Rot3 prior_rotation = Rot3::Quaternion(initial_state(6), initial_state(3),
//                                           initial_state(4), initial_state(5));
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
            (Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m
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

    boost::shared_ptr<PreintegratedImuMeasurements::Params> p =
            PreintegratedImuMeasurements::Params::MakeSharedD(9.8);

    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous



    NavState prev_state(prior_pose, prior_velocity);
    NavState prop_state = prev_state;
    imuBias::ConstantBias prev_bias = prior_imu_bias;

    ////Define the imu preintegration
    imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);

    /**
     *
     * Initial ekf
     */

    double last_zupt_flag = 0.0;


    int trace_id(0);
    Eigen::Isometry3d last_transform = Eigen::Isometry3d::Identity();
    double last_theta = 0.0;

    std::vector<double> index_score;


    int add_vertex_counter = 0;

    for (int index(0); index < imudata.rows(); ++index) {
        /**ZUPT DETECTOR FIRST*/
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {

            if (GLRT_Detector(imudata.block(index - initial_para.ZeroDetectorWindowSize_,
                                            0, initial_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                              initial_para)) {
                zupt_flag = 1.0;
            }
        }

        /** ZUPT METHOD **/
        auto tx = myekf.GetPosition(imudata.block(index, 0, 1, 6).transpose(), zupt_flag);
        if (0 == index || (zupt_flag < 0.5 & last_zupt_flag > 0.5)) {
//            std::cout << "index: " << index << "key step"
//                      << "ori:" << myekf.getOriente() << std::endl;

            auto the_transform = myekf.getTransformation();

            ix.push_back(tx(0));
            iy.push_back(tx(1));
        }

        /** GTSAM FOR INTEGRATE **/
        add_vertex_counter++;
        if (add_vertex_counter > 3) {
            /// first moment of zupt detected
            add_vertex_counter = 0;

            trace_id++;
//            std::cout << "trace id : " << trace_id << std::endl;
            ///Added to
            PreintegratedImuMeasurements *preint_imu =
                    dynamic_cast<PreintegratedImuMeasurements *>(imu_preintegrated_);


            try {

//                std::cout << "prev state :" << prev_state << std::endl;
//                std::cout << " pre bias: " << prev_bias << std::endl;
//                std::cout << "derect output:" << preint_imu->predict(prev_state, prev_bias);

                ImuFactor imu_factor(
                        X(trace_id - 1), V(trace_id - 1),
                        X(trace_id), V(trace_id),
                        B(trace_id), *preint_imu
                );


//                std::cout << "after built imu factor " << std::endl;
                graph->add(imu_factor);
//                std::cout << " after added imu factor " << std::endl;
                imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
//
                graph->add(BetweenFactor<imuBias::ConstantBias>(
                        B(trace_id - 1),
                        B(trace_id),
                        zero_bias, bias_noise_model
                ));


                // velocity constraint
                if (zupt_flag > 0.5) {
                    noiseModel::Diagonal::shared_ptr velocity_noise =
                            noiseModel::Isotropic::Sigma(3, 0.0000001);


                    PriorFactor<Vector3> zero_velocity(V(trace_id),
                                                       Vector3(0.0, 0.0, 0.0),
                                                       velocity_noise);

                    graph->add(zero_velocity);

                } else {
                    noiseModel::Diagonal::shared_ptr velocity_noise = noiseModel::Isotropic::Sigma(3,
                                                                                                   10.1);


                    PriorFactor<Vector3> zero_velocity(V(trace_id),
                                                       Vector3(0.0, 0.0, 0.0),
                                                       velocity_noise);
//                    graph->add(zero_velocity);
                }

//                PriorFactor<Rot3> orietation_constraint(X)


// / last moment of zupt detected
                prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
                initial_values.insert(X(trace_id), Pose3());
                initial_values.insert(V(trace_id), Vector3(0, 0, 0));
                initial_values.insert(B(trace_id), prev_bias);
                prev_state = prop_state;

                preint_imu->resetIntegration();


                //optimize
//                GaussNewtonOptimizer optimizer(*graph, initial_values);
//                for (int i(0); i < 100; ++i) {
//                    optimizer.iterate();
//                }
//                optimizer.optimizeSafely();
//                std::cout << "after : " << trace_id << std::endl;
//                initial_values = optimizer.values();

            } catch (const std::exception &e) {
                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : " << e.what() << std::endl;
            } catch (...) {
                std::cout << "unexpected error " << std::endl;
            }

            //velocity constraint

            /// Use zupt result as gps
            try {

                noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 11005.1);
                GPSFactor gps_factor(X(trace_id),
                                     Point3(0, 0, 0),
                                     correction_noise);
//                if (zupt_flag > 0.5) graph->add(gps_factor);

            } catch (std::exception &e) {
                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : " << e.what() << std::endl;
            } catch (...) {
                std::cout << "error at :" << __FILE__
                          << " " << __LINE__ << " : unkonw error " << std::endl;
            }

//            if(zupt_flag>0.5)
//            {
//
//            }

            /// reset integrated
//            imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

        }


        imu_preintegrated_->integrateMeasurement(imudata.block(index, 0, 1, 3).transpose(),
                                                 imudata.block(index, 3, 1, 3).transpose(),
                                                 initial_para.Ts_);



        /**
         * updata data
         */
        last_zupt_flag = zupt_flag;
    }


//    noiseModel::Diagonal::shared_ptr correction_noise = noiseModel::Isotropic::Sigma(3, 11005.1);
//    GPSFactor gps_factor(X(trace_id),
//                         Point3(0, 0, 0),
//                         correction_noise);
//    graph->add(gps_factor);

    ///optimization

    std::cout << "begin to optimization" << std::endl;
//    LevenbergMarquardtParams lm_para;
//    lm_para.setMaxIterations(10000);
    LevenbergMarquardtOptimizer optimizer(*graph, initial_values);//, lm_para);
//    GaussNewtonOptimizer optimizer(*graph, initial_values);

//    for (int i(0); i < 50000; i++) {
//        optimizer.iterate();
//        if (i % 100 == 0) std::cout << "i :'" << i << std::endl;
//    }
//    auto result = optimizer.values();

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

    auto result = initial_values;
//    try {

    result = optimizer.optimizeSafely();

//    } catch (std::exception &e) {
//        std::cout << e.what() << " :" << __FILE__ << ":" << __LINE__ << std::endl;
//        return 0;
//
//
//    }
//    std::cout << "trace id :" << trace_id << std::endl;
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

    plt::plot(gx, gy, "r-+");
    plt::plot(ix, iy, "b-");
//    plt::plot(ori_1,"r-+");
//    plt::plot(ori_2,"b-+");
//    plt::plot(ori_3,"g-+");
    plt::title("show");
    plt::show();


}