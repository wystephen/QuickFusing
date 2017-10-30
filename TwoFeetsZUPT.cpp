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
#include "ImuKeyPointInfo.h

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


    Vector3 prior_velocity(initial_state.tail<3>());
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


    std::vector<ImuKeyPointInfo> left_info_vec,right_info_vec;

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

            } catch (...) {
                assert(true);
            }


        }
        imu_preintegrated_l_->integrateMeasurement(
                imudata_l.block(index, 1, 1, 3).transpose(),
                imudata_l.block(index, 4, 1, 3).transpose(),
                initial_para.Ts_);


    }


}