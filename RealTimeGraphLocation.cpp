//
// Created by steve on 17-9-12.
//
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
/// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Zero_Detecter.h>

///OWN
#include "CSVReader.h"
#include "MyError.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "MYEKF.h"

///G2o
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

///G2o Own
#include "OwnEdge/ZoEdge.h"
#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"


/// For plot 2d
G2O_USE_TYPE_GROUP(slam3d)


namespace plt = matplotlibcpp;// matplotlib plot


/**
 * From offset and quaternion to Transform matrix(4x40
 * @param offset (x,y,z)
 * @param q (qw,qx,qy,qz)
 * @return  Transform matrix(4x4)
 */
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

    /// All paramaters for algorithm

    int only_method = 3;
    int only_particle_num = 1150;
    double only_transpose_sigma = 0.3;
    double only_eval_sigma = 5.0;

    int fus_particle_num = 30000;
    double fus_transpose_sigma = 1.3;
    double fus_eval_sigma = 1.0;



    /// g2o parameter

    double first_info = 1000.0;
    double second_info = 1000.0;


    double distance_info = 1.0;
    double distance_sigma = 3.0;


    double z_offset = 1.90 - 1.12;

    double turn_threshold = 1.0;
    double corner_ratio = 10.0;

    int max_optimize_times = 40;

    double time_offset = 0.0;


    double uwb_err_threshold = 0.5;

    int delay_times = 25;

    int out_delay_times = 4;

    int data_num = 92;


    if (argc == 14) {
        std::cout << "set para meter s" << std::endl;
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);

        distance_info = std::stod(argv[3]);
        distance_sigma = std::stod(argv[4]);

        z_offset = std::stod(argv[5]);

        turn_threshold = std::stod(argv[6]);
        corner_ratio = std::stod(argv[7]);

        max_optimize_times = std::stoi(argv[8]);

        time_offset = std::stod(argv[9]);

        uwb_err_threshold = std::stod(argv[10]);

        delay_times = std::stoi(argv[11]);

        out_delay_times = std::stoi(argv[12]);

        data_num = std::stoi(argv[13]);
    }


    std::string out_dir_name = "./";
    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/";


    dir_name = dir_name + std::to_string(data_num) + "/";
    if (argc != 10) {
        out_dir_name = dir_name;
    }

    std::cout.precision(20); //

    double first_t(TimeStamp::now());

    /**
     * Load data
     */
    CppExtent::CSVReader UwbRawReader(dir_name + "uwb_result.csv");

    Eigen::MatrixXd uwb_raw(UwbRawReader.GetMatrix().GetRows(), UwbRawReader.GetMatrix().GetCols());

    for (int i(0); i < uwb_raw.rows(); ++i) {
        for (int j(0); j < uwb_raw.cols(); ++j) {
            uwb_raw(i, j) = *(UwbRawReader.GetMatrix()(i, j));
        }
    }


    CppExtent::CSVReader ImuDataReader(dir_name + "sim_imu.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix());

    Eigen::MatrixXd imu_data, Zupt;
    imu_data.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            imu_data(i, j) = *ImuDataTmp(i, j);
            if (0 < j && j < 4) {
                imu_data(i, j) *= 9.81;
            } else if (4 <= j && j < 7) {
                imu_data(i, j) *= (M_PI / 180.0f);
            }
        }
    }


    /**
     * Build graph and optimizer
     */
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolverX SlamBlockSolver;
    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
//    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initial solver
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
    linearSolver->setWriteDebug(true);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);


    /// ROBUST KERNEL
    static g2o::RobustKernel *robustKernel =
            g2o::RobustKernelFactory::instance()->construct("Cauchy");

    /**
     * ZUPT initial
     */
    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    init_para.init_heading1_ = 0.0;


    init_para.sigma_acc_ = 0.5 * Eigen::Vector3d(1, 1, 1);
    init_para.sigma_gyro_ = 0.5 * Eigen::Vector3d(1, 1, 1) * M_PI / 180.0;
    init_para.ZeroDetectorWindowSize_ = 10;

    init_para.sigma_initial_pos1_ *= 1e-3;
    init_para.sigma_initial_att1_ = Eigen::Vector3d(0.1, 0.1, 0.1) * M_PI / 180.0;

    init_para.Ts_ = 1.0f / 200.0f;

    init_para.gravity_ = 9.8;

    MyEkf myekf(init_para);

    std::vector<double> zupt_v;

    myekf.InitNavEq(imu_data.block(1, 1, 40, 6));

    /**
     * Vector for save result
     */
    std::vector<double> online_gx, online_gy, online_gz;// online graph optimize output
    std::vector<double> gx, gy, gz;// graph_optimizer output
    std::vector<double> imu_x, imu_y, imu_z;// imu output

    std::vector<int> imu_vertex_id;// save imu_data_index for each SE3 vertex.


    /**
     * Main loop
     */
    int beacon_id_offset = 80000;// beacon_offset

    int trace_id = 0;

    for (int beacon_id(0); beacon_id < uwb_raw.cols() - 1; ++beacon_id) {
        auto *  v = new g2o::VertexSE3();
        double p[6] = {0};
        v->setEstimateData(p);
        v->setFixed(false);
        v->setId(beacon_id_offset + beacon_id);
        globalOptimizer.addVertex(v);

    }


    int uwb_data_index(0);
    int imu_data_index(0);


    bool last_zupt_flag = true;
    double last_theta = 0.0;

    Eigen::Isometry3d last_transform = myekf.getTransformation();// Eigen::Isometry3d::Identity();
    last_theta = myekf.getOriente();


    /**
  * Spacial preprocess !!!!
  * set z of beaconset to zero
  */



    /// add beacon vertex


    bool tmp_set_bool(true);// avoid the warning given by IDE...
    while (tmp_set_bool) {
        //
        if (uwb_data_index >= uwb_raw.rows() || imu_data_index >= imu_data.rows()) {
            tmp_set_bool=false;

            break;
        }


        if (uwb_raw(uwb_data_index, 0) < imu_data(imu_data_index, 0)) {
            /// next uwb raw
            uwb_data_index++;

        } else {

            bool zupt_flag = true;

            if (imu_data_index > init_para.ZeroDetectorWindowSize_) {
//            std::cout << "imu_data_index :" << imu_data_index << std::endl;
                zupt_flag = GLRT_Detector(imu_data.block(imu_data_index - init_para.ZeroDetectorWindowSize_, 1,
                                                         init_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                                          init_para);
            }
//        std::cout << "zupt flag:" << zupt_flag << std::endl;
            zupt_v.push_back(zupt_flag ? 1.0 : 0.0);

            auto ekf_output = myekf.GetPosition(
                    imu_data.block(imu_data_index, 1, 1, 6).transpose(),
                    (zupt_flag ? 1.0 : 0.0)
            );
            if (!zupt_flag && last_zupt_flag) {


                imu_x.push_back(ekf_output(0));
                imu_y.push_back(ekf_output(1));


                auto the_transform = myekf.getTransformation();


                /// add vertex
                auto *v = new g2o::VertexSE3();
                double p[6] = {0};
                v->setId(trace_id);
                v->setFixed(false);
                v->setEstimate(the_transform);
                globalOptimizer.addVertex(v);



                /// get delta theta
                double the_theta(myekf.getOriente());
                double delta_ori = the_theta - last_theta;

                if (delta_ori > M_PI) {
                    delta_ori -= (2 * M_PI);
                } else if (delta_ori < -M_PI) {
                    delta_ori += (2.0 * M_PI);
                }
                if (std::isnan(delta_ori)) {
                    delta_ori = 0.0;
                }
//                gekf.getDeltaOrientation();
                bool is_corner(false);
//                if(std::abs(the_theta-))
                if (std::abs(delta_ori) > turn_threshold) {
                    is_corner = true;
                }

                last_theta = the_theta;

                ///add transform edge

                if (trace_id > 0) {
                    auto *edge_se3 = new g2o::EdgeSE3();

                    edge_se3->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
                    edge_se3->vertices()[1] = globalOptimizer.vertex(trace_id);

                    std::cout << "traid and before id :"<< trace_id << "--"<< trace_id-1 << std::endl;

                    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


                    information(0, 0) = information(1, 1) = information(2, 2) = first_info;
                    information(3, 3) = information(4, 4) = information(5, 5) = second_info;

                    if (is_corner) {
                        information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
                        information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
                    }

                    edge_se3->setInformation(information);
                    edge_se3->setMeasurement(last_transform.inverse() * the_transform);


                    globalOptimizer.addEdge(edge_se3);


                }

                /// add range edge

                if (std::abs(uwb_raw(uwb_data_index, 0) - imu_data(imu_data_index, 0)) < 1.0) {

                    Eigen::VectorXd uwb_measure;

                    uwb_measure.resize(uwb_raw.cols() - 1);
                    uwb_measure.setZero();

                    uwb_measure = uwb_raw.block(uwb_data_index, 1, 1, uwb_measure.rows()).transpose();

                    for (int bi(0); bi < uwb_measure.rows(); ++bi) {
                        if ((uwb_measure(bi) - z_offset) < 0.0) {
                            continue;
                        } else {
                            auto *dist_edge = new DistanceEdge();
                            dist_edge->vertices()[0] = globalOptimizer.vertex(beacon_id_offset + bi);
                            dist_edge->vertices()[1] = globalOptimizer.vertex(trace_id);

                            std::cout << "trace id:"<<trace_id << "beaconid:" << bi << std::endl;

                            dist_edge->setMeasurement(
                                    std::sqrt(uwb_measure(bi) * uwb_measure(bi) - z_offset * z_offset));

                            Eigen::Matrix<double, 1, 1> information;
                            information(0, 0) = distance_info;

                            dist_edge->setInformation(information);
                            dist_edge->setSigma(distance_sigma);
//                            dist_edge->setRobustKernel(new g2o::RobustKernelHuber());

//                            globalOptimizer.addEdge(dist_edge);
                        }

                    }


                }



                /// updata transform matrix
                last_transform = the_transform;

                trace_id++;

            }


            imu_data_index++;


            last_zupt_flag = zupt_flag;
        }


    }


    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(2);


    /**
     * Save and show
     */
    for (int vid(0); vid < trace_id; ++vid) {
        double data[10] = {0};
        globalOptimizer.vertex(vid)->getEstimateData(data);
        gx.push_back(data[0]);
        gy.push_back(data[1]);
    }

    plt::plot(zupt_v, "r-+");
//    plt::named_plot("imu",imu_x, imu_y, "g-+");
//    plt::named_plot("online graph",online_gx, online_gy, "b-+");
//    plt::named_plot("graph",gx, gy, "r-+");
//    plt::legend();
    plt::grid(true);
    plt::show();


}