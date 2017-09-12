//
// Created by steve on 17-9-12.
//
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
    double distance_sigma = 2.0;


    double z_offset = 1.90 - 1.12;

    double turn_threshold = 1.0;
    double corner_ratio = 10.0;

    int max_optimize_times = 4000;

    double time_offset = 0.0;


    double uwb_err_threshold = 0.5;

    int delay_times = 25;

    int out_delay_times = 4;

    int data_num = 93;


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


    dir_name = dir_name + std::to_string(data_num);
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


    CppExtent::CSVReader ImuDataReader(dir_name + "sim_imu.csv"),
            ZuptReader(dir_name + "sim_zupt.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd imu_data, Zupt;
    imu_data.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            imu_data(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = int(*ZuptTmp(i, 0));
    }

    CppExtent::CSVReader ZuptResultReader(dir_name + "sim_pose.csv");
    CppExtent::CSVReader QuatReader(dir_name + "all_quat.csv");
    CppExtent::CSVReader VertexTime(dir_name + "vertex_time.csv");

    Eigen::MatrixXd v_high(1, 1);


    Eigen::MatrixXd zupt_res(ZuptResultReader.GetMatrix().GetRows(), ZuptResultReader.GetMatrix().GetCols());
    Eigen::MatrixXd quat(QuatReader.GetMatrix().GetRows(), QuatReader.GetMatrix().GetCols());
    Eigen::MatrixXd v_time(VertexTime.GetMatrix().GetRows(), VertexTime.GetMatrix().GetCols());

    for (int i(0); i < zupt_res.rows(); ++i) {
        for (int j(0); j < zupt_res.cols(); ++j) {
            zupt_res(i, j) = *(ZuptResultReader.GetMatrix()(i, j));
        }
    }

    for (int i(0); i < quat.rows(); ++i) {
        for (int j(0); j < quat.cols(); ++j) {
            quat(i, j) = *(QuatReader.GetMatrix()(i, j));
        }
    }

    for (int i(0); i < v_time.rows(); ++i) {
        for (int j(0); j < v_time.cols(); ++j) {
            v_time(i, j) = *(VertexTime.GetMatrix()(i, j));
        }
    }

    /**
     * Build graph and optimizer
     */
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolverX SlamBlockSolver;
//    typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

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
    init_para.init_heading1_ = 0.0;// -2.0;//M_PI / 2.0;


    init_para.sigma_acc_ = 0.5 * Eigen::Vector3d(1,1,1);
    init_para.sigma_gyro_= 0.5 * Eigen::Vector3d(1,1,1) * M_PI / 180.0;

    init_para.sigma_initial_pos1_ *= 1e-3;
//    init_para.sigma_initial_vel1_ *=
    init_para.sigma_initial_att1_ = Eigen::Vector3d(0.1,0.1,0.1) * M_PI / 180.0;

//    init_para.Ts_ = 0.005f;//1.0/ 200.0;
//    init_para.Ts_ = 1.0f/100.0f;//1.0/ 200.0;
    init_para.Ts_ = 1.0f / 200.0f;

    init_para.gravity_ = 9.8;

    MyEkf myekf(init_para);

    std::vector<double> zupt_v;

    myekf.InitNavEq(imu_data.block(1, 1, 40, 6));

    /**
     * Vector for save result
     */
    std::vector<double> online_gx,online_gy,online_gz;
    std::vector<double> gx,gy,gz;
    std::vector<double> imu_x,imu_y,imu_z;


    /**
     * Main loop
     */
    int beacon_offset = 100000;// beacon_offset

    int trace_id = 0;

    for(int beacon_id(0);beacon_id<uwb_raw.cols()-1;++beacon_id)
    {
        auto *v= new g2o::VertexSE3();
        double p[6] = {0};
        v->setEstimateData(p);
        v->setFixed(false);
        v->setId(beacon_offset+beacon_id);
        globalOptimizer.addVertex(v);

    }

    bool last_zupt_flag = false;

    for(int index(0);index < imu_data.rows();++index)
    {
        bool zupt_flag = false;

        if(index > init_para.ZeroDetectorWindowSize_)
        {
            zupt_flag=GLRT_Detector(imu_data.block(index-init_para.ZeroDetectorWindowSize_,1,
            init_para.ZeroDetectorWindowSize_,6),init_para);
        }





       last_zupt_flag = zupt_flag;
    }





    /**
     * Save and show
     */



}