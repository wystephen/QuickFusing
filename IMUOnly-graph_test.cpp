//
// Created by steve on 17-9-4.
//

#include <eigen3/Eigen/Dense>
#include <ImuIntegrate.h>

#include "CSVReader.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "EKF.hpp"

#include "ResultEvaluation.hpp"

/////stamp---------

#include "RangeKF.hpp"

#include "PUWBPF.hpp"

#include "EXUWBPF.hpp"


#include "MYEKF.h"
//#include "Zero_Detecter.h"
#include<Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <sophus/so3.h>

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



//#include "g2o/types/slam3d_addons/vertex_line3d.h"
//#include "g2o/types/slam3d_addons/edge_se3_line.h"

//#include "OwnEdge/ZoEdge.h"
//#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"

#include "OwnEdge/OrientationEdge.h"
#include "OwnEdge/OrientationEdge.cpp"

//#include "OwnEdge/Line2D.h"
//#include "OwnEdge/Line2D.cpp"
//#include "OwnEdge/Point2Line2D.h"
//#include "OwnEdge/Point2Line2D.cpp"

//#include "OwnEdge/DistanceSE3Line3D.h"
//#include "OwnEdge/DistanceSE3Line3D.cpp"
//#include "g2o_types_slam3d_addons_api.h"
//#include "g2o/types/slam3d_addons/line3d.h"



G2O_USE_TYPE_GROUP(slam3d)


namespace plt = matplotlibcpp;

bool GLRT_Detector_special(Eigen::MatrixXd u,
                           const SettingPara &para_) {
    Eigen::Vector3d ya_m;
    double g = para_.gravity_;

    double T(0.0);
    Eigen::MatrixXd Tmatrix(1, 1);

    for (int i(0); i < 3; ++i) {
        ya_m(i) = u.block(i, 0, 1, u.cols()).mean();
    }

    Eigen::Vector3d tmp;

    for (int i(0); i < u.cols(); ++i) {

        tmp = u.block(0, i, 3, 1) - g * ya_m / ya_m.norm();
        double tt(0.0);

//        std::cout << " u block size : " << u.block(3,i,3,1).rows()<< std::endl;
//        std::cout << "tmp size :" << tmp.rows()<< std::endl;

        Tmatrix += (u.block(3, i, 3, 1).transpose() * u.block(3, i, 3, 1) / para_.sigma_a_ +
                    tmp.transpose() * tmp / para_.sigma_g_);


    }

    if (Tmatrix.size() != 1) {
        MYERROR("Tmatrxi size is not equal to 1")
    }

    T = Tmatrix(0, 0);

    T = T / double(para_.ZeroDetectorWindowSize_);
    if (T < para_.gamma_) {
        return true;
    } else {
        return false;
    }

}

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
    std::string dir_name = "/home/steve/Data/XIMU&UWB/3/";

    /// Global parameters
    double first_info(10), second_info(10 * M_PI / 180.0);
    double ori_info(10);

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
     * Initial  graph parameters
     */
    g2o::SparseOptimizer globalOptimizer;


    typedef g2o::BlockSolverX SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initial solver
    SlamLinearSolver *linearSolver = new SlamLinearSolver();
//    linearSolver->setBlockOrdering(false);
    linearSolver->setWriteDebug(true);
    SlamBlockSolver *blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver =
            new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    globalOptimizer.setAlgorithm(solver);


    int zupt_id_offset(0.0);
    int airpre_id_offset(100000);

    int attitude_vertex_id(400000);

    /// insert attitude offset vertex. represent the globle offset of positioning(6dof) between imu and world frame
    auto *v = new g2o::VertexSE3();
    double p[6] = {0.0};
    v->setEstimateData(p);

    v->setFixed(false);
    v->setId(attitude_vertex_id);
    globalOptimizer.addVertex(v);




    /**
     * Initial ZUPT parameters
     */
    SettingPara initial_para(true);
    initial_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    initial_para.init_heading1_ = M_PI / 2.0;
    initial_para.Ts_ = 1.0f / 128.0f;

    initial_para.sigma_a_ *= 0.8;
    initial_para.sigma_g_ *= 0.8;

    initial_para.ZeroDetectorWindowSize_ = 10;// Time windows size fo zupt detector

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata.block(0, 0, 20, 6));

    double last_zupt_flag = 0.0;

    int trace_id(0);
    Eigen::Isometry3d last_transform = Eigen::Isometry3d::Identity();
    double last_theta = 0.0;

    for (int index(0); index < imudata.rows(); ++index) {
//        std::cout << "index:" << index << std::endl;
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {
            if (GLRT_Detector_special(imudata.block(index - initial_para.ZeroDetectorWindowSize_,
                                                    0, initial_para.ZeroDetectorWindowSize_, 6).transpose(),
                                      initial_para)) {
                zupt_flag = 1.0;
            }
        }
        std::cout << "index:" << index << " zupt state: " << zupt_flag << std::endl;
        ///ZUPT GET POSITION
        auto tx = myekf.GetPosition(imudata.block(index, 0, 1, 6).transpose(), zupt_flag);

        if ((zupt_flag < 0.5 && last_zupt_flag > 0.5)) {
            std::cout << "index: " << index << "key step"
                      << "ori:" << myekf.getOriente() << std::endl;

            auto the_transform = myekf.getTransformation();

            /// vertex zupt
            auto *v = new g2o::VertexSE3();
            v->setId(trace_id);
            v->setEstimate(the_transform);

            globalOptimizer.addVertex(v);

            /// add transform edge
            if (trace_id > 0) {
                auto *edge_se3 = new g2o::EdgeSE3();

                edge_se3->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
                edge_se3->vertices()[1] = globalOptimizer.vertex(trace_id);

                Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


                information(0, 0) = information(1, 1) = information(2, 2) = first_info;
                information(3, 3) = information(4, 4) = information(5, 5) = second_info;

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

                if (is_corner) {
                    information /= corner_ratio;
                }


                edge_se3->setInformation(information);

                edge_se3->setMeasurement(last_transform.inverse() * the_transform);
                globalOptimizer.addEdge(edge_se3);


            }

            /// add ori edg
            auto *edge_ori = new OrientationEdge();

            edge_ori->vertices()[0] = globalOptimizer.vertex(trace_id);
            edge_ori->vertices()[1] = globalOptimizer.vertex(attitude_vertex_id);


            Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
            information *= ori_info;

            edge_ori->setInformation(information);

            Sophus::SO3 ori_so3(
                    Eigen::Quaterniond(imudata(index, 12), imudata(index, 10), imudata(index, 11), imudata(index, 9)));
            ori_1.push_back(ori_so3.log()(0));
            ori_2.push_back(ori_so3.log()(1));
            ori_3.push_back(ori_so3.log()(2));

            edge_ori->setMeasurement(ori_so3);
            /// robust kernel
            static g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
            edge_ori->setRobustKernel(robustKernel);

            globalOptimizer.addEdge(edge_ori);


            trace_id++;
            last_transform = the_transform;
        }

//        globalOptimizer.setVerbose(true);
//        globalOptimizer.initializeOptimization();
//        globalOptimizer.optimize(100);

        last_zupt_flag = zupt_flag;
        ix.push_back(tx(0));
        iy.push_back(tx(1));
    }

    ///optimization

    globalOptimizer.setVerbose(true);
    globalOptimizer.initializeOptimization();
//    globalOptimizer.optimize(30000);

    for (int k(0); k < trace_id; ++k) {
        double t_data[10] = {0};
        globalOptimizer.vertex(k)->getEstimateData(t_data);

        gx.push_back(t_data[0]);
        gy.push_back(t_data[1]);
    }


    plt::plot(gx, gy, "r-+");
    plt::plot(ix, iy, "b-");
//    plt::plot(ori_1,"r-+");
//    plt::plot(ori_2,"b-+");
//    plt::plot(ori_3,"g-+");
    plt::title("show");
    plt::show();


}