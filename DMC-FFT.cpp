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
#include "EKFR.h"
#include "EKFEigen.h"
#include "EKFSimple.h"
//#include "Zero_Detecter.h"
#include<Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <Zero_Detecter.h>
#include <OwnEdge/RelativeMagEdge.h>
#include <OwnEdge/ZoEdge.h>
#include <OwnEdge/ZoEdge.cpp>
#include <OwnEdge/GravityZ.h>
#include <OwnEdge/GravityZ.cpp>

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

#include "OwnEdge/RelativeMagEdge.h"
#include "OwnEdge/RelativeMagEdge.cpp"

#include "OwnEdge/SimpleDistanceEdge.h"
#include "OwnEdge/SimpleDistanceEdge.cpp"

//#include "OwnEdge/Line2D.h"
//#include "OwnEdge/Line2D.cpp"
//#include "OwnEdge/Point2Line2D.h"
//#include "OwnEdge/Point2Line2D.cpp"

//#include "OwnEdge/DistanceSE3Line3D.h"
//#include "OwnEdge/DistanceSE3Line3D.cpp"
//#include "g2o_types_slam3d_addons_api.h"
//#include "g2o/types/slam3d_addons/line3d.h"


#include "ImuKeyPointInfo.h"


G2O_USE_TYPE_GROUP(slam3d)


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


// Good result by Version 2 :
// ./cmake-build-debug/DirectMagConstraint 100 1000 0.03 0.002 0.25 0.5 0.0001 17 0.01 17


int main(int argc, char *argv[]) {
    std::string dir_name = "/home/steve/Data/II/";
    /**
     * Should be used data:
     * 16,17,19,20,28,31,32,33,34
     * 33---Velocity is changed
     * 34---inverse attitude
     */



    /// Global parameters
    double first_info(100), second_info(100), ori_info(0.03);
    double gravity_info(0.002);
    double mag_threshold(0.25);
    double loop_threshold(0.5), loop_info(0.0001);
    double max_ite(100);
    double zero_z_info(-10.1);
    int data_dir(20);

    if (argc >= 4) {
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);
        ori_info = std::stod(argv[3]);
    }

    if (argc >= 5) {
        gravity_info = std::stod(argv[4]);
    }

    if (argc >= 6) {
//        zero_z_info = std::stod(argv[5]);
        mag_threshold = std::stod(argv[5]);
    }

    if (argc >= 7) {
        loop_threshold = std::stod(argv[6]);
    }

    if (argc >= 8) {
        loop_info = std::stod(argv[7]);
    }

    if (argc >= 9) {
        max_ite = int(std::stod(argv[8]));
    }

    if (argc >= 10) {
        zero_z_info = std::stod(argv[9]);
    }

    if (argc >= 11) {
        data_dir = std::stoi(argv[10]);
    }

    dir_name = dir_name + std::to_string(data_dir) + "/";
    double turn_threshold = 10.0 / 180.0 * M_PI;
    double corner_ratio = 10.0;

    //// Load data
    CppExtent::CSVReader imu_data_reader(dir_name + "vertex_all_data.csv");

    Eigen::MatrixXd imudata;
    imudata.resize(imu_data_reader.GetMatrix().GetRows(),
                   imu_data_reader.GetMatrix().GetCols());
    imudata.setZero();
    auto imu_data_tmp_matrix = imu_data_reader.GetMatrix();

    /**
     * '''
        id | time ax ay az wx wy wz mx my mz pressure| x  y  z  vx vy vz| qx qy qz qw
        0  |   1   2  3 4  5   6  7 8  9  10 11      | 12 13 14 15 16 17| 18 19 20 21
        1 + 11 + 6 + 4 = 22
    '''
     */

    Eigen::Vector3d central(-58.0512, -117.0970, 151.9001);//imu2
    Eigen::Vector3d scale(213.8826, 208.3894, 232.3945);//imu2
    for (int i(0); i < imudata.rows(); ++i) {
        for (int j(0); j < imudata.cols(); ++j) {
            imudata(i, j) = *(imu_data_tmp_matrix(i, j));
            if (7 < j && j < 11) {
                imudata(i, j) = (imudata(i, j) - central(j - 8)) / scale(j - 8);
            }
        }
    }
    std::cout << "imu data size: " << imudata.rows() << "x"
              << imudata.cols() << std::endl;

    std::cout << "source imu data :\n" << imudata.block(0, 0, 10, imudata.cols()) << std::endl;

    CppExtent::CSVReader pairs_index_file(dir_name+"pairs.csv");
    auto pairs_data_tmp_matrix = pairs_index_file.GetMatrix();

    Eigen::MatrixXi pairs_vec;
    pairs_vec.resize(pairs_data_tmp_matrix.GetRows(),pairs_data_tmp_matrix.GetCols());
    for(int i(0);i<pairs_data_tmp_matrix.GetRows();++i)
    {
        pairs_vec(i,0) = int(*(pairs_data_tmp_matrix(i,0)));
        pairs_vec(i,1) = int(*(pairs_data_tmp_matrix(i,1)));
    }

    std::cout << "pairs:" << pairs_vec.rows() << std::endl;



//    std::cout << "source imu data last 10 lines:\n" << imudata.block(imudata.rows() - 11, 0, 10, 10) << std::endl;


    std::vector<double> ix, iy, iz; //ix iy
    std::vector<double> gx, gy, gz;// graph x

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

    v->setFixed(true);
    v->setId(attitude_vertex_id);
    globalOptimizer.addVertex(v);


    int trace_id(0);
    Eigen::Isometry3d last_transform = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d current_transform = Eigen::Isometry3d::Identity();
    double last_theta = 0.0;

    int last_optimized_id(0);

    std::vector<ImuKeyPointInfo> key_info_mag;
    std::vector<int> mag_before, mag_after;
    std::vector<bool> corner_flag_vec;
    corner_flag_vec.push_back(false);

    std::vector<int> corner_before, corner_after;
    std::vector<double> corner_score;


    for (int index(0); index < imudata.rows(); ++index) {

        ///Add vertex
        auto *v = new g2o::VertexSE3();
        v->setId(trace_id);
        v->setEstimate(current_transform);

//        if(trace_id==0)
//        {
//            v->setFixed(true);
//
//        }
        globalOptimizer.addVertex(v);
        /**
         * '''
            id | time ax ay az wx wy wz mx my mz pressure| x  y  z  vx vy vz| qx qy qz qw
            0  |   1   2  3 4  5   6  7 8  9  10 11      | 12 13 14 15 16 17| 18 19 20 21
            1 + 11 + 6 + 4 = 22
        '''
         */


        auto tmp_quaternion = Eigen::Quaterniond(imudata(index, 21),
                                                 imudata(index, 18),
                                                 imudata(index, 19),
                                                 imudata(index, 20));

        current_transform = Eigen::Isometry3d::Identity();
//        current_transform.matrix().block(0, 0, 3, 3) = tmp_quaternion.toRotationMatrix();
//        current_transform.matrix().block(0, 2, 3, 1) = imudata.block(index, 12, 1, 3).transpose();
        current_transform = tq2Transform(imudata.block(index, 12, 1, 3).transpose(),
                                         tmp_quaternion);

        if (trace_id > 0) {
            bool is_corner = false;




            /// Add z 0 edge
            if (zero_z_info > 0.0) {
                auto *edge_z0 = new Z0Edge();
                edge_z0->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
                edge_z0->vertices()[1] = globalOptimizer.vertex(trace_id);

                edge_z0->setMeasurement(0.0);
                edge_z0->setInformation(Eigen::Matrix<double, 1, 1>(zero_z_info));
                globalOptimizer.addEdge(edge_z0);

            }



            /// Add transform constraint
            auto *edge_se3 = new g2o::EdgeSE3();
            edge_se3->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
            edge_se3->vertices()[1] = globalOptimizer.vertex(trace_id);

            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


            information(0, 0) = information(1, 1) = information(2, 2) = first_info;
            information(3, 3) = information(4, 4) = information(5, 5) = second_info;


            auto detector_vec = tmp_quaternion.toRotationMatrix().matrix() * Eigen::Vector3d(1, 0, 0);
            if (detector_vec(0) < 0.9) {
                information /= 10.0;
                is_corner = true;
            }
            corner_flag_vec.push_back(is_corner);

            edge_se3->setInformation(information);

            edge_se3->setMeasurement(last_transform.inverse() * current_transform);
            globalOptimizer.addEdge(edge_se3);

            /// Add gravity constraint
            if (gravity_info > 0.0) {
                auto *edge_gravity = new GravityZ(imudata.block(trace_id - 1, 2, 1, 3).transpose(),
                                                  imudata.block(trace_id, 2, 1, 3).transpose());

                edge_gravity->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
                edge_gravity->vertices()[1] = globalOptimizer.vertex(trace_id);

                Eigen::Matrix2d info;
                info.setIdentity();
                info *= gravity_info;

                edge_gravity->setInformation(info);
                edge_gravity->setMeasurement(Eigen::Vector2d(0, 0));
                globalOptimizer.addEdge(edge_gravity);

            }

            /// Add mag constraint

            if (true) {
                for (int before_id(0); before_id < trace_id; ++before_id) {
                    if ((imudata.block(before_id, 8, 1, 3)
                         - imudata.block(trace_id, 8, 1, 3)).norm() < mag_threshold) {

                        if (loop_info > 0.0) {
                            if (//is_corner && corner_flag_vec[before_id] &&
                                    before_id > 10 &&
                                    trace_id < imudata.rows() - 15) {

                                double tmp_score = (imudata.block(before_id - 5, 8, 10, 3) -
                                                    imudata.block(trace_id - 5, 8, 10, 3)).norm();

                                if (tmp_score < loop_threshold) {
                                    corner_before.push_back(before_id);
                                    corner_after.push_back(trace_id);
                                    corner_score.push_back(tmp_score);


                                    auto *dis_edge = new SimpleDistanceEdge();
                                    dis_edge->vertices()[0] = globalOptimizer.vertex(before_id);
                                    dis_edge->vertices()[1] = globalOptimizer.vertex(trace_id);

                                    dis_edge->setMeasurement(0.0);
                                    dis_edge->setInformation(Eigen::Matrix<double, 1, 1>(loop_info));

                                    globalOptimizer.addEdge(dis_edge);
                                }



//                            dis_edge->set

                            }
                        }


                        if (ori_info > 0.0) {
                            mag_before.push_back(before_id);
                            mag_after.push_back(trace_id);

                            auto *mag_edge = new RelativeMagEdge(imudata.block(before_id, 8, 1, 3).transpose(),
                                                                 imudata.block(trace_id, 8, 1, 3).transpose()
                            );

                            mag_edge->vertices()[0] = globalOptimizer.vertex(before_id);
                            mag_edge->vertices()[1] = globalOptimizer.vertex(trace_id);

                            Eigen::Matrix<double, 3, 3> information_matrix = Eigen::Matrix<double, 3, 3>::Identity();
                            information_matrix *= ori_info;

                            mag_edge->setInformation(information_matrix);

                            mag_edge->setMeasurement(Eigen::Vector3d(0, 0, 0));


                            static g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct(
                                    "Cauchy");
//                    mag_edge->setRobustKernel(robustKernel);

                            globalOptimizer.addEdge(mag_edge);
                        }


                    }

                }
            }


        }


        if (trace_id - last_optimized_id > 15) {

            last_optimized_id = trace_id;
            globalOptimizer.setVerbose(true);
            globalOptimizer.initializeOptimization();
//            globalOptimizer.updateInitialization()
            globalOptimizer.optimize(10, false);
        }

//        if(trace_id>50)
//        {
//            globalOptimizer.vertex(trace_id-50)->setFixed(true);
//        }

        last_transform = current_transform;

        trace_id++;
        ix.push_back(current_transform(0, 3));
        iy.push_back(current_transform(1, 3));
        iz.push_back(current_transform(2, 3));


    }

    globalOptimizer.setVerbose(true);
//    globalOptimizer.initMultiThreading();
    globalOptimizer.initializeOptimization();
//    globalOptimize
    globalOptimizer.optimize(max_ite);



    ///optimization

    std::ofstream test("./ResultData/test.txt");
    std::ofstream test_imu("./ResultData/text_imu.txt");
    std::ofstream test_pairs("./ResultData/pair.txt");
    std::ofstream test_corner_pairs("./ResultData/corner_pair.txt");


    for (int k(0); k < ix.size(); ++k) {
        test_imu << ix[k]
                 << ","
                 << iy[k]
                 << ","
                 << iz[k]
                 << std::endl;
    }
    auto *t_data = new double[10];
    for (int k(0); k < trace_id; ++k) {
        globalOptimizer.vertex(k)->getEstimateData(t_data);
        test << t_data[0] << "," << t_data[1] << "," << t_data[2] << std::endl;


        gx.push_back(t_data[0]);
        gy.push_back(t_data[1]);
        gz.push_back(t_data[2]);

    }

    delete[] t_data;

    for (int k(0); k < mag_before.size(); ++k) {
        test_pairs << mag_before[k]
                   << ","
                   << mag_after[k]
                   << std::endl;

    }
    for (int k(0); k < corner_before.size(); ++k) {
        test_corner_pairs << corner_before[k]
                          << ","
                          << corner_after[k]
                          << ","
                          << corner_score[k]
                          << std::endl;
    }

    test.close();
    test_imu.close();
    test_pairs.close();


    plt::grid(true);
    plt::plot(gx, gy, "r-+");
    plt::plot(ix, iy, "b-");
    plt::title(std::to_string(first_info) + "-" +
               std::to_string(second_info) + "-" +
               std::to_string(ori_info) + "-" +
               std::to_string(gravity_info) + "-" +
               std::to_string(mag_threshold) + "-" +
               std::to_string(loop_threshold) + "-" +
               std::to_string(loop_info) + "-" +
               std::to_string(zero_z_info));
    plt::show();
//    plt::save(std::to_string(first_info) + "-" +
//              std::to_string(second_info) + "-" +
//              std::to_string(ori_info) + "-" +
//              std::to_string(gravity_info) + "-" +
//              std::to_string(mag_threshold) + ".png");


}