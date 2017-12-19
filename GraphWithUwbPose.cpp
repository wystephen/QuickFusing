//
// Created by steve on 17-5-7.
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
#include "MYEKF.h"

#include "PUWBPF.hpp"

#include "EXUWBPF.hpp"


//#include "MYEKF.h"
#include<Eigen/Dense>
#include <Eigen/Geometry>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"

#include "OwnEdge/ZoEdge.h"
#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"


#include <sophus/so3.h>
#include <sophus/se3.h>

#include <algorithm>


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


int main(int argc, char *argv[]) {

    /**
     * Global value
     */

//    std::string dir_name = "/home/steve/Data/FastUwbDemo/3/";

//    std::string dir_name = "/home/steve/Data/IMUWB/27/";
//    std::string dir_name = "/home/steve/Data/NewRecord/Record2/";
//    std::string dir_name = "/home/steve/tmp/test/45/";
//    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/";

    std::string dir_name = "/home/steve/Data/NewIU/";

//    std::ofstream for_debug("/home/steve/Code/")
    double max_iterators(1000.0);//defualt paramet35s.
    double offset_cov(0.001), rotation_cov(0.002), range_cov(0.5);


    double valid_range(10.0), range_sigma(10.0), z0_info(5.0);

    int tmp_dir_num = 17;

    std::vector<int> beacon_mask;
    beacon_mask.push_back(0);
    beacon_mask.push_back(7);
    if (argc >= 2) {
        max_iterators = std::stod(argv[1]);
    }

//    if(0.01<=max_iterators<2)
//    {
//        max_iterators=5000;
//    }

    if (argc >= 5) {
        offset_cov = std::stod(argv[2]);
        rotation_cov = std::stod(argv[3]);
        range_cov = std::stod(argv[4]);
    }

    if (argc >= 8) {
        valid_range = std::stod(argv[5]);
        range_sigma = std::stod(argv[6]);
        z0_info = std::stod(argv[7]);
    }

    if (argc >= 9) {
        tmp_dir_num = std::stoi(argv[8]);
    }


    bool with_high(false);
    if (argc >= 10) {
        if (std::stoi(argv[9]) > 0) {
            with_high = true;
        } else {
            with_high = false;
        }
    }


    dir_name = dir_name + std::to_string(tmp_dir_num) + "/";
//    dir_name="/home/steve/Data/IU/76/";  // a good resutl.
//    dir_name = "/home/steve/Data/IU/76/";


    int trace_id = 0;
    const int beacon_id_offset(100000);


    /**
    * Build a global optimizer
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


    /**
     * Load Data from file
     */


    CppExtent::CSVReader UwbRawReader(dir_name + "uwb_result.csv");
    CppExtent::CSVReader BeaconSetReader(dir_name + "beaconset.csv");

    Eigen::MatrixXd uwb_raw(UwbRawReader.GetMatrix().GetRows(),
                            UwbRawReader.GetMatrix().GetCols());

    auto uwb_tmp_mat = UwbRawReader.GetMatrix();

    for (int i(0); i < uwb_raw.rows(); ++i) {
        for (int j(0); j < uwb_raw.cols(); ++j) {
            uwb_raw(i, j) = *(uwb_tmp_mat(i, j));
        }
    }

    auto beacon_tmp_mat = BeaconSetReader.GetMatrix();
    Eigen::MatrixXd beacon_raw(beacon_tmp_mat.GetRows(),
                               beacon_tmp_mat.GetCols());

    for (int i(0); i < beacon_raw.rows(); ++i) {
        for (int j(0); j < beacon_raw.cols(); ++j) {
            beacon_raw(i, j) = *(beacon_tmp_mat(i, j));
        }
    }


    CppExtent::CSVReader ImuDataReader(dir_name + "sim_imu.csv"),
            ZuptReader(dir_name + "sim_zupt.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = int(*ZuptTmp(i, 0));
    }

    CppExtent::CSVReader ZuptResultReader(dir_name + "sim_pose.csv");
    CppExtent::CSVReader QuatReader(dir_name + "all_quat.csv");
    CppExtent::CSVReader VertexTime(dir_name + "vertex_time.csv");


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


    /// ROBUST KERNEL
    static g2o::RobustKernel *robustKernel =
            g2o::RobustKernelFactory::instance()->construct("DCS");

    robustKernel->setDelta(1.5);
    /**
     * Build Graph
     */


    /// Add Beacon Vertex
    for (int i(0); i < beacon_raw.rows(); ++i) {
        auto *v = new g2o::VertexSE3();
        double p[6] = {0};
        p[0] = beacon_raw(i, 0);
        p[1] = beacon_raw(i, 1);
        p[2] = beacon_raw(i, 2);

        v->setEstimateData(p);
        v->setFixed(true);
        v->setId(beacon_id_offset + i);

        globalOptimizer.addVertex(v);
    }

    /**
     * SPECIALL
     * // TODO: REMOVE It!!!!
     */





    /// Add never used vertex

    int never_used_id = 9999910;
    auto *v = new g2o::VertexSE3();
    double p[6] = {0};

    v->setEstimateData(p);
    v->setFixed(true);
    v->setId(never_used_id);

    globalOptimizer.addVertex(v);

    ///Add ZUPT and ZUPT Edge
    Eigen::Isometry3d latest_transform = Eigen::Isometry3d::Identity();

    for (int index(0); index < zupt_res.rows(); ++index) {

        Eigen::Quaterniond qt;
        qt.x() = quat(index, 0);
        qt.y() = quat(index, 1);
        qt.z() = quat(index, 2);
        qt.w() = quat(index, 3);

        auto this_transform = tq2Transform(Eigen::Vector3d(zupt_res(index, 0),
                                                           zupt_res(index, 1),
                                                           zupt_res(index, 2)),
                                           qt);
        if (index == 0) {
            latest_transform = this_transform;
        }

        /// Add ZUPT Vertex
        auto *v = new g2o::VertexSE3();
        v->setId(index);
        v->setEstimate(this_transform);

        globalOptimizer.addVertex(v);


        // Add ZUPT Edge & z constrain
        if (index > 0) {

            /// Edge for z constraint
            auto *edge_zo = new Z0Edge();
            edge_zo->vertices()[0] = globalOptimizer.vertex(index - 1);
            edge_zo->vertices()[1] = globalOptimizer.vertex(index);

            Eigen::Matrix<double, 1, 1> info;
            info(0, 0) = z0_info;
            edge_zo->setInformation(info);
            edge_zo->setMeasurement(0.0);


            globalOptimizer.addEdge(edge_zo);


            /// ZUPT EDGE
            auto *edge_se3 = new g2o::EdgeSE3();

            edge_se3->vertices()[0] = globalOptimizer.vertex(index - 1);
            edge_se3->vertices()[1] = globalOptimizer.vertex(index);


            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


            information(0, 0) = information(1, 1) = information(2, 2) = 1.0 / offset_cov;
            information(3, 3) = information(4, 4) = information(5, 5) = 1.0 / rotation_cov;

            /// Compute the angle of each step
            auto current_transform = latest_transform.inverse() * this_transform;

            /**
             * Tu Fa Lian Gang
             */
            Eigen::Vector4d source_vec(1, 0, 0, 0);
            Eigen::Vector4d out_vec = current_transform * source_vec;

            double t_theta = std::atan(out_vec(1) / out_vec(0));
            if (std::abs(t_theta) > 0.1) {
                information(0, 0) =
                information(1, 1) =
                information(2, 2) = 1.0 / offset_cov;


                information(3, 3) =
                information(4, 4) =
                information(5, 5) = 1.0 / rotation_cov / 10.0;
            }


            edge_se3->setInformation(information);

            edge_se3->setMeasurement(latest_transform.inverse() * this_transform);


            globalOptimizer.addEdge(edge_se3);

        }

        latest_transform = this_transform;
    }



    /// ADD Range Edge

    int zupt_index(0);

    int uwb_index(0);


    double last_time = 0.0;
    double time_intervel = -10.0;

    std::ofstream range_file("./ResultData/range_file.txt");

    std::vector<DistanceEdge *> edge_vec;
    while (true) {
        if (zupt_index > v_time.rows() - 2) {
            break;
        }
        double zupt_time = v_time(zupt_index);//uwb_raw(uwb_index, 0);


        Eigen::VectorXd current_range(uwb_raw.cols());
        Eigen::VectorXd current_range_time_diff(uwb_raw.cols());
        current_range_time_diff.setOnes();
        current_range_time_diff *= 1000.0;
        current_range.setOnes();
        current_range *= -10;

        ///Find time diff smaller than 1.0(after find time diff smaller than 0.5)
        uwb_index = 0;

        while (true) {

            if (uwb_index > uwb_raw.rows() - 1) {
                break;

            }


            if (std::fabs(uwb_raw(uwb_index) - zupt_time) < 1.0) {
                if (zupt_time - last_time > time_intervel) {
                    last_time = zupt_time;

                    for (int bi(0); bi < uwb_raw.cols() - 1; ++bi) {
                        auto b_num = std::count(beacon_mask.begin(), beacon_mask.end(), bi);
                        if (uwb_raw(uwb_index, bi + 1) > 0
                            && uwb_raw(uwb_index, bi + 1) < 100.0
                            && b_num < 1) {
                            double range = uwb_raw(uwb_index, bi + 1);
                            int beacon_id = bi + beacon_id_offset;

                            int zupt_id = zupt_index;

                            auto *dist_edge = new DistanceEdge();
                            dist_edge->vertices()[0] = globalOptimizer.vertex(beacon_id);
                            dist_edge->vertices()[1] = globalOptimizer.vertex(zupt_id);

                            Eigen::Matrix<double, 1, 1> information;

                            information(0, 0) = 1 / range_cov;

                            dist_edge->setInformation(information);
                            dist_edge->setSigma(range_sigma);
                            dist_edge->setMeasurement(range);
                            current_range(bi) = range;


//                            dist_edge->setRobustKernel(robustKernel);

                            globalOptimizer.addEdge(dist_edge);
                            edge_vec.push_back(dist_edge);


                        }
                    }
                }

            }

            uwb_index++;
        }
//        globalOptimizer.edges()


        // Add edge after search all range:

        range_file << current_range(0);
        for (int tmp_k(1); tmp_k < current_range.rows(); ++tmp_k) {
            range_file << "," << current_range(tmp_k);
        }
        range_file << std::endl;

        zupt_index++;


    }

    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    for (int i(0); i < edge_vec.size(); ++i) {
        edge_vec[i]->setRobustKernel(robustKernel);
    }





    /// Initial graph and optimize
    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);

    if (max_iterators > 0) {
        globalOptimizer.optimize(max_iterators);
    }
//    globalOptimizer.optimize(6000);
    // Obtain start point pose
    double td[10] = {0};
    globalOptimizer.vertex(0)->getEstimateData(td);
    Eigen::Vector3d start_point(td[0], td[1], td[2]);

    /// Only-UWB PF
    int only_particle_num = 1;
    double only_eval_sigma = 3;
    double only_transpose_sigma = 1.5;

    std::vector<double> ux, uy;


    EXUWBPF<8> puwbpf(only_particle_num);
    puwbpf.SetMeasurementSigma(only_eval_sigma, beacon_raw.rows());
    puwbpf.SetInputNoiseSigma(only_transpose_sigma);

    puwbpf.SetBeaconSet(beacon_raw);
    puwbpf.Initial(Eigen::VectorXd(Eigen::Vector4d(
            start_point(0), start_point(1),
            0, 0
    )));
    double ppf_start_time = TimeStamp::now();

    for (int i(0); i < uwb_raw.rows(); ++i) {
//        if()

        puwbpf.StateTransmition(Eigen::Vector2d(0, 0), 3);
        puwbpf.Evaluation(uwb_raw.block(i, 1, 1, uwb_raw.cols() - 1).transpose(),
                          0);
        auto tmp = puwbpf.GetResult(0);
        puwbpf.Resample(-1, 0);
        ux.push_back(double(tmp(0)));
        uy.push_back(double(tmp(1)));
    }
    double ppf_end_time = TimeStamp::now();
    std::cout << " pure uwb pf cost time :" << ppf_end_time - ppf_start_time << std::endl;



    /**
     * output and Plot result
     */

    std::ofstream graph_res_file("./ResultData/graph.txt");
    std::ofstream zupt_res_file("./ResultData/zupt.txt");
    std::ofstream beacon_set("./ResultData/beacon_pose.txt");
    std::ofstream only_pf_file("./ResultData/only_pf.txt");
    std::vector<double> gx, gy, gz;
    for (int i(0); i < zupt_res.rows(); ++i) {
        double data[10] = {0};
        globalOptimizer.vertex(i)->getEstimateData(data);
        gx.push_back(data[0]);
        gy.push_back(data[1]);
        gz.push_back(data[2]);
        graph_res_file << data[0] << " " << data[1] << " " << data[2] << std::endl;
        zupt_res_file << zupt_res(i, 0) << " " << zupt_res(i, 1) << " " << zupt_res(i, 2) << std::endl;

    }

    std::vector<double> bx, by, bz;
    for (int i(0); i < uwb_raw.cols() - 1; ++i) {
        double data[10] = {0};
        globalOptimizer.vertex(i + beacon_id_offset)->getEstimateData(data);
        bx.push_back(data[0]);
        by.push_back(data[1]);
        bz.push_back(data[2]);
        beacon_set << data[0] << " " << data[1] << " " << data[2] << std::endl;
    }


    for (int i(0); i < uwb_raw.cols() - 1; ++i) {

        for (int j(0); j < i; ++j) {
            std::cout << std::sqrt(std::pow(bx[i] - bx[j], 2.0) +
                                   std::pow(by[i] - by[j], 2.0) + std::pow(bz[i] - bz[j], 2.0)) << "   ";
        }
        std::cout << std::endl;
    }

    for (int i(0); i < uwb_raw.cols() - 1; ++i) {
        std::cout << "i : " << i << " z = " << bz[i] << std::endl;
    }

    for (int i(0); i < ux.size(); ++i) {
        only_pf_file << ux[i] << " " << uy[i] << " 0.0" << std::endl;
    }

    int first_i = 0;
    int last_i = int(gx.size() - 1);

    std::cout << "distance between first and last: " <<
              std::sqrt(
                      std::pow(gx[first_i] - gx[last_i], 2.0) +
                      std::pow(gy[first_i] - gy[last_i], 2.0) +
                      std::pow(gz[first_i] - gz[last_i], 2.0)
              ) << std::endl;

//    plt::plot(gx, gy, "b-*");
    plt::named_plot("graph", gx, gy, "-*");
//    plt::plot(bx, by, "r*");
    plt::named_plot("beacon", bx, by, "D");
//    plt::plot(ux,uy,"g--");
    plt::named_plot("only_uwb_pf", ux, uy, "-*");
    plt::title("para:" + std::to_string(offset_cov) + ":"
               + std::to_string(rotation_cov) + ":" + std::to_string(range_cov) + ":"
               + std::to_string(valid_range) + ":" + std::to_string(range_sigma) +
               ":" + std::to_string(z0_info));

//    plt::save(std::to_string(TimeStamp::now())
//
//              + "test.jpg");
    plt::show();


}
