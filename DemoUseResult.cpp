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


//#include "g2o/types/slam3d_addons/vertex_line3d.h"
//#include "g2o/types/slam3d_addons/edge_se3_line.h"

#include "OwnEdge/ZoEdge.h"
#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"

//#include "OwnEdge/Line2D.h"
//#include "OwnEdge/Line2D.cpp"
//#include "OwnEdge/Point2Line2D.h"
//#include "OwnEdge/Point2Line2D.cpp"

//#include "OwnEdge/DistanceSE3Line3D.h"
//#include "OwnEdge/DistanceSE3Line3D.cpp"
//#include "g2o_types_slam3d_addons_api.h"
//#include "g2o/types/slam3d_addons/line3d.h"


#include <sophus/so3.h>
#include <sophus/se3.h>


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
    std::string dir_name = "/home/steve/Code/Mini_IMU/Scripts/IMUWB/";

//    std::ofstream for_debug("/home/steve/Code/")

    double offset_cov(0.001), rotation_cov(0.002), range_cov(0.5);
    double max_iterators(1000.0);//defualt paramet35s.

    double valid_range(10.0), range_sigma(10.0), z0_info(5.0);


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

    int tmp_dir_num = 54;
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
    dir_name="/home/steve/Data/IU/74/";


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

    Eigen::MatrixXd uwb_raw(UwbRawReader.GetMatrix().GetRows(), UwbRawReader.GetMatrix().GetCols());

    for (int i(0); i < uwb_raw.rows(); ++i) {
        for (int j(0); j < uwb_raw.cols(); ++j) {
            uwb_raw(i, j) = *(UwbRawReader.GetMatrix()(i, j));
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

//    Eigen::MatrixXd v_high(1, 1);
//
//    if (with_high) {
//        CppExtent::CSVReader VertexHigh(dir_name + "vertex_high_modified.csv");
//
//        v_high.resize(VertexHigh.GetMatrix().GetRows(), VertexHigh.GetMatrix().GetCols());
////        v_high.setZero(VertexHigh.GetMatrix().GetRows(), VertexHigh.GetMatrix().GetCols());
//
//        auto v_high_matrix = VertexHigh.GetMatrix();
//
//        for (int i(0); i < v_high.rows(); ++i) {
//            for (int j(0); j < v_high.cols(); ++j) {
//                v_high(i, j) = *v_high_matrix(i, j);
//            }
//        }
//    }

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
            g2o::RobustKernelFactory::instance()->construct("Cauchy");
    /**
     * Build Graph
     */


    /// Add Beacon Vertex
    for (int i(0); i < uwb_raw.cols() - 1; ++i) {
        auto *v = new g2o::VertexSE3();
        double p[6] = {0};

        v->setEstimateData(p);
        v->setFixed(false);
        v->setId(beacon_id_offset + i);

        globalOptimizer.addVertex(v);
    }

    /**
     * SPECIALL
     * // TODO: REMOVE It!!!!
     */
//
//    std::vector<int> low_b{0, 4, 6, 7, 8, 9, 0};
//    std::vector<int> high_b{6, 5, 1, 3, 2, 6};
//
//    double *beacon_high = new double[uwb_raw.cols()];
//
//    if (with_high) {
//        for (int i(0); i < low_b.size() - 1; ++i) {
//            auto *e = new Z0Edge();
//            e->vertices()[0] = globalOptimizer.vertex(beacon_id_offset + low_b[i]);
//            e->vertices()[1] = globalOptimizer.vertex(beacon_id_offset + low_b[i + 1]);
//
//            Eigen::Matrix<double, 1, 1> info;
//            info(0, 0) = 0.00010;
//
//            beacon_high[low_b[i]] = 3.5;
//            e->setMeasurement(3.5);
////                e->setRobustKernel(robustKernel);
//            globalOptimizer.addEdge(e);
//        }
//
//        for (int i(0); i < high_b.size() - 1; ++i) {
//            auto *e = new Z0Edge();
//            e->vertices()[0] = globalOptimizer.vertex(beacon_id_offset + high_b[i]);
//            e->vertices()[1] = globalOptimizer.vertex(beacon_id_offset + high_b[i + 1]);
//
//            Eigen::Matrix<double, 1, 1> info;
//            info(0, 0) = 0.00010;
//
//            beacon_high[high_b[i]] = -1.5;
//            e->setMeasurement(-1.5);
////                e->setRobustKernel(robustKernel);
//            globalOptimizer.addEdge(e);
//        }
//
//
//    }

//    if(uwb_raw.cols()==6)
//    {
//        for(int i(0);i<6;++i)
//        {
//             auto *e = new Z0Edge();
//            e->vertices()[0] = globalOptimizer.vertex(beacon_id_offset + i);
//            e->vertices()[1] = globalOptimizer.vertex(beacon_id_offset + i + 1);
//
//            Eigen::Matrix<double, 1, 1> info;
//            info(0, 0) = 10.0;
//
//            e->setMeasurement(0.45);
//            e->setRobustKernel(robustKernel);
//            globalOptimizer.addEdge(e);
//        }
//    }



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
//        v->setEstimateData(latest_transform.inverse()*this_transform);
        v->setEstimate(this_transform);
//        v->setFixed(true);
//        v->setFixed(false);
//        if(index==0)
//        {
//            v->setFixed(true);
//        }

        globalOptimizer.addVertex(v);


        // Add z = 0 Edge
//        auto *edge_z0=new Z0Edge();



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
//            if (with_high) {
//                edge_zo->setMeasurement(v_high(index, 0));
//                if (v_high(index, 0) > -1.0) {
//                    globalOptimizer.addEdge(edge_zo);
//                }
//            }
//            edge_zo->setMeasurement(0.0);

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





//            std::cout << t_theta << std::endl;


            globalOptimizer.addEdge(edge_se3);

        }

        latest_transform = this_transform;
    }


    /// ADD High Edge

    /**
     * TODO: Redifined a new version of z constraint for high.
     * NOTE:  now the zero edge added in the process above.
     */





    /// ADD Range Edge

    int zupt_index(0);

    int uwb_index(0);


    double last_time = 0.0;
    double time_intervel = -10.0;

    std::ofstream range_file("./ResultData/range_file.txt");
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
//        zupt_index = 0;
        uwb_index = 0;

        while (true) {

            if (uwb_index > uwb_raw.rows() - 1) {
                break;

            }


            if (std::fabs(uwb_raw(uwb_index) - zupt_time) < 1.0) {
                if (zupt_time - last_time > time_intervel) {
                    last_time = zupt_time;

                    for (int bi(0); bi < uwb_raw.cols() - 1; ++bi) {
                        if (uwb_raw(uwb_index, bi + 1) > 0 && uwb_raw(uwb_index, bi + 1) < 10.0) {
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


                            dist_edge->setRobustKernel(robustKernel);

                            if (with_high) {
//                                if (fabs(v_high(zupt_index) - beacon_high[bi]) < 1.2) {
//
//                                    globalOptimizer.addEdge(dist_edge);
//                                }
                            } else {
                                globalOptimizer.addEdge(dist_edge);
                            }


                        }
                    }
                }

            }

            uwb_index++;
        }

        // Add edge after search all range:


//            range_file
        range_file << current_range(0);
        for (int tmp_k(1); tmp_k < current_range.rows(); ++tmp_k) {
            range_file << "," << current_range(tmp_k);
        }
        range_file << std::endl;

        zupt_index++;



    }





    /// Initial graph and optimize
    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);

    if (max_iterators > 0) {
        globalOptimizer.optimize(max_iterators);
    }
    globalOptimizer.optimize(6000);


    /**
     * output and Plot result
     */



    std::ofstream imu("./ResultData/imu.txt");
    std::ofstream uwb_tmp("./ResultData/uwb_tmp.txt");
    std::ofstream uwb("./ResultData/uwb.txt");
    std::ofstream axis_file("./ResultData/axis.txt");
    std::vector<double> gx, gy, gz;
    for (int i(0); i < zupt_res.rows(); ++i) {
        double data[10] = {0};
        globalOptimizer.vertex(i)->getEstimateData(data);
        gx.push_back(data[0]);
        gy.push_back(data[1]);
        gz.push_back(data[2]);
        imu << data[0] << " " << data[1] << " " << data[2] << std::endl;
        // pose
//        axis_file << data[0] << "," << data[1] << "," << data[2] ;//<< ",";//<< ",0,0,1,0,1,0,1,0,0" << std::endl;

        //axis

        Sophus::SO3 so3_rotation(data[3], data[4], data[5]);
        Sophus::SE3 se3_transform(so3_rotation,
                                  Eigen::Vector3d(data[0], data[1], data[2]));

//        Eigen::Matrix3d rotation_matrix;
//        rotation_matrix= so3_rotation.matrix();

        Eigen::Matrix4d translate_matrix;
//        se3_transform = se3_transform.inverse();
        translate_matrix = se3_transform.matrix();
        Eigen::Vector4d last_vec;

        Eigen::Vector4d tmp_vec(0, 0, 0, 1);
        tmp_vec = translate_matrix * tmp_vec;
        axis_file << tmp_vec(0) << "," << tmp_vec(1) << "," << tmp_vec(2);

        for (int i(0); i < 3; ++i) {
            Eigen::Vector4d ori_vec(0, 0, 0, 1.0);
            ori_vec(i) = -1.0;

            ori_vec = translate_matrix * ori_vec;

            for (int j(0); j < 3; ++j) {
                axis_file << "," << ori_vec(j) - tmp_vec(j);
            }

//            if(i>0)
//            {
//                if(std::abs(double(last_vec.transpose() * ori_vec))>1e-3)
//                {
//                    std::cout << "error in inner product" << std::endl;
//                }else{
//                    std::cout << "axis ok" << std::endl;
//                }
//            }
//            last_vec = ori_vec;

        }
        axis_file << std::endl;


    }

    std::vector<double> bx, by, bz;
    for (int i(0); i < uwb_raw.cols() - 1; ++i) {
        double data[10] = {0};
        globalOptimizer.vertex(i + beacon_id_offset)->getEstimateData(data);
        bx.push_back(data[0]);
        by.push_back(data[1]);
        bz.push_back(data[2]);
        uwb << data[0] << " " << data[1] << " " << data[2] << std::endl;
        uwb_tmp << data[0] << "," << data[1] << "," << data[2] << std::endl;
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

    int first_i = 0;
    int last_i = gx.size() - 1;

    std::cout << "distance between first and last: " <<
              std::sqrt(
                      std::pow(gx[first_i] - gx[last_i], 2.0) +
                      std::pow(gy[first_i] - gy[last_i], 2.0) +
                      std::pow(gz[first_i] - gz[last_i], 2.0)
              ) << std::endl;

    plt::plot(gx, gy, "b-*");
    plt::plot(bx, by, "r*");
    plt::title("para:" + std::to_string(offset_cov) + ":"
               + std::to_string(rotation_cov) + ":" + std::to_string(range_cov) + ":"
               + std::to_string(valid_range) + ":" + std::to_string(range_sigma) +
               ":" + std::to_string(z0_info));

//    plt::save(std::to_string(TimeStamp::now())
//
//              + "test.jpg");
    plt::show();


}
