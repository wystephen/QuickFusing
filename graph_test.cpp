//
// Created by steve on 17-4-26.
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
#include<Eigen/Dense>
#include <Eigen/Geometry>

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

int main(int argc, char *argv[]) {
// 3 300 0.2 5.0 10000 0.2 5.0 5
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


    double distance_info = 100.0;
    double distance_sigma = 2.0;


    double z_offset = 1.90-1.12;

    double turn_threshold  = 1.0;
    double corner_ratio = 10.0;

    int max_optimize_times = 4000;

    double time_offset = 0.0;




    if(argc == 10)
    {
        std::cout << "set para meter s" << std::endl;
        first_info = std::stod(argv[1]);
        second_info=std::stod(argv[2]);

        distance_info=std::stod(argv[3]);
        distance_sigma=std::stod(argv[4]);

        z_offset = std::stod(argv[5]);

        turn_threshold = std::stod(argv[6]);
        corner_ratio = std::stod(argv[7]);

        max_optimize_times = std::stoi(argv[8]);

        time_offset = std::stod(argv[9]);
    }


    int data_num = 5;

    std::string out_dir_name = "./";
    std::string dir_name = "/home/steve/locate/";

    /**
     * Parameters:
     * ## pf only uwb
     * 1. only uwb methond 0-with x y a w 3- only x y
     * 2. particle_num
     * 3. transpose sigma
     * 4. evaluation sigma
     *
     * ## pf uwb and imu
     *
     * 1. particle num
     * 2. transpose sigma
     * 3. evaluation sigma
     *
     * ## which data
     * 1. data_number 1-5
     *
     * ## dir_name
     */
//    if (argc == 10 || argc == 9) {
//        only_method = atoi(argv[1]);
//        only_particle_num = atoi(argv[2]);
//        only_transpose_sigma = atof(argv[3]);
//        only_eval_sigma = atof(argv[4]);
//
//        fus_particle_num = atoi(argv[5]);
//        fus_transpose_sigma = atof(argv[6]);
//        fus_eval_sigma = atof(argv[7]);
//
//        data_num = atoi(argv[8]);
//        if (argc == 10) {
//            out_dir_name = std::string(argv[9]);
//        } else {
//            out_dir_name = dir_name;
//        }
//    }

    dir_name = dir_name + std::to_string(data_num);
    if (argc != 10) {
        out_dir_name = dir_name;
    }

    std::cout.precision(20); //

    double first_t(TimeStamp::now());

    /**
     * Build a global optimizer
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


    /**
     * Vertex id counter;
     */

    int trace_id(0);
    int beacon_id(100000);



    /*
     * Load Imu data.
     */


    // Load real pose
    CSVReader ImuRealPose(dir_name + "ImuRealPose.data.csv"),
            UwbRealPose(dir_name + "ImuRealPose.data.csv");

    std::vector<double> irx, iry, urx, ury;
    auto ImuRP(ImuRealPose.GetMatrix());
    auto UwbRP(UwbRealPose.GetMatrix());

    for (int i(0); i < ImuRP.GetRows(); ++i) {
//        std::cout << *(ImuRP(i, 0)) << ":" << *ImuRP(i, 1) << std::endl;
        irx.push_back(*(ImuRP(i, 0)));
        iry.push_back(*(ImuRP(i, 1)));
    }
    for (int i(0); i < UwbRP.GetRows(); ++i) {
        urx.push_back(*(UwbRP(i, 0)));
        ury.push_back(*(UwbRP(i, 1)));
    }


    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"),
            ZuptReader(dir_name + "Zupt.data.csv"),
            UwbResultReader(dir_name + "UwbResult.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    //////////ADD NOISE TO SOURCE DATA
    std::default_random_engine ee;
    std::uniform_real_distribution<double> u(-0.15, 0.15);
    std::normal_distribution<> n(0.0, 0.2);

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = int(*ZuptTmp(i, 0));
//        std::cout << i << " :  " << *ZuptTmp(i,0) << std::endl;
    }

    /**
     * sim uwb result
     */
    std::vector<double> spx, spy;
    for (int i(0); i < UwbResultReader.rows_; ++i) {
        spx.push_back(double(*UwbResultReader.GetMatrix()(i, 0)));
        spy.push_back(double(*UwbResultReader.GetMatrix()(i, 1)));
    }





    /**
     * MyEkf
     */
    double imu_start_time(TimeStamp::now());

    std::vector<double> mx, my;

    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(irx[0], iry[0], 0.0);
    init_para.init_heading1_ = M_PI / 2.0;

    init_para.Ts_ = 1.0 / 128.0;

    MyEkf myekf(init_para);

    myekf.InitNavEq(ImuData.block(0, 1, 20, 6));



    std::vector<double> wi, w1, w2, w3;

    for (int i(0); i < ImuData.rows(); ++i) {
        Eigen::VectorXd vec = myekf.GetPosition(
                ImuData.block(i, 1, 1, 6).transpose(),
                Zupt(i, 0));


        wi.push_back(double(i));


        mx.push_back(double(vec(0)));
        my.push_back(double(vec(1)));
    }
    std::cout << "IMU waste time :" << TimeStamp::now() - imu_start_time << std::endl;
//    std::cout << " zupt sum : " << Zupt.sum() << " size : " << Zupt.size()
//              << std::endl;


    /**
    * Load uwb data.
    */

    CSVReader BeaconsetReader(dir_name + "beaconset.data.csv");
    CSVReader UwbdataReader(dir_name + "UwbData.data.csv");

    Eigen::MatrixXd beaconset, UwbData;


    beaconset.resize(BeaconsetReader.GetMatrix().GetRows(),
                     BeaconsetReader.GetMatrix().GetCols());
    for (int i(0); i < beaconset.rows(); ++i) {
        for (int j(0); j < beaconset.cols(); ++j) {
            beaconset(i, j) = *BeaconsetReader.GetMatrix()(i, j);
        }
    }

    UwbData.resize(UwbdataReader.GetMatrix().GetRows(), UwbdataReader.GetMatrix().GetCols());
    for (int i(0); i < UwbData.rows(); ++i) {
        for (int j(0); j < UwbData.cols(); ++j) {
            UwbData(i, j) = *(UwbdataReader.GetMatrix()(i, j));
        }
    }
    std::vector<std::vector<double>> range_vec;


    /**
     * Graph optimizer build
     */

    //// for output data
    std::vector<int> vertex_index;


    ///////// try to add time offset to uwb data
    for(int i(0);i<UwbData.rows();++i)
    {
        UwbData(i,0) += time_offset;
    }

    /// 0. prepare some class
    double graph_start_time(TimeStamp::now());
    int uwb_index(0);
    int imu_index(0);

    MyEkf gekf(init_para);
    gekf.InitNavEq(ImuData.block(0,1,20,6));

    Eigen::Isometry3d latest_transform = (Eigen::Isometry3d::Identity());

    double latest_theta = 0.0;


    /**
     * Spacial preprocess !!!!
     * set z of beaconset to zero
     */
    for(int i(0);i<beaconset.rows();++i)
    {
        beaconset(i,2) = 0.0;
    }



    /// 1. add beacon vertex
    for(int i(0);i<beaconset.rows();++i)
    {
        auto *v = new  g2o::VertexSE3();
        double p[6] = {0};
        for(int j(0);j<3;++j)
        {
            p[j] = beaconset(i,j);
        }
        std::cout << " beacon " << i << " : " << p[0]<< " " <<p[1]<< " "<<p[2] << std::endl;
        v->setEstimateData(p);
        v->setFixed(true);
        v->setId(beacon_id+i);
        globalOptimizer.addVertex(v);
    }


    /// 2. Loop (add zero-velocity vertex and range constraint)

    while (true) {
        if (uwb_index >= UwbData.rows() || imu_index >= ImuData.rows()) {

            break;
        }

        if (UwbData(uwb_index, 0) < ImuData(imu_index, 0)) {
            // Do nothing
            uwb_index++;
        } else {
            /*
             * update imu data
             */
            gekf.GetPosition(ImuData.block(imu_index, 1, 1, 6).transpose(),
                               Zupt(imu_index, 0));

            /**
             *  Add a vertex to graph when first time of zero-velocity detected.
             *  (or first times of the imu data)
             */


            if (imu_index == 0 || (Zupt(imu_index, 0) > 0.5 && Zupt(imu_index - 1, 0) < 0.5)) {
                auto the_transform = gekf.getTransformation();

                vertex_index.push_back(imu_index);

                /// add vertex
                auto *v = new g2o::VertexSE3();
                v->setId(trace_id);
                v->setEstimate(the_transform);

                globalOptimizer.addVertex(v);

                /// get delta theta
                double the_theta(gekf.getOriente());
                double delta_ori = the_theta - latest_theta;

                if (delta_ori > M_PI) {
                    delta_ori -= (2 * M_PI);
                } else if (delta_ori < -M_PI) {
                    delta_ori += (2.0 * M_PI);
                }
                if (isnan(delta_ori)) {
                    delta_ori = 0.0;
                }
//                gekf.getDeltaOrientation();
                bool is_corner(false);
//                if(std::abs(the_theta-))
                if (std::abs(delta_ori) > turn_threshold) {
                    is_corner = true;
                }

                latest_theta = the_theta;


                std::cout << trace_id << " " << delta_ori << "   " << is_corner
                          << is_corner << is_corner << is_corner << std::endl;


                ///add transform edge

                if (trace_id > 0) {
                    auto *edge_se3 = new g2o::EdgeSE3();

                    edge_se3->vertices()[0] = globalOptimizer.vertex(trace_id - 1);
                    edge_se3->vertices()[1] = globalOptimizer.vertex(trace_id);

                    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


                    information(0, 0) = information(1, 1) = information(2, 2) = first_info;
                    information(3, 3) = information(4, 4) = information(5, 5) = second_info;

                    if (is_corner) {
                        information(0, 0) = information(1, 1) = information(2, 2) = first_info / corner_ratio;
                        information(3, 3) = information(4, 4) = information(5, 5) = second_info / corner_ratio;
                    }

                    edge_se3->setInformation(information);

                    edge_se3->setMeasurement(latest_transform.inverse() * the_transform);

                    globalOptimizer.addEdge(edge_se3);


                }

                /// add range edge

                //  get measurement
                if (std::abs(UwbData(uwb_index, 0) - ImuData(imu_index, 0)) < 1.0) {

                    Eigen::VectorXd uwb_measure;

                    uwb_measure.resize(UwbData.cols() - 1);
                    uwb_measure.setZero();

//                std::cout << "uwb measurement ros and cols :" << uwb_measure.rows() <<  " " << uwb_measure.cols() << std::endl;

                    if (uwb_index == 0 || uwb_index > UwbData.rows() - 3) {
                        uwb_measure = UwbData.block(uwb_index, 1, 1, uwb_measure.rows()).transpose();
                    } else {
                        uwb_measure += UwbData.block(uwb_index, 1, 1, uwb_measure.rows()).transpose();
                        uwb_measure += UwbData.block(uwb_index + 1, 1, 1, uwb_measure.rows()).transpose();

                        uwb_measure /= 2.0;
                    }

                    // build and add edge

                    for (int bi(0); bi < uwb_measure.rows(); ++bi) {
                        auto *dist_edge = new DistanceEdge();
                        dist_edge->vertices()[0] = globalOptimizer.vertex(beacon_id + bi);
                        dist_edge->vertices()[1] = globalOptimizer.vertex(trace_id);

                        dist_edge->setMeasurement(std::sqrt(uwb_measure(bi) * uwb_measure(bi) - z_offset * z_offset));

                        Eigen::Matrix<double, 1, 1> information;
                        information(0, 0) = distance_info;

                        dist_edge->setInformation(information);
                        dist_edge->setSigma(distance_sigma);
                        dist_edge->setRobustKernel(new g2o::RobustKernelHuber());

                        globalOptimizer.addEdge(dist_edge);
                    }

                }

                /// updata transform matrix
                latest_transform = the_transform;

                /// increase trace id
                trace_id ++;
            }

            imu_index++;
        }
    }


    /// 3. Solve the problem
    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);
    globalOptimizer.optimize(max_optimize_times);


    /// 4. plot result
    std::ofstream out_result("./ResultData/test.txt");
    std::vector<double> gx,gy;
    for(int vid(0);vid<trace_id;++vid)
    {
        double data[10] = {0};
        globalOptimizer.vertex(vid)->getEstimateData(data);
        gx.push_back(data[0]);
        gy.push_back(data[1]);
        out_result << data[0] << " " << data[1] <<" "<< data[2]<< std::endl;
    }
    out_result.close();

    //// 5.compute error
    std::ofstream out_err("./ResultData/err.txt");
    std::vector<double> error_vec;
    out_err.precision(10);
    double err_sum(0.0);
    for(int i(0);i<vertex_index.size();++i)
    {
        int index = vertex_index.at(i);

        error_vec.push_back(std::sqrt(std::pow(gx[i]-irx[index],2.0)+
        std::pow(gy[i]-iry[index],2.0)));
        out_err<< error_vec.at(i) << std::endl;
        err_sum+=std::sqrt(std::pow(gx[i]-irx[index],2.0)+
                           std::pow(gy[i]-iry[index],2.0));
//        plt::plot()
        std::vector<double> tmpx,tmpy;
        tmpx.push_back(gx[i]);
        tmpx.push_back(irx[index]);
        tmpy.push_back(gy[i]);
        tmpy.push_back(iry[index]);

        plt::plot(tmpx,tmpy,"y-");
    }
    std::cout << "average error is :" << err_sum/double(error_vec.size()) << std::endl;

//    std::cout << "average error is :" << double(std::accumulate(error_vec.begin(),
//    error_vec.end(),0))/double(error_vec.size()) << std::endl;



    plt::plot(gx,gy,"r-+");
    plt::plot(irx,iry,"b-");
    plt::grid(true);


    std::ofstream out_para_res("./ResultData/para_err.txt",std::ios::app);

    out_para_res.precision(10);

    out_para_res << "first_info:"<<first_info
                 << "second_info:"<<second_info
                 << "distance_info:"<<distance_info
                 <<"distance_sigma:"<<distance_sigma
                 <<"z_offset:"<<z_offset
                 <<"turn_threshold:"<<turn_threshold
                 <<"corner_ratio:"<<corner_ratio
                 <<"max_iterate:"<<max_optimize_times
                 <<"time_offset:"<<time_offset
            <<"err:"<<err_sum/double(error_vec.size())
            <<std::endl;


//    plt::title("")
//
    plt::show();



//    plt::show();


}
