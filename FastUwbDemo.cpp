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




Eigen::Isometry3d tq2Transform(Eigen::Vector3d offset,
                               Eigen::Quaterniond q){
    Eigen::Isometry3d T;
    T.setIdentity();
    T.rotate(q.toRotationMatrix());
    T(0,3) = offset(0);
    T(1,3) = offset(1);
    T(2,3) = offset(2);
    return T;
}


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


    double uwb_err_threshold = 0.5;

    int delay_times = 5;

    int out_delay_times = 2;

    int data_num = 5;


    if (argc == 14)
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

        uwb_err_threshold = std::stod(argv[10]);

        delay_times = std::stoi(argv[11]);

        out_delay_times = std::stoi(argv[12]);

        data_num = std::stoi(argv[13]);
    }




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
    dir_name  = "/home/steve/Data/FastUwbDemo/1/";
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

    CSVReader ImuDataReader(dir_name + "sim_imu.csv"),
            ZuptReader(dir_name + "sim_zupt.csv");
//            UwbResultReader(dir_name + "UwbResult.data.csv");

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
//    std::vector<double> spx, spy;
//    for (int i(0); i < UwbResultReader.rows_; ++i) {
//        spx.push_back(double(*UwbResultReader.GetMatrix()(i, 0)));
//        spy.push_back(double(*UwbResultReader.GetMatrix()(i, 1)));
//    }





    /**
     * MyEkf
     */
    double imu_start_time(TimeStamp::now());

    std::vector<double> mx, my;

    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.0,0.0,0.0);//irx[0], iry[0], 0.0);
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

//    CSVReader BeaconsetReader(dir_name + "beaconset.data.csv");
    CSVReader UwbdataReader(dir_name + "uwb_result.csv");
//    CSVReader UwbValidReader(dir_name+"UwbValid.data.csv");

    Eigen::MatrixXd beaconset, UwbData,UwbValid;


//    beaconset.resize(BeaconsetReader.GetMatrix().GetRows(),
//                     BeaconsetReader.GetMatrix().GetCols());
//    for (int i(0); i < beaconset.rows(); ++i) {
//        for (int j(0); j < beaconset.cols(); ++j) {
//            beaconset(i, j) = *BeaconsetReader.GetMatrix()(i, j);
//        }
//    }

    UwbData.resize(UwbdataReader.GetMatrix().GetRows(), UwbdataReader.GetMatrix().GetCols());
    for (int i(0); i < UwbData.rows(); ++i) {
        for (int j(0); j < UwbData.cols(); ++j) {
            UwbData(i, j) = *(UwbdataReader.GetMatrix()(i, j));
        }
    }

//    UwbValid.resize(UwbValidReader.GetMatrix().GetRows(),
//                    UwbValidReader.GetMatrix().GetCols());

//    for(int i(0);i<UwbValid.rows();++i)
//    {
//        for(int j(0);j<UwbValid.cols();++j)
//        {
//            UwbValid(i,j) = *(UwbValidReader.GetMatrix()(i,j));
//        }
//    }


    std::vector<std::vector<double>> range_vec;


    /**
     * Graph optimizer build
     */

    //// for output data
    std::vector<int> vertex_index;


    std::vector <Eigen::Isometry3d> edge_vector;

    std::vector<double> onx, ony;


    std::ofstream out_v_before("./ResultData/" + std::to_string(data_num)+ "out_v_before.txt");
    std::ofstream out_v_after("./ResultData/" + std::to_string(data_num)+ "out_v_after.txt");


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
//    for(int i(0);i<beaconset.rows();++i)
//    {
//        beaconset(i,2) = 0.0;
//    }



    /// 1. add beacon vertex
    for(int i(0);i<beaconset.rows();++i)
    {
        auto *v = new  g2o::VertexSE3();
        double p[6] = {0};
        for(int j(0);j<3;++j)
        {
//            p[j] = beaconset(i, j);
            p[j] = 0.0;
        }
//        std::cout << " beacon " << i << " : " << p[0]<< " " <<p[1]<< " "<<p[2] << std::endl;
        v->setEstimateData(p);
//        if(0==i||1==i)
        v->setFixed(false);
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
                bool not_nan = true;
                for(int i(0);i<the_transform.rows();++i)
                {
                    for(int j(0);j<the_transform.cols();++j)
                    {
                        if(std::isnan(the_transform(i,j)))
                        {
                            not_nan = false;
                        }
                    }
                }

                if(!not_nan)
                {
//                    continue;
                    the_transform = latest_transform;

                }


//                for(int i(0);i<)

                vertex_index.push_back(imu_index);


                /// generate better estimate

                Eigen::Isometry3d before_state = Eigen::Isometry3d::Identity();
                if (imu_index > 0) {
                    double before_data[10] = {0};
                    globalOptimizer.vertex(trace_id - 1)->getEstimateData(before_data);

                    Eigen::Vector3d offset(before_data[0], before_data[1], before_data[2]);

                    Eigen::Quaterniond qq(before_data[6], before_data[3], before_data[4], before_data[5]);

                    before_state = tq2Transform(offset, qq);

                }

                /// add vertex
                auto *v = new g2o::VertexSE3();
                v->setId(trace_id);
                v->setEstimate(before_state * latest_transform.inverse() * the_transform);

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
                std::cout << "current delta theta : " << delta_ori << std::endl;
//                std::cout << trace_id << " " << delta_ori << "   " << is_corner
//                          << is_corner << is_corner << is_corner << std::endl;


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

                    edge_vector.push_back(latest_transform.inverse()*the_transform);


//                    double tmp_data[10]={0};
//                    edge_se3->getMeasurementData(tmp_data);
//                    for(int ti(0);ti<7;++ti)
//                    {
//                        out_v_before<< tmp_data[ti] << " ";
//                    }
//                    out_v_before << std::endl;

                    out_v_before<< delta_ori/180.0*M_PI <<std::endl;

                    globalOptimizer.addEdge(edge_se3);


                }

                /// add range edge

                //  get measurement
                if (std::abs(UwbData(uwb_index, 0) - ImuData(imu_index, 0)) < 3.0 ) {

                    Eigen::VectorXd uwb_measure;

                    uwb_measure.resize(UwbData.cols() - 1);
                    uwb_measure.setZero();


                    uwb_measure = UwbData.block(uwb_index,1,1,uwb_measure.rows()).transpose();

//                    if (uwb_index == 0 || uwb_index > UwbData.rows() - 3) {
//                        uwb_measure = UwbData.block(uwb_index, 1, 1, uwb_measure.rows()).transpose();
//                    } else {
//                        uwb_measure += UwbData.block(uwb_index, 1, 1, uwb_measure.rows()).transpose();
//                        uwb_measure += UwbData.block(uwb_index + 1, 1, 1, uwb_measure.rows()).transpose();
//
//                        uwb_measure /= 2.0;
//                    }

                    // build and add edge

                    for (int bi(0); bi < uwb_measure.rows(); ++bi) {
                        if( bi == 10

                                )
                        {
                            break;
                        }
                        auto *dist_edge = new DistanceEdge();
                        dist_edge->vertices()[0] = globalOptimizer.vertex(beacon_id + bi);
                        dist_edge->vertices()[1] = globalOptimizer.vertex(trace_id);

                        dist_edge->setMeasurement((uwb_measure(bi)));

                        Eigen::Matrix<double, 1, 1> information;
                        if(uwb_err_threshold>1.0)
                        {
                            information(0, 0) = 1/UwbValid(uwb_index,0)+1.0;
//                                          distance_sigma = UwbValid(uwb_index,0);
                        }
                        else{
                            information(0,0) = distance_info;
                        }

                        dist_edge->setInformation(information);
                        dist_edge->setSigma(distance_sigma);

                        if(uwb_measure(bi)>0)
                        {

                            globalOptimizer.addEdge(dist_edge);
                        }
                    }

                    /// try online optimize

                }

                /// updata transform matrix
                latest_transform = the_transform;

                /// increase trace id
                trace_id ++;

            }

            imu_index++;
        }
    }



    /// add extra to out data
//


    std::cout << "sum time :" << TimeStamp::now() - graph_start_time << std::endl;


    /// 3. Solve the problem
    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);
    globalOptimizer.optimize(10000);
//    globalOptimizer.optimize(max_optimize_times);


    /// 4. plot result
    std::ofstream out_result("./ResultData/" + std::to_string(data_num)+ "test.txt");
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
    std::ofstream out_err("./ResultData/" + std::to_string(data_num)+ "err.txt");
    std::vector<double> error_vec;
    out_err.precision(10);
    double err_sum(0.0);
    double online_err_sum(0.0);
    std::vector<double> rix,riy;

    std::ofstream("./ResultData/"+std::to_string(data_num)+"real_pose_ir.txt");


    //// PLOT BEACONSET
    std::vector<double> bx, by, bz;

    for (int i(0); i < 4; ++i) {
        double data[10] = {0};

        globalOptimizer.vertex(beacon_id + i)->getEstimateData(data);

        bx.push_back(data[0]);
        by.push_back(data[1]);
        bz.push_back(data[2]);
    }


    plt::plot(bx, by, "r*");

    plt::plot(gx,gy,"r-+");
    plt::grid(true);


    std::ofstream out_para_res("./ResultData/para_err.txt",std::ios::app);

    out_para_res.precision(10);

    out_para_res << "first_info:" << first_info
                 << "second_info:" << second_info
                 << "distance_info:" << distance_info
                 << "distance_sigma:" << distance_sigma
                 << "z_offset:" << z_offset
                 << "turn_threshold:" << turn_threshold
                 << "corner_ratio:" << corner_ratio
                 << "max_iterate:" << max_optimize_times
                 << "time_offset:" << time_offset
                 << "delay_times:" << delay_times
                 << "err:" <<err_sum/double(error_vec.size())
                 << std::endl;


    plt::title("erro is :"+std::to_string(err_sum/double(error_vec.size())));
//
    plt::show();



//    plt::show();


}