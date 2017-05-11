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
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

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


int main(int argc,char *argv[]) {

    /**
     * Global value
     */

    std::string dir_name = "/home/steve/Data/FastUwbDemo/2/";


    double offset_cov(0.001),rotation_cov(0.002),range_cov(5.0);
    double time_offset(72.0);//defualt parameters.

    if(argc>=2)
    {
        time_offset = std::stod(argv[1]);
    }

    if(argc==5)
    {
        offset_cov=std::stod(argv[2]);
        rotation_cov=std::stod(argv[3]);
        range_cov=std::stod(argv[4]);

    }



    int trace_id = 0;
    const int beacon_id_offset(100000);


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
     * Load Data from file
     */


    CSVReader UwbRawReader(dir_name+"uwb_result.csv");

    Eigen::MatrixXd uwb_raw(UwbRawReader.GetMatrix().GetRows(),UwbRawReader.GetMatrix().GetCols());

    for(int i(0);i<uwb_raw.rows();++i)
    {
        for(int j(0);j<uwb_raw.cols();++j)
        {
            uwb_raw(i,j) = *(UwbRawReader.GetMatrix()(i,j));
        }
    }


    CSVReader ImuDataReader(dir_name + "sim_imu.csv"),
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

    CSVReader ZuptResultReader(dir_name+"sim_pose.csv");
    CSVReader QuatReader(dir_name+"all_quat.csv");
    CSVReader VertexTime(dir_name+"vertex_time.csv");

    Eigen::MatrixXd zupt_res(ZuptResultReader.GetMatrix().GetRows(),ZuptResultReader.GetMatrix().GetCols());
    Eigen::MatrixXd quat(QuatReader.GetMatrix().GetRows(),QuatReader.GetMatrix().GetCols());
    Eigen::MatrixXd v_time(VertexTime.GetMatrix().GetRows(),VertexTime.GetMatrix().GetCols());

    for(int i(0);i<zupt_res.rows();++i)
    {
        for(int j(0);j<zupt_res.cols();++j)
        {
            zupt_res(i,j)  = *(ZuptResultReader.GetMatrix()(i,j));
        }
    }

    for(int i(0);i<quat.rows();++i)
    {
        for(int j(0);j<quat.cols();++j)
        {
            quat(i,j) = *(QuatReader.GetMatrix()(i,j));
        }
    }

    for(int i(0);i<v_time.rows();++i)
    {
        for(int j(0);j<v_time.cols();++j)
        {
            v_time(i,j) = *(VertexTime.GetMatrix()(i,j));
        }
    }


    /**
     * Build Graph
     */


    /// Add Beacon Vertex
    for(int i(0);i<uwb_raw.cols()-1;++i)
    {
        auto *v = new g2o::VertexSE3();
        double p[6]={0};

        v->setEstimateData(p);
        v->setFixed(false);
        v->setId(beacon_id_offset+i);

        globalOptimizer.addVertex(v);
    }

    ///Add ZUPT and ZUPT Edge
    Eigen::Isometry3d latest_transform=Eigen::Isometry3d::Identity();

    for(int index(0);index<zupt_res.rows();++index)
    {

        Eigen::Quaterniond qt;
        qt.x() = quat(index,0);
        qt.y() = quat(index,1);
        qt.z() = quat(index,2);
        qt.w() = quat(index,3);

        auto this_transform = tq2Transform(Eigen::Vector3d(zupt_res(index,0),
        zupt_res(index,1),
        zupt_res(index,2)),
        qt);
        if(index == 0)
        {
           latest_transform = this_transform;
        }

        /// Add ZUPT Vertex
        auto *v = new g2o::VertexSE3();
        v->setId(index);
//        v->setEstimateData(latest_transform.inverse()*this_transform);
        v->setEstimate(this_transform);
//        v->setFixed(true);
//        v->setFixed(false);

        globalOptimizer.addVertex(v);


        // Add z = 0 Edge
//        auto *edge_z0=new Z0Edge();



        /// Add ZUPT Edge
        if(index>0)
        {
            auto *edge_zo = new Z0Edge();
            edge_zo->vertices()[0] = globalOptimizer.vertex(index-1);
            edge_zo->vertices()[1] = globalOptimizer.vertex(index);

            Eigen::Matrix<double,1,1> info;
            info(0,0) = 100.0;
            edge_zo->setInformation(info);
            edge_zo->setMeasurement(0.0);

            globalOptimizer.addEdge(edge_zo);



            auto *edge_se3 = new g2o::EdgeSE3();

            edge_se3->vertices()[0] = globalOptimizer.vertex(index-1);
            edge_se3->vertices()[1] = globalOptimizer.vertex(index);


            Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();


            information(0, 0) = information(1, 1) = information(2, 2) = 1.0/offset_cov;
            information(3, 3) = information(4, 4) = information(5, 5) = 1.0/rotation_cov;

            edge_se3->setInformation(information);

            edge_se3->setMeasurement(latest_transform.inverse()*this_transform);


            globalOptimizer.addEdge(edge_se3);

        }

        latest_transform = this_transform;
    }



    /// ADD Range Edge

    int zupt_index(0);

    int uwb_index(0);

    while(true)
    {
        if(uwb_index>uwb_raw.rows()-2)
        {
            break;
        }
        double uwb_time = uwb_raw(uwb_index,0)-time_offset;


        zupt_index = 0;


//        int raw_zupt_index = zupt_index;
        while(true)
        {

            if(zupt_index>zupt_res.rows()-2)
            {
                std::cout << "not found right way" << std::endl;
                break;

            }

            if(std::fabs(v_time(zupt_index)-uwb_time)<1.0)
            {
                for(int bi(0);bi<uwb_raw.cols()-1;++bi)
                {
                    if(uwb_raw(uwb_index,bi+1)>0&&uwb_raw(uwb_index,bi+1)<60.0)
                    {
                        double range = uwb_raw(uwb_index,bi+1);
                        int beacon_id = bi+beacon_id_offset;

                        int zupt_id = zupt_index;

                        auto *dist_edge = new DistanceEdge();
                        dist_edge->vertices()[0] = globalOptimizer.vertex(beacon_id);
                        dist_edge->vertices()[1] = globalOptimizer.vertex(zupt_id);

                        Eigen::Matrix<double,1,1> information;

                        information(0,0) = 1/range_cov;

                        dist_edge->setInformation(information);
                        dist_edge->setSigma(10.0);
                        dist_edge->setMeasurement(range);

                        globalOptimizer.addEdge(dist_edge);
                        std::cout << "add distance edge" << std::endl;
                    }
                }
                break;
            }

            zupt_index++;
        }

        uwb_index++;
    }

    globalOptimizer.initializeOptimization();
    globalOptimizer.setVerbose(true);
    globalOptimizer.optimize(10000);






    /**
     * output and Plot result
     */



    std::ofstream imu("./ResultData/imu.txt");
    std::ofstream uwb("./ResultData/uwb.txt");
    std::vector<double> gx,gy,gz;
    for(int i(0);i<zupt_res.rows();++i)
    {
        double data[10]={0};
        globalOptimizer.vertex(i)->getEstimateData(data);
        gx.push_back(data[0]);
        gy.push_back(data[1]);
        gz.push_back(data[2]);
        imu << data[0]<<" "<<data[1]<<" "<<data[2] << std::endl;
    }

    std::vector<double> bx,by,bz;
    for(int i(0);i<uwb_raw.cols()-1;++i)
    {
        double data[10]={0};
        globalOptimizer.vertex(i+beacon_id_offset)->getEstimateData(data);
        bx.push_back(data[0]);
        by.push_back(data[1]);
        bz.push_back(data[2]);
        uwb << data[0] << " "<<data[1] << " " << data[2] << std::endl;
    }


    for(int i(0);i<uwb_raw.cols()-1;++i)
    {
        for(int j(0);j<i;++j)
        {
            std::cout << std::sqrt(std::pow(bx[i]-bx[j],2.0)+
            std::pow(by[i]-by[j],2.0)+std::pow(bz[i]-bz[j],2.0))<< "   ";
        }
        std::cout << std::endl;
    }

    plt::plot(gx,gy,"b-*");
    plt::plot(bx,by,"r*");
    plt::title("offset :"+std::to_string(time_offset));
    plt::save(std::to_string(time_offset)+"test.jpg");
    plt::show();




}
