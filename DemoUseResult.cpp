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
                               Eigen::Quaterniond q) {
    Eigen::Isometry3d T;
    T.setIdentity();
    T.rotate(q.toRotationMatrix());
    T(0, 3) = offset(0);
    T(1, 3) = offset(1);
    T(2, 3) = offset(2);
    return T;
}


int main() {

    /**
     * Global value
     */

    std::string dir_name = "/home/steve/Data/FastUwbDemo/1/";


    double offset_cov(0.1),rotation_cov(0.1),range_cov(1.0);


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
    for(int i(0);i<)







    /**
     * output and Plot result
     */
}
