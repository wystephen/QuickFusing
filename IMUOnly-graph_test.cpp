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
#include "Zero_Detecter.h"
#include<Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.h>

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


int main(int argc, char *argv[]) {
    std::string dir_name = "/home/steve/Data/XIMU&UWB/1/";

    // Load data
    CppExtent::CSVReader imu_data_reader(dir_name+"ImuData.csv");

    Eigen::MatrixXd imudata(imu_data_reader.GetMatrix().GetRows(),
    imu_data_reader.GetMatrix().GetCols());
    auto imu_data_tmp_matrix = imu_data_reader.GetMatrix();

    for(int i(0);i<imudata.rows();++i)
    {
        for(int j(0);j<imudata.cols();++j)
        {
            imudata(i,j) = *(imu_data_tmp_matrix(i,j));
        }
    }
    std::cout << "imu data size: " << imudata.rows() << "x"
              << imudata.cols() << std::endl;


    std::vector<double> ix,iy; //ix iy
    std::vector<double> gx,gy;// graph x


    SettingPara initial_para(true);

    initial_para.Ts_ = 1.0f / 128.0f;




    for(int index(0);index < imudata.rows();++index)
    {
        double zupt_flag = 0.0;
        if(index < initial_para.ZeroDetectorWindowSize_)
        {

            zupt_flag = 1.0;
        }else{
            if(GLRT_Detector(imudata.block(index-initial_para.ZeroDetectorWindowSize_,
            0,initial_para.ZeroDetectorWindowSize_,6),initial_para.sigma_a_,
            initial_para.sigma_g_,initial_para.ZeroDetectorWindowSize_))
            {
                zupt_flag = 1.0;
            }
        }
        auto tx =
    }






    plt::plot(gx,gy,"r-+");
    plt::title("show");
    plt::show();


}