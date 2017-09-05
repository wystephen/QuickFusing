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

bool GLRT_Detector(Eigen::MatrixXd u,
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

        Tmatrix += u.block(3, i, 3, 1).transpose() * u.block(3, i, 3, 1) / para_.sigma_g_ +
                   tmp.transpose() * tmp / para_.sigma_a_;


    }

    if (Tmatrix.size() != 1) {
        MYERROR("Tmatrxi size is not equal to 1")
    }

    T = Tmatrix(0, 0);

    T = T / para_.ZeroDetectorWindowSize_;
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
    std::string dir_name = "/home/steve/Data/XIMU&UWB/1/";

    // Load data
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


    SettingPara initial_para(true);

    initial_para.Ts_ = 1.0f / 128.0f;

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata.block(0, 0, 20, 6));

    for (int index(0); index < imudata.rows(); ++index) {
        std::cout << "index:" << index << std::endl;
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {
            if (GLRT_Detector(imudata.block(index - initial_para.ZeroDetectorWindowSize_,
                                            0, initial_para.ZeroDetectorWindowSize_, 6).transpose().eval(),
                              initial_para)) {
                zupt_flag = 1.0;
            }
        }
        auto tx = myekf.GetPosition(imudata.block(index, 0, 1, 6).transpose(), zupt_flag);

        ix.push_back(tx(0));
        iy.push_back(tx(1));
    }


    plt::plot(gx, gy, "r-+");
    plt::plot(ix, iy, "b-+");
    plt::title("show");
    plt::show();


}