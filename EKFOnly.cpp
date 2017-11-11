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


int main(int argc, char *argv[]) {
    std::string dir_name = "/home/steve/Data/II/20/";

    /// Global parameters
    double first_info(0.01), second_info(2.5), ori_info(1);
    double gravity_info(9.8);
    double zero_z_info(-10.1);

    if (argc >= 4) {
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);
        ori_info = std::stod(argv[3]);
    }

    if (argc >= 5) {
        gravity_info = std::stod(argv[4]);
    }

    if (argc >= 6) {
        zero_z_info = std::stod(argv[5]);
    }

    double turn_threshold = 10.0 / 180.0 * M_PI;
    double corner_ratio = 10.0;

    //// Load data
    CppExtent::CSVReader imu_data_reader(dir_name + "imu2.txt");

    Eigen::MatrixXd imudata;
    imudata.resize(imu_data_reader.GetMatrix().GetRows(),
                   imu_data_reader.GetMatrix().GetCols());
    imudata.setZero();
    auto imu_data_tmp_matrix = imu_data_reader.GetMatrix();

//    central =
//
//            5   105   283
//
//
//    Scale_axis =
//
//            238   263   269
    Eigen::Vector3d central(5, 105, 283);//imu
    Eigen::Vector3d scale(238, 263, 269);//imu
//    Eigen::Vector3d central(-63, -108, 151);//imu2
//    Eigen::Vector3d scale(241, 264, 283);//imu2


    for (int i(0); i < imudata.rows(); ++i) {
        for (int j(0); j < imudata.cols(); ++j) {
            imudata(i, j) = *(imu_data_tmp_matrix(i, j));
            if (0 < j && j < 4) {
                imudata(i, j) *= 9.8;
            } else if (4 <= j && j < 7) {
                imudata(i, j) *= (M_PI / 180.0f);
            } else if (7 <= j && j < 10) {
                imudata(i, j) = (imudata(i, j) - central(j - 7)) / scale(j - 7);
            }
        }
    }
    std::cout << "imu data size: " << imudata.rows() << "x"
              << imudata.cols() << std::endl;

    std::cout << "source imu data :\n" << imudata.block(0, 0, 10, 10) << std::endl;

    std::cout << "source imu data last 10 lines:\n" << imudata.block(imudata.rows() - 11, 0, 10, 10) << std::endl;


    std::vector<double> ix, iy, iz; //ix iy
    std::vector<double> gx, gy;// graph x

    std::vector<double> ori_1, ori_2, ori_3;




    /**
     * Initial ZUPT parameters
     */
    SettingPara initial_para(true);
    initial_para.init_pos1_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    initial_para.init_heading1_ = 0.0;
    initial_para.Ts_ = 1.0f / 200.0f;


    initial_para.sigma_a_ = 0.01*ori_info;
    initial_para.sigma_g_ = 0.01*ori_info / 180.0 * M_PI;


    initial_para.gravity_ = gravity_info;
//    initial_para.sigma_a_ /= 3.0;
//    initial_para.sigma_g_ /= 3.0;
    initial_para.sigma_acc_ = Eigen::Vector3d(1,1,1)*first_info;
    initial_para.sigma_gyro_ = Eigen::Vector3d(1,1,1)/180.0*M_PI*second_info;
//    initial_para.sigma_acc_ *= 6.0;
//    initial_para.sigma_gyro_ *= 6.0;

    initial_para.ZeroDetectorWindowSize_ = 5;// Time windows size fo zupt detector

    MyEkf myekf(initial_para);
    myekf.InitNavEq(imudata.block(0, 1, 20, 6));

    double last_zupt_flag = 0.0;

    int trace_id(0);
    Eigen::Isometry3d last_transform = Eigen::Isometry3d::Identity();
    double last_theta = 0.0;

    int last_optimized_id(0);

    std::vector<ImuKeyPointInfo> key_info_mag;

    for (int index(0); index < imudata.rows(); ++index) {
//        std::cout << "index:" << index << std::endl;
        double zupt_flag = 0.0;
        if (index <= initial_para.ZeroDetectorWindowSize_) {
            zupt_flag = 1.0;
        } else {
//

            if (std::isnan(imudata.block(index - initial_para.ZeroDetectorWindowSize_, 1,
                                         initial_para.ZeroDetectorWindowSize_, 6).sum())) {
                std::cout << " input data of GLRT is nana " << std::endl;
            }
            if (GLRT_Detector(
                    imudata.block(index - initial_para.ZeroDetectorWindowSize_,
                                  1, initial_para.ZeroDetectorWindowSize_, 6).transpose(),
                    initial_para)) {
                zupt_flag = 1.0;
            }
        }
        ///ZUPT GET POSITION
        auto tx = myekf.GetPosition(imudata.block(index, 1, 1, 6).transpose(), zupt_flag);


        last_optimized_id = trace_id;
//

        last_zupt_flag = zupt_flag;
        ix.push_back(tx(0));
        iy.push_back(tx(1));
        iz.push_back(tx(2));
    }

//    std::cout << "after imu data :\n" << imudata.block(0, 0, 10, 10) << std::endl;

//    std::cout << "after imu data last 10 lines:\n" << imudata.block(imudata.rows() - 11, 0, 10, 10) << std::endl;



    ///optimization

    std::ofstream test("./ResultData/test.txt");
    std::ofstream test_imu("./ResultData/text_imu.txt");

    for (int k(0); k < ix.size(); ++k) {
        test_imu << ix[k]
                 << ","
                 << iy[k]
                 << ","
                 << iz[k]
                 << std::endl;
    }


    plt::plot(gx, gy, "r-+");
    plt::plot(ix, iy, "b-");
//    plt::plot(ori_1,"r-+");
//    plt::plot(ori_2,"b-+");
//    plt::plot(ori_3,"g-+");
    plt::title("show");
//    plt::save(std::to_string(first_info)+":"
//    +std::to_string(second_info)+":"
//    +std::to_string(ori_info)+".png");
    plt::show();


}