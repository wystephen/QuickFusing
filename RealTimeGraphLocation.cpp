//
// Created by steve on 17-9-12.
//
/// EIGEN
#include <Eigen/Dense>
#include <Eigen/Geometry>

///OWN
#include "CSVReader.h"
#include "MyError.h"
#include "matplotlib_interface.h"
#include "time_stamp.h"

#include "SettingPara.h"
#include "MYEKF.h"

///G2o
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

///G2o Own
#include "OwnEdge/ZoEdge.h"
#include "OwnEdge/ZoEdge.cpp"
#include "OwnEdge/DistanceEdge.h"
#include "OwnEdge/DistanceEdge.cpp"


/// For plot 2d
G2O_USE_TYPE_GROUP(slam3d)


namespace plt = matplotlibcpp;// matplotlib plot


/**
 * From offset and quaternion to Transform matrix(4x40
 * @param offset (x,y,z)
 * @param q (qw,qx,qy,qz)
 * @return  Transform matrix(4x4)
 */
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


int main(int argc, char *argv[])
{
    /// All paramaters for algorithm

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


    double distance_info = 1.0;
    double distance_sigma = 2.0;


    double z_offset = 1.90 - 1.12;

    double turn_threshold = 1.0;
    double corner_ratio = 10.0;

    int max_optimize_times = 4000;

    double time_offset = 0.0;


    double uwb_err_threshold = 0.5;

    int delay_times = 25;

    int out_delay_times = 4;

    int data_num = 5;


    if (argc == 14) {
        std::cout << "set para meter s" << std::endl;
        first_info = std::stod(argv[1]);
        second_info = std::stod(argv[2]);

        distance_info = std::stod(argv[3]);
        distance_sigma = std::stod(argv[4]);

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
    if (argc != 10) {
        out_dir_name = dir_name;
    }

    std::cout.precision(20); //

    double first_t(TimeStamp::now());


}