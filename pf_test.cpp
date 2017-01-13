//Create by steve in 16-12-31 at 上午9:34
//
// Created by steve on 16-12-31.
//

#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

#include <omp.h>

#include <eigen3/Eigen/Dense>

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

/////stamp---------
namespace plt = matplotlibcpp;

/*
 * return the probability of N(x|miu,sigma);
 */
double NormalPdf(double x,
                 double miu,
                 double sigma) {
//    std::cout << "dis :" << x << " range:" << miu << std::endl;
    double para1((x - miu) * (x - miu) / 2 / sigma / sigma);
    double para2(1 / std::sqrt(2 * sigma * sigma * M_PI));
    return para2 * std::exp(-para1);
}

/*
 * Evaluation function.
 */
double Pdf(Eigen::Vector2d vecx,
           Eigen::MatrixXd beaconset,
           Eigen::VectorXd Range,
           double z_offset, double sigma) {
    try {
        double res(1.0);
        double dis(0.0);
        for (int i(0); i < beaconset.rows(); ++i) {
            dis = 0.0;
            dis += (vecx(0) - beaconset(i, 0)) * (vecx(0) - beaconset(i, 0));
            dis += (vecx(1) - beaconset(i, 1)) * (vecx(1) - beaconset(i, 1));
            dis += (z_offset - beaconset(i, 2)) * (z_offset - beaconset(i, 2));

            res += NormalPdf(Range(i), std::sqrt(dis), sigma);
        }
        return res;
    } catch (...) {
        std::cout << "ERROR When output likelihood probability distribution function."
                  << std::endl;
        return 1.0;
    }

}


int main(int argc, char *argv[]) {

    std::cout.precision(20); //

    double first_t(TimeStamp::now());

    /*
     * Load Imu data.
     */
    std::string dir_name = "tmp_file_dir---/";

    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"), ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    ////////////ADD NOISE TO SOURCE DATA
    std::default_random_engine ee;
    std::uniform_real_distribution<double> u(-0.15, 0.15);
    std::normal_distribution<> n(0.0, 0.2);

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = *ZuptTmp(i, 0);
    }

    /**
    * Load uwb data.
    */

    CSVReader BeaconsetReader(dir_name + "beaconset.data.csv");
    CSVReader UwbdataReader(dir_name + "UwbData.data.csv");

    Eigen::MatrixXd beaconset, UwbData;


    beaconset.resize(BeaconsetReader.GetMatrix().GetRows(), BeaconsetReader.GetMatrix().GetCols());
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


    //Fusing result.
    std::vector<double> fx, fy;
    std::vector<double> ux, uy;

    std::cout << TimeStamp::now() - first_t << std::endl;

//    PUWBPF<4> puwbpf(1000);
    EXUWBPF<4> puwbpf(4000);


    puwbpf.SetMeasurementSigma(5.0, 4);
    puwbpf.SetInputNoiseSigma(0.20);

    puwbpf.SetBeaconSet(beaconset);
    std::cout << "result:" << puwbpf.GetResult(0) << std::endl;

    puwbpf.OptimateInitial(UwbData.block(10, 1, 1, UwbData.cols() - 1).transpose(), 0);


    for (int i(0); i < UwbData.rows(); ++i) {
//        std::cout << "1.1\n";
        if ((i / UwbData.rows() * 100) % 10 == 0) {
            std::cout << "finished :" << double(i) / double(UwbData.rows()) * 100.0 << "  %.\n";
        }

        puwbpf.StateTransmition(Eigen::Vector2d(2, 2), 1);

//        if (i == 0) {
//
//            puwbpf.StateTransmition(Eigen::Vector2d(2, 2), 0);
//        } else {
//
//            puwbpf.StateTransmition(Eigen::Vector2d(2, 2), 0);
////            puwbpf.StateTransmitionWithTimeStep(Eigen::Vector2d(2, 2), UwbData(i, 0) - UwbData(i - 1, 0));
//        }

//        std::cout << UwbData.block(i, 1, 1, UwbData.cols() - 1) << std::endl;


//        std::cout << "1.2\n";
        puwbpf.Evaluation(UwbData.block(i, 1, 1, UwbData.cols() - 1).transpose(),
                          0);
//        std::cout << puwbpf.GetResult(0).transpose() << std::endl;
//        puwbpf.Evaluation(Eigen::Vector4d(UwbData(i,1),UwbData(i,2)
//        ,UwbData(i,3),UwbData(i,4)).transpose(),0);
//        std::cout << "1.3\n";
        Eigen::VectorXd tmp = puwbpf.GetResult(0);


//        std::cout << "1.4\n";
        puwbpf.Resample(-1, 0);

//        std::cout << tmp.transpose() << std::endl;

        ux.push_back(tmp(0));
        uy.push_back(tmp(1));
    }


    std::cout << "end time:" << TimeStamp::now() - first_t << std::endl;

    /**
     * Show result.
     */
    plt::named_plot("ux,uy", ux, uy, "r-+");

//    plt::named_plot("ux1", ux, ux);
    plt::grid(true);
    plt::show();


}

