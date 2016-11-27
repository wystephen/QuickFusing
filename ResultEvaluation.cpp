//
// Created by steve on 16-11-26.
//

//Create by steve in 16-11-26 at 下午9:25
#include "ResultEvaluation.hpp"


#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

#include "EKF.hpp"
#include "SettingPara.h"

#include <iostream>

#include <vector>

#include "matplotlib_interface.h"
#include "CSVReader.h"
#include "time_stamp.h"

#include <eigen3/Eigen/Dense>

namespace plt = matplotlibcpp;

int main(int argc,char *argv[])
{

    std::string dir_name = "tmp_file_dir/";
    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"), ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()), ZuptTmp(ZuptReader.GetMatrix());

    Eigen::MatrixXd ImuData, Zupt;
    ImuData.resize(ImuDataTmp.GetRows(), ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(), ZuptTmp.GetCols());

    for (int i(0); i < ImuDataTmp.GetRows(); ++i) {
        for (int j(0); j < ImuDataTmp.GetCols(); ++j) {
            ImuData(i, j) = *ImuDataTmp(i, j);
        }
        Zupt(i, 0) = *ZuptTmp(i, 0);
    }


    /*
     * Set initial parameter for ekf.
     */
    SettingPara init_para(true);

    init_para.init_pos1_ = Eigen::Vector3d(0.8, -5.6, 0.0);
    init_para.init_heading1_ = -180 / 180 * M_PI;

    init_para.Ts_ = 1.0 / 128.0;

    /*
     * Initial filter.
     */

    double b_time(TimeStamp::now());


    Ekf ekf(init_para);

    ekf.InitNavEq(ImuData.block(0, 1, 20, 6));

    std::vector<double> imux, imuy;
    for (int i(0); i < ImuData.rows(); ++i) {

        Eigen::VectorXd vec = ekf.GetPosition(ImuData.block(i, 1, 1, 6).transpose(), Zupt(i));
        if (isnan(vec(0))) {
            std::cout << "ddd" << std::endl;
            MYERROR("value change to nan")
            break;

        } else {
            imux.push_back(double(vec(0)));
            imuy.push_back(double(vec(1)));
        }
//        std::cout << i << ":" << ImuData.rows() << ":" << Zupt(i) << "   :   " << vec.transpose() << std::endl;
    }
//    std::cout << " Cost time :" << TimeStamp::now() - b_time << std::endl;



    /*
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

    /////////////-----------------Load UWB RESULT --------------------
    CSVReader UwbresultReader(dir_name + "UwbResult.data.csv");

    std::vector<double> ux, uy;
    for (int i(0); i < UwbresultReader.GetMatrix().GetRows(); ++i) {
        ux.push_back(*UwbresultReader.GetMatrix()(i, 1));
        uy.push_back(*UwbresultReader.GetMatrix()(i, 2));
    }

    std::vector<double> type_data;

    for(int k(0);k<imux.size();++k)
    {
        if(k<10 || imux.size() - k < 10)
        {
            type_data.push_back(0.0);

        }else if(std::abs(imux[k] - imux[k-7]) > 0.1 || std::abs(imux[k] - imux[k+7]) > 0.5)
        {
            type_data.push_back(2.0);

        } else{
            type_data.push_back(0.0);
        }
    }

//    plt::plot(ux,"r-+");
//    plt::plot(uy,"b-+");

//    plt::plot(ux,uy,"r+-");
    plt::plot(imux,"r-+");
    plt::plot(imuy,"b-+");
    plt::plot(type_data,"g-+");


    plt::show();



    return true;
}