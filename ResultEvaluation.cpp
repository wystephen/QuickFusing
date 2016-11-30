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
        if (std::isnan(vec(0))) {
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

    std::ofstream outf("tmp_file_dir/keypoint.csv");

    outf << "0.8,-5.6,"<<std::to_string(ImuData(0,0))<<std::endl;

    std::vector<double> rx,ry,rt;
    rx.push_back(0.8);
    ry.push_back(-5.6);
    rt.push_back(double(ImuData(1086,0)));

    rx.push_back(-8.0);
    ry.push_back(-5.6);
    rt.push_back(double(ImuData(2144,0)));

    rx.push_back(-8.0);
    ry.push_back(-2.4);
    rt.push_back(double(ImuData(2700,0)));

    rx.push_back(0.8);
    ry.push_back(-2.4);
    rt.push_back(double(ImuData(3924,0)));

    rx.push_back(0.8);
    ry.push_back(2.4);
    rt.push_back(ImuData(4648,0));

    rx.push_back(-8.0);
    ry.push_back(2.4);
    rt.push_back(ImuData(6025,0));

    rx.push_back(-8.0);
    ry.push_back(5.6);
    rt.push_back(ImuData(6679,0));

    rx.push_back(0.8);
    ry.push_back(5.6);
    rt.push_back(ImuData(7806,0));

    rx.push_back(0.8);
    ry.push_back(-5.6);
    rt.push_back(ImuData(9614,0));


    rx.push_back(-8.0);
    ry.push_back(-5.6);
    rt.push_back(ImuData(10811,0));

    rx.push_back(-8.0);
    ry.push_back(-2.4);
    rt.push_back(ImuData(11423,0));

    rx.push_back(0.8);
    ry.push_back(-2.4);
    rt.push_back(ImuData(12550,0));

    rx.push_back(0.8);
    ry.push_back(2.4);
    rt.push_back(ImuData(13468,0));


    rx.push_back(-8.0);
    ry.push_back(2.4);
    rt.push_back(ImuData(14581,0));

    rx.push_back(-8.0);
    ry.push_back(5.6);
    rt.push_back(ImuData(15290,0));

    rx.push_back(0.8);
    ry.push_back(5.6);
    rt.push_back(ImuData(16584,0));

    rx.push_back(0.8);
    ry.push_back(-5.6);
    rt.push_back(ImuData(18518,0));


    for(int i(0);i<rx.size();++i)
    {
        outf << std::to_string(rx[i]) << ","
                                      <<std::to_string(ry[i])
                                      << ","
                                      <<std::to_string(rt[i])
                                      <<std::endl;
    }

    std::cout << "0.8,-5.6,"<<std::to_string(ImuData(19297,0))<<std::endl;

    outf.close();

    std::vector<double> imut;
    for(int k(0);k<ImuData.rows();++k)
    {
        imut.push_back(ImuData(k,0));
    }

    ResultEvaluation re("tmp_file_dir/keypoint.csv");

    std::vector<double> eval_vec;
    for(int i(0);i<imux.size();++i)
    {
        Eigen::Vector2d tmp(double(imux[i]),double(imuy[i]));
        eval_vec.push_back(re.Distance(
                tmp,
                ImuData(i,0)
        ));
    }
    plt::plot(imut,eval_vec,"y-*");


//    plt::plot(imux,"r-+");
//    plt::plot(imuy,"b-+");
    plt::plot(imut,imux,"r-+");
    plt::plot(imut,imuy,"b-+");
//    plt::plot(type_data,"g-+");
//    plt::plot(rx,ry,"r-+");
    plt::plot(rt,rx,"y-+");
    plt::plot(rt,ry,"g-+");

    plt::grid(true);


    plt::show();



    return true;
}