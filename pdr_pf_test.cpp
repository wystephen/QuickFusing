//
// Created by steve on 17-3-9.
//

#include "PDRPF.hpp"

#include <iostream>
#include <random>
#include <time_stamp.h>
#include <CSVReader.h>
#include <Eigen/Dense>


int main()
{
    std::cout.precision(20); //

    double first_t(TimeStamp::now());

    /*
     * Load Imu data.
     */

    std::string dir_name = "tmp_file_dir---/";

    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"),
            ZuptReader(dir_name + "Zupt.data.csv"),
            UwbResultReader(dir_name + "UwbResult.data.csv");

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

}