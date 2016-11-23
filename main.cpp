#include <iostream>
#include <fstream>
#include <cmath>



#include <eigen3/Eigen/Dense>

#include "CSVReader.h"
#include "matplotlib_interface.h"


namespace plt = matplotlibcpp;

int main() {

    std::string dir_name = "tmp_file_dir/";

//    std::ifstream f_list(dir_name+"file_list.txt");
//
//    std::string str;
//
//    std::cout << " name : " << dir_name + "file_name.txt" << std::endl;
//
//    while(!f_list.eof())
//    {
//        f_list >> str;
//        std::cout << str;
//    }


    CSVReader ImuDataReader(dir_name + "ImuData.data.csv"),ZuptReader(dir_name + "Zupt.data.csv");

    auto ImuDataTmp(ImuDataReader.GetMatrix()),ZuptTmp(ZuptReader.GetMatrix());

//    std::cout << "IMUDATA SIZE:" << ImuDataTmp.GetRows() << " " << ImuDataTmp.GetCols() << std::endl;
//    std::cout << "Zupt SIZE:" << ZuptTmp.GetRows() << " " << ZuptTmp.GetCols() << std::endl;

    Eigen::MatrixXd ImuData,Zupt;
    ImuData.resize(ImuDataTmp.GetRows(),ImuDataTmp.GetCols());
    Zupt.resize(ZuptTmp.GetRows(),ZuptTmp.GetCols());

    for(int i(0);i<ImuDataTmp.GetRows();++i)
    {
        for(int j(0);j<ImuDataTmp.GetCols();++j)
        {
            ImuData(i,j) = *ImuDataTmp(i,j);
        }
        Zupt(i,0) = *ZuptTmp(i,0);
    }















    return 0;


}