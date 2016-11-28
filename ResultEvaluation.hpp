#pragma once
//
// Created by steve on 16-11-26.
//

//Create by steve in 16-11-26 at 下午9:25
#ifndef QUICKFUSING_RESULTEVALUATION_HPP
#define QUICKFUSING_RESULTEVALUATION_HPP


#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include "CSVReader.h"

class ResultEvaluation {
public:
    ResultEvaluation(std::string keypoint_file_name){
        CSVReader KeyPoint(keypoint_file_name);

        keypoint_.resize(KeyPoint.rows_,KeyPoint.cols_);
        for(int i(0);i<KeyPoint.rows_;++i)
        {
            keypoint_(i,0) = *KeyPoint.GetMatrix()(i,0);
            keypoint_(i,1) = *KeyPoint.GetMatrix()(i,1);
            keypoint_(i,2) = *KeyPoint.GetMatrix()(i,2);
        }

        keypoint_(0,2) = keypoint_(0,2) - 20.0;
        keypoint_(keypoint_.rows()-1,2)=keypoint_(keypoint_.rows()-1,2) +20.0;

    }

    double Distance(Eigen::Vector2d x,double time)
    {
        int before_index(0),after_index(0);

        for(int i(1);i<keypoint_.rows();++i)
        {
            if(keypoint_(i-1,2)<time && keypoint_(i,2)>time)
            {

            }
        }
    }

    Eigen::MatrixXd keypoint_;

};


#endif //QUICKFUSING_RESULTEVALUATION_HPP
