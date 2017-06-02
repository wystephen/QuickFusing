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
    ResultEvaluation(std::string keypoint_file_name) {
        CppExtent::CSVReader KeyPoint(keypoint_file_name);

        keypoint_.resize(KeyPoint.GetMatrix().GetRows(),
                         KeyPoint.GetMatrix().GetCols());
        for (int i(0); i < KeyPoint.rows_; ++i) {
            keypoint_(i, 0) = *KeyPoint.GetMatrix()(i, 0);
            keypoint_(i, 1) = *KeyPoint.GetMatrix()(i, 1);
            keypoint_(i, 2) = *KeyPoint.GetMatrix()(i, 2);
        }

        keypoint_(0, 2) = keypoint_(0, 2) - 20.0;
        keypoint_(keypoint_.rows() - 1, 2) = keypoint_(keypoint_.rows() - 1, 2) + 20.0;

    }

    double Distance(Eigen::Vector2d x, double time) {
        int before_index(0), after_index(0);

        for (int i(1); i < keypoint_.rows(); ++i) {
            if (keypoint_(i - 1, 2) < time && keypoint_(i, 2) > time) {
                before_index = i - 1;
                after_index = i;
                break;
            }

        }
        if (before_index == 0 && after_index == 0) {
            MYERROR("wrong time stamp");
            return 0.0;
        }

        Eigen::Vector2d v1(keypoint_(before_index, 0) - x(0),
                           keypoint_(before_index, 1) - x(1));
        Eigen::Vector2d v2(keypoint_(after_index, 0) - x(0),
                           keypoint_(after_index, 1) - x(1));
//
//        if((v1-v2).norm() > 0.1){
//            return std::abs(v1(0)*v2(1)-v2(0)*v1(1))/2.0/(v1-v2).norm();
//        }else{
//            return v1.norm();
//        }
        double x0(x(0)), y0(x(1));
        double x1(keypoint_(before_index, 0)), y1(keypoint_(before_index, 1));
        double x2(keypoint_(after_index, 0)), y2(keypoint_(after_index, 1));

        if (std::sqrt(std::pow(x2 - x1, 2.0) + std::pow(y2 - y1, 2.0)) > 0.1) {
            return std::abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1)) /
                   std::sqrt(std::pow(x2 - x1, 2.0) + std::pow(y2 - y1, 2.0));
        } else {
            return (v1.norm() + v2.norm()) / 2.0;
        }

    }

    Eigen::MatrixXd keypoint_;

};


#endif //QUICKFUSING_RESULTEVALUATION_HPP
