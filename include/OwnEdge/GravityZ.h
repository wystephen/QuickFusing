/** 
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
*/
//
// Created by steve on 17-11-7.
//

#ifndef QUICKFUSING_GRAVITYZ_H
#define QUICKFUSING_GRAVITYZ_H


#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam3d/vertex_se3.h>

class GravityZ: public g2o::BaseBinaryEdge<1,double,g2o::VertexSE3,g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    GravityZ();

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    virtual void setMeasurement(const double &m) {
        _measurement = m;
//        _inverseMeasurement = 10000;
    }

    virtual bool getMeasurementData(double *d) const {
        *d = _measurement;
        return true;
    }

//    void linearizeOplus();

    virtual int measurementDimension() const { return 1; }

    virtual bool setMeasurementFromState();

    virtual double initialEstimatePossible(
            const g2o::OptimizableGraph::VertexSet &,
            g2o::OptimizableGraph::Vertex */*to*/) {
        return 1.0;
    }

    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                 g2o::OptimizableGraph::Vertex *to);

};


#endif //QUICKFUSING_GRAVITYZ_H
