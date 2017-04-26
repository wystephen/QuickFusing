//
// Created by steve on 17-4-15.
//

#ifndef ARSLAM_DISTANCEEDGE_H
#define ARSLAM_DISTANCEEDGE_H


#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/types/slam3d_addons/types_slam3d_addons.h"

//G2O_USE_TYPE_GROUP(slam3d);

class DistanceEdge :
        public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    DistanceEdge();

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    /**
     * distance
     * @param m
     */
    virtual void setMeasurement(const double &m) {
        _measurement = m;
    }

    virtual bool getMeasurementData(double *d) const {
        *d = _measurement;
        return true;
    }

//    void linearizeOplus();

    virtual int measurementDimension() const {
        return 1;
    }

    virtual bool setMeasurementFromState();

    virtual double initialEstimatePossible(
            const g2o::OptimizableGraph::VertexSet &/*from*/,
            g2o::OptimizableGraph::Vertex */*to*/) {
//        //TODO:
//        std::cout << __FILE__ << __FUNCTION__
//                  << __LINE__ << "this function not implement" << std::endl;
        return 1.0;
    }


    virtual void initialEstimate(const g2o::OptimizableGraph::VertexSet &from,
                                 g2o::OptimizableGraph::Vertex *to);


};


#endif //ARSLAM_DISTANCEEDGE_H
