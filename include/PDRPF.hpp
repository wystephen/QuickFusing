//
// Created by steve on 17-3-9.
//

#include "PFBase.hpp"


class pdrpf:public PFBase<double,7,1>{
public:
    pdrpf(int Particle_num=10000):
            PFBase(Particle_num)
    {
        particle_num_ = Particle_num;
        e_.seed(TimeStamp::now());
    }
};
