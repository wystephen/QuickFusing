#pragma once
//Create by steve in 17-1-7 at 下午12:37
//
// Created by steve on 17-1-7.
//

#ifndef QUICKFUSING_ORIUWBPF_HPP
#define QUICKFUSING_ORIUWBPF_HPP


#include "PFBase.hpp"

#define ISDEBUG false
/**
 * Oriontion based state represent.
 * @tparam uwb_number
 */
template<int uwb_number>
class ORIUWBPF:public PFBase<double,6,uwb_number>{
public:
    ORIUWBPF(int particle_num):PFBase<double,6,uwb_number>(particle_num){
        try{

        }
    }

};

#endif //QUICKFUSING_ORIUWBPF_HPP
