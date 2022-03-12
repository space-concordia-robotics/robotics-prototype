//
// Created by nikolas on 3/12/22.
//

#ifndef NIR_NIR_NODE_H
#define NIR_NIR_NODE_H

#include "ros/ros.h"
#include "nir/scan_srv.h"
#include "std_msgs/String.h"
#include "nir_spectrum.h"


class nir_node{
    using Request = nir::scan_srv::Request;
    using Response = nir::scan_srv::Response;

    nir_spectrum spectrum ;
public:
        nir_node(int , char**,const char* );

        void initSpectrum();
        void startScan(Request& req,Response& res);

};

#endif //NIR_NIR_NODE_H


