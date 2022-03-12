//
// Created by nikolas on 3/12/22.
//
#include "nir_node.h"
int main(int argc, char** argv){

    nir_node(argc,argv,"nir");
 //   ros::NodeHandle node_handle;

}

nir_node::nir_node(int argc, char ** argv, const char * name) {
    ros::init(argc,argv,name);

}

void nir_node::startScan(nir::scan_srv::Request &req, nir::scan_srv::Response &res) {
    uint16_t start = req.start_wavelength;
    uint16_t end = req.end_wavelength;

    if(spectrum.createScanConfiguration(start,end) == -1){
        //return "Error in creating scan configuration";
        ROS_ERROR("Scan configuration error");
        return;
    }

    ROS_INFO("Performing scan on [%d,%d] nm",start, end);
    if(spectrum.scan() == -1){
        ROS_ERROR("Error in scanning");
        return;
    }
    ROS_INFO("Finished scan...");

}

void nir_node::initSpectrum() {
    if(spectrum.init() == -1){
        ROS_ERROR("Scan configuration error");
    }
    ROS_INFO("Opened NIR device");

}
