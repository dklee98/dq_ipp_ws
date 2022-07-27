#ifndef VOXBLOX_H
#define VOXBLOX_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/ros_params.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class voxblox_class {
public:
    explicit voxblox_class();
    void get_param();

    // states
    const static unsigned char OCCUPIED = 0;  // NOLINT
    const static unsigned char FREE = 1;      // NOLINT
    const static unsigned char UNKNOWN = 2;   // NOLINT

    // check observed voxel
    bool isObserved(const Eigen::Vector3d& point);

    ////////////////////////////// occupancy map ///////////////////////////////////////
    // get occupancy
    unsigned char getVoxelState(const Eigen::Vector3d& point);
    // get voxel size
    double getVoxelSize();
    // get the center of a voxel from input point
    bool getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point);

    ////////////////////////////// tsdf map ///////////////////////////////////////
    // get the stored distance
    double getVoxelDistance(const Eigen::Vector3d& point);
    // get the stored weight
    double getVoxelWeight(const Eigen::Vector3d& point);
    // get the maximum allowed weight (return 0 if using uncapped weights)
    double getMaximumWeight();


private:
    std::unique_ptr<voxblox::EsdfServer> esdf_server;

    //param
    double p_collision_radius;

    //constants
    double c_voxel_size;
    double c_block_size;
    double c_maximum_weight;
};

#endif