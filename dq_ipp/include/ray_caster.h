#ifndef RAY_CASTER_H
#define RAY_CASTER_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include "voxblox.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class ray_caster_class{
public:
    explicit ray_caster_class(voxblox_class& map_, const ros::NodeHandle& nh);
    virtual ~ray_caster_class() = default;

    void get_param(const ros::NodeHandle& nh);

    bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result, 
                            const Eigen::Vector3d& position, 
                            const Eigen::Quaterniond& orientation);
    bool countVisibleFrontiers(int& result, 
                                const Eigen::Vector3d& position, 
                                const Eigen::Quaterniond& orientation,
                                const vector<Eigen::Vector3d>& cluster);

    void markNeighboringRays(int x, int y, int segment, int value);
    void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                                     double relative_y);

private:
    voxblox_class& map_;

    // params
    double p_ray_step;
    double p_downsampling_factor;   // reduce the minimum resolution for performance
    double p_ray_length;  // params for camera model
    double p_focal_length;
    int p_resolution_x;
    int p_resolution_y;
    
    // constants
    double c_fov_x;
    double c_fov_y;
    int c_res_x;    // factual resolution that is used for ray casting
    int c_res_y;
    int c_n_sections;  // number of ray duplications
    std::vector<double> c_split_distances; // distances where rays are duplicated
    std::vector<int> c_split_widths; // number of max distance rays that are covered per split

    // variables
    Eigen::ArrayXXi ray_table;
};

#endif