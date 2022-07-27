#include "ray_caster.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

ray_caster_class::ray_caster_class(voxblox_class& map) : map_(map)  {
    get_param();

    // cache param dependent constants
    c_fov_x = 2.0 * atan2(p_resolution_x, p_focal_length * 2.0);
    c_fov_y = 2.0 * atan2(p_resolution_y, p_focal_length * 2.0);
}

void ray_caster_class::get_param()    {
    p_ray_step = map_.getVoxelSize();  // "default voxel_size"
    p_downsampling_factor = 5.0;            // "default 1.0"

    // Downsample to voxel size resolution at max range
    c_res_x = std::min(
        static_cast<int>(ceil(p_ray_length * c_fov_x /
                        (map_.getVoxelSize() * p_downsampling_factor))), p_resolution_x);
    c_res_y = std::min(
        static_cast<int>(ceil(p_ray_length * c_fov_y /
                        (map_.getVoxelSize() * p_downsampling_factor))), p_resolution_y);

    // Determine number of splits + split distances
    c_n_sections = std::floor(std::log2(
        std::min(static_cast<double>(c_res_x), static_cast<double>(c_res_y))));
    c_split_widths.push_back(0);
    for (int i = 0; i < c_n_sections; ++i) {
        c_split_widths.push_back(std::pow(2, i));
        c_split_distances.push_back(p_ray_length / std::pow(2.0, static_cast<double>(i)));
    }
    c_split_distances.push_back(0.0);
    std::reverse(c_split_distances.begin(), c_split_distances.end());
    std::reverse(c_split_widths.begin(), c_split_widths.end());
}

bool ray_caster_class::getVisibleVoxels(std::vector<Eigen::Vector3d>* result, 
                                        const Eigen::Vector3d& position, 
                                        const Eigen::Quaterniond& orientation) {
    // Setup ray table (contains at which segment to start, -1 if occluded
    ray_table = Eigen::ArrayXXi::Zero(c_res_x, c_res_y);
  
    // Ray-casting
    Eigen::Vector3d camera_direction;
    Eigen::Vector3d direction;
    Eigen::Vector3d current_position;
    Eigen::Vector3d voxel_center;
    double distance;
    bool cast_ray;
    double map_distance;
    for (int i = 0; i < c_res_x; ++i) {
        for (int j = 0; j < c_res_y; ++j) {
            int current_segment = ray_table(i, j);  // get ray starting segment
            if (current_segment < 0) {
                continue;  // already occluded ray
            }
            getDirectionVector(&camera_direction,
                static_cast<double>(i) / (static_cast<double>(c_res_x) - 1.0),
                static_cast<double>(j) / (static_cast<double>(c_res_y) - 1.0));
            direction = orientation * camera_direction;
            distance = c_split_distances[current_segment];
            cast_ray = true;
            while (cast_ray) {
                // iterate through all splits (segments)
                while (distance < c_split_distances[current_segment + 1]) {
                    current_position = position + distance * direction;
                    distance += p_ray_step;

                    map_.getVoxelCenter(&voxel_center, current_position);
                    result->push_back(voxel_center);

                    // Check voxel occupied
                    if (map_.getVoxelState(current_position) == voxblox_class::OCCUPIED) {
                        // Occlusion, mark neighboring rays as occluded
                        markNeighboringRays(i, j, current_segment, -1);
                        cast_ray = false;
                        break;
                    }
                }
                if (cast_ray) {
                    current_segment++;
                    if (current_segment >= c_n_sections) {
                        cast_ray = false;  // done
                    } else {
                        // update ray starts of neighboring rays
                        markNeighboringRays(i, j, current_segment - 1, current_segment);
                    }
                }
            }
        }
    }
    return true;
}

void ray_caster_class::markNeighboringRays(int x, int y, int segment, int value) {
// Set all nearby (towards bottom right) ray starts, depending on the segment depth, to a value.
    for (int i = x; i < std::min(c_res_x, x + c_split_widths[segment]); ++i) {
        for (int j = y; j < std::min(c_res_y, y + c_split_widths[segment]); ++j) {
            ray_table(i, j) = value;
        }
    }
}

void ray_caster_class::getDirectionVector(Eigen::Vector3d* result, double relative_x,
                                     double relative_y) {
    *result = Eigen::Vector3d(p_focal_length,
                (0.5 - relative_x) * static_cast<double>(p_resolution_x),
                (0.5 - relative_y) * static_cast<double>(p_resolution_y)).normalized();
}