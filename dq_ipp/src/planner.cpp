#include "planner.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_class::planner_class() : f_planning(false)  {
    get_param();

}

void planner_class::get_param()    {
    p_verbose = true;
    p_checking_distance = 1.0;
    c_voxel_size = map_->getVoxelSize();
    auto vs = c_voxel_size * p_checking_distance;
    p_accurate_frontiers = false;
    if (!p_accurate_frontiers)  {
        c_neighbor_voxels[0] = Eigen::Vector3d(vs, 0, 0);
        c_neighbor_voxels[1] = Eigen::Vector3d(-vs, 0, 0);
        c_neighbor_voxels[2] = Eigen::Vector3d(0, vs, 0);
        c_neighbor_voxels[3] = Eigen::Vector3d(0, -vs, 0);
        c_neighbor_voxels[4] = Eigen::Vector3d(0, 0, vs);
        c_neighbor_voxels[5] = Eigen::Vector3d(0, 0, -vs);
    }   else{
        c_neighbor_voxels[0] = Eigen::Vector3d(vs, 0, 0);
        c_neighbor_voxels[1] = Eigen::Vector3d(vs, vs, 0);
        c_neighbor_voxels[2] = Eigen::Vector3d(vs, -vs, 0);
        c_neighbor_voxels[3] = Eigen::Vector3d(vs, 0, vs);
        c_neighbor_voxels[4] = Eigen::Vector3d(vs, vs, vs);
        c_neighbor_voxels[5] = Eigen::Vector3d(vs, -vs, vs);
        c_neighbor_voxels[6] = Eigen::Vector3d(vs, 0, -vs);
        c_neighbor_voxels[7] = Eigen::Vector3d(vs, vs, -vs);
        c_neighbor_voxels[8] = Eigen::Vector3d(vs, -vs, -vs);
        c_neighbor_voxels[9] = Eigen::Vector3d(0, vs, 0);
        c_neighbor_voxels[10] = Eigen::Vector3d(0, -vs, 0);
        c_neighbor_voxels[11] = Eigen::Vector3d(0, 0, vs);
        c_neighbor_voxels[12] = Eigen::Vector3d(0, vs, vs);
        c_neighbor_voxels[13] = Eigen::Vector3d(0, -vs, vs);
        c_neighbor_voxels[14] = Eigen::Vector3d(0, 0, -vs);
        c_neighbor_voxels[15] = Eigen::Vector3d(0, vs, -vs);
        c_neighbor_voxels[16] = Eigen::Vector3d(0, -vs, -vs);
        c_neighbor_voxels[17] = Eigen::Vector3d(-vs, 0, 0);
        c_neighbor_voxels[18] = Eigen::Vector3d(-vs, vs, 0);
        c_neighbor_voxels[19] = Eigen::Vector3d(-vs, -vs, 0);
        c_neighbor_voxels[20] = Eigen::Vector3d(-vs, 0, vs);
        c_neighbor_voxels[21] = Eigen::Vector3d(-vs, vs, vs);
        c_neighbor_voxels[22] = Eigen::Vector3d(-vs, -vs, vs);
        c_neighbor_voxels[23] = Eigen::Vector3d(-vs, 0, -vs);
        c_neighbor_voxels[24] = Eigen::Vector3d(-vs, vs, -vs);
        c_neighbor_voxels[25] = Eigen::Vector3d(-vs, -vs, -vs);
    }
}

void planner_class::get_frontiers(std::vector<Eigen::Vector3d>* surface_result,
                                std::vector<Eigen::Vector3d>* spatial_result,
                                std::vector<Eigen::Vector3d> voxels)    {

    update_frontiers(surface_result, spatial_result);
    for (int i = 0; i < voxels.size(); ++i) {
        if (map_->getVoxelState(voxels[i]) == voxblox_class::UNKNOWN)   {
            int state = isFrontierVoxel(voxels[i]);
            if (state == NOT_FRONTIER)  {
                continue;
            }
            else if (state == SPATIAL_FRONTIER) {
                spatial_result->push_back(voxels[i]);
            }
            else if (state == SURFACE_FRONTIER) {
                surface_result->push_back(voxels[i]);
            }
        }
    }
}

void planner_class::update_frontiers(std::vector<Eigen::Vector3d>* surface_result,
                                std::vector<Eigen::Vector3d>* spatial_result)    {
    std::vector<Eigen::Vector3d> temp_surface;
    std::vector<Eigen::Vector3d> temp_spatial;

    for (int i = 0; i < surface_result->size(); ++i) {
        if (map_->getVoxelState((*surface_result)[i]) == voxblox_class::UNKNOWN)   {
            int state = isFrontierVoxel((*surface_result)[i]);
            if (state == NOT_FRONTIER)  {
                continue;
            }
            else if (state == SPATIAL_FRONTIER) {
                temp_spatial.push_back((*surface_result)[i]);
            }
            else if (state == SURFACE_FRONTIER) {
                temp_surface.push_back((*surface_result)[i]);
            }
        }
    }
    for (int i = 0; i < spatial_result->size(); ++i) {
        if (map_->getVoxelState((*spatial_result)[i]) == voxblox_class::UNKNOWN)   {
            int state = isFrontierVoxel((*spatial_result)[i]);
            if (state == NOT_FRONTIER)  {
                continue;
            }
            else if (state == SPATIAL_FRONTIER) {
                temp_spatial.push_back((*spatial_result)[i]);
            }
            else if (state == SURFACE_FRONTIER) {
                temp_surface.push_back((*spatial_result)[i]);
            }
        }
    }
    *surface_result = temp_surface;
    *spatial_result = temp_spatial;
}

int planner_class::isFrontierVoxel(const Eigen::Vector3d& voxel) {
    // input voxel is arleay UNKNOWN!!!!!!!!!!!!!!!!!!!!!!
    // Check all neighboring voxels
    unsigned char voxel_state;
    if (!p_accurate_frontiers)  {
        for (int i = 0; i < 6; ++i) {
            voxel_state = map_->getVoxelState(voxel + c_neighbor_voxels[i]);
            if (voxel_state ==  voxblox_class::UNKNOWN) {
                continue;
            }
            else if (voxel_state == voxblox_class::FREE)   {
                return SPATIAL_FRONTIER;
            }
            else if (voxel_state == voxblox_class::OCCUPIED)   {
                return SURFACE_FRONTIER;
            }
        }
    }   else{
        for (int i = 0; i < 26; ++i) {
            voxel_state = map_->getVoxelState(voxel + c_neighbor_voxels[i]);
            if (voxel_state ==  voxblox_class::UNKNOWN) {
                continue;
            }
            else if (voxel_state == voxblox_class::FREE)   {
                return SPATIAL_FRONTIER;
            }
            else if (voxel_state == voxblox_class::OCCUPIED)   {
                return SURFACE_FRONTIER;
            }
        }
    }
    return NOT_FRONTIER;
}

void planner_class::test()  {
    // test getVisibleVoxels
    std::vector<Eigen::Vector3d> new_voxels;
    ROS_INFO("getVisiblevoxels");
    tic();
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);  // free space : 920 voxels
    new_voxels.erase(std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end());
    toc();
    ROS_INFO("getVisiblevoxels");
    tic();
    get_frontiers(&surface_f, &spatial_f, new_voxels);
    toc();
    if (p_verbose)  {
        v_voxels(new_voxels, c_voxel_size, V_visible_voxels);
        v_voxels(surface_f, c_voxel_size, V_surface_frontiers);
        v_voxels(spatial_f, c_voxel_size, V_spatial_frontiers);
    }
    
}