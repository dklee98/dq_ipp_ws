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

bool planner_class::isFrontierVoxel(const Eigen::Vector3d& voxel) {
    // input voxel is arleay UNKNOWN!!!!!!!!!!!!!!!!!!!!!!
    // Check all neighboring voxels
    unsigned char voxel_state;
    for (int i = 0; i < 26; ++i) {
        voxel_state = map_->getVoxelState(voxel + c_neighbor_voxels[i]);
        if (voxel_state ==  voxblox_class::UNKNOWN) {
            continue;
        }
        if (voxel_state == voxblox_class::FREE)   {
            return GENERAL_F;
        }
        if (voxel_state == voxblox_class::OCCUPIED)   {
            return SURFACE_F;
        }
    }
    return false;
}

void planner_class::test()  {
    // test getVisibleVoxels
    std::vector<Eigen::Vector3d> new_voxels;
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);  // free space : 920 voxels
    if (p_verbose)  {
        v_visible_voxels(new_voxels, c_voxel_size);
    }
    
}