#include "voxblox.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

voxblox_class::voxblox_class()  {
    get_param();
    // create an esdf server
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");
    esdf_server.reset(new voxblox::EsdfServer(nh, nh_private));
    esdf_server->setTraversabilityRadius(p_collision_radius);

    c_voxel_size = esdf_server->getEsdfMapPtr()->voxel_size();
    c_block_size = esdf_server->getEsdfMapPtr()->block_size();
    c_maximum_weight = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private).max_weight;  
    // direct access is not exposed
}

void voxblox_class::get_param()    {
    p_collision_radius = 1.0;
}

bool voxblox_class::isObserved(const Eigen::Vector3d& point) {
    return esdf_server->getEsdfMapPtr()->isObserved(point);
}

// get occupancy
unsigned char voxblox_class::getVoxelState(const Eigen::Vector3d& point) {
    double distance = 0.0;
    if (esdf_server->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
        // This means the voxel is observed
        if (distance < c_voxel_size) {
            return OCCUPIED;
        } else {
            return FREE;
        }
    } else {
        return UNKNOWN;
    }
}

// get voxel size
double voxblox_class::getVoxelSize() { return c_voxel_size; }

// get the center of a voxel from input point
bool voxblox_class::getVoxelCenter(Eigen::Vector3d* center, const Eigen::Vector3d& point) {
    voxblox::BlockIndex block_id = esdf_server->getEsdfMapPtr()
                                    ->getEsdfLayerPtr()
                                    ->computeBlockIndexFromCoordinates(
                                        point.cast<voxblox::FloatingPoint>());
    *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size).cast<double>();
    voxblox::VoxelIndex voxel_id =
        voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
            (point - *center).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size);
    *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size).cast<double>();
    return true;
}

// get the stored TSDF distance
double voxblox_class::getVoxelDistance(const Eigen::Vector3d& point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        esdf_server->getTsdfMapPtr()
                    ->getTsdfLayerPtr()
                    ->getBlockPtrByCoordinates(voxblox_point);
    if (block) {
        voxblox::TsdfVoxel* tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
        if (tsdf_voxel) {
            return tsdf_voxel->distance;
        }
    }
    return 0.0;
}

// get the stored weight
double voxblox_class::getVoxelWeight(const Eigen::Vector3d& point) {
    voxblox::Point voxblox_point(point.x(), point.y(), point.z());
    voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
        esdf_server->getTsdfMapPtr()
                    ->getTsdfLayerPtr()
                    ->getBlockPtrByCoordinates(voxblox_point);
    if (block) {
        voxblox::TsdfVoxel* tsdf_voxel = block->getVoxelPtrByCoordinates(voxblox_point);
        if (tsdf_voxel) {
            return tsdf_voxel->weight;
        }
    }
    return 0.0;
}

// get the maximum allowed weight (return 0 if using uncapped weights)
double voxblox_class::getMaximumWeight() { return c_maximum_weight; }