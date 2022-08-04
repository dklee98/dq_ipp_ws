#include "frontier.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

frontier_class::frontier_class(voxblox_class& map, ray_caster_class& ray, const ros::NodeHandle& nh) 
        : map_(map), ray_(ray)  {
    get_param(nh);

}

void frontier_class::get_param(const ros::NodeHandle& nh)    {
    double x_size, y_size, z_size;
    nh.param("map_size_x", x_size, 50.0);
    nh.param("map_size_y", y_size, 50.0);
    nh.param("map_size_z", z_size, 30.0);
    nh.param("p_down_sample", p_down_sample, 3);
    nh.param("p_cluster_min", p_cluster_min, 100);
    nh.param("p_cluster_size_xy", p_cluster_size_xy, 2.0);
    nh.param("p_cluster_size_z", p_cluster_size_z, 10.0);
    
    c_voxel_size = map_.getVoxelSize();
    c_voxel_size_inv = 1 / c_voxel_size;
    c_map_origin = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, 0.5);
    c_map_size = Eigen::Vector3d(x_size, y_size, z_size);
    for (int i = 0; i < 3; ++i) {
        c_map_voxel_num(i) = ceil(c_map_size(i) / c_voxel_size);
    }
    int voxel_num = c_map_voxel_num(0) * c_map_voxel_num(1) * c_map_voxel_num(2);
    frontier_flag = std::vector<char>(voxel_num, 0);    // size voxel_num with initialized 0
    fill(frontier_flag.begin(), frontier_flag.end(), 0);
}

void frontier_class::searchFrontiers(std::vector<Eigen::Vector3d> new_voxels) {
    tmp_frontiers.clear();
    // Removed changed frontiers in updated map
    auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& ftrs) {
        Eigen::Vector3i idx;
        for (auto cell : iter->cells_) {
            posToIndex(cell, idx);
            frontier_flag[toAddress(idx)] = 0;
        }
        iter = ftrs.erase(iter);
    };

    std::cout << "Before remove: " << frontiers.size() << std::endl;
    
    removed_ids.clear();
    int rmv_idx = 0;
    for (auto iter = frontiers.begin(); iter != frontiers.end();)   {
        if (isFrontierChanged(*iter))   {
            resetFlag(iter, frontiers);
            removed_ids.push_back(rmv_idx);
        } else{
            ++rmv_idx;
            ++iter;
        }
    }
    std::cout << "After remove: " << frontiers.size() << std::endl;

    Eigen::Vector3i idx;
    for (int i = 0; i < new_voxels.size(); ++i) {
        posToIndex(new_voxels[i], idx);
        Eigen::Vector3i cur(idx(0), idx(1), idx(2));
        if (frontier_flag[toAddress(cur)] == 0 && knownfree(cur) && isNeighborUnknown(cur)) {
            // ROS_INFO("\n******************** expand frontier ********************\n");
            expandFrontier(cur);
        }
    }
    splitLargeFrontiers(tmp_frontiers);
}

void frontier_class::expandFrontier(const Eigen::Vector3i& first) {

    // Data for clustering
    std::queue<Eigen::Vector3i> cell_queue;
    std::vector<Eigen::Vector3d> expanded;
    Eigen::Vector3d pos;

    indexToPos(first, pos);
    expanded.push_back(pos);
    cell_queue.push(first);
    frontier_flag[toAddress(first)] = 1;

    // Search frontier cluster based on region growing (distance clustering)
    while (!cell_queue.empty()) {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = allNeighbors(cur);
        for (auto nbr : nbrs) {
        // Qualified cell should be inside bounding box and frontier cell not clustered
        int adr = toAddress(nbr);
        if (frontier_flag[adr] == 1  || !(knownfree(nbr) && isNeighborUnknown(nbr)))
            continue;

        indexToPos(nbr, pos);
        if (pos[2] < 0.4) continue;  // Remove noise close to ground
        expanded.push_back(pos);
        cell_queue.push(nbr);
        frontier_flag[adr] = 1;
        }
    }
    if (expanded.size() > p_cluster_min) {
        // Compute detailed info
        Frontier frontier;
        frontier.cells_ = expanded;
        computeFrontierInfo(frontier);
        tmp_frontiers.push_back(frontier);
    }
}

void frontier_class::splitLargeFrontiers(list<Frontier>& ftrs) {
  list<Frontier> splits, tmps;
  for (auto it = ftrs.begin(); it != ftrs.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitHorizontally(*it, splits)) {
        tmps.insert(tmps.end(), splits.begin(), splits.end());
        splits.clear();
    } else
        tmps.push_back(*it);
  }
  ftrs = tmps;
}

bool frontier_class::splitHorizontally(const Frontier& frontier, list<Frontier>& splits) {
    // Split a frontier into small piece if it is too large
    auto mean = frontier.average_.head<2>();    //x, y
    bool need_split = false;
    for (auto cell : frontier.filtered_cells_) {
        if ((cell.head<2>() - mean).norm() > p_cluster_size_xy) {
            need_split = true;
            break;
        }
    }
    if (!need_split) return false;

    // Compute principal component
    // Covariance matrix of cells
    Eigen::Matrix2d cov;
    cov.setZero();
    for (auto cell : frontier.filtered_cells_) {
        Eigen::Vector2d diff = cell.head<2>() - mean;
        cov += diff * diff.transpose();
    }
    cov /= double(frontier.filtered_cells_.size());

    // Find eigenvector corresponds to maximal eigenvector
    Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int max_idx;
    double max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i) {
        if (values[i] > max_eigenvalue) {
            max_idx = i;
            max_eigenvalue = values[i];
        }
    }
    Eigen::Vector2d first_pc = vectors.col(max_idx);
    std::cout << "max idx: " << max_idx << std::endl;
    std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;

    // Split the frontier into two groups along the first PC
    Frontier ftr1, ftr2;
    for (auto cell : frontier.cells_) {
        if ((cell.head<2>() - mean).dot(first_pc) >= 0)
            ftr1.cells_.push_back(cell);
        else
            ftr2.cells_.push_back(cell);
    }
    computeFrontierInfo(ftr1);
    computeFrontierInfo(ftr2);

    // Recursive call to split frontier that is still too large
    list<Frontier> splits2;
    if (splitHorizontally(ftr1, splits2)) {
        splits.insert(splits.end(), splits2.begin(), splits2.end());
        splits2.clear();
    } else
        splits.push_back(ftr1);

    if (splitHorizontally(ftr2, splits2))
        splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
        splits.push_back(ftr2);

    return true;
}

bool frontier_class::isFrontierChanged(const Frontier& ft) {
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    posToIndex(cell, idx);
    if (!(knownfree(idx) && isNeighborUnknown(idx))) return true;
  }
  return false;
}

void frontier_class::computeFrontierInfo(Frontier& ftr) {
    // Compute average position and bounding box of cluster
    ftr.average_.setZero();
    //   ftr.box_max_ = ftr.cells_.front();
    //   ftr.box_min_ = ftr.cells_.front();
    for (auto cell : ftr.cells_) {
        ftr.average_ += cell;
        // for (int i = 0; i < 3; ++i) {
        //   ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
        //   ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
        // }
    }
    ftr.average_ /= double(ftr.cells_.size());

    // Compute downsampled cluster
    downsample(ftr.cells_, ftr.filtered_cells_);
}

void frontier_class::downsample(
        const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out) {
    // downsamping cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto cell : cluster_in)
        cloud->points.emplace_back(cell[0], cell[1], cell[2]);

    const double leaf_size = c_voxel_size * p_down_sample;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloudf);

    cluster_out.clear();
    for (auto pt : cloudf->points)
        cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

void frontier_class::computeFrontiersToVisit() {
    first_new_ftr = frontiers.end();
    int new_num = 0;
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_ftr : tmp_frontiers) {
        // Search viewpoints around frontier
        // sampleViewpoints(tmp_ftr);
        // if (!tmp_ftr.viewpoints_.empty()) {
        //     ++new_num;
        //     list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
        //     // Sort the viewpoints by coverage fraction, best view in front
        //     sort(
        //         inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
        //         [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
        //     if (first_new_ftr_ == frontiers_.end()) first_new_ftr_ = inserted;
        //     } else {
        //     // Find no viewpoint, move cluster to dormant list
        //     dormant_frontiers_.push_back(tmp_ftr);
        //     ++new_dormant_num;
        // }
        frontiers.insert(frontiers.end(), tmp_ftr);
    }
    // Reset indices of frontiers
    int idx = 0;
    for (auto& ft : frontiers) {
        ft.id_ = idx++;
        std::cout << ft.id_ << ", ";
        // std::cout << ft.size() << std::endl;
    }
    std::cout << "to visit: " << frontiers.size() << std::endl;
}

/////////////////////////

void frontier_class::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - c_map_origin(i)) * c_voxel_size_inv);
}

void frontier_class::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * c_voxel_size + c_map_origin(i);
}

int frontier_class::toAddress(const Eigen::Vector3i& id) {
    return id[0] * c_map_voxel_num(1) * c_map_voxel_num(2) + id[1] * c_map_voxel_num(2) + id[2];
}

bool frontier_class::knownfree(const Eigen::Vector3i& idx) {
    Eigen::Vector3d pos;
    indexToPos(idx, pos);
    return map_.getVoxelState(pos) == voxblox_class::FREE;
}

bool frontier_class::isNeighborUnknown(const Eigen::Vector3i& voxel) {
    // At least one neighbor is unknown
    auto nbrs = sixNeighbors(voxel);
    Eigen::Vector3d pos;
    for (auto nbr : nbrs) {
        indexToPos(nbr, pos);
        if (map_.getVoxelState(pos) == voxblox_class::UNKNOWN) return true;
    }
    return false;
}

std::vector<Eigen::Vector3i> frontier_class::sixNeighbors(const Eigen::Vector3i& voxel) {
    std::vector<Eigen::Vector3i> neighbors(6);
    Eigen::Vector3i tmp;

    tmp = voxel - Eigen::Vector3i(1, 0, 0);
    neighbors[0] = tmp;
    tmp = voxel + Eigen::Vector3i(1, 0, 0);
    neighbors[1] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 1, 0);
    neighbors[2] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 1, 0);
    neighbors[3] = tmp;
    tmp = voxel - Eigen::Vector3i(0, 0, 1);
    neighbors[4] = tmp;
    tmp = voxel + Eigen::Vector3i(0, 0, 1);
    neighbors[5] = tmp;

    return neighbors;
}

std::vector<Eigen::Vector3i> frontier_class::allNeighbors(const Eigen::Vector3i& voxel) {
  std::vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0) continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

////////////////
