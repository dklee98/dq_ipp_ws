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
    nh.param("p_verbose_ft", p_verbose_ft, true);
    nh.param("p_down_sample", p_down_sample, 1);
    nh.param("p_spatial_cluster_min", p_spatial_cluster_min, 50);
    nh.param("p_surface_cluster_min", p_surface_cluster_min, 5);
    nh.param("p_cluster_size_xy", p_cluster_size_xy, 2.0);
    nh.param("p_cluster_size_yz", p_cluster_size_yz, 2.0);
    nh.param("p_vp_min_visible_num", p_vp_min_visible_num, 15);
    

    nh.param("p_vp_rmax", p_vp_rmax, 4.0);
    nh.param("p_vp_rmin", p_vp_rmin, 3.0);
    nh.param("p_vp_rnum", p_vp_rnum, 3);
    double dphi = 15 * M_PI / 180.0;
    nh.param("p_vp_dphi", p_vp_dphi, dphi);
    nh.param("p_vp_min_dist", p_vp_min_dist, 0.3);
    nh.param("p_vp_clearance", p_vp_clearance, 0.41);
    
    c_voxel_size = map_.getVoxelSize();
    c_voxel_size_inv = 1 / c_voxel_size;
    c_map_origin = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, 1.0);
    c_map_size = Eigen::Vector3d(x_size, y_size, z_size);
    for (int i = 0; i < 3; ++i) {
        c_map_voxel_num(i) = ceil(c_map_size(i) / c_voxel_size);
    }
    int voxel_num = c_map_voxel_num(0) * c_map_voxel_num(1) * c_map_voxel_num(2);
    frontier_flag = std::vector<char>(voxel_num, 0);    // size voxel_num with initialized 0
    fill(frontier_flag.begin(), frontier_flag.end(), 0);
}

void frontier_class::searchFrontiers(std::vector<Eigen::Vector3d> new_voxels) {
    tmp_spatial_frontiers.clear();
    tmp_surface_frontiers.clear();
    Eigen::Vector3d new_bmin = new_voxels.front();
    Eigen::Vector3d new_bmax = new_voxels.front();
    getVisibleBox(new_bmin, new_bmax, new_voxels);

    // Removed changed frontiers in updated map
    auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& ftrs) {
        Eigen::Vector3i idx;
        for (auto cell : iter->cells_) {
            posToIndex(cell, idx);
            frontier_flag[toAddress(idx)] = 0;
        }
        iter = ftrs.erase(iter);
    };
    if (p_verbose_ft)
        std::cout << "Before remove: " << spatial_frontiers.size() + surface_frontiers.size() << std::endl;
    
    removed_ids.clear();
    int rmv_idx = 0;
    for (auto iter = spatial_frontiers.begin(); iter != spatial_frontiers.end();)   {
        if (haveOverlap(iter->box_min_, iter->box_max_, new_bmin, new_bmax) && 
                isFrontierChanged(*iter, 0))   {
            resetFlag(iter, spatial_frontiers);
            removed_ids.push_back(rmv_idx);
        } else{
            ++rmv_idx;
            ++iter;
        }
    }
    for (auto iter = surface_frontiers.begin(); iter != surface_frontiers.end();)   {
        if (haveOverlap(iter->box_min_, iter->box_max_, new_bmin, new_bmax) && 
                isFrontierChanged(*iter, floor(iter->cells_.size()/3)))   {
            resetFlag(iter, surface_frontiers);
            removed_ids.push_back(rmv_idx);
        } else{
            ++rmv_idx;
            ++iter;
        }
    }
    if (p_verbose_ft)
        std::cout << "After remove: " << spatial_frontiers.size() + surface_frontiers.size() << std::endl;

    Eigen::Vector3i idx;
    for (int i = 0; i < new_voxels.size(); ++i) {
        posToIndex(new_voxels[i], idx);
        Eigen::Vector3i cur(idx(0), idx(1), idx(2));
        if (frontier_flag[toAddress(cur)] == 0 && isSpatialFrontiers(idx)) {
            expandFrontier(cur, new_bmin, new_bmax);
        }
    }
    splitLargeFrontiers(tmp_spatial_frontiers, false);
    splitLargeFrontiers(tmp_surface_frontiers, true);
}

void frontier_class::expandFrontier(const Eigen::Vector3i& first,
                    Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax) {

    // Data for clustering
    std::queue<Eigen::Vector3i> cell_queue;
    std::vector<Eigen::Vector3d> spatial_expanded, surface_expanded;
    Eigen::Vector3d pos;

    indexToPos(first, pos);
    spatial_expanded.push_back(pos);
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
            if (frontier_flag[adr] == 1  || !isSpatialFrontiers(nbr))
                continue;

            indexToPos(nbr, pos);
            if (pos[2] < 0.5) continue;  // Remove noise close to ground

            if (isSurfaceFrontiers(nbr))    {
                surface_expanded.push_back(pos);
            }
            else
                spatial_expanded.push_back(pos);
            
            cell_queue.push(nbr);
            frontier_flag[adr] = 1;
        }
    }
    if (spatial_expanded.size() > p_spatial_cluster_min) {
        // Compute detailed info
        Frontier frontier;
        frontier.cells_ = spatial_expanded;
        computeFrontierInfo(frontier);
        tmp_spatial_frontiers.push_back(frontier);
    }
    if (surface_expanded.size() > p_surface_cluster_min)   {
        Frontier frontier;
        frontier.cells_ = surface_expanded;
        computeFrontierInfo(frontier);
        tmp_surface_frontiers.push_back(frontier);
    }
}

void frontier_class::splitLargeFrontiers(list<Frontier>& ftrs, bool isSurface) {
  std::list<Frontier> splits, tmps;
  for (auto it = ftrs.begin(); it != ftrs.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (isSurface)  {
        if (splitYZ(*it, splits)) {
            tmps.insert(tmps.end(), splits.begin(), splits.end());
            splits.clear();
        }
        else
            tmps.push_back(*it);
    }
    else{
        if (splitXY(*it, splits)) {
            tmps.insert(tmps.end(), splits.begin(), splits.end());
            splits.clear();
        } 
        else
            tmps.push_back(*it);
    }
  }
  ftrs = tmps;
}

bool frontier_class::splitXY(const Frontier& frontier, list<Frontier>& splits) {
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
    if (p_verbose_ft) {
        std::cout << "max idx: " << max_idx << std::endl;
        std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;
    }

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
    if (splitXY(ftr1, splits2)) {
        splits.insert(splits.end(), splits2.begin(), splits2.end());
        splits2.clear();
    } else
        splits.push_back(ftr1);

    if (splitXY(ftr2, splits2))
        splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
        splits.push_back(ftr2);

    return true;
}

bool frontier_class::splitYZ(const Frontier& frontier, list<Frontier>& splits) {
    // Split a frontier into small piece if it is too large
    auto mean = frontier.average_.tail<2>();    //y, z
    bool need_split = false;
    for (auto cell : frontier.filtered_cells_) {
        if ((cell.tail<2>() - mean).norm() > p_cluster_size_yz) {
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
        Eigen::Vector2d diff = cell.tail<2>() - mean;
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
    if (p_verbose_ft){       
        std::cout << "max idx: " << max_idx << std::endl;
        std::cout << "mean: " << mean.transpose() << ", first pc: " << first_pc.transpose() << std::endl;
    }
    // Split the frontier into two groups along the first PC
    Frontier ftr1, ftr2;
    for (auto cell : frontier.cells_) {
        if ((cell.tail<2>() - mean).dot(first_pc) >= 0)
            ftr1.cells_.push_back(cell);
        else
            ftr2.cells_.push_back(cell);
    }
    computeFrontierInfo(ftr1);
    computeFrontierInfo(ftr2);

    // Recursive call to split frontier that is still too large
    list<Frontier> splits2;
    if (splitYZ(ftr1, splits2)) {
        splits.insert(splits.end(), splits2.begin(), splits2.end());
        splits2.clear();
    } else
        splits.push_back(ftr1);

    if (splitYZ(ftr2, splits2))
        splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
        splits.push_back(ftr2);

    return true;
}

bool frontier_class::isFrontierChanged(const Frontier& ft, int cnt_th) {
    int cnt = 0;
    for (auto cell : ft.cells_) {
        if (cnt > cnt_th)   return true;
        Eigen::Vector3i idx;
        posToIndex(cell, idx);
        if (!isSpatialFrontiers(idx)) ++cnt;
        // if (!isSpatialFrontiers(idx)) return true;
    }
    return false;
}

void frontier_class::computeFrontierInfo(Frontier& ftr) {
    // Compute average position and bounding box of cluster
    ftr.average_.setZero();
    ftr.box_max_ = ftr.cells_.front();
    ftr.box_min_ = ftr.cells_.front();
    for (auto cell : ftr.cells_) {
        ftr.average_ += cell;
        for (int i = 0; i < 3; ++i) {
            ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
            ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
        }
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
    first_spatial_ftr = spatial_frontiers.end();
    first_surface_ftr = surface_frontiers.end();
    // Try find viewpoints for each cluster and categorize them according to viewpoint number
    for (auto& tmp_ftr : tmp_spatial_frontiers) {
        // Search viewpoints around frontier
        sampleViewpoints(tmp_ftr);
        if (!tmp_ftr.viewpoints_.empty()) {
            list<Frontier>::iterator inserted = spatial_frontiers.insert(spatial_frontiers.end(), tmp_ftr);
            // Sort the viewpoints by coverage fraction, best view in front
            sort(
                inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
            if (first_spatial_ftr == spatial_frontiers.end()) first_spatial_ftr = inserted;
        }
        // spatial_frontiers.insert(spatial_frontiers.end(), tmp_ftr);
    }
    for (auto& tmp_ftr : tmp_surface_frontiers) {
        // Search viewpoints around frontier
        sampleViewpoints(tmp_ftr);
        if (!tmp_ftr.viewpoints_.empty()) {
            list<Frontier>::iterator inserted = surface_frontiers.insert(surface_frontiers.end(), tmp_ftr);
            // Sort the viewpoints by coverage fraction, best view in front
            sort(
                inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
                [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
            if (first_surface_ftr == surface_frontiers.end()) first_surface_ftr = inserted;
        }
        // surface_frontiers.insert(surface_frontiers.end(), tmp_ftr);
    }
    // Reset indices of frontiers
    int idx = 0;
    if (p_verbose_ft) std::cout << "spatial ";
    for (auto& ft : spatial_frontiers) {
        ft.id_ = idx++;
        if (p_verbose_ft) std::cout << ft.id_ << ", ";
    }
    if (p_verbose_ft) std::cout << std::endl << "surface ";
    for (auto& ft : surface_frontiers) {
        ft.id_ = idx++;
        if (p_verbose_ft) std::cout << ft.id_ << ", ";
    }
    int ss = spatial_frontiers.size() + surface_frontiers.size();
    if (p_verbose_ft) std::cout << std::endl << "to visit: " << ss << std::endl;
}

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void frontier_class::sampleViewpoints(Frontier& frontier) {
    // Evaluate sample viewpoints on circles, find ones that cover most cells
    for (double rc = p_vp_rmin, dr = (p_vp_rmax - p_vp_rmin) / p_vp_rnum;
            rc <= p_vp_rmax + 1e-3; rc += dr)   {
        for (double phi = -M_PI; phi < M_PI; phi += p_vp_dphi) {
            const Eigen::Vector3d sample_pos = frontier.average_ + rc * Eigen::Vector3d(cos(phi), sin(phi), 0);
            if (sample_pos[2] < 0.5)    // height
                continue;
            // Qualified viewpoint is in bounding box and in safe region
            
            if (!map_.isObserved(sample_pos) || 
                    map_.getVoxelState(sample_pos) != voxblox_class::FREE ||
                    isNearNotFREE(sample_pos))
                continue;

            // Compute average yaw
            auto& cells = frontier.filtered_cells_;
            Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
            double avg_yaw = 0.0;
            for (int i = 1; i < cells.size(); ++i) {
                Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
                double yaw = acos(dir.dot(ref_dir));
                if (ref_dir.cross(dir)[2] < 0) yaw = -yaw;
                avg_yaw += yaw;
            }
            avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
            yawSaturation(avg_yaw);
            // Compute the fraction of covered and visible cells
            Eigen::Quaterniond sample_ori;
            int visib_num = 0;
            yaw2orientation(avg_yaw, sample_ori);
            ray_.countVisibleFrontiers(visib_num, sample_pos, sample_ori, cells);
            
            // // int visib_num = countVisibleCells(sample_pos, avg_yaw, cells);
            if (visib_num > p_vp_min_visible_num) {
                Viewpoint vp = { sample_pos, avg_yaw, visib_num };
                frontier.viewpoints_.push_back(vp);
            }
        }
    }
}

void frontier_class::getTopViewpointsInfo(const Eigen::Vector3d& cur_pos, const Eigen::Quaterniond& cur_ori, 
                                        std::vector<Eigen::Vector3d>& points, std::vector<double>& yaws) {
    points.clear();
    yaws.clear();
    if (surface_frontiers.size() == 0 && spatial_frontiers.size() == 0) {
        points.push_back(cur_pos);
        yaws.push_back(cur_ori.toRotationMatrix().eulerAngles(0,1,2)(2));
        return;
    }
    if (surface_frontiers.size() != 0)  {
        for (auto frontier : surface_frontiers) {
            bool no_view = true;
            for (auto view : frontier.viewpoints_) {
                // Retrieve the first viewpoint that is far enough and has highest coverage
                if ((view.pos_ - cur_pos).norm() < p_vp_min_dist) continue;
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
                no_view = false;
                break;
            }
            if (no_view) {
                // All viewpoints are very close, just use the first one (with highest coverage).
                auto view = frontier.viewpoints_.front();
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
            }
        }
    }
    else    {
        for (auto frontier : spatial_frontiers) {
            bool no_view = true;
            for (auto view : frontier.viewpoints_) {
                // Retrieve the first viewpoint that is far enough and has highest coverage
                if ((view.pos_ - cur_pos).norm() < p_vp_min_dist) continue;
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
                no_view = false;
                break;
            }
            if (no_view) {
                // All viewpoints are very close, just use the first one (with highest coverage).
                auto view = frontier.viewpoints_.front();
                points.push_back(view.pos_);
                yaws.push_back(view.yaw_);
            }
        }
    }
}

/////////////////////////

void frontier_class::yaw2orientation(double yaw, Eigen::Quaterniond& orientation)   {
    double roll = 0, pitch = 0;
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    orientation = q;
}

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

bool frontier_class::knownFree(const Eigen::Vector3i& idx) {
    Eigen::Vector3d pos;
    indexToPos(idx, pos);
    return map_.getVoxelState(pos) == voxblox_class::FREE;
}

bool frontier_class::knownOccupied(const Eigen::Vector3i& idx) {
    Eigen::Vector3d pos;
    indexToPos(idx, pos);
    return map_.getVoxelState(pos) == voxblox_class::OCCUPIED;
}

bool frontier_class::knownUnknown(const Eigen::Vector3i& idx) {
    Eigen::Vector3d pos;
    indexToPos(idx, pos);
    return map_.getVoxelState(pos) == voxblox_class::UNKNOWN;
}

bool frontier_class::isNeighborUnknown(const Eigen::Vector3i& voxel, int th) {
    // At least one neighbor is unknown
    auto nbrs = sixNeighbors(voxel);
    Eigen::Vector3d pos;
    int cnt = 0;
    for (auto nbr : nbrs) {
        if (cnt > th)    return true;
        indexToPos(nbr, pos);
        if (map_.getVoxelState(pos) == voxblox_class::UNKNOWN) ++cnt;
    }
    if (cnt > th)    return true;
    return false;
}

bool frontier_class::isNearOccupied(const Eigen::Vector3i& voxel) {
    // At least one neighbor is occupied
    // auto nbrs = sixNeighbors(voxel);
    auto nbrs = allNeighbors(voxel);
    Eigen::Vector3d pos;
    for (auto nbr : nbrs) {
        indexToPos(nbr, pos);
        if (map_.getVoxelState(pos) == voxblox_class::OCCUPIED) return true;
    }
    return false;
}

bool frontier_class::isNearNotFREE(const Eigen::Vector3d& pos) {
    const int vox_num = floor(p_vp_clearance / c_voxel_size);
    for (int x = -vox_num; x <= vox_num; ++x)
        for (int y = -vox_num; y <= vox_num; ++y)
            for (int z = -1; z <= 1; ++z) {
                Eigen::Vector3d vox;
                vox << pos[0] + x * c_voxel_size, pos[1] + y * c_voxel_size, pos[2] + z * c_voxel_size;
                if (map_.getVoxelState(vox) != voxblox_class::FREE) return true;
            }
    return false;
}

bool frontier_class::haveOverlap(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1, 
                                const Eigen::Vector3d& min2, const Eigen::Vector3d& max2) {
    // Check if two box have overlap part
    Eigen::Vector3d bmin, bmax;
    for (int i = 0; i < 3; ++i) {
        bmin[i] = max(min1[i], min2[i]);
        bmax[i] = min(max1[i], max2[i]);
        if (bmin[i] > bmax[i] + 1e-3) return false;
    }
    return true;
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

bool frontier_class::isSpatialFrontiers(const Eigen::Vector3i& id)  {
    return (knownFree(id) && isNeighborUnknown(id, 0));
}

bool frontier_class::isSurfaceFrontiers(const Eigen::Vector3i& id)  {
    return isNearOccupied(id); // 26 check
}

void frontier_class::getVisibleBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax,
                       std::vector<Eigen::Vector3d> new_voxels) {
    for (int i = 0; i < new_voxels.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            bmin[j] = min(bmin[j], new_voxels[i][j]);
            bmax[j] = max(bmax[j], new_voxels[i][j]);
        }
    }
}

