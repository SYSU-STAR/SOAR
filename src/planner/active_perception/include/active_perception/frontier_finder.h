#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>
#include <plan_env/edt_environment.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>

#include <random>
#include <unordered_set>

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace hetero_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};
// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();
  void updateFrontierCostMatrix();

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
      vector<Vector3d>& averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
      const double& max_decay, vector<vector<Vector3d>>& points, vector<vector<double>>& yaws);
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
      Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  bool isFrontierCovered();
  bool isExplorer();

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitFrontierIn3D(const Frontier& frontier, list<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  void computeFrontierInfo(Frontier& frontier);
  void sampleViewpoints(Frontier& frontier);
  bool satisfyFrontierCell(const Eigen::Vector3i& idx);

  // Utils
  void wrapAngle(double& angle);
  bool haveOverlap(
      const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  int countVisibleFrontierCells(const Vector3d& pos, const double& yaw,
      const vector<Vector3d>& cluster, vector<Eigen::Vector3d>& visib_cells);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> eighteenNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  bool isNeighborOccupied(const Eigen::Vector3i& voxel);
  bool isNeighborFree(const Eigen::Vector3i& voxel);
  bool isInLidarFOV(const Eigen::Vector3d& vp_pos, const double& vp_yaw, const Vector3d& frt_cell);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownFree(const Eigen::Vector3i& idx);
  bool knownOccupied(const Eigen::Vector3i& idx);
  bool knownUnknown(const Eigen::Vector3i& idx);
  bool inMap(const Eigen::Vector3i& idx);

  // Deprecated
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xyz_;
  double candidate_rmax_, candidate_rmin_, candidate_hmax_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_, candidate_hnum_;
  bool is_lidar_;
  double lidar_fov_up_, lidar_fov_down_, lidar_max_dist_, lidar_pitch_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

inline bool FrontierFinder::isInBoxes(
    const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx)
{
  Vector3d pt;
  edt_env_->sdf_map_->indexToPos(idx, pt);
  for (auto box : boxes) {
    // Check if contained by a box
    bool inbox = true;
    for (int i = 0; i < 3; ++i) {
      inbox = inbox && pt[i] > box.first[i] && pt[i] < box.second[i];
      if (!inbox)
        break;
    }
    if (inbox)
      return true;
  }
  return false;
}

inline bool FrontierFinder::haveOverlap(
    const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2)
{
  // Check if two box have overlap part
  Vector3d bmin, bmax;
  for (int i = 0; i < 3; ++i) {
    bmin[i] = max(min1[i], min2[i]);
    bmax[i] = min(max1[i], max2[i]);
    if (bmin[i] > bmax[i] + 1e-3)
      return false;
  }
  return true;
}

inline void FrontierFinder::wrapAngle(double& angle)
{
  while (angle < -M_PI) angle += 2 * M_PI;
  while (angle > M_PI) angle -= 2 * M_PI;
}

inline void FrontierFinder::downsample(
    const vector<Eigen::Vector3d>& cluster_in, vector<Eigen::Vector3d>& cluster_out)
{
  // downsamping cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudf(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto cell : cluster_in) cloud->points.emplace_back(cell[0], cell[1], cell[2]);

  const double leaf_size = edt_env_->sdf_map_->getResolution() * down_sample_;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);
  sor.filter(*cloudf);

  cluster_out.clear();
  for (auto pt : cloudf->points) cluster_out.emplace_back(pt.x, pt.y, pt.z);
}

inline vector<Eigen::Vector3i> FrontierFinder::sixNeighbors(const Eigen::Vector3i& voxel)
{
  vector<Eigen::Vector3i> neighbors(6);
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

inline vector<Eigen::Vector3i> FrontierFinder::tenNeighbors(const Eigen::Vector3i& voxel)
{
  vector<Eigen::Vector3i> neighbors(10);
  Eigen::Vector3i tmp;
  int count = 0;

  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      if (x == 0 && y == 0)
        continue;
      tmp = voxel + Eigen::Vector3i(x, y, 0);
      neighbors[count++] = tmp;
    }
  }
  neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
  neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::eighteenNeighbors(const Eigen::Vector3i& voxel)
{
  vector<Eigen::Vector3i> neighbors(18);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        if (abs(x) + abs(y) + abs(z) > 2)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline vector<Eigen::Vector3i> FrontierFinder::allNeighbors(const Eigen::Vector3i& voxel)
{
  vector<Eigen::Vector3i> neighbors(26);
  Eigen::Vector3i tmp;
  int count = 0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        if (x == 0 && y == 0 && z == 0)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, z);
        neighbors[count++] = tmp;
      }
  return neighbors;
}

inline bool FrontierFinder::isNeighborOccupied(const Eigen::Vector3i& voxel)
{
  // At least one neighbor is occupied
  auto nbrs = eighteenNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->isInBox(nbr) &&
        edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED)
      return true;
  }
  return false;
}

inline bool FrontierFinder::isNeighborFree(const Eigen::Vector3i& voxel)
{
  // At least one neighbor is free
  auto nbrs = allNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->isInBox(nbr) && edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::FREE)
      return true;
  }
  return false;
}
inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i& voxel)
{
  // At least one neighbor is unknown
  auto nbrs = sixNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->isInBox(nbr) &&
        edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
      return true;
  }
  return false;
}

inline bool FrontierFinder::isNearUnknown(const Eigen::Vector3d& pos)
{
  const int vox_num = floor(min_candidate_clearance_ / resolution_);
  for (int x = -vox_num; x <= vox_num; ++x)
    for (int y = -vox_num; y <= vox_num; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d vox;
        vox << pos[0] + x * resolution_, pos[1] + y * resolution_, pos[2] + z * resolution_;
        if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN)
          return true;
      }
  return false;
}

inline int FrontierFinder::toadr(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool FrontierFinder::knownFree(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool FrontierFinder::knownOccupied(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED;
}
inline bool FrontierFinder::knownUnknown(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN;
}

inline bool FrontierFinder::inMap(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->isInMap(idx);
}
}  // namespace hetero_planner
#endif