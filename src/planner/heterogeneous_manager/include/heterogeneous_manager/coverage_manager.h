#ifndef _COVERAGE_MANAGER_H_
#define _COVERAGE_MANAGER_H_

#include <ros/ros.h>
#include <active_perception/perception_utils.h>
#include <active_perception/graph_node.h>
#include <plan_env/raycast.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <ikd-Tree/ikd_Tree.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace Eigen;
// namespace std
// {
//     template <>
//     struct hash<Eigen::Vector3d>
//     {
//         size_t operator()(const Eigen::Vector3d &key) const
//         {
//             // Eigen::Vector3d的哈希函数
//             return std::hash<double>()(key.x()) ^ std::hash<double>()(key.y()) ^
//             std::hash<double>()(key.z());
//         }
//     };
// }

namespace hetero_planner {
class PerceptionUtils;

struct Vector3dCompare {
  bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const
  {
    if (v1(0) != v2(0))
      return v1(0) < v2(0);
    if (v1(1) != v2(1))
      return v1(1) < v2(1);
    return v1(2) < v2(2);
  }
};

struct SurfaceViewpoint {
  Vector3d pos_;
  double yaw_;
  double pitch_;
  int visib_num_;
  vector<Eigen::Vector3d> visib_cells_;
};

struct Surface {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<SurfaceViewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;

  bool state_;            // ture -> active  false -> dormant
  vector<Vector3d> pts_;  // all point cloud in the cluster
};

struct PrunedViewpoint {
  int vp_id;
  int global_id;
  Eigen::VectorXd pose;
  int vox_count;
  int iter_num;
};

struct ViewpointResult {
  /* incremental */
  int last_vps_num_;
  int all_iter_num_;
  vector<Vector3d> new_filtered_pts_;

  /* all */
  int all_vp_num_;
  vector<Vector3d> all_filtered_pts_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_pts_;
  vector<int> uncovered_filtered_pt_ids_;

  /* viewpoints info */
  Eigen::MatrixXd vps_pose_;
  vector<int> vps_voxcount_;  // contain the number of voxels covered by each viewpoint
  vector<int> vps_contri_num_;

  /* point cloud info */
  vector<bool> cover_state_all_;
  vector<int> cover_contrib_num_all_;  // 该pt所归属的vp对应的可见点云个数
  vector<int> contrib_id_all_;         // [sub-space id, vp id]

  /* viewpoints prune */
  double prune_radius_;
  map<Eigen::Vector3d, int, Vector3dCompare> inverse_idx_;
  map<int, Eigen::VectorXd> idx_viewpoints_;  // [vp_id]: viewpoint pose
  map<int, int> idx_ctrl_voxels_;             // [vp_id]: viewpoint voxel num
  map<int, bool> idx_live_state_;             // [vp_id]: if use this viewpoint finally?
  map<int, bool> idx_query_state_;            // [vp_id]: the query state of this viewpoint

  /* iter update */
  map<int, Eigen::Vector3d> pt_idx_normal_pairs_;

  /* result */
  vector<PrunedViewpoint> final_vps_;

  pcl::PointCloud<pcl::PointXYZ> pub_seen_cloud;
};

class SurfaceCoverage {
public:
  SurfaceCoverage(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh);
  ~SurfaceCoverage();

  void setVpGenStartFlag(bool flag);

  void searchSurfaces();
  void computeSurfacesToVisit();
  void updateSurfaceState();
  void surfaceViewpointsGeneration();
  void getFullViewpointsCostMatrix(
      const VectorXd& drone_pose, const vector<VectorXd>& viewpoints, Eigen::MatrixXd& mat);
  void getFullViewpointsDronesCostMatrix(
      const vector<VectorXd>& viewpoints, const vector<VectorXd>& drones_pos, Eigen::MatrixXd& mat);

  void getSurfaces(vector<vector<Eigen::Vector3d>>& clusters);
  void getSurfaceFrontiers(vector<vector<Eigen::Vector3d>>& clusters);
  void getDormantSurfaces(vector<vector<Eigen::Vector3d>>& clusters);
  void getSurfacesState(vector<char>& states);
  void getFinalViewpoints(vector<VectorXd>& viewpoints, vector<int>& viewpoint_ids,
      vector<int>& iter_nums, vector<int>& counts);
  void getTopSurfaceViewpointsInfo(vector<Eigen::Vector3d>& points, vector<double>& yaws);
  void getAllSurfaceViewpointsInfo(
      vector<Eigen::Vector3d>& points, vector<double>& yaws, vector<double>& pitchs);
  void updateSeenCells(const Vector3d& pos, const double& pitch, const double& yaw);
  void setSurfaceFrontiers(vector<vector<Vector3d>>& clusters);
  unique_ptr<PerceptionUtils> percep_utils_;

private:
  /* Functions */
  void lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void visCallback(const ros::TimerEvent& e);
  void publishIkdtree();
  void publishClusterKdtree();

  // Surface
  void expandSurface(const Eigen::Vector3i& first);
  void splitLargeSurfaces(list<Surface>& surfaces);
  bool splitSurfaceIn3D(const Surface& surface, list<Surface>& splits);
  bool splitSurfaceIn3DV2(const Surface& surface, list<Surface>& splits);
  bool splitSurfaceInXY(const Surface& surface, list<Surface>& splits);
  bool splitSurfaceInYZ(const Surface& surface, list<Surface>& splits);
  bool splitSurfaceInXZ(const Surface& surface, list<Surface>& splits);
  void sampleViewpoints(Surface& surface);

  // Surface Utils
  void computeSurfaceInfo(Surface& surface);
  int countSurfaceDecreaseNum(const Surface& sf);
  int countSurfaceIncreaseNum(const Surface& sf);
  int countVisibleOccupiedCells(const Vector3d& pos, const double& pitch, const double& yaw,
      vector<Eigen::Vector3d>& visib_cells);
  int countVisibleSurfaceCells(const Vector3d& pos, const double& pitch, const double& yaw,
      const vector<Vector3d>& cluster, vector<Eigen::Vector3d>& visib_cells);
  bool isSurfaceChanged(const Surface& sf);
  bool satisfySurfaceCell(const Eigen::Vector3i& idx);
  bool satisfySurfaceFrontierCell(const Eigen::Vector3i& idx);

  // Viewpoint Prune and Iter Update
  void updateNewPts(const vector<Vector3d>& pts, vector<bool>& cover_state,
      vector<int>& cover_contrib_num, vector<int>& contrib_id);
  void updateViewpointInfo(Eigen::VectorXd& pos, const int& vp_id);
  void updatePoseGravitation(
      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const vector<int>& inner_ids, const int& cur_idx);
  void viewpointsPrune(Eigen::MatrixXd vps_pose, vector<int> vps_voxcount);
  void computeUncoveredViewpoints(
      const Vector3d& pt_pos, const Vector3d& pt_normal, vector<pcl::PointNormal>& unc_vps);

  // Utils
  bool isPointInBox(const Eigen::Vector3d& pos, const Eigen::Vector3d& update_min,
      const Eigen::Vector3d& update_max);
  Eigen::Vector2d getPitchYaw(const Eigen::Vector3d& vec);
  double roundToDecimals(double value, int decimalPlaces);
  void getCellBox(const Vector3d& pos, Vector3d& bmin, Vector3d& bmax);
  void getFovBox(const Vector3d& pos, const double& pitch, const double& yaw,
      Eigen::Vector3i& box_min_id, Eigen::Vector3i& box_max_id);
  void wrapAngle(double& angle);
  bool haveOverlap(
      const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> eighteenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel, bool all_nbrs = false);
  bool isNeighborOccupied(const Eigen::Vector3i& voxel, bool all_nbrs = false);
  bool isNeighborFree(const Eigen::Vector3i& voxel, bool all_nbrs = false);
  bool isNearUnknown(const Vector3d& pos);
  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownFree(const Eigen::Vector3i& idx);
  bool knownOccupied(const Eigen::Vector3i& idx);
  bool inMap(const Eigen::Vector3i& idx);

  /* Data */
  bool vis_flag_ = false;
  bool start_vp_gen_flag_ = true;
  bool global_viewpoint_generation_ = false;
  int frozen_surface_min_num_;
  ros::Timer vis_timer_;

  // ikdtree
  ros::Publisher ikdtree_cloud_pub_, cluster_kdtree_cloud_pub_, uncovered_cloud_pub_, cloud_pub_;
  ros::Subscriber lidar_cloud_sub_;
  KD_TREE<PointType> ikd_tree_;

  double ikdtree_downsample_resolution_, coverage_downsample_resolution_;
  double pitch_upper_, pitch_lower_;
  double max_ray_length_;
  string frame_id_;

  // surface
  int cur_surface_num_;
  vector<char> surface_flag_;
  list<Surface> surfaces_, dormant_surfaces_, tmp_surfaces_;
  vector<int> removed_surface_ids_;
  list<Surface>::iterator first_new_surface_ftr_;

  vector<char> surface_frontier_flag_;
  vector<vector<Vector3d>> surface_frontiers_;
  ViewpointResult VR;

  // parameters
  int iter_update_num_;
  int surface_cluster_min_;
  double surface_cluster_size_xyz_, surface_cluster_size_xy_, surface_cluster_size_yz_,
      surface_cluster_size_xz_;
  double candidate_rmax_, candidate_rmin_, candidate_hmax_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int candidate_rnum_, candidate_hnum_, surface_min_visib_num_, min_visib_num_;
  double sf_decrease_percentage_max_, sf_increase_percentage_max_;
  bool need_init_viewpoints_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

// Utils
inline bool SurfaceCoverage::isPointInBox(const Eigen::Vector3d& point,
    const Eigen::Vector3d& update_min, const Eigen::Vector3d& update_max)
{
  bool insideX = (point.x() >= update_min.x()) && (point.x() <= update_max.x());
  bool insideY = (point.y() >= update_min.y()) && (point.y() <= update_max.y());
  bool insideZ = (point.z() >= update_min.z()) && (point.z() <= update_max.z());

  return insideX && insideY && insideZ;
}

inline void SurfaceCoverage::downsample(
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

inline Eigen::Vector2d SurfaceCoverage::getPitchYaw(const Eigen::Vector3d& vec)
{
  Eigen::Vector2d PY;
  double pitch = std::asin(vec.z() / (vec.norm() + 1e-3));  // calculate pitch angle
  double yaw = std::atan2(vec.y(), vec.x());                // calculate yaw angle

  PY(0) = pitch;
  PY(1) = yaw;

  return PY;
}

inline double SurfaceCoverage::roundToDecimals(double value, int decimalPlaces)
{
  double multiplier = std::pow(10.0, decimalPlaces);
  return std::round(value * multiplier) / multiplier;
}

inline void SurfaceCoverage::getCellBox(const Vector3d& pos, Vector3d& bmin, Vector3d& bmax)
{
  bmin = pos - Vector3d(resolution_, resolution_, resolution_) / 2.0;
  bmax = pos + Vector3d(resolution_, resolution_, resolution_) / 2.0;
}

inline void SurfaceCoverage::getFovBox(const Vector3d& pos, const double& pitch, const double& yaw,
    Eigen::Vector3i& box_min_id, Eigen::Vector3i& box_max_id)
{
  Eigen::Vector3d box_min, box_max;
  percep_utils_->setPose_PY(pos, pitch, yaw);
  percep_utils_->getFOVBoundingBox(box_min, box_max);
  box_min -= Eigen::Vector3d(1.0, 1.0, 1.0);
  box_max += Eigen::Vector3d(1.0, 1.0, 1.0);
  edt_env_->sdf_map_->posToIndex(box_min, box_min_id);
  edt_env_->sdf_map_->posToIndex(box_max, box_max_id);

  Eigen::Vector3d all_min, all_max;
  edt_env_->sdf_map_->getBox(all_min, all_max);
  Eigen::Vector3i all_min_id, all_max_id;
  edt_env_->sdf_map_->posToIndex(all_min, all_min_id);
  edt_env_->sdf_map_->posToIndex(all_max, all_max_id);
  for (int i = 0; i < 3; i++) {
    box_min_id[i] = std::max(box_min_id[i], all_min_id[i]);
    box_max_id[i] = std::min(box_max_id[i], all_max_id[i]);
  }
}

inline bool SurfaceCoverage::haveOverlap(
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

inline void SurfaceCoverage::wrapAngle(double& angle)
{
  while (angle < -M_PI) angle += 2 * M_PI;
  while (angle > M_PI) angle -= 2 * M_PI;
}

inline vector<Eigen::Vector3i> SurfaceCoverage::sixNeighbors(const Eigen::Vector3i& voxel)
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

inline vector<Eigen::Vector3i> SurfaceCoverage::tenNeighbors(const Eigen::Vector3i& voxel)
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

inline vector<Eigen::Vector3i> SurfaceCoverage::eighteenNeighbors(const Eigen::Vector3i& voxel)
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

inline vector<Eigen::Vector3i> SurfaceCoverage::allNeighbors(const Eigen::Vector3i& voxel)
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

inline bool SurfaceCoverage::isNeighborOccupied(const Eigen::Vector3i& voxel, bool all_nbrs)
{
  // At least one neighbor is unknown
  vector<Eigen::Vector3i> nbrs;
  if (!all_nbrs)
    nbrs = sixNeighbors(voxel);
  else
    nbrs = allNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED)
      return true;
  }
  return false;
}

inline bool SurfaceCoverage::isNeighborFree(const Eigen::Vector3i& voxel, bool all_nbrs)
{
  // At least one neighbor is unknown
  vector<Eigen::Vector3i> nbrs;
  if (!all_nbrs)
    nbrs = sixNeighbors(voxel);
  else
    nbrs = allNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::FREE)
      return true;
  }
  return false;
}

inline bool SurfaceCoverage::isNeighborUnknown(const Eigen::Vector3i& voxel, bool all_nbrs)
{
  // At least one neighbor is unknown
  vector<Eigen::Vector3i> nbrs;
  if (!all_nbrs)
    nbrs = sixNeighbors(voxel);
  else
    nbrs = allNeighbors(voxel);
  for (auto nbr : nbrs) {
    if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN &&
        edt_env_->sdf_map_->isInBox(nbr))
      return true;
  }
  return false;
}

inline int SurfaceCoverage::toadr(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->toAddress(idx);
}

inline bool SurfaceCoverage::knownFree(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
}

inline bool SurfaceCoverage::knownOccupied(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED;
}

inline bool SurfaceCoverage::isNearUnknown(const Eigen::Vector3d& pos)
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

inline bool SurfaceCoverage::inMap(const Eigen::Vector3i& idx)
{
  return edt_env_->sdf_map_->isInMap(idx);
}
}  // namespace hetero_planner

#endif
