#include <heterogeneous_manager/coverage_manager.h>

#include <random>
#include <unordered_set>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

namespace hetero_planner {
SurfaceCoverage::~SurfaceCoverage()
{
}

SurfaceCoverage::SurfaceCoverage(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh)
{
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  surface_flag_ = vector<char>(voxel_num, 0);
  fill(surface_flag_.begin(), surface_flag_.end(), 0);
  surface_frontier_flag_ = vector<char>(voxel_num, 0);
  fill(surface_frontier_flag_.begin(), surface_frontier_flag_.end(), 0);

  nh.param("coverage_manager/ikdtree_downsample_resolution", ikdtree_downsample_resolution_, 0.1);
  nh.param("coverage_manager/iter_update_num", iter_update_num_, 0);
  nh.param("coverage_manager/coverage_downsample_resolution", coverage_downsample_resolution_, 0.2);
  nh.param("coverage_manager/pitch_upper", pitch_upper_, 80.0);
  nh.param("coverage_manager/pitch_lower", pitch_lower_, -70.0);

  nh.param("map_ros/frame_id_", frame_id_, string("world"));
  nh.param("sdf_map/max_ray_length", max_ray_length_, 0.0);

  nh.param("surface/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("surface/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("surface/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("surface/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("surface/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("surface/candidate_rnum", candidate_rnum_, -1);
  nh.param("surface/candidate_hnum", candidate_hnum_, -1);
  nh.param("surface/candidate_hmax", candidate_hmax_, -1.0);
  nh.param("surface/down_sample", down_sample_, -1);
  nh.param("surface/min_visib_num", min_visib_num_, -1);

  nh.param("surface/cluster_size_xyz", surface_cluster_size_xyz_, -1.0);
  nh.param("surface/cluster_size_xy", surface_cluster_size_xy_, -1.0);
  nh.param("surface/cluster_size_yz", surface_cluster_size_yz_, -1.0);
  nh.param("surface/cluster_size_xz", surface_cluster_size_xz_, -1.0);

  nh.param("surface/cluster_min", surface_cluster_min_, -1);
  nh.param("surface/sf_decrease_percentage_max_", sf_decrease_percentage_max_, 0.0);
  nh.param("surface/sf_increase_percentage_max", sf_increase_percentage_max_, 0.0);
  nh.param("surface/need_init_viewpoints", need_init_viewpoints_, false);

  nh.param("surface/frozen_surface_min_num", frozen_surface_min_num_, -1);

  percep_utils_.reset(new PerceptionUtils(nh, 1));
  vis_timer_ = nh.createTimer(ros::Duration(0.1), &SurfaceCoverage::visCallback, this);

  vis_flag_ = false;

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  lidar_cloud_sub_ = nh.subscribe("/map_ros/cloud", 10, &SurfaceCoverage::lidarCloudCallback, this);
  ikdtree_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/ikdtree_point_cloud", 10);
  cluster_kdtree_cloud_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("/cluster_kdtree_point_cloud", 10);
  uncovered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/uncovered_cloud_point_cloud", 10);
  cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/test_cloud_point_cloud", 10);

  ikd_tree_.set_downsample_param(ikdtree_downsample_resolution_);
  VR.all_cloud_pts_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  double vp_dist = (candidate_rmax_ + candidate_rmin_) / 2.0;
  VR.prune_radius_ = 0.7 * vp_dist * tan(30.0 * M_PI / 180.0);

  if (global_viewpoint_generation_) {
    iter_update_num_ += 2;
    VR.prune_radius_ = 0.85 * vp_dist * tan(30.0 * M_PI / 180.0);
  }
  start_vp_gen_flag_ = !global_viewpoint_generation_;
}

/*****  Surface Generation  *****/
void SurfaceCoverage::searchSurfaces()
{
  ros::Time t1 = ros::Time::now();
  tmp_surfaces_.clear();

  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed surfaces in updated map
  auto resetFlag = [&](list<Surface>::iterator& iter, list<Surface>& surfaces) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      surface_flag_[toadr(idx)] = 0;
    }
    iter = surfaces.erase(iter);
  };

  std::cout << "[Surface] Before remove: " << surfaces_.size() << std::endl;

  removed_surface_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = surfaces_.begin(); iter != surfaces_.end();) {
    // 还要判断是否休眠，如果休眠直接跳过即可
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) && iter->state_) {
      // 判断如果变化太大则去移除surface并重新生成
      double change_rate = (double)countSurfaceDecreaseNum(*iter) / (double)iter->cells_.size();
      double add_rate = (double)countSurfaceIncreaseNum(*iter) / (double)iter->cells_.size();
      if (change_rate > sf_decrease_percentage_max_ || add_rate > sf_increase_percentage_max_) {
        resetFlag(iter, surfaces_);
        removed_surface_ids_.push_back(rmv_idx);
      }
      else {
        ++rmv_idx;
        ++iter;
      }
    }
    else {
      ++rmv_idx;
      ++iter;
    }
  }
  std::cout << "[Surface] After remove: " << surfaces_.size() << std::endl;
  for (auto iter = dormant_surfaces_.begin(); iter != dormant_surfaces_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isSurfaceChanged(*iter))
      resetFlag(iter, dormant_surfaces_);
    else
      ++iter;
  }

  // Search new surface within box slightly inflated from updated box
  Vector3d search_min = update_min - Vector3d(1, 1, 0.5);
  Vector3d search_max = update_max + Vector3d(1, 1, 0.5);
  Vector3d box_min, box_max;
  edt_env_->sdf_map_->getBox(box_min, box_max);
  for (int k = 0; k < 3; ++k) {
    search_min[k] = max(search_min[k], box_min[k]);
    search_max[k] = min(search_max[k], box_max[k]);
  }
  Eigen::Vector3i min_id, max_id;
  edt_env_->sdf_map_->posToIndex(search_min, min_id);
  edt_env_->sdf_map_->posToIndex(search_max, max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z) {
        // Scanning the updated region to find seeds of surfaces
        Eigen::Vector3i cur(x, y, z);
        if (surface_flag_[toadr(cur)] == 0 && satisfySurfaceCell(cur)) {
          expandSurface(cur);
        }
      }
  splitLargeSurfaces(tmp_surfaces_);

  ROS_WARN_THROTTLE(
      5.0, "[Coverage Manager] surface search time: %lf", (ros::Time::now() - t1).toSec());
}

void SurfaceCoverage::expandSurface(const Eigen::Vector3i& first)
{
  auto t1 = ros::Time::now();

  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  surface_flag_[toadr(first)] = 1;

  // Search surface cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and surface cell not clustered
      int adr = toadr(nbr);
      if (surface_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) || !satisfySurfaceCell(nbr))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.5)
        continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      surface_flag_[adr] = 1;
    }
  }
  if ((int)expanded.size() > surface_cluster_min_) {
    // Compute detailed info
    Surface surface;
    surface.cells_ = expanded;
    computeSurfaceInfo(surface);
    tmp_surfaces_.push_back(surface);
  }
  // 回溯
  else {
    for (auto cell : expanded) {
      Vector3i cell_idx;
      edt_env_->sdf_map_->posToIndex(cell, cell_idx);
      surface_flag_[toadr(cell_idx)] = 0;
    }
  }

  ROS_INFO_THROTTLE(
      1.0, "[expandSurface]  expandSurface time: %lf", (ros::Time::now() - t1).toSec());
}

void SurfaceCoverage::computeSurfacesToVisit()
{
  first_new_surface_ftr_ = surfaces_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_surface : tmp_surfaces_) {
    // Search viewpoints around surface
    sampleViewpoints(tmp_surface);
    if (!tmp_surface.viewpoints_.empty()) {
      ++new_num;
      list<Surface>::iterator inserted = surfaces_.insert(surfaces_.end(), tmp_surface);
      // Sort the viewpoints by coverage fraction, best view in front
      sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const SurfaceViewpoint& v1, const SurfaceViewpoint& v2) {
            return v1.visib_num_ > v2.visib_num_;
          });
      if (first_new_surface_ftr_ == surfaces_.end())
        first_new_surface_ftr_ = inserted;
    }
    else {
      // Find no viewpoint, move cluster to dormant list
      dormant_surfaces_.push_back(tmp_surface);
      ++new_dormant_num;
    }
  }
  // Reset indices of surfaces
  int idx = 0;
  for (auto& sf : surfaces_) {
    sf.id_ = idx++;
    std::cout << sf.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << surfaces_.size() << ", dormant: " << dormant_surfaces_.size()
            << std::endl;
}

void SurfaceCoverage::sampleViewpoints(Surface& surface)
{
  int max_visib_num = 0;
  SurfaceViewpoint top_vp;
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      for (double h = 0.0, dh = candidate_hmax_ / (double)candidate_hnum_;
           h <= candidate_hmax_ + 1e-3; h += dh) {
        const Vector3d sample_pos =
            surface.average_ + rc * Vector3d(cos(phi), sin(phi), 0) + Vector3d(0, 0, h);

        // Qualified viewpoint is in bounding box and in safe region
        if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
            edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
          continue;

        // Compute average yaw
        const auto& cells = surface.filtered_cells_;
        Eigen::Vector3d ref_dir = (cells.front() - sample_pos).normalized();
        double avg_yaw = 0.0;
        for (size_t i = 1; i < cells.size(); ++i) {
          Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
          double yaw = acos(dir.dot(ref_dir));
          if (ref_dir.cross(dir)[2] < 0)
            yaw = -yaw;
          avg_yaw += yaw;
        }
        avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
        wrapAngle(avg_yaw);

        // Compute the fraction of covered and visible cells
        vector<Vector3d> visib_cells;
        int visib_num =
            countVisibleSurfaceCells(sample_pos, 0.0, avg_yaw, surface.cells_, visib_cells);
        if (visib_num > min_visib_num_) {
          SurfaceViewpoint vp;
          vp.pos_ = sample_pos;
          vp.pitch_ = 0.0;
          vp.yaw_ = avg_yaw;
          vp.visib_num_ = visib_num;
          vp.visib_cells_ = visib_cells;
          // surface.viewpoints_.push_back(vp);

          if (visib_num > max_visib_num) {
            top_vp = vp;
            max_visib_num = visib_num;
          }
        }
      }
    }
  if (max_visib_num != 0) {
    surface.viewpoints_.push_back(top_vp);
  }
}

void SurfaceCoverage::splitLargeSurfaces(list<Surface>& surfaces)
{
  list<Surface> splits, tmps, xy_tmps, yz_tmps, xz_tmps;
  for (auto it = surfaces.begin(); it != surfaces.end(); ++it) {
    xy_tmps.clear();
    if (splitSurfaceInYZ(*it, splits)) {
      // tmps.insert(tmps.end(), splits.begin(), splits.end());
      xy_tmps.insert(xy_tmps.end(), splits.begin(), splits.end());
      splits.clear();
    }
    else
      xy_tmps.push_back(*it);

    yz_tmps.clear();
    for (auto it2 = xy_tmps.begin(); it2 != xy_tmps.end(); ++it2) {
      if (splitSurfaceInXZ(*it2, splits)) {
        yz_tmps.insert(yz_tmps.end(), splits.begin(), splits.end());
        splits.clear();
      }
      else
        yz_tmps.push_back(*it2);
    }

    for (auto it2 = yz_tmps.begin(); it2 != yz_tmps.end(); ++it2) {
      if (splitSurfaceInXY(*it2, splits)) {
        tmps.insert(tmps.end(), splits.begin(), splits.end());
        splits.clear();
      }
      else
        tmps.push_back(*it2);
    }
  }
  surfaces = tmps;
}

bool SurfaceCoverage::splitSurfaceInYZ(const Surface& surface, list<Surface>& splits)
{
  auto mean = surface.average_;

  bool need_split_yz = false;

  // Project the data onto the YZ plane
  Eigen::Vector2d mean_yz(mean[1], mean[2]);

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d cell_yz(cell[1], cell[2]);

    if ((cell_yz - mean_yz).norm() > surface_cluster_size_yz_)
      need_split_yz = true;
  }

  if (!need_split_yz)
    return false;

  // Compute covariance matrices of cells projected onto the YZ plane
  Eigen::Matrix2d cov_yz;
  cov_yz.setZero();

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d diff_yz(cell[1] - mean[1], cell[2] - mean[2]);
    cov_yz += diff_yz * diff_yz.transpose();
  }

  cov_yz /= double(surface.filtered_cells_.size());

  // Find eigenvectors corresponds to maximal eigenvalues for the YZ plane
  Eigen::Vector2d first_pc_yz;
  Eigen::EigenSolver<Eigen::Matrix2d> es_yz(cov_yz);
  auto values_yz = es_yz.eigenvalues().real();
  auto vectors_yz = es_yz.eigenvectors().real();
  int max_idx_yz = 0;
  double max_eigenvalue_yz = values_yz[0];
  for (int i = 1; i < values_yz.rows(); ++i) {
    if (values_yz[i] > max_eigenvalue_yz) {
      max_idx_yz = i;
      max_eigenvalue_yz = values_yz[i];
    }
  }
  first_pc_yz = vectors_yz.col(max_idx_yz);

  // Split the surface into two groups along the first PC for the YZ plane
  Surface ftr1, ftr2;
  for (auto cell : surface.cells_) {
    if (need_split_yz) {
      Eigen::Vector2d cell_yz(cell[1], cell[2]);
      double proj_yz = (cell_yz - mean_yz).dot(first_pc_yz);
      if (proj_yz >= 0)
        ftr1.cells_.push_back(cell);
      else
        ftr2.cells_.push_back(cell);
    }
  }

  computeSurfaceInfo(ftr1);
  computeSurfaceInfo(ftr2);

  // Recursive call to split surface that is still too large
  list<Surface> splits2;
  if (splitSurfaceInYZ(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  }
  else
    splits.push_back(ftr1);

  if (splitSurfaceInYZ(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool SurfaceCoverage::splitSurfaceInXZ(const Surface& surface, list<Surface>& splits)
{
  auto mean = surface.average_;

  bool need_split_xz = false;

  // Project the data onto the XZ plane
  Eigen::Vector2d mean_xz(mean[0], mean[2]);

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d cell_xz(cell[0], cell[2]);

    if ((cell_xz - mean_xz).norm() > surface_cluster_size_xz_)
      need_split_xz = true;
  }

  if (!need_split_xz)
    return false;

  // Compute covariance matrices of cells projected onto the XZ plane
  Eigen::Matrix2d cov_xz;
  cov_xz.setZero();

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d diff_xz(cell[0] - mean[0], cell[2] - mean[2]);
    cov_xz += diff_xz * diff_xz.transpose();
  }

  cov_xz /= double(surface.filtered_cells_.size());

  // Find eigenvectors corresponding to maximal eigenvalues for the XZ plane
  Eigen::Vector2d first_pc_xz;
  Eigen::EigenSolver<Eigen::Matrix2d> es_xz(cov_xz);
  auto values_xz = es_xz.eigenvalues().real();
  auto vectors_xz = es_xz.eigenvectors().real();
  int max_idx_xz = 0;
  double max_eigenvalue_xz = values_xz[0];
  for (int i = 1; i < values_xz.rows(); ++i) {
    if (values_xz[i] > max_eigenvalue_xz) {
      max_idx_xz = i;
      max_eigenvalue_xz = values_xz[i];
    }
  }
  first_pc_xz = vectors_xz.col(max_idx_xz);

  // Split the surface into two groups along the first PC for the XZ plane
  Surface ftr1, ftr2;
  for (auto cell : surface.cells_) {
    if (need_split_xz) {
      Eigen::Vector2d cell_xz(cell[0], cell[2]);
      double proj_xz = (cell_xz - mean_xz).dot(first_pc_xz);
      if (proj_xz >= 0)
        ftr1.cells_.push_back(cell);
      else
        ftr2.cells_.push_back(cell);
    }
  }

  computeSurfaceInfo(ftr1);
  computeSurfaceInfo(ftr2);

  // Recursive call to split surface that is still too large
  list<Surface> splits2;
  if (splitSurfaceInXZ(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  }
  else
    splits.push_back(ftr1);

  if (splitSurfaceInXZ(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool SurfaceCoverage::splitSurfaceInXY(const Surface& surface, list<Surface>& splits)
{
  auto mean = surface.average_;

  bool need_split_xy = false;

  // Project the data onto three 2D planes
  Eigen::Vector2d mean_xy(mean[0], mean[1]);

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d cell_xy(cell[0], cell[1]);

    if ((cell_xy - mean_xy).norm() > surface_cluster_size_xy_)
      need_split_xy = true;
  }

  if (!need_split_xy)
    return false;

  // Compute covariance matrices of cells projected onto each 2D plane
  Eigen::Matrix2d cov_xy, cov_xz, cov_yz;
  cov_xy.setZero();

  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector2d diff_xy(cell[0] - mean[0], cell[1] - mean[1]);
    cov_xy += diff_xy * diff_xy.transpose();
  }

  cov_xy /= double(surface.filtered_cells_.size());

  // Find eigenvectors corresponds to maximal eigenvalues for each 2D plane
  Eigen::Vector2d first_pc_xy, first_pc_xz, first_pc_yz;
  Eigen::EigenSolver<Eigen::Matrix2d> es_xy(cov_xy);
  auto values_xy = es_xy.eigenvalues().real();
  auto vectors_xy = es_xy.eigenvectors().real();
  int max_idx_xy = 0;
  double max_eigenvalue_xy = values_xy[0];
  for (int i = 1; i < values_xy.rows(); ++i) {
    if (values_xy[i] > max_eigenvalue_xy) {
      max_idx_xy = i;
      max_eigenvalue_xy = values_xy[i];
    }
  }
  first_pc_xy = vectors_xy.col(max_idx_xy);

  // Split the surface into two groups along the first PC for each 2D plane
  Surface ftr1, ftr2;
  for (auto cell : surface.cells_) {
    if (need_split_xy) {
      Eigen::Vector2d cell_xy(cell[0], cell[1]);
      double proj_xy = (cell_xy - mean_xy).dot(first_pc_xy);
      if (proj_xy >= 0)
        ftr1.cells_.push_back(cell);
      else
        ftr2.cells_.push_back(cell);
    }
  }

  computeSurfaceInfo(ftr1);
  computeSurfaceInfo(ftr2);

  // Recursive call to split surface that is still too large
  list<Surface> splits2;
  if (splitSurfaceInXY(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  }
  else
    splits.push_back(ftr1);

  if (splitSurfaceInXY(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

bool SurfaceCoverage::splitSurfaceIn3D(const Surface& surface, list<Surface>& splits)
{
  auto mean = surface.average_;
  bool need_split = false;
  for (auto cell : surface.filtered_cells_) {
    if ((cell - mean).norm() > surface_cluster_size_xyz_) {
      need_split = true;
      break;
    }
  }
  if (!need_split)
    return false;

  // Compute covariance matrix of cells
  Eigen::Matrix3d cov;
  cov.setZero();
  for (auto cell : surface.filtered_cells_) {
    Eigen::Vector3d diff = cell - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(surface.filtered_cells_.size());

  // Find eigenvector corresponds to maximal eigenvalue
  Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
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
  Eigen::Vector3d first_pc = vectors.col(max_idx);

  // Split the surface into two groups along the first PC
  Surface ftr1, ftr2;
  for (auto cell : surface.cells_) {
    if ((cell - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeSurfaceInfo(ftr1);
  computeSurfaceInfo(ftr2);

  // Recursive call to split surface that is still too large
  list<Surface> splits2;
  if (splitSurfaceIn3D(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  }
  else
    splits.push_back(ftr1);

  if (splitSurfaceIn3D(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

void SurfaceCoverage::updateSurfaceState()
{
  for (auto& surface : surfaces_) {
    if (!surface.state_)
      continue;
    bool nbr_frontier = false;
    for (const auto& cell : surface.cells_) {
      Vector3i idx;
      edt_env_->sdf_map_->posToIndex(cell, idx);
      auto nbrs = sixNeighbors(idx);
      for (auto nbr : nbrs) {
        // 邻居有surface frontier则说明还可能会更新
        if ((edt_env_->sdf_map_->isInBox(nbr) && surface_frontier_flag_[toadr(nbr)]) ||
            (int)surface.cells_.size() < frozen_surface_min_num_) {
          nbr_frontier = true;
          break;
        }
      }
    }
    // surface第一次被冻结
    if (!nbr_frontier) {
      surface.state_ = false;
      surface.id_ = cur_surface_num_;
      cur_surface_num_++;
      pcl::PointCloud<PointType>::Ptr cluster_pts(new pcl::PointCloud<PointType>);
      cluster_pts.reset(new pcl::PointCloud<PointType>);

      for (auto cell : surface.cells_) {
        Vector3d bmin, bmax;
        getCellBox(cell, bmin, bmax);
        BoxPointType cell_box;
        for (int i = 0; i < 3; i++) {
          cell_box.vertex_min[i] = static_cast<float>(bmin[i]);
          cell_box.vertex_max[i] = static_cast<float>(bmax[i]);
        }
        PointVector pts;
        ikd_tree_.Box_Search(cell_box, pts);
        cluster_pts->insert(cluster_pts->end(), pts.begin(), pts.end());
        Vector3i cell_idx;
        edt_env_->sdf_map_->posToIndex(cell, cell_idx);
      }

      pcl::VoxelGrid<PointType> sor;
      sor.setInputCloud(cluster_pts);
      sor.setLeafSize(coverage_downsample_resolution_, coverage_downsample_resolution_,
          coverage_downsample_resolution_);

      pcl::PointCloud<PointType>::Ptr filtered_pts(new pcl::PointCloud<PointType>);
      sor.filter(*filtered_pts);
      for (const auto& pt : filtered_pts->points) surface.pts_.emplace_back(pt.x, pt.y, pt.z);

      pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
      cloud->insert(cloud->end(), cluster_pts->begin(), cluster_pts->end());

      // VR add points
      VR.all_cloud_pts_->insert(VR.all_cloud_pts_->end(), cloud->begin(), cloud->end());
      VR.all_filtered_pts_.insert(VR.all_filtered_pts_.end(), surface.pts_.begin(),
          surface.pts_.end());  // 降采样过后的
      VR.all_vp_num_ += (int)surface.viewpoints_.size();
      VR.new_filtered_pts_.insert(
          VR.new_filtered_pts_.end(), surface.pts_.begin(), surface.pts_.end());
    }
  }
}

void SurfaceCoverage::setVpGenStartFlag(bool flag)
{
  start_vp_gen_flag_ = flag;
}

/*****  Surface Viewpoints Generation(Prune and Iter Update)  *****/
void SurfaceCoverage::surfaceViewpointsGeneration()
{
  if (VR.new_filtered_pts_.size() == 0)
    return;
  if (!start_vp_gen_flag_)
    return;

  auto t1 = ros::Time::now();

  // update newly added point cloud
  vector<bool> new_cover_state_all;
  vector<int> new_cover_contrib_num_all;
  vector<int> new_contrib_id_all;
  updateNewPts(
      VR.new_filtered_pts_, new_cover_state_all, new_cover_contrib_num_all, new_contrib_id_all);
  VR.cover_state_all_.insert(
      VR.cover_state_all_.end(), new_cover_state_all.begin(), new_cover_state_all.end());
  VR.cover_contrib_num_all_.insert(VR.cover_contrib_num_all_.end(),
      new_cover_contrib_num_all.begin(), new_cover_contrib_num_all.end());
  VR.contrib_id_all_.insert(
      VR.contrib_id_all_.end(), new_contrib_id_all.begin(), new_contrib_id_all.end());

  // find uncovered point cloud
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(VR.all_cloud_pts_);

  VR.uncovered_filtered_pt_ids_.clear();
  vector<int> uncovered_all_pt_ids;  // find the id corresponding to the global point cloud
  pcl::PointCloud<pcl::PointXYZ> uncovered_cloud;
  for (int i = 0; i < (int)VR.cover_state_all_.size(); ++i) {
    if (VR.cover_state_all_[i] == false) {
      VR.uncovered_filtered_pt_ids_.push_back(i);
      Vector3d unc_pt = VR.all_filtered_pts_[i];
      pcl::PointXYZ point;
      point.x = unc_pt.x();
      point.y = unc_pt.y();
      point.z = unc_pt.z();
      uncovered_cloud.push_back(point);
      std::vector<int> nearest_indices;
      std::vector<float> nearest_distances;
      kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances);
      uncovered_all_pt_ids.push_back(nearest_indices[0]);
    }
  }

  int sum_seen = 0;
  for (auto final_vp : VR.final_vps_) {
    sum_seen += final_vp.vox_count;
  }

  ROS_WARN(
      "\033[1;32m[Coverage Manager] Number of Total Voxels = %d", (int)VR.cover_state_all_.size());
  ROS_WARN("\033[1;32m[Coverage Manager] Seen all voxels num = %d", sum_seen);
  // ROS_WARN("\033[1;32m[Coverage Manager] Number of Uncovered Voxels = %d",
  //     (int)uncovered_all_pt_ids.size());

  // Generate viewpoints corresponding to uncovered point clouds.
  VR.pt_idx_normal_pairs_.clear();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(VR.all_cloud_pts_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(1.0);

  ne.setIndices(boost::make_shared<std::vector<int>>(uncovered_all_pt_ids));
  pcl::PointCloud<pcl::Normal>::Ptr uncovered_normals(new pcl::PointCloud<pcl::Normal>());
  ne.compute(*uncovered_normals);

  map<int, vector<pcl::PointNormal>> all_uncovered_vps;
  for (int i = 0; i < (int)uncovered_all_pt_ids.size(); ++i) {
    int unc_id = uncovered_all_pt_ids[i];

    Eigen::Vector3d normal_vector;
    normal_vector(0) = uncovered_normals->points[i].normal_x;
    normal_vector(1) = uncovered_normals->points[i].normal_y;
    normal_vector(2) = uncovered_normals->points[i].normal_z;

    VR.pt_idx_normal_pairs_[unc_id] = normal_vector.normalized();

    vector<pcl::PointNormal> tmp_uncovered_vps;
    Vector3d pt_pos(VR.all_cloud_pts_->points[unc_id].x, VR.all_cloud_pts_->points[unc_id].y,
        VR.all_cloud_pts_->points[unc_id].z);
    computeUncoveredViewpoints(pt_pos, normal_vector, tmp_uncovered_vps);
    all_uncovered_vps[unc_id] = tmp_uncovered_vps;
  }

  int unc_iter = 0, last_unc_num = -1;
  int iter_max = iter_update_num_;
  while ((int)uncovered_all_pt_ids.size() > 0 && unc_iter < iter_max &&
         last_unc_num != (int)uncovered_all_pt_ids.size()) {
    VR.all_iter_num_++;
    ROS_INFO("\033[1;34m[Coverage Manager] Iter_%d Uncovered Start", unc_iter);
    last_unc_num = (int)uncovered_all_pt_ids.size();

    // The overall state of the point cloud before the last iteration
    vector<bool> tempCoverState;
    vector<int> tempCoverContribNum;
    vector<int> tempContribId;
    tempCoverState = VR.cover_state_all_;
    tempCoverContribNum = VR.cover_contrib_num_all_;
    tempContribId = VR.contrib_id_all_;

    pcl::PointCloud<pcl::PointNormal>::Ptr uncovered_vps(new pcl::PointCloud<pcl::PointNormal>);
    for (auto p : uncovered_all_pt_ids) {
      for (auto uvp : all_uncovered_vps[p]) uncovered_vps->points.push_back(uvp);
    }

    // Get all viewpoints information
    vector<pcl::PointXYZ> tempVps;
    vector<Vector3d> tempVpsDir;
    for (int i = 0; i < (int)uncovered_vps->points.size(); i++) {
      pcl::PointXYZ vp_position;
      vp_position.x = uncovered_vps->points[i].x;
      vp_position.y = uncovered_vps->points[i].y;
      vp_position.z = uncovered_vps->points[i].z;

      Eigen::Vector3d posi_, dir_;
      posi_ << uncovered_vps->points[i].x, uncovered_vps->points[i].y, uncovered_vps->points[i].z;
      dir_ << uncovered_vps->points[i].normal_x, uncovered_vps->points[i].normal_y,
          uncovered_vps->points[i].normal_z;
      tempVpsDir.push_back(dir_);
      tempVps.push_back(vp_position);
    }

    int oriVpNum = VR.vps_pose_.rows();

    Eigen::MatrixXd pose_set_unc;
    pose_set_unc.resize((int)tempVps.size(), 5);
    vector<int> vps_count(tempVps.size(), 0);
    vector<int> vps_contri_num(tempVps.size(), 0);
    VR.vps_voxcount_.insert(VR.vps_voxcount_.end(), vps_count.begin(), vps_count.end());
    VR.vps_contri_num_.insert(
        VR.vps_contri_num_.end(), vps_contri_num.begin(), vps_contri_num.end());

    vector<int> before_vps_voxcount, temp_vps_voxcount;
    before_vps_voxcount = VR.vps_voxcount_;

    for (int i = 0; i < (int)tempVps.size(); ++i) {
      Eigen::VectorXd pose_unc(5);
      Vector2d dir = getPitchYaw(tempVpsDir[i]);
      Vector3d pos = Vector3d(tempVps[i].x, tempVps[i].y, tempVps[i].z);
      pose_unc << pos(0), pos(1), pos(2), dir(0), dir(1);
      int vp_id = oriVpNum + i;
      updateViewpointInfo(pose_unc, vp_id);
      pose_set_unc.row(i) = pose_unc;

      // For viewpoint visualization
      SurfaceViewpoint sf_vp;
      sf_vp.pos_ = pos;
      sf_vp.pitch_ = dir(0);
      sf_vp.yaw_ = dir(1);
      surfaces_.front().viewpoints_.push_back(sf_vp);
    }

    Eigen::MatrixXd oriVps = VR.vps_pose_;
    if ((int)oriVps.rows() > 0) {
      VR.vps_pose_.resize(oriVps.rows() + (int)tempVps.size(), 5);
      VR.vps_pose_.topRows(oriVps.rows()) = oriVps;
      VR.vps_pose_.bottomRows((int)tempVps.size()) = pose_set_unc;
    }
    else
      VR.vps_pose_ = pose_set_unc;

    VR.cover_state_all_.clear();
    VR.cover_state_all_ = tempCoverState;
    VR.cover_contrib_num_all_.clear();
    VR.cover_contrib_num_all_ = tempCoverContribNum;
    VR.contrib_id_all_.clear();
    VR.contrib_id_all_ = tempContribId;
    temp_vps_voxcount = VR.vps_voxcount_;
    VR.vps_voxcount_ = before_vps_voxcount;
    viewpointsPrune(VR.vps_pose_, temp_vps_voxcount);

    // Initialization and data update after pruning
    VR.uncovered_filtered_pt_ids_.clear();
    uncovered_all_pt_ids.clear();
    uncovered_cloud.clear();
    for (int i = 0; i < (int)VR.cover_state_all_.size(); ++i) {
      if (VR.cover_state_all_[i] == false) {
        VR.uncovered_filtered_pt_ids_.push_back(i);
        Vector3d unc_pt = VR.all_filtered_pts_[i];
        pcl::PointXYZ point;
        point.x = unc_pt.x();
        point.y = unc_pt.y();
        point.z = unc_pt.z();
        uncovered_cloud.push_back(point);
        vector<int> nearest_indices;
        vector<float> nearest_distances;
        kdtree.nearestKSearch(point, 1, nearest_indices, nearest_distances);
        uncovered_all_pt_ids.push_back(nearest_indices[0]);
      }
    }
    // Publish uncovered cloud
    uncovered_cloud.width = uncovered_cloud.points.size();
    uncovered_cloud.height = 1;
    uncovered_cloud.is_dense = true;
    uncovered_cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 uncovered_cloud_msg;
    pcl::toROSMsg(uncovered_cloud, uncovered_cloud_msg);
    uncovered_cloud_pub_.publish(uncovered_cloud_msg);

    sum_seen = 0;
    for (auto final_vp : VR.final_vps_) {
      // ROS_WARN("Iter_%d  vp_%d : count = %d",
      //           VR.all_iter_num_, final_vp.vp_id, final_vp.vox_count);
      sum_seen += final_vp.vox_count;
    }
    ROS_WARN("\033[1;34m[Coverage Manager] Iter_%d Number of Total Voxels = %d", VR.all_iter_num_,
        (int)VR.cover_state_all_.size());
    ROS_WARN("\033[1;34m[Coverage Manager] Iter_%d Seen all voxels num = %d", VR.all_iter_num_,
        sum_seen);
    ROS_WARN("\033[1;34m[Coverage Manager] Iter_%d Now final viewpoint num = %d", VR.all_iter_num_,
        (int)VR.final_vps_.size());
    // ROS_WARN("\033[1;34m[Coverage Manager] Iter_%d Number of Uncovered Points = %d",
    //     VR.all_iter_num_, (int)uncovered_all_pt_ids.size());

    unc_iter++;
    VR.last_vps_num_ = VR.vps_pose_.rows();
  }

  VR.new_filtered_pts_.clear();
  ROS_WARN("\033[1;32m[Coverage Manager] SurfaceViewpointsGeneration Total time = %f",
      (ros::Time::now() - t1).toSec());
}

void SurfaceCoverage::viewpointsPrune(Eigen::MatrixXd vps_pose, vector<int> vps_voxcount)
{
  VR.inverse_idx_.clear();
  VR.idx_viewpoints_.clear();
  VR.idx_ctrl_voxels_.clear();
  VR.idx_live_state_.clear();
  VR.idx_query_state_.clear();
  vector<int> prune_order_;

  pcl::PointXYZ vp_pt_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vp_cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = VR.last_vps_num_; i < (int)vps_pose.rows(); i++) {
    if (vps_voxcount[i] > 0) {
      Eigen::VectorXd qualified_vp_ = vps_pose.row(i);
      int index_ = i;
      vp_pt_.x = qualified_vp_(0);
      vp_pt_.y = qualified_vp_(1);
      vp_pt_.z = qualified_vp_(2);
      vp_cloud_->points.push_back(vp_pt_);

      Vector3d tmp_vp;
      tmp_vp(0) = vp_pt_.x;
      tmp_vp(1) = vp_pt_.y;
      tmp_vp(2) = vp_pt_.z;
      VR.inverse_idx_[tmp_vp] = index_;
      VR.idx_viewpoints_[index_] = qualified_vp_;
      VR.idx_ctrl_voxels_[index_] = vps_voxcount[i];
      VR.idx_live_state_[index_] = true;  // all initialized as active
      VR.idx_query_state_[index_] = false;
    }
  }

  if (vp_cloud_->points.size() == 0) {
    ROS_ERROR("[Coverage Manager] No need prune viewpoints!");
    return;
  }

  /* sort vps according to ctrl voxels */
  vector<pair<int, int>> ctrlVector(VR.idx_ctrl_voxels_.begin(), VR.idx_ctrl_voxels_.end());
  sort(ctrlVector.begin(), ctrlVector.end(),
      [](const auto& a, const auto& b) { return a.second > b.second; });

  for (const auto& p : ctrlVector) prune_order_.push_back(p.first);

  /* construct viewpoints kdtree */
  pcl::KdTreeFLANN<pcl::PointXYZ> vps_tree_;
  vps_tree_.setInputCloud(vp_cloud_);

  /* Gravitation-like model */
  double bubble_radius = VR.prune_radius_;
  for (auto prune_idx : prune_order_) {
    vector<int> index_set;
    vector<float> radius_set;
    vp_pt_.x = VR.idx_viewpoints_.find(prune_idx)->second(0);
    vp_pt_.y = VR.idx_viewpoints_.find(prune_idx)->second(1);
    vp_pt_.z = VR.idx_viewpoints_.find(prune_idx)->second(2);
    vps_tree_.radiusSearch(vp_pt_, bubble_radius, index_set, radius_set);

    // The viewpoint status is active and there are multiple viewpoints within the bubble radius.
    if (VR.idx_live_state_.find(prune_idx)->second == true && (int)index_set.size() > 1) {
      updatePoseGravitation(vp_cloud_, index_set, prune_idx);
    }
  }

  // 存之前的
  for (auto& pair : VR.idx_viewpoints_) {
    if (VR.idx_live_state_.find(pair.first)->second == true) {
      updateViewpointInfo(pair.second, pair.first);
    }
  }
  // 赋值之前的

  // 过滤看到点云少的
  PrunedViewpoint struct_vp;
  for (auto& pair : VR.idx_viewpoints_) {
    if (VR.idx_live_state_.find(pair.first)->second == true && VR.vps_voxcount_[pair.first] > 1) {
      struct_vp.vp_id = pair.first;
      struct_vp.pose = pair.second;
      struct_vp.iter_num = VR.all_iter_num_;
      struct_vp.global_id = (int)VR.final_vps_.size();
      VR.final_vps_.push_back(struct_vp);
    }
    updateSeenCells(pair.second.head(3), pair.second(3), pair.second(4));
  }
  // 重新updateViewpointInfo

  for (auto& final_vp : VR.final_vps_) final_vp.vox_count = VR.vps_voxcount_[final_vp.vp_id];
  ROS_INFO("final_vps num = %ld  raw_vps num = %d", VR.final_vps_.size(), VR.all_vp_num_);
}

void SurfaceCoverage::computeUncoveredViewpoints(
    const Vector3d& pt_pos, const Vector3d& pt_normal, vector<pcl::PointNormal>& unc_vps)
{
  unc_vps.clear();
  vector<int> normal_dir{ 1, -1 };
  for (auto dir : normal_dir) {
    for (double dist = candidate_rmin_,
                delta_d = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
         dist <= candidate_rmax_ + 1e-3; dist = dist + delta_d) {
      Vector3d updated_normal = pt_normal;
      double pitch = atan2(updated_normal.z(), sqrt(updated_normal.x() * updated_normal.x() +
                                                    updated_normal.y() * updated_normal.y())) *
                     180 / M_PI;
      double max_pitch = pitch_upper_;
      double min_pitch = pitch_lower_;
      if (pitch > max_pitch)
        pitch = max_pitch;
      else if (pitch < min_pitch)
        pitch = min_pitch;
      double horizontal_length =
          sqrt(updated_normal.x() * updated_normal.x() + updated_normal.y() * updated_normal.y());
      double updated_z = tan(pitch * M_PI / 180.0) * horizontal_length;
      updated_normal.z() = updated_z;
      updated_normal.normalize();

      Vector3d vp_pos = pt_pos + dir * dist * updated_normal;
      if (!edt_env_->sdf_map_->isInBox(vp_pos) ||
          edt_env_->sdf_map_->getInflateOccupancy(vp_pos) == 1 || isNearUnknown(vp_pos) ||
          edt_env_->sdf_map_->getOccupancy(vp_pos) != SDFMap::FREE)
        continue;

      pcl::PointNormal vp;
      vp.x = pt_pos.x() + dir * dist * updated_normal.x();
      vp.y = pt_pos.y() + dir * dist * updated_normal.y();
      vp.z = pt_pos.z() + dir * dist * updated_normal.z();
      vp.normal_x = -dir * updated_normal.x();
      vp.normal_y = -dir * updated_normal.y();
      vp.normal_z = -dir * updated_normal.z();

      Vector3d pos1 = vp_pos;
      Vector3d pos2 = pt_pos;
      raycaster_->input(pos1, pos2);
      Eigen::Vector3i idx, idx_cell;
      edt_env_->sdf_map_->posToIndex(pos2, idx_cell);
      while (raycaster_->nextId(idx)) {
        if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE)
          break;
      }
      vector<Eigen::Vector3i> nbrs;
      nbrs = allNeighbors(idx_cell);
      nbrs.push_back(idx_cell);
      for (auto nbr : nbrs) {
        if (idx == nbr) {
          unc_vps.push_back(vp);
          break;
        }
      }
    }
  }
}

void SurfaceCoverage::updateNewPts(const vector<Vector3d>& pts, vector<bool>& cover_state,
    vector<int>& cover_contrib_num, vector<int>& contrib_id)
{
  cover_state.clear();
  cover_contrib_num.clear();
  contrib_id.clear();
  cover_state.resize(pts.size(), false);
  cover_contrib_num.resize(pts.size(), 0);
  contrib_id.resize(pts.size(), -1);
  for (int i = 0; i < (int)pts.size(); i++) {
    Vector3d pt_pos = pts[i];
    for (auto& final_vp : VR.final_vps_) {
      if (cover_state[i])
        break;

      Vector3d vp_pos = final_vp.pose.head(3);
      double vp_pitch = final_vp.pose(3);
      double vp_yaw = final_vp.pose(4);
      percep_utils_->setPose_PY(vp_pos, vp_pitch, vp_yaw);
      if (!percep_utils_->insideFOV(pt_pos))
        continue;

      Vector3d pos1 = vp_pos;
      Vector3d pos2 = pt_pos;
      raycaster_->input(pos1, pos2);
      Eigen::Vector3i idx, idx_cell;
      edt_env_->sdf_map_->posToIndex(pos2, idx_cell);
      while (raycaster_->nextId(idx)) {
        if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE)
          break;
      }
      // TODO
      vector<Eigen::Vector3i> nbrs;
      nbrs = allNeighbors(idx_cell);
      nbrs.push_back(idx_cell);
      for (auto nbr : nbrs) {
        if (idx == nbr) {
          VR.vps_voxcount_[final_vp.vp_id] += 1;
          cover_state[i] = true;
          cover_contrib_num[i] = 1000000;
          contrib_id[i] = final_vp.vp_id;
          break;
        }
      }
    }
  }

  for (auto& final_vp : VR.final_vps_) {
    updateSeenCells(final_vp.pose.head(3), final_vp.pose(3), final_vp.pose(4));
    final_vp.vox_count = VR.vps_voxcount_[final_vp.vp_id];
  }
}

void SurfaceCoverage::updateViewpointInfo(Eigen::VectorXd& pose, const int& vp_id)
{
  vector<int> index_set;
  Vector3d pos = Vector3d(pose(0), pose(1), pose(2));
  double pitch = pose(3);
  double yaw = pose(4);
  percep_utils_->setPose_PY(pos, pitch, yaw);
  // 去看未覆盖点云
  for (int i = 0; i < (int)VR.uncovered_filtered_pt_ids_.size(); i++) {
    Vector3d pt = VR.all_filtered_pts_[VR.uncovered_filtered_pt_ids_[i]];
    if (percep_utils_->insideFOV(pt)) {
      index_set.push_back(VR.uncovered_filtered_pt_ids_[i]);
    }
  }

  int contribute_voxel_num = 0;
  map<int, Eigen::Vector3d> valid_vox;

  for (int i = 0; i < (int)index_set.size(); i++) {
    Vector3d pos1 = pos;
    Vector3d pos2 = VR.all_filtered_pts_[index_set[i]];
    Vector3i idx, idx_rev;

    raycaster_->input(pos1, pos2);
    Eigen::Vector3i idx_cell;
    edt_env_->sdf_map_->posToIndex(pos2, idx_cell);
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE) {
        break;
      }
    }
    vector<Eigen::Vector3i> nbrs;
    nbrs = allNeighbors(idx_cell);
    nbrs.push_back(idx_cell);
    for (auto nbr : nbrs) {
      if (idx == nbr) {
        contribute_voxel_num++;
        valid_vox[index_set[i]] = pos2;
        break;
      }
    }
  }

  if (contribute_voxel_num == 0)
    return;

  VR.vps_contri_num_[vp_id] = contribute_voxel_num;

  for (const auto& val_id_vox : valid_vox) {
    // 该点云第一次被覆盖
    if (VR.cover_state_all_[val_id_vox.first] == false) {
      VR.cover_state_all_[val_id_vox.first] = true;
      VR.cover_contrib_num_all_[val_id_vox.first] = contribute_voxel_num;
      VR.contrib_id_all_[val_id_vox.first] = vp_id;

      VR.vps_voxcount_[vp_id] += 1;
    }
    else {
      // 现在的贡献数比之前的高
      if (contribute_voxel_num > VR.cover_contrib_num_all_[val_id_vox.first]) {
        int before_vp_id = VR.contrib_id_all_[val_id_vox.first];
        // 这是只会更新这次迭代更新所新增加的点云（不会更新之前迭代更新的（可以理解成冻结了））
        if (before_vp_id >= VR.last_vps_num_) {
          VR.vps_voxcount_[before_vp_id] -= 1;

          VR.cover_contrib_num_all_[val_id_vox.first] = contribute_voxel_num;
          VR.contrib_id_all_[val_id_vox.first] = vp_id;
          VR.vps_voxcount_[vp_id] += 1;
        }
      }
    }
  }
}

void SurfaceCoverage::updatePoseGravitation(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const vector<int>& inner_ids, const int& cur_idx)
{
  // 设置为已查询，防止后续将自己送引力模型里
  VR.idx_query_state_.find(cur_idx)->second = true;

  double pitch_bound = 30 * M_PI / 180.0;
  double yaw_bound = 30 * M_PI / 180.0;

  Eigen::VectorXd cur_pose = VR.idx_viewpoints_.find(cur_idx)->second;
  Eigen::Vector3d cur_position = cur_pose.head(3);
  double cur_pitch = cur_pose(3);
  double cur_yaw = cur_pose(4);
  int cur_vox = VR.idx_ctrl_voxels_.find(cur_idx)->second;

  int index_finder;
  Eigen::Vector3d vp_finder;
  Eigen::VectorXd inner_pose;
  int inner_vox;
  double diff_yaw, diff_pitch;
  bool update_flag = false;
  vector<int> updated_indexer;
  for (auto id_ : inner_ids) {
    update_flag = false;
    vp_finder(0) = cloud->points[id_].x;
    vp_finder(1) = cloud->points[id_].y;
    vp_finder(2) = cloud->points[id_].z;

    if (VR.inverse_idx_.find(vp_finder) == VR.inverse_idx_.end()) {
      ROS_ERROR("[Coverage Manager][BUG] I can't find this viewpoint in vp_finder");
      ROS_ERROR("[Coverage Manager][BUG] id = %d vp_finder pos = (%f %f %f)", id_, vp_finder(0),
          vp_finder(1), vp_finder(2));
    }

    index_finder = VR.inverse_idx_.find(vp_finder)->second;
    inner_pose = VR.idx_viewpoints_.find(index_finder)->second;
    inner_vox = VR.idx_ctrl_voxels_.find(index_finder)->second;
    if (inner_vox > cur_vox)
      continue;
    else {
      diff_pitch = cur_pose(3) - inner_pose(3);
      wrapAngle(diff_pitch);
      diff_yaw = cur_pose(4) - inner_pose(4);
      wrapAngle(diff_yaw);
      if (fabs(diff_pitch) < pitch_bound && fabs(diff_yaw) < yaw_bound)
        update_flag = true;

      if (update_flag == true && VR.idx_query_state_.find(index_finder)->second == false &&
          VR.idx_live_state_.find(index_finder)->second == true) {
        VR.idx_live_state_.find(index_finder)->second = false;
        updated_indexer.push_back(index_finder);
      }
    }
  }

  /* gravitation-like model */
  Eigen::Vector3d updated_position = cur_position;
  double updated_pitch = cur_pitch;
  double updated_yaw = cur_yaw;
  if ((int)updated_indexer.size() > 0) {
    ROS_INFO("Really update position!!");
    for (auto ui_ : updated_indexer) {
      // Position model
      double grav_factor = (double)VR.idx_ctrl_voxels_.find(ui_)->second / (double)cur_vox;
      Eigen::Vector3d up_position = VR.idx_viewpoints_.find(ui_)->second.head(3);
      Eigen::Vector3d pull_vec = grav_factor * (up_position - cur_position);
      updated_position = updated_position + pull_vec;

      // Pitch model
      double up_pitch = VR.idx_viewpoints_.find(ui_)->second(3);
      double pitch_diff = abs(up_pitch - cur_pitch) > M_PI ?
                              (2 * M_PI - abs(up_pitch - cur_pitch)) :
                              abs(up_pitch - cur_pitch);
      int cal_flag_pitch = up_pitch - cur_pitch > 0 ? 1 : -1;
      int cal_dir_pitch = abs(up_pitch - cur_pitch) > M_PI ? -1 : 1;
      double pull_pitch = grav_factor * cal_flag_pitch * cal_dir_pitch * pitch_diff;
      updated_pitch = updated_pitch + pull_pitch;

      // Yaw model
      double up_yaw = VR.idx_viewpoints_.find(ui_)->second(4);
      double yaw_diff =
          abs(up_yaw - cur_yaw) > M_PI ? (2 * M_PI - abs(up_yaw - cur_yaw)) : abs(up_yaw - cur_yaw);
      int cal_flag_yaw = up_yaw - cur_yaw > 0 ? 1 : -1;
      int cal_dir_yaw = abs(up_yaw - cur_yaw) > M_PI ? -1 : 1;
      double pull_yaw = grav_factor * cal_flag_yaw * cal_dir_yaw * yaw_diff;
      updated_yaw = updated_yaw + pull_yaw;
    }
  }

  VR.idx_viewpoints_.find(cur_idx)->second(0) = updated_position(0);
  VR.idx_viewpoints_.find(cur_idx)->second(1) = updated_position(1);
  VR.idx_viewpoints_.find(cur_idx)->second(2) = updated_position(2);

  wrapAngle(updated_pitch);
  wrapAngle(updated_yaw);
  VR.idx_viewpoints_.find(cur_idx)->second(3) = updated_pitch;
  VR.idx_viewpoints_.find(cur_idx)->second(4) = updated_yaw;
}

/*****  Surface Utils  *****/
bool SurfaceCoverage::isSurfaceChanged(const Surface& sf)
{
  for (auto cell : sf.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!satisfySurfaceCell(idx))
      return true;
  }
  return false;
}

int SurfaceCoverage::countSurfaceDecreaseNum(const Surface& sf)
{
  int count = 0;
  int all_frt_num = sf.cells_.size();
  for (auto cell : sf.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!satisfySurfaceCell(idx))
      count++;
    if (count > all_frt_num * sf_decrease_percentage_max_ + 1)
      break;
  }
  return count;
}

int SurfaceCoverage::countSurfaceIncreaseNum(const Surface& sf)
{
  int count = 0;
  int all_frt_num = sf.cells_.size();
  for (auto cell : sf.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    queue<Eigen::Vector3i> cell_queue;
    vector<Eigen::Vector3i> expanded;
    cell_queue.push(idx);
    while (!cell_queue.empty()) {
      auto cur = cell_queue.front();
      cell_queue.pop();
      auto nbrs = allNeighbors(cur);
      for (auto nbr : nbrs) {
        // Qualified cell should be inside bounding box and surface cell not clustered
        int adr = toadr(nbr);
        if (surface_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
            !satisfySurfaceCell(nbr))
          continue;

        count++;
        expanded.push_back(nbr);
        cell_queue.push(nbr);
        surface_flag_[adr] = 1;

        if (count > all_frt_num * sf_increase_percentage_max_ + 1)
          break;
      }
      if (count > all_frt_num * sf_increase_percentage_max_ + 1)
        break;
    }
    // Backtrace
    for (auto expanded_cell : expanded) surface_flag_[toadr(expanded_cell)] = 0;
  }
  return count;
}

void SurfaceCoverage::computeSurfaceInfo(Surface& surface)
{
  // Compute average position and bounding box of cluster
  surface.average_.setZero();
  surface.box_max_ = surface.cells_.front();
  surface.box_min_ = surface.cells_.front();
  surface.state_ = true;  // All surfaces initialize to active
  for (auto cell : surface.cells_) {
    surface.average_ += cell;
    for (int i = 0; i < 3; ++i) {
      surface.box_min_[i] = min(surface.box_min_[i], cell[i]);
      surface.box_max_[i] = max(surface.box_max_[i], cell[i]);
    }
  }
  surface.average_ /= double(surface.cells_.size());

  // Compute downsampled cluster
  downsample(surface.cells_, surface.filtered_cells_);
}

int SurfaceCoverage::countVisibleSurfaceCells(const Eigen::Vector3d& pos, const double& pitch,
    const double& yaw, const vector<Eigen::Vector3d>& cluster, vector<Eigen::Vector3d>& visib_cells)
{
  visib_cells.clear();
  percep_utils_->setPose_PY(pos, pitch, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if surface cell is inside FOV
    if (!percep_utils_->insideFOV(cell))
      continue;

    // Check if surface cell is visible (not occulded by obstacles)
    Vector3i idx_cell;
    edt_env_->sdf_map_->posToIndex(cell, idx_cell);
    raycaster_->input(pos, cell);
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE)
        break;
    }
    if (idx == idx_cell) {
      visib_num += 1;
      visib_cells.push_back(cell);
    }
  }
  return visib_num;
}

int SurfaceCoverage::countVisibleOccupiedCells(const Vector3d& pos, const double& pitch,
    const double& yaw, vector<Eigen::Vector3d>& visib_cells)
{
  int count = 0;
  visib_cells.clear();
  percep_utils_->setPose_PY(pos, pitch, yaw);
  Eigen::Vector3i idx;
  Vector3i box_min, box_max;
  getFovBox(pos, pitch, yaw, box_min, box_max);
  // Traverse all grids in FOV
  for (int x = box_min[0]; x <= box_max[0]; x++)
    for (int y = box_min[1]; y <= box_max[1]; y++)
      for (int z = box_min[2]; z <= box_max[2]; z++) {
        Vector3i idx_cell = Vector3i(x, y, z);
        Vector3d cell;
        edt_env_->sdf_map_->indexToPos(idx_cell, cell);
        // Only look for occupied grids within the FOV
        if (edt_env_->sdf_map_->getOccupancy(cell) != SDFMap::OCCUPIED ||
            !percep_utils_->insideFOV(cell))
          continue;

        raycaster_->input(pos, cell);
        while (raycaster_->nextId(idx)) {
          if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED ||
              edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
            break;
        }
        // It can raycast the occupied grid
        if (idx == idx_cell) {
          visib_cells.push_back(cell);
          count++;
        }
      }
  return count;
}

bool SurfaceCoverage::satisfySurfaceCell(const Eigen::Vector3i& idx)
{
  if (knownOccupied(idx) && isNeighborFree(idx, true))
    return true;
  return false;
}

bool SurfaceCoverage::satisfySurfaceFrontierCell(const Eigen::Vector3i& idx)
{
  if (knownFree(idx) && isNeighborUnknown(idx) && isNeighborOccupied(idx))
    return true;
  return false;
}

/*****  Get Information (Public)  *****/
void SurfaceCoverage::getSurfaces(vector<vector<Eigen::Vector3d>>& clusters)
{
  clusters.clear();
  for (auto surface : surfaces_) clusters.push_back(surface.cells_);
}

void SurfaceCoverage::getDormantSurfaces(vector<vector<Eigen::Vector3d>>& clusters)
{
  clusters.clear();
  for (auto dormant_surface : dormant_surfaces_) clusters.push_back(dormant_surface.cells_);
}

void SurfaceCoverage::getSurfacesState(vector<char>& states)
{
  states.clear();
  for (auto surface : surfaces_) states.push_back(surface.state_);
}

void SurfaceCoverage::getSurfaceFrontiers(vector<vector<Eigen::Vector3d>>& clusters)
{
  clusters.clear();
  for (auto surface_frontier : surface_frontiers_) clusters.push_back(surface_frontier);
}

void SurfaceCoverage::getFinalViewpoints(vector<VectorXd>& viewpoints, vector<int>& viewpoint_ids,
    vector<int>& iter_nums, vector<int>& counts)
{
  viewpoints.clear();
  iter_nums.clear();
  counts.clear();
  viewpoint_ids.clear();
  for (auto vp : VR.final_vps_) {
    viewpoints.push_back(vp.pose);
    viewpoint_ids.push_back(vp.global_id);
    counts.push_back(vp.vox_count);
    iter_nums.push_back(vp.iter_num);
  }
}

void SurfaceCoverage::getTopSurfaceViewpointsInfo(
    vector<Eigen::Vector3d>& points, vector<double>& yaws)
{
  points.clear();
  yaws.clear();
  for (auto surface : surfaces_) {
    for (auto view : surface.viewpoints_) {
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      break;
    }
  }
}

void SurfaceCoverage::getAllSurfaceViewpointsInfo(
    vector<Eigen::Vector3d>& points, vector<double>& yaws, vector<double>& pitchs)
{
  points.clear();
  yaws.clear();
  pitchs.clear();
  for (auto surface : surfaces_) {
    for (auto view : surface.viewpoints_) {
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      pitchs.push_back(view.pitch_);
    }
  }
}

void SurfaceCoverage::getFullViewpointsDronesCostMatrix(
    const vector<VectorXd>& viewpoints, const vector<VectorXd>& drones_pos, Eigen::MatrixXd& mat)
{
  // [Virtual depot + Drones + Viewpoints] Cost Matrix (MTSP)
  int vp_num = viewpoints.size();
  int drone_num = drones_pos.size();
  int dimen = vp_num + drone_num + 1;
  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = 0;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to viewpoints
  for (int i = 0; i < vp_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }

  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to surfaces
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < vp_num; j++) {
      double cost = ViewNode::computeCostPY(drones_pos[i], viewpoints[j]);
      mat(1 + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + i) = 10000;
    }
  }

  // Costs between surfaces
  for (int i = 0; i < vp_num; i++) {
    for (int j = i + 1; j < vp_num; j++) {
      double cost = ViewNode::computeCostPY(viewpoints[i], viewpoints[j]);
      mat(1 + drone_num + i, 1 + drone_num + j) = cost;
      mat(1 + drone_num + j, 1 + drone_num + i) = cost;
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 10000;
  }
}

void SurfaceCoverage::getFullViewpointsCostMatrix(
    const VectorXd& drone_pose, const vector<VectorXd>& viewpoints, Eigen::MatrixXd& mat)
{
  // [Drone + Viewpoints] Cost Matrix (ATSP)
  int dimen = viewpoints.size() + 1;
  mat.resize(dimen, dimen);

  // drone to viewpoints
  for (int i = 1; i < dimen; i++) {
    mat(0, i) = ViewNode::computeCostPY(drone_pose, viewpoints[i - 1]);
    mat(i, 0) = 0;
  }

  // Costs between viewpoints
  for (int i = 1; i < dimen; ++i) {
    for (int j = i + 1; j < dimen; ++j) {
      double cost = ViewNode::computeCostPY(viewpoints[i - 1], viewpoints[j - 1]);
      mat(i, j) = cost;
      mat(j, i) = cost;
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 10000;
  }
}

/*****  Ikdtree Build (All Map)  *****/
void SurfaceCoverage::lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  PointVector pointCloud;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  if (cloud.points.size() == 0) {
    ROS_ERROR_THROTTLE(1.0, "Explorer have no lidar cloud input!!!!");
    return;
  }

  for (const auto& point : cloud.points) {
    pointCloud.push_back(point);
  }
  // init build ikdtree
  if (ikd_tree_.Root_Node == nullptr) {
    ikd_tree_.Build(pointCloud);
    ROS_WARN("[IkdTree] build num = %d", ikd_tree_.size());
  }
  // ikdtree add points
  else {
    ikd_tree_.Add_Points(pointCloud, true);  // downsample = true
    ROS_WARN_THROTTLE(5.0, "[IkdTree] add points all num = %d valid num = %d", ikd_tree_.size(),
        ikd_tree_.validnum());
  }
}

/*****  Visualization  *****/
void SurfaceCoverage::visCallback(const ros::TimerEvent& e)
{
  if (vis_flag_) {
    publishIkdtree();
    publishClusterKdtree();
    // publish raycast results
    VR.pub_seen_cloud.width = VR.pub_seen_cloud.points.size();
    VR.pub_seen_cloud.height = 1;
    VR.pub_seen_cloud.is_dense = true;
    VR.pub_seen_cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(VR.pub_seen_cloud, cloud_msg);
    cloud_pub_.publish(cloud_msg);
  }
}

void SurfaceCoverage::publishIkdtree()
{
  PointVector ikdtree_points;
  ikd_tree_.flatten(ikd_tree_.Root_Node, ikdtree_points, NOT_RECORD);
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (auto& point : ikdtree_points) {
    pcl::PointXYZ pcl_point;
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_cloud.push_back(pcl_point);
  }
  sensor_msgs::PointCloud2 ros_cloud;
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  pcl_cloud.header.frame_id = frame_id_;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  ikdtree_cloud_pub_.publish(ros_cloud);
}

void SurfaceCoverage::publishClusterKdtree()
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  for (auto surface : surfaces_) {
    for (auto& point : surface.pts_) {
      pcl::PointXYZ pcl_point;
      pcl_point.x = point(0);
      pcl_point.y = point(1);
      pcl_point.z = point(2);
      pcl_cloud.push_back(pcl_point);
    }
  }
  sensor_msgs::PointCloud2 ros_cloud;
  pcl_cloud.width = pcl_cloud.points.size();
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;
  pcl_cloud.header.frame_id = frame_id_;
  pcl::toROSMsg(pcl_cloud, ros_cloud);
  cluster_kdtree_cloud_pub_.publish(ros_cloud);
}

void SurfaceCoverage::setSurfaceFrontiers(vector<vector<Vector3d>>& clusters)
{
  // 清除上次的surface frontier
  for (auto surface_frontier_cells : surface_frontiers_) {
    for (auto cell : surface_frontier_cells) {
      Eigen::Vector3i idx;
      edt_env_->sdf_map_->posToIndex(cell, idx);
      surface_frontier_flag_[toadr(idx)] = 0;
    }
  }
  surface_frontiers_.clear();
  // 更新surface frontier
  for (auto cluster : clusters) {
    surface_frontiers_.push_back(cluster);
    // 更新surface_frontier_flag_
    for (auto cell : cluster) {
      Eigen::Vector3i idx;
      edt_env_->sdf_map_->posToIndex(cell, idx);
      surface_frontier_flag_[toadr(idx)] = 1;
    }
  }
}

void SurfaceCoverage::updateSeenCells(const Vector3d& pos, const double& pitch, const double& yaw)
{
  percep_utils_->setPose_PY(pos, pitch, yaw);
  Eigen::Vector3d bmin, bmax;
  Vector3i bbmin, bbmax;
  percep_utils_->getFOVBoundingBox(bmin, bmax);
  edt_env_->sdf_map_->posToIndex(bmin, bbmin);
  edt_env_->sdf_map_->posToIndex(bmax, bbmax);

  for (int x = bbmin(0); x <= bbmax(0); x++)
    for (int y = bbmin(1); y <= bbmax(1); y++)
      for (int z = bbmin(2); z <= bbmax(2); z++) {
        Vector3i idxx(x, y, z);
        Vector3d ppos;
        edt_env_->sdf_map_->indexToPos(idxx, ppos);
        if (edt_env_->sdf_map_->getGlobalOccupancy(idxx) != SDFMap::OCCUPIED)
          continue;
        if (!percep_utils_->insideFOV(ppos))
          continue;

        Vector3d pos1 = pos;
        Vector3d pos2 = ppos;
        raycaster_->input(pos1, pos2);
        Eigen::Vector3i idx, idx_cell;
        edt_env_->sdf_map_->posToIndex(pos2, idx_cell);
        while (raycaster_->nextId(idx)) {
          if (edt_env_->sdf_map_->getGlobalOccupancy(idx) != SDFMap::FREE)
            break;
        }
        if (idx == idx_cell) {
          edt_env_->sdf_map_->setSeen(idx);
        }
      }
}

}  // namespace hetero_planner