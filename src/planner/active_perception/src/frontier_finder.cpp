#include <active_perception/frontier_finder.h>

namespace hetero_planner {
FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh)
{
  this->edt_env_ = edt;
  int voxel_num = edt->sdf_map_->getVoxelNum();
  frontier_flag_ = vector<char>(voxel_num, 0);
  fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

  nh.param("frontier/cluster_min", cluster_min_, -1);
  nh.param("frontier/cluster_size_xyz", cluster_size_xyz_, -1.0);
  nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
  nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
  nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
  nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
  nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
  nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
  nh.param("frontier/candidate_hnum", candidate_hnum_, -1);
  nh.param("frontier/candidate_hmax", candidate_hmax_, -1.0);
  nh.param("frontier/down_sample", down_sample_, -1);
  nh.param("frontier/min_visib_num", min_visib_num_, -1);
  nh.param("frontier/min_view_finish_fraction", min_view_finish_fraction_, -1.0);
  nh.param("frontier/is_lidar", is_lidar_, false);  // true: explorer  false: photographer

  nh.param("perception_utils/lidar_top_angle", lidar_fov_up_, -1.0);
  nh.param("perception_utils/lidar_bottom_angle", lidar_fov_down_, -1.0);
  nh.param("perception_utils/lidar_max_dist", lidar_max_dist_, -1.0);
  nh.param("perception_utils/lidar_pitch", lidar_pitch_, 0.0);

  raycaster_.reset(new RayCaster);
  resolution_ = edt_env_->sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  edt_env_->sdf_map_->getRegion(origin, size);
  raycaster_->setParams(resolution_, origin);

  percep_utils_.reset(new PerceptionUtils(nh, 0));
}

FrontierFinder::~FrontierFinder()
{
}

void FrontierFinder::searchFrontiers()
{
  ros::Time t1 = ros::Time::now();
  tmp_frontiers_.clear();

  // Bounding box of updated region
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);

  // Removed changed frontiers in updated map
  auto resetFlag = [&](list<Frontier>::iterator& iter, list<Frontier>& frontiers) {
    Eigen::Vector3i idx;
    for (auto cell : iter->cells_) {
      edt_env_->sdf_map_->posToIndex(cell, idx);
      frontier_flag_[toadr(idx)] = 0;
    }
    iter = frontiers.erase(iter);
  };

  std::cout << "Before remove: " << frontiers_.size() << std::endl;

  removed_ids_.clear();
  int rmv_idx = 0;
  for (auto iter = frontiers_.begin(); iter != frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter)) {
      resetFlag(iter, frontiers_);
      removed_ids_.push_back(rmv_idx);
    }
    else {
      ++rmv_idx;
      ++iter;
    }
  }
  std::cout << "After remove: " << frontiers_.size() << std::endl;
  for (auto iter = dormant_frontiers_.begin(); iter != dormant_frontiers_.end();) {
    if (haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max) &&
        isFrontierChanged(*iter))
      resetFlag(iter, dormant_frontiers_);
    else
      ++iter;
  }

  // Search new frontier within box slightly inflated from updated box
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
        // Scanning the updated region to find seeds of frontiers
        Eigen::Vector3i cur(x, y, z);
        if (is_lidar_) {
          if (frontier_flag_[toadr(cur)] == 0 && satisfyFrontierCell(cur)) {
            // Expand from the seed cell to find a complete frontier cluster
            expandFrontier(cur);
          }
        }
      }
  splitLargeFrontiers(tmp_frontiers_);

  ROS_WARN_THROTTLE(5.0, "Frontier t: %lf", (ros::Time::now() - t1).toSec());
}

void FrontierFinder::expandFrontier(const Eigen::Vector3i& first)
{
  // Data for clustering
  queue<Eigen::Vector3i> cell_queue;
  vector<Eigen::Vector3d> expanded;
  Vector3d pos;

  edt_env_->sdf_map_->indexToPos(first, pos);
  expanded.push_back(pos);
  cell_queue.push(first);
  frontier_flag_[toadr(first)] = 1;

  // Search frontier cluster based on region growing (distance clustering)
  while (!cell_queue.empty()) {
    auto cur = cell_queue.front();
    cell_queue.pop();
    auto nbrs = allNeighbors(cur);
    for (auto nbr : nbrs) {
      // Qualified cell should be inside bounding box and frontier cell not clustered
      int adr = toadr(nbr);
      if (frontier_flag_[adr] == 1 || !edt_env_->sdf_map_->isInBox(nbr) ||
          !satisfyFrontierCell(nbr))
        continue;

      edt_env_->sdf_map_->indexToPos(nbr, pos);
      if (pos[2] < 0.5)
        continue;  // Remove noise close to ground
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    }
  }
  if ((int)expanded.size() > cluster_min_) {
    // Compute detailed info
    Frontier frontier;
    frontier.cells_ = expanded;
    computeFrontierInfo(frontier);
    tmp_frontiers_.push_back(frontier);
  }
  // 回溯
  else {
    for (auto cell : expanded) {
      Vector3i cell_idx;
      edt_env_->sdf_map_->posToIndex(cell, cell_idx);
      frontier_flag_[toadr(cell_idx)] = 0;
    }
  }
}

void FrontierFinder::splitLargeFrontiers(list<Frontier>& frontiers)
{
  list<Frontier> splits, tmps;
  for (auto it = frontiers.begin(); it != frontiers.end(); ++it) {
    // Check if each frontier needs to be split horizontally
    if (splitFrontierIn3D(*it, splits)) {
      tmps.insert(tmps.end(), splits.begin(), splits.end());
      splits.clear();
    }
    else
      tmps.push_back(*it);
  }
  frontiers = tmps;
}

bool FrontierFinder::splitFrontierIn3D(const Frontier& frontier, list<Frontier>& splits)
{
  auto mean = frontier.average_;
  bool need_split = false;
  for (auto cell : frontier.filtered_cells_) {
    if ((cell - mean).norm() > cluster_size_xyz_) {
      need_split = true;
      break;
    }
  }
  if (!need_split)
    return false;

  // Compute covariance matrix of cells
  Eigen::Matrix3d cov;
  cov.setZero();
  for (auto cell : frontier.filtered_cells_) {
    Eigen::Vector3d diff = cell - mean;
    cov += diff * diff.transpose();
  }
  cov /= double(frontier.filtered_cells_.size());

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

  // Split the frontier into two groups along the first PC
  Frontier ftr1, ftr2;
  for (auto cell : frontier.cells_) {
    if ((cell - mean).dot(first_pc) >= 0)
      ftr1.cells_.push_back(cell);
    else
      ftr2.cells_.push_back(cell);
  }
  computeFrontierInfo(ftr1);
  computeFrontierInfo(ftr2);

  // Recursive call to split frontier that is still too large
  list<Frontier> splits2;
  if (splitFrontierIn3D(ftr1, splits2)) {
    splits.insert(splits.end(), splits2.begin(), splits2.end());
    splits2.clear();
  }
  else
    splits.push_back(ftr1);

  if (splitFrontierIn3D(ftr2, splits2))
    splits.insert(splits.end(), splits2.begin(), splits2.end());
  else
    splits.push_back(ftr2);

  return true;
}

void FrontierFinder::updateFrontierCostMatrix()
{
  std::cout << "cost mat size before remove: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;

  std::cout << "cost mat size remove: " << std::endl;
  if (!removed_ids_.empty()) {
    // Delete path and cost for removed clusters
    for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it) {
      auto cost_iter = it->costs_.begin();
      auto path_iter = it->paths_.begin();
      int iter_idx = 0;
      for (size_t i = 0; i < removed_ids_.size(); ++i) {
        // Step iterator to the item to be removed
        while (iter_idx < removed_ids_[i]) {
          ++cost_iter;
          ++path_iter;
          ++iter_idx;
        }
        cost_iter = it->costs_.erase(cost_iter);
        path_iter = it->paths_.erase(path_iter);
      }
      std::cout << "(" << it->costs_.size() << "," << it->paths_.size() << "), ";
    }
    removed_ids_.clear();
  }
  std::cout << "" << std::endl;

  auto updateCost = [](const list<Frontier>::iterator& it1, const list<Frontier>::iterator& it2) {
    std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
    // Search path from old cluster's top viewpoint to new cluster'
    Viewpoint& vui = it1->viewpoints_.front();
    Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    double cost_ij = ViewNode::computeCost(
        vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
    // Insert item for both old and new clusters
    it1->costs_.push_back(cost_ij);
    it1->paths_.push_back(path_ij);
    reverse(path_ij.begin(), path_ij.end());
    it2->costs_.push_back(cost_ij);
    it2->paths_.push_back(path_ij);
  };

  std::cout << "cost mat add: " << std::endl;
  // Compute path and cost between old and new clusters
  for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
    for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2) updateCost(it1, it2);

  // Compute path and cost between new clusters
  for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
    for (auto it2 = it1; it2 != frontiers_.end(); ++it2) {
      if (it1 == it2) {
        std::cout << "(" << it1->id_ << "," << it2->id_ << "), ";
        it1->costs_.push_back(0);
        it1->paths_.push_back({});
      }
      else
        updateCost(it1, it2);
    }
  std::cout << "" << std::endl;
  std::cout << "cost mat size final: " << std::endl;
  for (auto ftr : frontiers_)
    std::cout << "(" << ftr.costs_.size() << "," << ftr.paths_.size() << "), ";
  std::cout << "" << std::endl;
}

bool FrontierFinder::isFrontierChanged(const Frontier& ft)
{
  for (auto cell : ft.cells_) {
    Eigen::Vector3i idx;
    edt_env_->sdf_map_->posToIndex(cell, idx);
    if (!satisfyFrontierCell(idx))
      return true;
  }
  return false;
}

void FrontierFinder::computeFrontierInfo(Frontier& ftr)
{
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

void FrontierFinder::computeFrontiersToVisit()
{
  first_new_ftr_ = frontiers_.end();
  int new_num = 0;
  int new_dormant_num = 0;
  // Try find viewpoints for each cluster and categorize them according to viewpoint number
  for (auto& tmp_ftr : tmp_frontiers_) {
    // Search viewpoints around frontier
    sampleViewpoints(tmp_ftr);
    if (!tmp_ftr.viewpoints_.empty()) {
      ++new_num;
      list<Frontier>::iterator inserted = frontiers_.insert(frontiers_.end(), tmp_ftr);
      // Sort the viewpoints by coverage fraction, best view in front
      sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(),
          [](const Viewpoint& v1, const Viewpoint& v2) { return v1.visib_num_ > v2.visib_num_; });
      if (first_new_ftr_ == frontiers_.end())
        first_new_ftr_ = inserted;
    }
    else {
      // Find no viewpoint, move cluster to dormant list
      dormant_frontiers_.push_back(tmp_ftr);
      ++new_dormant_num;
    }
  }
  // Reset indices of frontiers
  int idx = 0;
  for (auto& sf : frontiers_) {
    sf.id_ = idx++;
    std::cout << sf.id_ << ", ";
  }
  std::cout << "\nnew num: " << new_num << ", new dormant: " << new_dormant_num << std::endl;
  std::cout << "to visit: " << frontiers_.size() << ", dormant: " << dormant_frontiers_.size()
            << std::endl;
}

// Sample viewpoints around frontier's average position, check coverage to the frontier cells
void FrontierFinder::sampleViewpoints(Frontier& frontier)
{
  // Evaluate sample viewpoints on circles, find ones that cover most cells
  for (double rc = candidate_rmin_,
              dr = (candidate_rmax_ - candidate_rmin_) / (double)candidate_rnum_;
       rc <= candidate_rmax_ + 1e-3; rc += dr)
    for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_) {
      for (double h = -1, dh = (candidate_hmax_ + 1) / (double)candidate_hnum_;
           h <= candidate_hmax_ + 1e-3; h += dh) {
        const Vector3d sample_pos =
            frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0) + Vector3d(0, 0, h);

        // Qualified viewpoint is in bounding box and in safe region
        if (!edt_env_->sdf_map_->isInBox(sample_pos) ||
            edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 || isNearUnknown(sample_pos))
          continue;

        // Compute average yaw
        auto& cells = frontier.filtered_cells_;
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
        auto& raw_cells = frontier.cells_;
        vector<Vector3d> visib_cells;
        // Compute the fraction of covered and visible cells
        int visib_num = countVisibleFrontierCells(sample_pos, avg_yaw, raw_cells, visib_cells);
        if (visib_num > min_visib_num_) {
          Viewpoint vp = { sample_pos, avg_yaw, visib_num };
          frontier.viewpoints_.push_back(vp);
        }
      }
    }
}

void FrontierFinder::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages)
{
  points.clear();
  yaws.clear();
  averages.clear();
  for (auto frontier : frontiers_) {
    bool no_view = true;
    for (auto view : frontier.viewpoints_) {
      // Retrieve the first viewpoint that is far enough and has highest coverage
      if ((view.pos_ - cur_pos).norm() < min_candidate_dist_)
        continue;
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
      no_view = false;
      break;
    }
    if (no_view) {
      // All viewpoints are very close, just use the first one (with highest coverage).
      auto view = frontier.viewpoints_.front();
      points.push_back(view.pos_);
      yaws.push_back(view.yaw_);
      averages.push_back(frontier.average_);
    }
  }
}

void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>>& clusters)
{
  clusters.clear();
  for (auto frontier : frontiers_) clusters.push_back(frontier.cells_);
}

void FrontierFinder::getDormantFrontiers(vector<vector<Vector3d>>& clusters)
{
  clusters.clear();
  for (auto sf : dormant_frontiers_) clusters.push_back(sf.cells_);
}

void FrontierFinder::getFrontierBoxes(vector<pair<Eigen::Vector3d, Eigen::Vector3d>>& boxes)
{
  boxes.clear();
  for (auto frontier : frontiers_) {
    Vector3d center = (frontier.box_max_ + frontier.box_min_) * 0.5;
    Vector3d scale = frontier.box_max_ - frontier.box_min_;
    boxes.push_back(make_pair(center, scale));
  }
}

void FrontierFinder::getPathForTour(
    const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path)
{
  path.clear();
  // Make an frontier_indexer to access the frontier list easier
  vector<list<Frontier>::iterator> frontier_indexer;
  for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it) frontier_indexer.push_back(it);

  // Compute the path from current pos to the first frontier
  vector<Vector3d> segment;
  ViewNode::searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
  path.insert(path.end(), segment.begin(), segment.end());

  // Get paths of tour passing all clusters
  for (size_t i = 0; i < frontier_ids.size() - 1; ++i) {
    // Move to path to next cluster
    auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
    int next_idx = frontier_ids[i + 1];
    for (int j = 0; j < next_idx; ++j) ++path_iter;
    path.insert(path.end(), path_iter->begin(), path_iter->end());
  }
}

void FrontierFinder::getFullCostMatrix(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, Eigen::MatrixXd& mat)
{
  // Use Asymmetric TSP
  int dimen = frontiers_.size();
  mat.resize(dimen + 1, dimen + 1);
  // std::cout << "mat size: " << mat.rows() << ", " << mat.cols() << std::endl;
  // Fill block for clusters
  int i = 1, j = 1;
  for (auto ftr : frontiers_) {
    for (auto cs : ftr.costs_) {
      // std::cout << "(" << i << ", " << j << ")"
      // << ", ";
      mat(i, j++) = cs;
    }
    ++i;
    j = 1;
  }
  // std::cout << "" << std::endl;

  // Fill block from current state to clusters
  mat.leftCols<1>().setZero();
  for (auto ftr : frontiers_) {
    // std::cout << "(0, " << j << ")"
    // << ", ";
    Viewpoint vj = ftr.viewpoints_.front();
    vector<Vector3d> path;
    mat(0, j++) =
        ViewNode::computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
  }
  // std::cout << "" << std::endl;
}

bool FrontierFinder::isFrontierCovered()
{
  Vector3d update_min, update_max;
  edt_env_->sdf_map_->getUpdatedBox(update_min, update_max);

  auto checkChanges = [&](const list<Frontier>& frontiers) {
    for (auto ftr : frontiers) {
      if (!haveOverlap(ftr.box_min_, ftr.box_max_, update_min, update_max))
        continue;
      const int change_thresh = min_view_finish_fraction_ * ftr.cells_.size();
      int change_num = 0;
      for (auto cell : ftr.cells_) {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownFree(idx) && isNeighborUnknown(idx)) && ++change_num >= change_thresh)
          return true;
      }
    }
    return false;
  };

  if (checkChanges(frontiers_) || checkChanges(dormant_frontiers_))
    return true;

  return false;
}

int FrontierFinder::countVisibleFrontierCells(const Eigen::Vector3d& pos, const double& yaw,
    const vector<Eigen::Vector3d>& cluster, vector<Eigen::Vector3d>& visib_cells)
{
  visib_cells.clear();
  percep_utils_->setPose(pos, yaw);
  int visib_num = 0;
  Eigen::Vector3i idx;
  for (auto cell : cluster) {
    // Check if frontier cell is inside FOV
    if (!isExplorer()) {
      if (!percep_utils_->insideFOV(cell))
        continue;
    }
    else {
      if (!isInLidarFOV(pos, yaw, cell))
        continue;
    }

    // Check if frontier cell is visible (not occulded by obstacles)
    raycaster_->input(cell, pos);
    bool visib = true;
    while (raycaster_->nextId(idx)) {
      if (edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED ||
          edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
        visib = false;
        break;
      }
    }
    if (visib) {
      visib_num += 1;
      visib_cells.push_back(cell);
    }
  }
  return visib_num;
}

bool FrontierFinder::isExplorer()
{
  return is_lidar_;
}

bool FrontierFinder::satisfyFrontierCell(const Eigen::Vector3i& idx)
{
  if (knownFree(idx) && isNeighborUnknown(idx) && isNeighborOccupied(idx))
    return true;
  return false;

  // if (knownFree(idx))
  // {
  //   auto nbrs = eighteenNeighbors(idx);
  //   vector<Vector3i> occ_cells, unknown_cells;
  //   for (auto nbr : nbrs)
  //   {
  //     if (knownOccupied(nbr))
  //       occ_cells.push_back(nbr);
  //     if (knownUnknown(nbr))
  //       unknown_cells.push_back(nbr);
  //   }
  //   if (occ_cells.size() == 0 || unknown_cells.size() == 0)
  //     return false;
  //   for (auto occ_cell : occ_cells)
  //   {
  //     auto occ_nbrs = sixNeighbors(occ_cell);
  //     for (auto occ_nbr : occ_nbrs)
  //     {
  //       for (auto unknown_cell : unknown_cells)
  //       {
  //         if (occ_nbr == unknown_cell)
  //         {
  //           return true;
  //         }
  //       }
  //     }
  //   }
  // }
  // return false;
}

bool FrontierFinder::isInLidarFOV(
    const Eigen::Vector3d& vp_pos, const double& vp_yaw, const Vector3d& frt_cell)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // transform.rotate(Eigen::AngleAxisd(-lidar_roll_, Eigen::Vector3d::UnitX()));
  transform.rotate(Eigen::AngleAxisd(-lidar_pitch_, Eigen::Vector3d::UnitY()));
  transform.rotate(Eigen::AngleAxisd(-vp_yaw, Eigen::Vector3d::UnitZ()));

  if ((vp_pos - frt_cell).norm() > lidar_max_dist_)
    return false;
  Eigen::Vector3d pt2see = transform * (frt_cell - vp_pos);
  // pitch
  float pitch = atan2(pt2see.z(), sqrt(pt2see.x() * pt2see.x() + pt2see.y() * pt2see.y()));
  if (pitch > lidar_fov_up_ || pitch < -lidar_fov_down_)
    return false;

  return true;
}
}  // namespace hetero_planner