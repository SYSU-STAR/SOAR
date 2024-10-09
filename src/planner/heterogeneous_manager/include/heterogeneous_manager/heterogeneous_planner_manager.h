#ifndef _HETEROGENEOUS_PALNNER_MANAGER_H_
#define _HETEROGENEOUS_PALNNER_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <heterogeneous_manager/coverage_manager.h>
#include <traj_manager/traj_generator.h>
#include <active_perception/frontier_finder.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace std {
template <>
struct hash<Eigen::VectorXd> {
  size_t operator()(const Eigen::VectorXd& vec) const
  {
    return std::hash<double>()(vec[0]) ^ std::hash<double>()(vec[1]) ^ std::hash<double>()(vec[2]) ^
           std::hash<double>()(vec[3]) ^ std::hash<double>()(vec[4]);
  }
};
}  // namespace std
namespace hetero_planner {
class EDTEnvironment;
class SDFMap;
class FrontierFinder;
class SurfaceCoverage;
struct HeteroParam;
struct HeteroData;
struct ViewpointsTask;

// TODO 这些结果足够吗
enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, NO_RESULT };

struct Site {
  Site* Pred;
  Site* Suc;
  bool Start;
  bool End;
  Vector3d Pos;
  double Pitch;
  double Yaw;
  int GlobalID;
  int LocalID;
};

struct PathSite {
  vector<Site*> sites_;
  Site* start_site_;
  Site* end_site_;
};

// photographer
struct ViewpointTourResult {
  int vp_num_ = 0;
  bool init_tour_ = false;
  PathSite final_path_site_;
};

struct ViewpointCluster {
  int cluster_id_;
  Vector3d average_;
  double cost_;
  vector<VectorXd> viewpoints_;
};

struct ClusterNode {
  ViewpointCluster start_cluster_;
  ViewpointCluster end_cluster_;
  vector<int> cluster_ids_;
  double cost_;
  bool is_valid_;  // 如果根本没有boundary_cluster 则为false
  bool is_none_;   // 如果就是boundary包含全任务，则为true
};

// explorer
struct ViewpointAssignmentResult {
  double cluster_h_cost_;
  bool init_assignment_ = false;
  int last_cluster_num_ = 0;
  unordered_map<VectorXd, bool> vp_cluster_flag_;
  unordered_map<VectorXd, int> vp_cluster_finder_;  // vp在那个vp_cluster中
  vector<ViewpointCluster> vp_clusters_;
  int cluster_num_;
  MatrixXd cluster_matrix_;

  vector<vector<int>> drone_assigned_cluster_ids_;  // 每个飞机的任务（cluster的id）
  unordered_map<int, int> cluster_drone_finder_;    // 这个类归属哪个飞机
};

class HeterogenousPlannerManager {
public:
  HeterogenousPlannerManager();
  ~HeterogenousPlannerManager();

  void initialize(ros::NodeHandle& nh);

  int planNextMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
      const Vector3d& yaw, const double& camera_pitch, const double& camera_yaw);
  bool checkTrajCollision();
  double getResolution();
  double isHaveMap();

  shared_ptr<HeteroData> hd_;
  shared_ptr<HeteroParam> hp_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<SurfaceCoverage> surface_coverage_;
  shared_ptr<TrajGenerator> traj_generator_;

  std::map<int, std::string> color_map_ = {
    { 0, "\033[0m" },     // default
    { 1, "\033[1;31m" },  // red
    { 2, "\033[1;32m" },  // green
    { 3, "\033[1;33m" },  // yellow
    { 4, "\033[1;34m" },  // blue
    { 5, "\033[1;35m" },  // purple
    { 6, "\033[1;36m" },  // cyan-blue
    { 7, "\033[1;37m" },  // white
  };

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<Astar> path_finder_;

  ros::ServiceClient tsp_client_;
  double max_cluster_dist_;
  double max_boundary_clusters_dist_;
  double viewpoint_h_cost_;
  bool vis_flag_ = false;
  bool final_task_assignment_;
  double surface_min_update_time_;
  double consistent_alpha_;
  double consistent_value_;
  double mdmtsp_iterations_;
  int max_local_num_;

  ViewpointTourResult VTR;
  ViewpointAssignmentResult VAR;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      vector<int>& indices);

  void findViewpointsTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const double& cur_yaw,
      const double& cur_pitch, const vector<VectorXd>& viewpoints, vector<int>& indices);

  void findClustersDronesTourV2(const vector<VectorXd>& drones_pos,
      const vector<int>& useful_cluster_ids, vector<vector<int>>& drone_indices);
  void findClustersDronesTour(const vector<ViewpointCluster>& clusters,
      const vector<VectorXd>& drones_pos, vector<vector<int>>& drone_indices);
  void getViewpointClustersMatrix(const vector<ViewpointCluster>& clusters,
      const vector<VectorXd>& drones_pos, Eigen::MatrixXd& cost_mat);
  void updateViewpointClustersMatrix();

  bool taskAssignment(const vector<VectorXd>& drones_pos, const vector<int>& useful_cluster_ids,
      const vector<vector<int>>& last_drone_indices, vector<vector<int>>& drone_indices,
      const vector<int>& last_vector, const vector<int>& new_vector);

  void getMdmtspInitValue(const vector<vector<int>>& last_useful_allocated_indexs,
      const vector<int>& last_cls_indexs, const vector<int>& new_cls_indexs, VectorXi& p_rte,
      vector<int>& p_brk);
  void MDMTSP_GA(const MatrixXd& dmat, const MatrixXd& D0, const MatrixXd& D1, int max_iter,
      vector<vector<int>>& drone_indices, const vector<vector<int>>& last_useful_allocated_indexs,
      const vector<int>& last_cls_indexs, const vector<int>& new_cls_indexs);
  void getMdmtspCostMatrix(const vector<VectorXd>& drones_pos,
      const vector<int>& useful_cluster_ids, Eigen::MatrixXd& dmat, Eigen::MatrixXd& D0,
      Eigen::MatrixXd& D1);
  void getMTSPCostMatrixV2(const vector<VectorXd>& drones_pos,
      const vector<int>& useful_cluster_ids, Eigen::MatrixXd& cost_mat);

  std::vector<int> randBreakIdx(int max_salesmen, int n);
  MatrixXi calcRange(const vector<int>& p_brk, int n);
  double calculateTourLength(
      const vector<int>& Tour, const MatrixXd& dmat, const MatrixXd& D0, const Eigen::MatrixXd& D1);
  double calculateTourConsistent(int drone_id, const vector<int>& Tour,
      const vector<vector<int>>& last_useful_allocated_indexs, const MatrixXd& dmat,
      const MatrixXd& D0);
  vector<int> randperm(int n);

  void shortenPath(vector<Vector3d>& path);
  void AngleInterpolation(const Eigen::VectorXd& start, const Eigen::VectorXd& end,
      const vector<Eigen::Vector3d>& waypts_, vector<Eigen::VectorXd>& updates_waypts_);

  void getAddAndDeleteViewpoints(const ViewpointsTask& drone_task, const ViewpointsTask& last_task,
      vector<VectorXd>& add_viewpoints, vector<VectorXd>& delete_viewpoints);

  bool viewpointsClustering(vector<VectorXd> viewpoints);

  bool raycastFunc(const Vector3d& pos1, const Vector3d& pos2);
  vector<int> randomInsert(const vector<int>& a, const vector<int>& b);

public:
  typedef shared_ptr<HeterogenousPlannerManager> Ptr;
};

inline double HeterogenousPlannerManager::getResolution()
{
  return sdf_map_->getResolution();
}

inline double HeterogenousPlannerManager::isHaveMap()
{
  return sdf_map_->isStartMap();
}

inline bool HeterogenousPlannerManager::raycastFunc(const Vector3d& pos1, const Vector3d& pos2)
{
  ViewNode::caster_->input(pos1, pos2);
  Vector3i idx;
  while (ViewNode::caster_->nextId(idx)) {
    if (sdf_map_->getOccupancy(idx) != SDFMap::FREE) {
      return false;
    }
  }
  return true;
}

inline vector<int> HeterogenousPlannerManager::randomInsert(
    const vector<int>& a, const vector<int>& b)
{
  vector<int> c = a;                 // Initialize c with the elements of vector a
  srand(ros::Time::now().toNSec());  // Seed the random number generator with current time

  // Insert elements of vector b into vector c at random positions
  for (int i = 0; i < b.size(); ++i) {
    int randomIndex = rand() % (c.size() + 1);  // Generate random index
    c.insert(c.begin() + randomIndex, b[i]);    // Insert element of b into c at random index
  }

  return c;
}
}  // namespace hetero_planner

#endif