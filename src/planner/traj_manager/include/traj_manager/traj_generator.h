#ifndef _HCTRAJ_H_
#define _HCTRAJ_H_

#include <quadrotor_msgs/PolynomialTraj.h>
#include <gcopter/gcopter.hpp>
#include <gcopter/minco.hpp>
#include <gcopter/lbfgs_new.hpp>
#include <gcopter/firi.hpp>
#include <gcopter/voxel_map.hpp>
#include <gcopter/sfc_gen.hpp>
#include "gcopter/quickhull.hpp"
#include "gcopter/geo_utils.hpp"
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <traj_utils/planning_visualization.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <plan_env/edt_environment.h>

using namespace std;
using std::shared_ptr;

namespace hetero_planner {
class EDTEnvironment;
class PlanningVisualization;

class TrajGenerator {
public:
  typedef Eigen::Matrix3Xd PolyhedronV;
  typedef Eigen::MatrixX4d PolyhedronH;
  typedef std::vector<PolyhedronV> PolyhedraV;
  typedef std::vector<PolyhedronH> PolyhedraH;

  TrajGenerator();
  ~TrajGenerator();
  /* Func */
  void init(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  void wpsTraj(Eigen::MatrixXd& wps, Eigen::Matrix3d& iniState, Eigen::Matrix3d& finState,
      const Eigen::Vector3d& now_odom, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
      std::vector<Eigen::Vector3d>& given_wps, vector<bool>& given_indi, bool add_odom);
  void TrajOpt();
  void PitchTrajOpt(vector<double>& given_pitch);
  void YawTrajOpt(vector<double>& given_yaw);
  void HCTraj(Eigen::Vector3d& now_odom, const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
      vector<Eigen::Vector3d>& waypts, vector<bool> waypt_indi, vector<double>& given_pitch,
      vector<double>& given_yaw);
  void getMincoPosTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal);
  void getMincoPitchTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal);
  void getMincoYawTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal);

  shared_ptr<EDTEnvironment> edt_env_;

  /* Param */
  double floor_height_, ceil_height_;
  Eigen::Vector3d curOdom;
  Eigen::MatrixXd opt_wps, way_wps, view_wps, fused_wps;
  vector<bool> opt_indi;
  Eigen::SparseMatrix<double> select_waypt, select_viewpt;
  int piece_nums, waypt_count;
  Eigen::Matrix3d opt_inistate, opt_finstate;
  Eigen::VectorXd opt_times;
  double rho_T, rho_z, rho_p, rho_a, rho_v, rho_j, rho_e;
  double vel_bound, acc_bound, jerk_bound, yawd_bound, max_v_squared, max_a_squared, max_j_squared;
  double safe_dist_;
  Eigen::VectorXd opt_times_Pitch;
  Eigen::VectorXd opt_times_Yaw;
  int traj_id_{ 1 };
  Trajectory<7> minco_traj;
  Eigen::MatrixX3d posCoeff;
  int posPieceNum;
  double posDuration;
  Trajectory<7> mincoPitch_traj;
  Eigen::MatrixX3d pitchCoeff;
  int pitchPieceNum;
  double pitchDuration;
  Trajectory<7> mincoYaw_traj;
  Eigen::MatrixX3d yawCoeff;
  int yawPieceNum;
  double yawDuration;
  double holyProg;
  bool bmk_, tripod_head_trigger_;
  string TrajFile, CloudFile;
  string PosFile, PitchFile, YawFile;
  bool visFlag;
  /* Data */
  vector<double> velocityBound;
  vector<Eigen::Vector3d> route;
  vector<double> pitchRoute;
  vector<double> yawRoute;
  vector<Eigen::MatrixX4d> hPolys;  // H-Representation corridor
  vector<Eigen::Vector3d> pc;
  Eigen::VectorXi hPolyIdx;
  vector<bool> hPolyState;
  int polyN, pieceN;
  vector<Eigen::VectorXd> TrajPose;
  /* Vis */
  ros::Publisher routePub;
  ros::Publisher wayPointsPub;
  ros::Publisher appliedTrajectoryPub;
  ros::Publisher textPub;
  ros::Publisher PitchPub;
  ros::Publisher YawPub;
  /* Benchmark */
  Trajectory<7> FullATSP_traj;
  void TrajOptFullATSP();
  void FullATSPTraj(Eigen::Vector3d& now_odom_fa, vector<Eigen::Vector3d>& waypts_fa);
  void visualizeFullATSP(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  ros::Publisher FAroutePub;
  ros::Publisher FAwayPointsPub;
  ros::Publisher FAappliedTrajectoryPub;
  ros::Publisher FAtextPub;
  /* Tools */
  void velocityPieceBound(
      vector<Eigen::Vector3d>& waypts, vector<double>& given_pitch, vector<double>& given_yaw);
  Eigen::Vector3d jetColorMap(double value);
  void genCorridor(Eigen::Vector3d& start, vector<Eigen::Vector3d>& waypts, double& Progress);
  void setCorridor();
  bool isPointInsidePolytope(
      Eigen::MatrixX4d& polytope, Eigen::Vector3d& point1, Eigen::Vector3d& point2);
  void YawInterpolation(double& duration, double& start, double& end, vector<double>& newYaw,
      vector<double>& newDur, double& CompT);
  void getAngularVelInter(double& t, double& gap, double& omega);
  void getPitch(double& t, double& pitch);
  void getPitchd(double& t, double& pd_);
  void getYaw(double& t, double& yaw);
  void getYawd(double& t, double& yd_);
  /* Utils */
  voxel_map::VoxelMap corridorMap;
  minco::MINCO_S4NU minco_anal, minco_pos_anal, minco_pitch_anal, minco_yaw_anal;
  shared_ptr<PlanningVisualization> vis_utils_;
  gcopter::GCOPTER_PolytopeSFC gcopter;

  static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);
  void TrajVisCallback(const ros::TimerEvent& e);

  void visualizePolytope(vector<Eigen::MatrixX4d>& hPolys);
  void visualize(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizePitch(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void visualizeYaw(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT);
  void polynomialTrajConverter(
      const Trajectory<7>& traj, quadrotor_msgs::PolynomialTraj& msg, const ros::Time& iniStamp);

  void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCxy, Eigen::VectorXd& gdTxy)
  {
    cost = 0.0;
    gdCxy.resize(8 * piece_nums, 3);
    gdCxy.setZero();
    gdTxy.resize(piece_nums);
    gdTxy.setZero();

    Eigen::Vector3d pos, vel, acc, jer, sna, outerNormal;
    Eigen::Vector3d grad_z_upper = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_z_lower = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_v = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_a = Eigen::Vector3d::Zero();
    Eigen::Vector3d grad_j = Eigen::Vector3d::Zero();
    double grad_time = 0.0;
    Eigen::Matrix<double, 8, 1> beta0_xy, beta1_xy, beta2_xy, beta3_xy, beta4_xy;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha, omg;
    double violaPosPena, violaPosPenaD, smoothFactor;
    smoothFactor = 1e-2;

    int int_K = 32;
    for (int i = 0; i < piece_nums; i++) {
      const Eigen::Matrix<double, 8, 3>& c_xy = minco_anal.getCoeffs().block<8, 3>(i * 8, 0);
      step = minco_anal.T1(i) / int_K;
      s1 = 0.0;

      max_v_squared = velocityBound[i] * velocityBound[i];

      for (int j = 0; j <= int_K; j++) {
        alpha = 1.0 / int_K * j;

        // set zero
        grad_z_upper.setZero();
        grad_z_lower.setZero();
        grad_p.setZero();
        grad_v.setZero();
        grad_a.setZero();
        grad_j.setZero();
        grad_time = 0.0;

        // analyse xy
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s4 * s2;
        s7 = s4 * s3;
        beta0_xy << 1.0, s1, s2, s3, s4, s5, s6, s7;
        beta1_xy << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
        beta2_xy << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
        beta3_xy << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
        beta4_xy << 0.0, 0.0, 0.0, 0.0, 24.0, 120 * s1, 360.0 * s2, 840.0 * s3;
        pos = c_xy.transpose() * beta0_xy;
        vel = c_xy.transpose() * beta1_xy;
        acc = c_xy.transpose() * beta2_xy;
        jer = c_xy.transpose() * beta3_xy;
        sna = c_xy.transpose() * beta4_xy;

        omg = (j == 0 || j == int_K) ? 0.5 : 1.0;

        double pos_z = pos(2);
        double v_snorm = vel.squaredNorm();
        double a_snorm = acc.squaredNorm();
        double j_snorm = jer.squaredNorm();

        double safe_dist = safe_dist_ + 0.3;
        double dist_obs;
        Eigen::Vector3d grad_obs;
        edt_env_->evaluateEDTWithGrad(pos, 0, dist_obs, grad_obs);
        double pen = safe_dist - dist_obs;
        if (smoothedL1(pen, smoothFactor, violaPosPena, violaPosPenaD)) {
          grad_p -= rho_p * violaPosPenaD * grad_obs;
          double cost_p = rho_p * violaPosPena;
          cost += cost_p * omg * step;
          grad_time += omg * (cost_p / int_K + step * alpha * grad_p.dot(vel));
        }

        double ceil_height = ceil_height_;
        pen = pos(2) - ceil_height;
        if (smoothedL1(pen, smoothFactor, violaPosPena, violaPosPenaD)) {
          grad_z_upper += rho_p * violaPosPenaD * Eigen::Vector3d(0, 0, 1);
          double cost_p = rho_p * violaPosPena;
          cost += cost_p * omg * step;
          grad_time += omg * (cost_p / int_K + step * alpha * grad_z_upper.dot(vel));
        }

        double floor_height = floor_height_;
        pen = floor_height - pos(2);
        if (smoothedL1(pen, smoothFactor, violaPosPena, violaPosPenaD)) {
          grad_z_lower -= rho_p * violaPosPenaD * Eigen::Vector3d(0, 0, 1);
          double cost_p = rho_p * violaPosPena;
          cost += cost_p * omg * step;
          grad_time += omg * (cost_p / int_K + step * alpha * grad_z_lower.dot(vel));
        }

        // if (zFlag == true)
        // {
        //   double zViola = zPos - pos_z;
        //   Eigen::Vector3d zvec(0, 0, pos_z);
        //   Eigen::Vector3d vzvec(0, 0, vel(2));
        //   if (zViola > 0)
        //   {
        //     grad_z -= rho_z * 6 * zViola * zViola * zvec;
        //     double cost_z = rho_z * zViola * zViola * zViola;
        //     cost += cost_z * omg * step;
        //     grad_time += omg * (cost_z / int_K + step * alpha * grad_z.dot(vzvec));
        //   }
        // }

        // vel
        double vViola = v_snorm - max_v_squared;
        if (vViola > 0) {
          grad_v += rho_v * 6 * vViola * vViola * vel;
          double cost_v = rho_v * vViola * vViola * vViola;
          cost += cost_v * omg * step;
          grad_time += omg * (cost_v / int_K + step * alpha * grad_v.dot(acc));
        }

        // acc
        double aViola = a_snorm - max_a_squared;
        if (aViola > 0) {
          grad_a += rho_a * 6 * aViola * aViola * acc;
          double cost_a = rho_a * aViola * aViola * aViola;
          cost += cost_a * omg * step;
          grad_time += omg * (cost_a / int_K + step * alpha * grad_a.dot(jer));
        }

        // jer
        double jViola = j_snorm - max_j_squared;
        if (jViola > 0) {
          grad_j += rho_j * 6 * jViola * jViola * jer;
          double cost_j = rho_j * jViola * jViola * jViola;
          cost += cost_j * omg * step;
          grad_time += omg * (cost_j / int_K + step * alpha * grad_j.dot(sna));
        }

        // add all grad into C,T
        // note that xy = Cxy*β(j/K*T_xy), yaw = Cyaw*β(i*T_xy+j/K*T_xy-yaw_idx*T_yaw)
        // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
        gdCxy.block<8, 3>(i * 8, 0) +=
            (beta0_xy * grad_p.transpose() + beta0_xy * grad_z_upper.transpose() +
                beta0_xy * grad_z_lower.transpose() + beta1_xy * grad_v.transpose() +
                beta2_xy * grad_a.transpose() + beta3_xy * grad_j.transpose()) *
            omg * step;
        // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
        gdTxy(i) += grad_time;

        s1 += step;
      }
    }
  }
  // T = e^τ
  double expC2(const double& tau)
  {
    return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
  }
  // τ = ln(T)
  double logC2(const double& T)
  {
    return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
  }

  bool smoothedL1(const double& x, const double& mu, double& f, double& df)
  {
    if (x < 0.0) {
      return false;
    }
    else if (x > mu) {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return true;
    }
  }

  static inline void forwardT(const Eigen::VectorXd& tau, Eigen::VectorXd& T)
  {
    const int sizeTau = tau.size();
    T.resize(sizeTau);
    for (int i = 0; i < sizeTau; i++) {
      T(i) = tau(i) > 0.0 ? ((0.5 * tau(i) + 1.0) * tau(i) + 1.0) :
                            1.0 / ((0.5 * tau(i) - 1.0) * tau(i) + 1.0);
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd& T, EIGENVEC& tau)
  {
    const int sizeT = T.size();
    tau.resize(sizeT);
    for (int i = 0; i < sizeT; i++) {
      tau(i) = T(i) > 1.0 ? (sqrt(2.0 * T(i) - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T(i) - 1.0));
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardGradT(
      const Eigen::VectorXd& tau, const Eigen::VectorXd& gradT, EIGENVEC& gradTau)
  {
    const int sizeTau = tau.size();
    gradTau.resize(sizeTau);
    double denSqrt;
    for (int i = 0; i < sizeTau; i++) {
      if (tau(i) > 0) {
        gradTau(i) = gradT(i) * (tau(i) + 1.0);
      }
      else {
        denSqrt = (0.5 * tau(i) - 1.0) * tau(i) + 1.0;
        gradTau(i) = gradT(i) * (1.0 - tau(i)) / (denSqrt * denSqrt);
      }
    }
    return;
  }

private:
  /* Timer */
  ros::Timer traj_vis_timer_;
};

inline void TrajGenerator::setCorridor()
{
  pieceN = (int)route.size() - 1;
  hPolyIdx.resize(pieceN);
  hPolyState.resize(pieceN, false);
  for (int i = 0; i < pieceN; ++i) {
    Eigen::Vector3d pt1 = route[i];
    Eigen::Vector3d pt2 = route[i + 1];
    for (int j = 0; j < (int)hPolys.size(); ++j) {
      if (isPointInsidePolytope(hPolys[j], pt1, pt2)) {
        hPolyIdx[i] = j;
        hPolyState[i] = true;
        break;
      }
    }
  }
  for (int i = 0; i < (int)hPolyIdx.size(); ++i) {
    if (hPolyState[i] == false) {
      vector<Eigen::MatrixX4d> tempPolys;
      Eigen::Vector3d pt1 = route[i];
      Eigen::Vector3d pt2 = route[i + 1];
      vector<Eigen::Vector3d> tempRoute = { pt1, pt2 };
      if (i > 1) {
        Eigen::Vector3d pt1Pre = route[i - 1];
        tempRoute.insert(tempRoute.begin(), pt1Pre);
      }
      if (i < pieceN - 1) {
        Eigen::Vector3d pt2Suc = route[i + 2];
        tempRoute.push_back(pt2Suc);
      }
      sfc_gen::convexCover(tempRoute, pc, corridorMap.getOrigin(), corridorMap.getCorner(),
          1.5 * holyProg, 0.5 * holyProg, tempPolys);
      int tempFlag = hPolys.size();
      hPolys.insert(hPolys.end(), tempPolys.begin(), tempPolys.end());
      for (int k = tempFlag; k < (int)hPolys.size(); ++k) {
        if (isPointInsidePolytope(hPolys[k], pt1, pt2)) {
          hPolyIdx[i] = k;
          hPolyState[i] = true;
          break;
        }
      }
      cout << "state_" << i << ": " << hPolyState[i] << ", hPoly: " << hPolyIdx[i] << endl;
    }
  }
  cout << "sajdkasjkdhasjkdhn end" << std::endl;
}

inline bool TrajGenerator::isPointInsidePolytope(
    Eigen::MatrixX4d& polytope, Eigen::Vector3d& point1, Eigen::Vector3d& point2)
{
  int numHalfSpaces = polytope.rows();
  for (int i = 0; i < numHalfSpaces; ++i) {
    Eigen::Vector3d outerNormal = polytope.block<1, 3>(i, 0);
    if ((outerNormal.dot(point1) + polytope(i, 3)) > 0 ||
        (outerNormal.dot(point2) + polytope(i, 3)) > 0)
      return false;
  }

  return true;
}

inline void TrajGenerator::YawInterpolation(double& duration, double& start, double& end,
    vector<double>& newYaw, vector<double>& newDur, double& CompT)
{
  double yaw_start = start * 180.0 / M_PI;
  double yaw_end = end * 180.0 / M_PI;
  double yaw_gap = abs(yaw_end - yaw_start) > 180.0 ? (360.0 - abs(yaw_end - yaw_start)) :
                                                      abs(yaw_end - yaw_start);
  double omega = yaw_gap / duration;
  int cal_flag_yaw = yaw_end - yaw_start > 0 ? 1 : -1;
  int cal_dir = abs(yaw_end - yaw_start) > 180.0 ? -1 : 1;
  int num = floor(duration / CompT);
  double lastDur = duration - num * CompT;

  if (lastDur < 0.4 * CompT) {
    num = num - 1;
    lastDur += CompT;
  }

  double totalDur = 0.0;
  double tempYaw;
  for (int i = 1; i < num + 1; ++i) {
    totalDur += CompT;
    tempYaw = (yaw_start + cal_dir * cal_flag_yaw * omega * totalDur) * M_PI / 180.0;
    while (tempYaw < -M_PI) tempYaw += 2 * M_PI;
    while (tempYaw > M_PI) tempYaw -= 2 * M_PI;
    newYaw.push_back(tempYaw);
    newDur.push_back(CompT);
  }
  newYaw.push_back(end);
  newDur.push_back(lastDur);
}

inline void TrajGenerator::getAngularVelInter(double& t, double& gap, double& omega)
{
  Eigen::Vector3d YAW = mincoYaw_traj.getPos(t);
  double yawAng = atan2(YAW(1), YAW(0));
  Eigen::Vector3d YAWPre = mincoYaw_traj.getPos(t - gap);
  double yawAngPre = atan2(YAWPre(1), YAWPre(0));
  double yaw_gap = abs(yawAng - yawAngPre) > M_PI ? (2 * M_PI - abs(yawAng - yawAngPre)) :
                                                    abs(yawAng - yawAngPre);
  omega = yaw_gap / gap;
}

inline void TrajGenerator::getPitch(double& t, double& pitch)
{
  Eigen::Vector3d ap = mincoPitch_traj.getPos(t);
  pitch = atan2(ap(1), ap(0) + 1e-6);
}

inline void TrajGenerator::getPitchd(double& t, double& pd_)
{
  Eigen::Vector3d av = mincoPitch_traj.getVel(t);
  pd_ = av.norm();
}

inline void TrajGenerator::getYaw(double& t, double& yaw)
{
  Eigen::Vector3d ap = mincoYaw_traj.getPos(t);
  yaw = atan2(ap(1), ap(0) + 1e-6);
}
/* angular velocity = d(string_length)/dt */
inline void TrajGenerator::getYawd(double& t, double& yd_)
{
  Eigen::Vector3d av = mincoYaw_traj.getVel(t);
  yd_ = av.norm();
}

}  // namespace hetero_planner

#endif