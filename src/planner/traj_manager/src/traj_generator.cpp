#include <traj_manager/traj_generator.h>
#include <chrono>

namespace hetero_planner {
TrajGenerator::TrajGenerator()
{
}

TrajGenerator::~TrajGenerator()
{
}

void TrajGenerator::init(const EDTEnvironment::Ptr& edt, ros::NodeHandle& nh)
{
  /* Initialization */
  nh.param("traj_generator/vmax_", vel_bound, -1.0);
  nh.param("traj_generator/amax_", acc_bound, -1.0);
  nh.param("traj_generator/jmax_", jerk_bound, -1.0);
  nh.param("traj_generator/ydmax_", yawd_bound, -1.0);
  nh.param("traj_generator/rho_T", rho_T, -1.0);
  nh.param("traj_generator/rho_j", rho_j, -1.0);
  nh.param("traj_generator/rho_a", rho_a, -1.0);
  nh.param("traj_generator/rho_v", rho_v, -1.0);
  nh.param("traj_generator/rho_p", rho_p, -1.0);
  nh.param("sdf_map/box_min_z", floor_height_, -1.0);
  nh.param("sdf_map/box_max_z", ceil_height_, -1.0);
  // nh.param("traj_generator/rho_z", rho_z, -1.0);
  nh.param("traj_generator/rho_e", rho_e, -1.0);
  nh.param("sdf_map/obstacles_inflation", safe_dist_, -1.0);
  this->edt_env_ = edt;
  /* Utils Init */
  vis_utils_.reset(new PlanningVisualization(nh));
  visFlag = false;
  traj_vis_timer_ = nh.createTimer(ros::Duration(1), &TrajGenerator::TrajVisCallback, this);
  /* Vis Traj */
  routePub = nh.advertise<visualization_msgs::Marker>("/hcoppTraj/route", 1);
  wayPointsPub = nh.advertise<visualization_msgs::Marker>("/hcoppTraj/waypoints", 1);
  appliedTrajectoryPub =
      nh.advertise<visualization_msgs::Marker>("/hcoppTraj/applied_trajectory", 1);
  textPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/drone_status", 1);
  PitchPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/PitchState", 1);
  YawPub = nh.advertise<visualization_msgs::MarkerArray>("/hcoppTraj/YawState", 1);
}

// Pass in the aircraft's current odometry + path obtained from A* (position, pitch, yaw).
void TrajGenerator::HCTraj(Eigen::Vector3d& now_odom, const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc, vector<Eigen::Vector3d>& waypts, vector<bool> waypt_indi,
    vector<double>& given_pitch, vector<double>& given_yaw)
{
  velocityBound.clear();
  route.clear();
  pitchRoute.clear();
  yawRoute.clear();
  hPolys.clear();
  pc.clear();
  hPolyState.clear();
  TrajPose.clear();
  opt_indi.clear();

  auto hctraj_t1 = std::chrono::high_resolution_clock::now();
  vector<Eigen::Vector3d> allwaypts;
  allwaypts.push_back(now_odom);
  allwaypts.insert(allwaypts.end(), waypts.begin(), waypts.end());
  velocityPieceBound(allwaypts, given_pitch, given_yaw);
  // 得到优化的wps & init_state finish_state
  wpsTraj(opt_wps, opt_inistate, opt_finstate, now_odom, vel, acc, waypts, waypt_indi, false);
  TrajOpt();
  if ((int)given_pitch.size() > 0)
    PitchTrajOpt(given_pitch);
  if ((int)given_yaw.size() > 0)
    YawTrajOpt(given_yaw);
  auto hctraj_t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> hctraj_ms = hctraj_t2 - hctraj_t1;
  cout << "Trajectory generation: " << hctraj_ms.count() << "ms." << std::endl;
  visFlag = true;
}

void TrajGenerator::wpsTraj(Eigen::MatrixXd& wps, Eigen::Matrix3d& iniState,
    Eigen::Matrix3d& finState, const Eigen::Vector3d& now_odom, const Eigen::Vector3d& vel,
    const Eigen::Vector3d& acc, std::vector<Eigen::Vector3d>& given_wps, vector<bool>& given_indi,
    bool add_odom)
{
  if (given_wps.size() == 0) {
    ROS_ERROR("TrajGenerator: No waypoints given! ");
    wps.resize(3, 1);
    wps.col(0) = now_odom;
    iniState << now_odom, vel, acc;
    finState << now_odom, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    return;
  }
  else if (given_wps.size() == 1) {
    wps.resize(3, 1);
    wps.col(0) = 0.5 * (now_odom + given_wps[0]);
    iniState << now_odom, vel, acc;
    finState << given_wps[0], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
    return;
  }

  wps.resize(3, int(given_wps.size()) - 1);
  opt_indi.clear();
  opt_indi.resize(given_indi.size() - 1, false);
  for (int i = 0; i < int(given_wps.size()) - 1; i++) {
    if (given_indi[i] == true)
      opt_indi[i] = true;
    else
      opt_indi[i] = false;
    if (add_odom) {
      wps(0, i) = given_wps[i](0) + now_odom(0);
      wps(1, i) = given_wps[i](1) + now_odom(1);
      wps(2, i) = given_wps[i](2);
    }
    else {
      wps(0, i) = given_wps[i](0);
      wps(1, i) = given_wps[i](1);
      wps(2, i) = given_wps[i](2);
    }
  }
  iniState << now_odom, vel, acc;
  finState << given_wps[given_wps.size() - 1], Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
}

void TrajGenerator::TrajOpt()
{
  /*
  suppose N points including M waypoints and N-M viewpoints
  optimize M waypoints and freeze N-M viewpoints
  */
  Eigen::SparseMatrix<double> onehot((int)opt_wps.cols(), (int)opt_wps.cols());
  onehot.setIdentity();

  waypt_count = count(opt_indi.begin(), opt_indi.end(), true);
  select_waypt.resize((int)opt_wps.cols(), waypt_count);
  select_viewpt.resize((int)opt_wps.cols(), (int)opt_wps.cols() - waypt_count);
  way_wps.resize(3, waypt_count);
  view_wps.resize(3, (int)opt_wps.cols() - waypt_count);

  int index1 = 0, index2 = 0;
  for (int i = 0; i < (int)opt_indi.size(); ++i) {
    if (opt_indi[i] == true) {
      select_waypt.col(index1) = onehot.col(i);
      way_wps.col(index1) = opt_wps.col(i);
      index1++;
    }
    else {
      select_viewpt.col(index2) = onehot.col(i);
      view_wps.col(index2) = opt_wps.col(i);
      index2++;
    }
  }
  if (opt_wps.cols() < 1) {
    ROS_ERROR("TrajGenerator: TrajOpt: No Waypoints!");
    return;
  }

  piece_nums = opt_wps.cols() + 1;
  /* max vel, acc, and jerk */
  max_v_squared = vel_bound * vel_bound;
  max_a_squared = acc_bound * acc_bound;
  max_j_squared = jerk_bound * jerk_bound;
  /* set initial Time */
  opt_times.resize(piece_nums);
  opt_times[0] = ((opt_inistate.col(0) - opt_wps.col(0)).norm() + 0.1) * 2.0 / vel_bound;
  for (int i = 1; i < opt_wps.cols(); i++) {
    opt_times[i] = ((opt_wps.col(i) - opt_wps.col(i - 1)).norm() + 0.1) * 2.0 / vel_bound;
  }
  opt_times[piece_nums - 1] =
      ((opt_finstate.col(0) - opt_wps.col(opt_wps.cols() - 1)).norm() + 0.1) * 2.0 / vel_bound;

  double traj_time_init = 0.0;
  for (int i = 0; i < (int)opt_times.size(); ++i) traj_time_init += opt_times(i);
  ROS_INFO("\033[32m[TrajInit] init traj time = %lf s\033[32m", traj_time_init);

  minco_anal.setConditions(opt_inistate, opt_finstate, piece_nums);
  minco_anal.setParameters(opt_wps, opt_times);

  /* set initial tau */
  Eigen::VectorXd x, init_tau, waypt_vec;
  int opt_dim = piece_nums + 3 * waypt_count;
  x.resize(opt_dim);
  backwardT(opt_times, init_tau);
  waypt_vec = Eigen::Map<Eigen::VectorXd>(way_wps.data(), way_wps.size());
  x.segment(0, piece_nums) = init_tau;
  x.segment(piece_nums, 3 * waypt_count) = waypt_vec;

  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs_params.mem_size = 16;
  lbfgs_params.past = 3;
  // lbfgs_params.g_epsilon = 1.0e-3;
  lbfgs_params.g_epsilon = 0;
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.delta = 1.0e-4;
  // lbfgs_params.max_linesearch = 256;
  lbfgs_params.max_iterations = 1000;
  double inner_cost;

  bmk_ = false;
  int result =
      lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, nullptr, this, lbfgs_params);

  if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
      result == lbfgs::LBFGS_STOP || result == lbfgs::LBFGSERR_MAXIMUMITERATION) {
    ROS_INFO_STREAM("[Eight] optimization success! cost: " << inner_cost);
  }
  else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH) {
    ROS_WARN("[Eight] The line-search routine reaches the maximum number of evaluations.");
  }
  else {
    ROS_ERROR("[Eight] Solver error. Return = %d, %s.", result, lbfgs::lbfgs_strerror(result));
  }

  minco_anal.getTrajectory(minco_traj);
  minco_pos_anal = minco_anal;
  posCoeff = minco_anal.getCoeffs();
  posPieceNum = minco_traj.getPieceNum();
  posDuration = minco_traj.getTotalDuration();

  if (minco_traj.getPieceNum()) {
    double length = 0.0, T = 0.01;
    Eigen::Vector3d lastX = minco_traj.getPos(0.0);
    for (double t = T; t < minco_traj.getTotalDuration(); t += T) {
      Eigen::Vector3d X = minco_traj.getPos(t);
      length += (X - lastX).norm();
      lastX = X;
    }
    int highVelNum = 0, totalNum = 0;
    double max_vel = minco_traj.getMaxVelRate();
    for (double t = 0.0; t < minco_traj.getTotalDuration(); t += T) {
      if (minco_traj.getVel(t).norm() > 0.8 * max_vel)
        highVelNum++;
      totalNum++;
    }
    double highVelRate = (double)highVelNum / (double)totalNum;
    double max_jerk = -1.0;
    for (double t = 0.0; t < minco_traj.getTotalDuration(); t += T) {
      double jerk = minco_traj.getJer(t).norm();
      if (jerk > max_jerk)
        max_jerk = jerk;
    }

    ROS_INFO("\033[32m[TrajAnalyzer] traj max vel = %lf m/s\033[32m", minco_traj.getMaxVelRate());
    ROS_INFO("\033[32m[TrajAnalyzer] traj max acc = %lf m/s^2\033[32m", minco_traj.getMaxAccRate());
    ROS_INFO("\033[32m[TrajAnalyzer] traj max jer = %lf m/s^3\033[32m", max_jerk);
    ROS_INFO(
        "\033[35m[TrajAnalyzer] traj exec time = %lf s\033[32m", minco_traj.getTotalDuration());
    ROS_INFO("\033[36m[TrajAnalyzer] traj length = %lf m\033[32m", length);
    ROS_INFO("\033[33m[TrajAnalyzer] traj high vel rate = %lf \033[32m", highVelRate);
  }
}

void TrajGenerator::getMincoPosTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal)
{
  traj = minco_traj;
  anal = minco_pos_anal;
}

void TrajGenerator::getMincoPitchTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal)
{
  traj = mincoPitch_traj;
  anal = minco_pitch_anal;
}

void TrajGenerator::getMincoYawTraj(Trajectory<7>& traj, minco::MINCO_S4NU& anal)
{
  traj = mincoYaw_traj;
  anal = minco_yaw_anal;
}

double TrajGenerator::innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
{
  TrajGenerator& obj = *(TrajGenerator*)ptrObj;
  /* get x */
  Eigen::Map<const Eigen::MatrixXd> tau(x.data(), obj.piece_nums, 1);
  Eigen::Map<const Eigen::MatrixXd> wps(x.data() + obj.piece_nums, 3, obj.waypt_count);
  Eigen::Map<Eigen::VectorXd> grad_tau(grad.data(), obj.piece_nums);
  Eigen::Map<Eigen::MatrixXd> grad_wps(grad.data() + obj.piece_nums, 3, obj.waypt_count);
  grad_tau.setZero();
  grad_wps.setZero();

  /* get T from τ, generate MINCO trajectory */
  Eigen::VectorXd Txy;
  obj.forwardT(tau, Txy);
  obj.fused_wps = wps * obj.select_waypt.transpose() + obj.view_wps * obj.select_viewpt.transpose();
  obj.minco_anal.setParameters(obj.fused_wps, Txy);

  /* get snap cost with grad (C,T) */
  double snap_cost = 0.0;
  Eigen::MatrixX3d gdCxy_snap;
  Eigen::VectorXd gdTxy_snap;
  obj.minco_anal.getEnergy(snap_cost);
  snap_cost = obj.rho_e * snap_cost;
  obj.minco_anal.getEnergyPartialGradByCoeffs(gdCxy_snap);
  gdCxy_snap = obj.rho_e * gdCxy_snap;
  obj.minco_anal.getEnergyPartialGradByTimes(gdTxy_snap);
  gdTxy_snap = obj.rho_e * gdTxy_snap;

  /* get constrain cost with grad (C,T) */
  double constrain_cost = 0.0;
  Eigen::MatrixXd gdCxy_constrain;
  Eigen::VectorXd gdTxy_constrain;
  // add vel, acc, jerk bound
  obj.calConstrainCostGrad(constrain_cost, gdCxy_constrain, gdTxy_constrain);

  /* get grad (q, T) from (C, T) */
  Eigen::MatrixXd gdCxy = gdCxy_snap + gdCxy_constrain;
  Eigen::VectorXd gdTxy = gdTxy_snap + gdTxy_constrain;
  Eigen::Matrix3Xd gradPxy_temp;
  Eigen::VectorXd gradTxy_temp;
  obj.minco_anal.propogateGrad(gdCxy, gdTxy, gradPxy_temp, gradTxy_temp);

  /* get tau cost with grad */
  double tau_cost = obj.rho_T * Txy.sum();
  gradTxy_temp.array() += obj.rho_T;
  grad_wps = gradPxy_temp * obj.select_waypt;
  obj.backwardGradT(tau, gradTxy_temp, grad_tau);

  return snap_cost + constrain_cost + tau_cost;
}

void TrajGenerator::velocityPieceBound(
    vector<Eigen::Vector3d>& waypts, vector<double>& given_pitch, vector<double>& given_yaw)
{
  double pos_time, yaw_time, pitch_time, bound_time;
  double bound_velocity;
  int pieces = (int)waypts.size() - 1;

  for (int i = 0; i < pieces; ++i) {
    pos_time = (waypts[i + 1] - waypts[i]).norm() / vel_bound;
    if (given_yaw.size() > 0)
      yaw_time = 1.2 *
                 min(abs(given_yaw[i + 1] - given_yaw[i]),
                     2 * M_PI - abs(given_yaw[i + 1] - given_yaw[i])) /
                 yawd_bound;
    if (given_pitch.size() > 0)
      pitch_time = 1.2 *
                   min(abs(given_pitch[i + 1] - given_pitch[i]),
                       2 * M_PI - abs(given_pitch[i + 1] - given_pitch[i])) /
                   yawd_bound;
    bound_time = max(max(pos_time, yaw_time), pitch_time);

    bound_velocity = min(vel_bound, (waypts[i + 1] - waypts[i]).norm() / bound_time);
    velocityBound.push_back(bound_velocity);
  }
}

void TrajGenerator::PitchTrajOpt(vector<double>& given_pitch)
{
  /* p: [cos(Pitch), sin(Pitch), 0], v: [0, 0, 0], a: [0, 0, 0] */
  pitchRoute = given_pitch;
  Eigen::Matrix3d iniStatePitch, finStatePitch;
  Eigen::MatrixXd wpsPitch;

  iniStatePitch << Eigen::Vector3d(cos(given_pitch.front()), sin(given_pitch.front()), 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finStatePitch << Eigen::Vector3d(cos(given_pitch.back()), sin(given_pitch.back()), 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::VectorXd Duration = minco_traj.getDurations();
  vector<double> Pitchs, Durs;
  double compT = 0.2;
  for (int i = 0; i < (int)Duration.size(); ++i) {
    vector<double> tempPitchs, tempDurs;
    YawInterpolation(Duration(i), pitchRoute[i], pitchRoute[i + 1], tempPitchs, tempDurs, compT);
    Pitchs.insert(Pitchs.end(), tempPitchs.begin(), tempPitchs.end());
    Durs.insert(Durs.end(), tempDurs.begin(), tempDurs.end());
  }
  int pieceNUM = (int)Durs.size();
  opt_times_Pitch.resize(pieceNUM);
  for (int i = 0; i < pieceNUM; ++i) opt_times_Pitch[i] = Durs[i];
  wpsPitch.resize(3, int(Pitchs.size()) - 1);
  for (int i = 0; i < (int)Pitchs.size() - 1; ++i) {
    wpsPitch(0, i) = cos(Pitchs[i]);
    wpsPitch(1, i) = sin(Pitchs[i]);
    wpsPitch(2, i) = 0.0;
  }
  /* MINCO */
  minco_anal.setConditions(iniStatePitch, finStatePitch, pieceNUM);
  minco_anal.setParameters(wpsPitch, opt_times_Pitch);
  minco_anal.getTrajectory(mincoPitch_traj);
  minco_pitch_anal = minco_anal;
  pitchCoeff = minco_anal.getCoeffs();
  pitchPieceNum = mincoPitch_traj.getPieceNum();
  pitchDuration = mincoPitch_traj.getTotalDuration();

  double T = 0.01, max_omega_ = -1.0;
  for (double t = 0.0; t < mincoPitch_traj.getTotalDuration(); t += T) {
    double omega_;
    getPitchd(t, omega_);
    if (omega_ > max_omega_)
      max_omega_ = omega_;
  }
  ROS_INFO("\033[32m[TrajAnalyzer] traj max pitch angular vel = %lf deg/s\033[32m",
      180.0 * max_omega_ / M_PI);
}

void TrajGenerator::YawTrajOpt(vector<double>& given_yaw)
{
  /* p: [cos(Yaw), sin(Yaw), 0], v: [0, 0, 0], a: [0, 0, 0] */
  yawRoute = given_yaw;
  Eigen::Matrix3d iniStateYaw, finStateYaw;
  Eigen::MatrixXd wpsYaw;

  iniStateYaw << Eigen::Vector3d(cos(given_yaw.front()), sin(given_yaw.front()), 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  finStateYaw << Eigen::Vector3d(cos(given_yaw.back()), sin(given_yaw.back()), 0.0),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
  Eigen::VectorXd Duration = minco_traj.getDurations();
  vector<double> Yaws, Durs;
  double compT = 0.2;
  for (int i = 0; i < (int)Duration.size(); ++i) {
    vector<double> tempYaws, tempDurs;
    YawInterpolation(Duration(i), yawRoute[i], yawRoute[i + 1], tempYaws, tempDurs, compT);
    Yaws.insert(Yaws.end(), tempYaws.begin(), tempYaws.end());
    Durs.insert(Durs.end(), tempDurs.begin(), tempDurs.end());
  }
  int pieceNUM = (int)Durs.size();
  opt_times_Yaw.resize(pieceNUM);
  for (int i = 0; i < pieceNUM; ++i) {
    opt_times_Yaw[i] = Durs[i];
  }
  wpsYaw.resize(3, int(Yaws.size()) - 1);
  for (int i = 0; i < (int)Yaws.size() - 1; ++i) {
    wpsYaw(0, i) = cos(Yaws[i]);
    wpsYaw(1, i) = sin(Yaws[i]);
    wpsYaw(2, i) = 0.0;
  }
  /* MINCO */
  minco_anal.setConditions(iniStateYaw, finStateYaw, pieceNUM);
  minco_anal.setParameters(wpsYaw, opt_times_Yaw);
  minco_anal.getTrajectory(mincoYaw_traj);
  minco_yaw_anal = minco_anal;
  yawCoeff = minco_anal.getCoeffs();
  yawPieceNum = mincoYaw_traj.getPieceNum();
  yawDuration = mincoYaw_traj.getTotalDuration();

  double yaw_end;
  double time = mincoYaw_traj.getTotalDuration();
  getYaw(time, yaw_end);

  double T = 0.1, max_omega_ = -1.0;
  double yaw_cur;
  // double max_omega = -1.0;
  // double gap = 0.0; int count = 0;
  int over = 0, count = 0;
  double overRate = 0.0;
  for (double t = 0.0; t < mincoYaw_traj.getTotalDuration(); t += T) {
    getYaw(t, yaw_cur);
    double omega_;
    getYawd(t, omega_);
    if (omega_ > max_omega_)
      max_omega_ = omega_;

    if (omega_ > yawd_bound)
      over++;
    count++;
  }
  overRate = (double)over / (double)count;
  ROS_INFO("\033[33m[TrajAnalyzer] traj angular vel over rate = %lf \033[32m", overRate);
  ROS_INFO(
      "\033[32m[TrajAnalyzer] traj max angular vel = %lf deg/s\033[32m", 180.0 * max_omega_ / M_PI);
  ROS_INFO("\033[35m[TrajAnalyzer] traj angular exec time = %lf s\033[32m",
      mincoYaw_traj.getTotalDuration());
}

void TrajGenerator::visualize(const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
{
  visualization_msgs::Marker routeMarker, wayPointsMarker, appliedTrajMarker;
  visualization_msgs::MarkerArray Order;
  double max_vel = appliedTraj.getMaxVelRate();

  routeMarker.id = 0;
  routeMarker.type = visualization_msgs::Marker::LINE_LIST;
  routeMarker.header.stamp = timeStamp;
  routeMarker.header.frame_id = "world";
  routeMarker.pose.orientation.w = 1.00;
  routeMarker.action = visualization_msgs::Marker::ADD;
  routeMarker.ns = "route";
  routeMarker.color.r = 1.00;
  routeMarker.color.g = 0.00;
  routeMarker.color.b = 0.00;
  routeMarker.color.a = 1.00;
  routeMarker.scale.x = 0.4;

  wayPointsMarker = routeMarker;
  wayPointsMarker.type = visualization_msgs::Marker::SPHERE_LIST;
  wayPointsMarker.ns = "waypoints";
  wayPointsMarker.color.r = 1.00;
  wayPointsMarker.color.g = 0.00;
  wayPointsMarker.color.b = 0.00;
  wayPointsMarker.scale.x = 0.40;
  wayPointsMarker.scale.y = 0.40;
  wayPointsMarker.scale.z = 0.40;

  appliedTrajMarker.header.frame_id = "world";
  appliedTrajMarker.header.stamp = ros::Time::now();
  appliedTrajMarker.id = 0;
  appliedTrajMarker.ns = "applied_trajectory";
  appliedTrajMarker.type = visualization_msgs::Marker::SPHERE_LIST;
  appliedTrajMarker.action = visualization_msgs::Marker::ADD;
  appliedTrajMarker.scale.x = 0.20;
  appliedTrajMarker.scale.y = 0.20;
  appliedTrajMarker.scale.z = 0.20;
  appliedTrajMarker.pose.orientation.w = 1.0;

  Eigen::MatrixXd route = appliedTraj.getPositions();

  if (route.cols() > 0) {
    bool first = true;
    Eigen::Vector3d last;
    for (int i = 0; i < route.cols(); i++) {
      if (first) {
        first = false;
        last = route.col(i);
        continue;
      }
      geometry_msgs::Point point;

      point.x = last(0);
      point.y = last(1);
      point.z = last(2);
      routeMarker.points.push_back(point);
      point.x = route.col(i)(0);
      point.y = route.col(i)(1);
      point.z = route.col(i)(2);
      routeMarker.points.push_back(point);
      last = route.col(i);
    }

    routePub.publish(routeMarker);
  }

  if (route.cols() > 0) {
    for (int i = 0; i < route.cols(); i++) {
      geometry_msgs::Point point;
      point.x = route.col(i)(0);
      point.y = route.col(i)(1);
      point.z = route.col(i)(2);
      wayPointsMarker.points.push_back(point);
    }

    wayPointsPub.publish(wayPointsMarker);
  }

  if (appliedTraj.getPieceNum() > 0) {
    double T = 0.01;
    geometry_msgs::Point point;
    for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T) {
      Eigen::Vector3d X = appliedTraj.getPos(t);
      double Vel = appliedTraj.getVel(t).norm() / max_vel;

      point.x = X(0);
      point.y = X(1);
      point.z = X(2);
      Eigen::Vector3d rgb = jetColorMap(Vel);
      appliedTrajMarker.color.r = rgb(0);
      appliedTrajMarker.color.g = rgb(1);
      appliedTrajMarker.color.b = rgb(2);
      appliedTrajMarker.color.a = 1.0;
      appliedTrajMarker.scale.x = 0.1;
      appliedTrajMarker.scale.y = 0.1;
      appliedTrajMarker.scale.z = 0.1;

      appliedTrajMarker.points.push_back(point);
      appliedTrajMarker.colors.push_back(appliedTrajMarker.color);
    }
    appliedTrajectoryPub.publish(appliedTrajMarker);
  }

  if (route.cols() > 0) {
    int count = 0;
    for (int i = 0; i < route.cols(); ++i) {
      visualization_msgs::Marker textMarker;
      textMarker.header.frame_id = "world";
      textMarker.header.stamp = timeStamp;
      textMarker.ns = "text";
      textMarker.id = count;
      textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      textMarker.action = visualization_msgs::Marker::ADD;

      textMarker.pose.position.x = route.col(i)(0);
      textMarker.pose.position.y = route.col(i)(1);
      textMarker.pose.position.z = route.col(i)(2);
      textMarker.pose.orientation.x = 0.0;
      textMarker.pose.orientation.y = 0.0;
      textMarker.pose.orientation.z = 0.0;
      textMarker.pose.orientation.w = 1.0;
      textMarker.scale.x = 2.0;
      textMarker.scale.y = 2.0;
      textMarker.scale.z = 2.0;
      textMarker.color.r = 0.0;
      textMarker.color.g = 0.0;
      textMarker.color.b = 0.0;
      textMarker.color.a = 1.0;
      textMarker.text = to_string(i);

      Order.markers.push_back(textMarker);
      count++;
    }

    textPub.publish(Order);
  }
}

void TrajGenerator::visualizePitch(
    const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
{
  visualization_msgs::MarkerArray pitch_traj;
  int counter = 0;
  double scale = 3.0;
  double T = 0.3;
  for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T) {
    Eigen::Vector3d X = minco_traj.getPos(t);
    // Eigen::Vector3d PITCH = appliedTraj.getPos(t);
    double pitchAng;
    getPitch(t, pitchAng);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "pitch_traj";
    mk.type = visualization_msgs::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.4;
    mk.scale.z = 0.3;
    mk.color.r = 0.8;
    mk.color.g = 0.0;
    mk.color.b = 0.4;
    mk.color.a = 1.0;

    geometry_msgs::Point pt_;
    pt_.x = X(0);
    pt_.y = X(1);
    pt_.z = X(2);
    mk.points.push_back(pt_);

    pt_.x = X(0) + scale * cos(pitchAng);
    pt_.y = X(1);
    pt_.z = X(2) + scale * sin(pitchAng);
    mk.points.push_back(pt_);

    pitch_traj.markers.push_back(mk);
    counter++;
  }

  PitchPub.publish(pitch_traj);
}

void TrajGenerator::visualizeYaw(
    const Trajectory<7>& appliedTraj, ros::Time timeStamp, double compT)
{
  visualization_msgs::MarkerArray yaw_traj;
  int counter = 0;
  double scale = 3.0;
  double T = 0.5;
  for (double t = 0.0; t < appliedTraj.getTotalDuration(); t += T) {
    Eigen::Vector3d X = minco_traj.getPos(t);
    // Eigen::Vector3d YAW = appliedTraj.getPos(t);
    double yawAng;
    getYaw(t, yawAng);

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = counter;
    mk.ns = "yaw_traj";
    mk.type = visualization_msgs::Marker::ARROW;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.2;
    mk.scale.y = 0.4;
    mk.scale.z = 0.3;
    mk.color.r = 0.0;
    mk.color.g = 0.8;
    mk.color.b = 0.4;
    mk.color.a = 1.0;

    geometry_msgs::Point pt_;
    pt_.x = X(0);
    pt_.y = X(1);
    pt_.z = X(2);
    mk.points.push_back(pt_);

    pt_.x = X(0) + scale * cos(yawAng);
    pt_.y = X(1) + scale * sin(yawAng);
    pt_.z = X(2);
    mk.points.push_back(pt_);

    yaw_traj.markers.push_back(mk);
    counter++;
  }

  YawPub.publish(yaw_traj);
}

void TrajGenerator::polynomialTrajConverter(
    const Trajectory<7>& traj, quadrotor_msgs::PolynomialTraj& msg, const ros::Time& iniStamp)
{
  msg.trajectory_id = traj_id_;
  msg.header.stamp = iniStamp;
  msg.action = msg.ACTION_ADD;
  int piece_num = traj.getPieceNum();

  for (int i = 0; i < piece_num; ++i) {
    quadrotor_msgs::PolynomialMatrix piece;
    piece.num_dim = traj[i].getDim();
    piece.num_order = traj[i].getDegree();
    piece.duration = traj[i].getDuration();
    auto cMat = traj[i].getCoeffMat();
    piece.data.assign(cMat.data(), cMat.data() + cMat.rows() * cMat.cols());
    msg.trajectory.emplace_back(piece);
  }
  traj_id_++;
}

void TrajGenerator::TrajVisCallback(const ros::TimerEvent& e)
{
  if (visFlag == true) {
    // quadrotor_msgs::PolynomialTraj position_traj_msg, pitch_traj_msg, yaw_traj_msg;
    // polynomialTrajConverter(minco_traj, position_traj_msg, ros::Time::now());
    // polynomialTrajConverter(mincoPitch_traj, pitch_traj_msg, ros::Time::now());
    // polynomialTrajConverter(mincoYaw_traj, yaw_traj_msg, ros::Time::now());

    visualize(minco_traj, ros::Time::now(), 0.0);
    visualizePitch(mincoPitch_traj, ros::Time::now(), 0.0);
    visualizeYaw(mincoYaw_traj, ros::Time::now(), 0.0);
  }
}
// * /* Tools */
Eigen::Vector3d TrajGenerator::jetColorMap(double value)
{
  double r, g, b;
  if (value < 0.0)
    value = 0.0;
  else if (value > 1.0)
    value = 1.0;

  if (value < 0.25) {
    r = 0.0;
    g = 4.0 * value;
    b = 1.0;
  }
  else if (value < 0.5) {
    r = 0.0;
    g = 1.0;
    b = 1.0 - 4.0 * (value - 0.25);
  }
  else if (value < 0.75) {
    r = 4.0 * (value - 0.5);
    g = 1.0;
    b = 0.0;
  }
  else {
    r = 1.0;
    g = 1.0 - 4.0 * pow((value - 0.75), 1);
    b = 0.0;
  }

  return Eigen::Vector3d(r, g, b);
}

void TrajGenerator::genCorridor(
    Eigen::Vector3d& start, vector<Eigen::Vector3d>& waypts, double& Progress)
{
  route.push_back(start);
  for (auto p : waypts) route.push_back(p);

  holyProg = Progress;
  corridorMap.getSurf(pc);
  // ROS_ERROR_STREAM("corridorMap.getOrigin()" << corridorMap.getOrigin().transpose() <<
  // "corridorMap.getCorner()" << corridorMap.getCorner().transpose());
  sfc_gen::convexCover(route, pc, corridorMap.getOrigin(), corridorMap.getCorner(), Progress,
      0.3 * Progress, hPolys);

  ROS_ERROR_STREAM("origin :" << corridorMap.getOrigin().transpose()
                              << "\ncorner :" << corridorMap.getCorner().transpose());
  ROS_INFO("\033[34m[TrajAnalyzer] convexHull number = %d \033[32m", (int)hPolys.size());
}
}  // namespace hetero_planner