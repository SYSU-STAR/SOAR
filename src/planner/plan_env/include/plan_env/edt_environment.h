#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <utility>
#include <memory>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace hetero_planner {
class SDFMap;

class EDTEnvironment {
private:
  /* data */
  double resolution_inv_;

public:
  EDTEnvironment(/* args */)
  {
  }
  ~EDTEnvironment()
  {
  }

  shared_ptr<SDFMap> sdf_map_;

  void init();
  void setMap(shared_ptr<SDFMap>& map);
  void evaluateEDTWithGrad(
      const Eigen::Vector3d& pos, double time, double& dist, Eigen::Vector3d& grad);

  // deprecated
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
  void interpolateTrilinear(
      double values[2][2][2], const Eigen::Vector3d& diff, double& value, Eigen::Vector3d& grad);

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace hetero_planner

#endif