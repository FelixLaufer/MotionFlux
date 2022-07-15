#ifndef _MOTION_FLUX_H_
#define _MOTION_FLUX_H_

#include "Skeleton.h"
#include "HomogenousTransform.h"
#include <unsupported/Eigen/Splines>

class MotionFlux
{
public:

  struct Joint
  {
    std::string name;
    unsigned int jointIdx;
    unsigned int pointIdx;
  };

  struct Group
  {
    std::string name;
    Joint parent;
    std::vector<Joint> joints;
    std::pair<Joint, Joint> gVector;
  };

  struct Hierarchy
  {
    Joint root;
    std::vector<Group> groups;
  };

  MotionFlux(const Skeleton& skeleton, const std::vector<Vector>& animation, const ScalarType samplingFreq = 60);

  Hierarchy getHierarchy() { return hierarchy_; };
  std::vector<std::vector<std::vector<HomogenousTransform>>> getRNSTrajectories() { return rsnTrajectories_; };

  std::vector<Vector> rnsAnimation();

  void buildSplineTrajectories(const ScalarType cutoffFreq);
  ScalarType objective(const unsigned int groupIdx, ScalarType t0, const ScalarType t1, const ScalarType betaV = 0, const ScalarType betaS = 0, const ScalarType dt = 0.001);
  ScalarType objective(const std::string& groupName, ScalarType t0, const ScalarType t1, const ScalarType betaV = 0, const ScalarType betaS = 0, const ScalarType dt = 0.001);
  ScalarType objectiveDeriv(const unsigned int groupIdx, ScalarType t, const ScalarType betaV = 0, const ScalarType betaS = 0);
  ScalarType objectiveDeriv(const std::string& groupName, ScalarType t, const ScalarType betaV = 0, const ScalarType betaS = 0);


private:
  void constructMFHierarchy();
  void rootSequenceNormalize();

  Skeleton skeleton_;
  std::vector<Vector> animation_;
  ScalarType samplingFreq_;
  Hierarchy hierarchy_;
  std::vector<std::vector<std::vector<HomogenousTransform>>> rsnTrajectories_;
  std::vector<std::vector<Eigen::Spline<ScalarType, 3, 3>>> trajectorySplines_;
};

#endif