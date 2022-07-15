#include "MotionFlux.h"

#include "math/ButterworthLP.h"

MotionFlux::MotionFlux(const Skeleton& skeleton, const std::vector<Vector>& animation, const ScalarType samplingFreq)
  : skeleton_(skeleton)
  , animation_(animation)
  , hierarchy_(Hierarchy())
  , samplingFreq_(samplingFreq)
{
  constructMFHierarchy();
  rootSequenceNormalize();
}

void MotionFlux::constructMFHierarchy()
{
  // Define the motion flux hierarchy in relation to the skeleton

  const auto segIdx2Name = [this](const unsigned int segIdx)
  {
    const auto segments = (*skeleton_.getSegments());
    if (segIdx > segments.size())
      return std::string();
    return segments[segIdx].getName();
  };

  const auto segName2Idx = [this](const std::string& segName)
  {
    const auto segments = (*skeleton_.getSegments());
    for (unsigned int s = 0; s < segments.size(); ++s)
    {
      if (segments[s].getName() == segName)
        return s;
    }
    return std::numeric_limits<unsigned int>::quiet_NaN();
  };

  // 1) Joints
  // Torso
  const Joint mfjPelvis{ "Pelvis", segName2Idx("Pelvis"), 0 };
  const Joint mfjT8{ "T8", segName2Idx("T8"), 0 };
  const Joint mfjT12{ "T12", segName2Idx("T12"), 0 };

  // Head
  const Joint mfjHead{ "Head", segName2Idx("Head"), 0 };
  const Joint mfjNeck{ "Neck", segName2Idx("Neck"), 0 };

  // Left arm
  const Joint mfjLeftUpperArm{ "LeftUpperArm", segName2Idx("LeftUpperArm"), 0 };
  const Joint mfjLeftForeArm{ "LeftForeArm", segName2Idx("LeftForeArm"), 0 };
  const Joint mfjLeftHand{ "LeftHand", segName2Idx("LeftHand"), 0 };

  // Right arm
  const Joint mfjRightUpperArm{ "RightUpperArm", segName2Idx("RightUpperArm"), 0 };
  const Joint mfjRightForeArm{ "RightForeArm", segName2Idx("RightForeArm"), 0 };
  const Joint mfjRightHand{ "RightHand", segName2Idx("RightHand"), 0 };

  // Left leg
  const Joint mfjLeftUpperLeg{ "LeftUpperLeg", segName2Idx("LeftUpperLeg"), 0 };
  const Joint mfjLeftLowerLeg{ "LeftLowerLeg", segName2Idx("LeftLowerLeg"), 0 };
  const Joint mfjLeftFoot{ "LeftFoot", segName2Idx("LeftFoot"), 0 };

  // Right leg
  const Joint mfjRightUpperLeg{ "RightUpperLeg", segName2Idx("RightUpperLeg"), 0 };
  const Joint mfjRightLowerLeg{ "RightLowerLeg", segName2Idx("RightLowerLeg"), 0 };
  const Joint mfjRightFoot{ "RightFoot", segName2Idx("RightFoot"), 0 };

  // 2) Groups
  const Group mfgTorso{ "Torso", mfjPelvis, { mfjPelvis, mfjT8, mfjT12 }, { mfjPelvis, mfjT12 } };
  const Group mfgHead{ "Head", mfjPelvis, { mfjNeck, mfjHead }, { mfjNeck, mfjHead } };
  const Group mfgLeftArm{ "LeftArm", mfjPelvis, { mfjLeftUpperArm, mfjLeftForeArm, mfjLeftHand }, { mfjNeck, mfjHead } };
  const Group mfgRightArm{ "RightArm", mfjPelvis, { mfjRightUpperArm, mfjRightForeArm, mfjRightHand }, { mfjNeck, mfjHead } };
  const Group mfgLeftLeg{ "LeftLeg", mfjPelvis, { mfjLeftUpperLeg, mfjLeftLowerLeg, mfjLeftFoot }, { mfjPelvis, mfjT12 } };
  const Group mfgRightLeg{ "RightLeg", mfjPelvis, { mfjRightUpperLeg, mfjRightLowerLeg, mfjRightFoot }, { mfjPelvis, mfjT12 } };

  // 3) Hierarchy
  const Hierarchy mfHierarchy{ mfjPelvis, { mfgTorso, mfgHead, mfgLeftArm, mfgRightArm, mfgLeftLeg, mfgRightLeg } };

  hierarchy_ = mfHierarchy;
}

void MotionFlux::rootSequenceNormalize()
{
  const auto homogenousTFromSegPoint = [](const Skeleton& skeleton, const unsigned int segIdx, const unsigned int pointIdx)
  {
    const Segments& segs = *skeleton.getSegments();
    return HomogenousTransform(Transform(segs[segIdx].quat, segs[segIdx](pointIdx)));
  };

  // Set skeleton to the initial animation state
  skeleton_.setState(animation_[0]);
  // Extract root transform of the initial animation state
  const HomogenousTransform TrootFrame0G = homogenousTFromSegPoint(skeleton_, hierarchy_.root.jointIdx, hierarchy_.root.pointIdx);
  // Extract the group transforms of the initial animation state
  std::vector<HomogenousTransform> TgroupFrame0Gs;
  for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
    TgroupFrame0Gs.emplace_back(homogenousTFromSegPoint(skeleton_, hierarchy_.groups[g].parent.jointIdx, hierarchy_.groups[g].parent.pointIdx));

  // Do the normalization for each frame for each joint in each group
  rsnTrajectories_.clear();
  for (unsigned int f = 0; f < animation_.size(); ++f)
  {
    const Vector& state = animation_[f];
    skeleton_.setState(state);

    rsnTrajectories_.emplace_back();
    for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
    {
      const Group& group = hierarchy_.groups[g];
      const HomogenousTransform TgroupFrame0G = TgroupFrame0Gs[g];
      const HomogenousTransform TgroupG = homogenousTFromSegPoint(skeleton_, group.parent.jointIdx, group.parent.pointIdx);

      rsnTrajectories_.back().emplace_back();
      for (unsigned int j = 0; j < group.joints.size(); ++j)
      {
        const Joint& joint = group.joints[j];
        const HomogenousTransform TjointG = homogenousTFromSegPoint(skeleton_, joint.jointIdx, joint.pointIdx);

        // Root sequence normalization (paper eq. 3) 
        const HomogenousTransform TjointNorm = (TrootFrame0G.inverse() * TgroupFrame0G) * (TgroupG.inverse() * TjointG);

        rsnTrajectories_.back().back().emplace_back();
        rsnTrajectories_[f][g][j] = TjointNorm;
      }
    }
  }
}

std::vector<Vector> MotionFlux::rnsAnimation()
{
  const auto groupJointIdxs = [this](const unsigned int segmentIdx)
  {
    const std::string segName = (*skeleton_.getSegments())[segmentIdx].getName();
    for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
    {
      for (unsigned int j = 0; j < hierarchy_.groups[g].joints.size(); ++j)
      {
        if (hierarchy_.groups[g].joints[j].name == segName)
          return std::make_pair((int)g, (int)j);
      }
    }
    return std::make_pair<int, int>(-1, -1);
  };

  const Segments& segments = *skeleton_.getSegments();
  const unsigned int numSegments = segments.size();
  std::vector<Vector> ret;
  for (unsigned int f = 0; f < animation_.size(); ++f)
  {
    Vector newState = Vector::Zero(14 * numSegments);
    for (unsigned int s = 0; s < numSegments; ++s)
    {
      const auto gjIdxs = groupJointIdxs(s);
      const int g = gjIdxs.first;
      const int j = gjIdxs.second;
      if (g < 0 || j < 0)
      {
        newState.segment(14 * s, 14).segment(0, 3) = Vector3(0, 0, -100);
        newState.segment(14 * s, 14).segment(3, 4) = QuaternionMath::toVector(Quaternion::Identity());
        newState.segment(14 * s, 14).segment(7, 7) = Transform().asVector();
      }
      else
      {
        newState.segment(14 * s, 14).segment(0, 3) = rsnTrajectories_[f][g][j].pos();
        newState.segment(14 * s, 14).segment(3, 4) = QuaternionMath::toVector(rsnTrajectories_[f][g][j].quat());
        newState.segment(14 * s, 14).segment(7, 7) = Transform().asVector();
      }
    }
    ret.emplace_back(newState);
  }
  return ret;
}

void MotionFlux::buildSplineTrajectories(const ScalarType cutoffFreq)
{
  const auto extractTrajectory = [this](const unsigned int groupIdx, const unsigned int jointIdx)
  {
    std::vector<Vector3> ret;
    for (unsigned int f = 0; f < rsnTrajectories_.size(); ++f)
      ret.emplace_back(rsnTrajectories_[f][groupIdx][jointIdx].pos());
    return ret;
  };

  const auto calculateTrajectorySpline = [this, cutoffFreq](const std::vector<Vector3>& trajectory)
  {
    // Smooth the trajectory
    Vector x(trajectory.size()), y(trajectory.size()), z(trajectory.size());
    for (unsigned int f = 0; f < trajectory.size(); ++f)
    {
      x[f] = trajectory[f].x();
      y[f] = trajectory[f].y();
      z[f] = trajectory[f].z();
    }

    ButterworthLP butter(samplingFreq_, cutoffFreq, 4);
    x = butter.filter(x, x[0], true);
    y = butter.filter(y, y[0], true);
    z = butter.filter(z, z[0], true);

    // Construct spline based on smoothed controls
    Vector knots(trajectory.size());
    for (unsigned int f = 0; f < trajectory.size(); ++f)
      knots[f] = ScalarType(f) / samplingFreq_;

    Matrix ctrls(trajectory.size(), 3);
    for (unsigned int f = 0; f < trajectory.size(); ++f)
      ctrls.row(f) = Vector3(x[f], y[f], z[f]);

    return Eigen::SplineFitting<Eigen::Spline<ScalarType, 3, 3>>::Interpolate(ctrls.transpose(), 3, knots);
  };

  if (rsnTrajectories_.empty())
    return;

  // Prepare flattened spline trajectory vector
  unsigned int numTrajectories = 0;
  std::vector<std::pair<unsigned int, unsigned int>> mapping;
  for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
  {
    for (unsigned int j = 0; j < hierarchy_.groups[g].joints.size(); ++j)
    {
      mapping.push_back({ g, j });
      numTrajectories++;
    }
  }

  // Spline construction is quite expensive => use OpenMP; hence flattened structure for performance reasons
  std::vector<Eigen::Spline<ScalarType, 3, 3>> flattenedSplineTrajectories(numTrajectories);
  #pragma omp parallel for
  for (int t = 0; t < numTrajectories; ++t)
  {
    const unsigned int g = mapping[t].first;
    const unsigned int j = mapping[t].second;
    flattenedSplineTrajectories[t] = calculateTrajectorySpline(extractTrajectory(g, j)); 
  }

  // De-flatten spline trajectory vector
  trajectorySplines_.clear();
  for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
  {
    trajectorySplines_.emplace_back();
    for (unsigned int j = 0; j < hierarchy_.groups[g].joints.size(); ++j)
      trajectorySplines_.back().emplace_back();
  }
  for (unsigned int t = 0; t < numTrajectories; ++t)
  {
    const unsigned int g = mapping[t].first;
    const unsigned int j = mapping[t].second;
    trajectorySplines_[g][j] = flattenedSplineTrajectories[t];
  }
}

ScalarType MotionFlux::objective(const unsigned int groupIdx, ScalarType t0, const ScalarType t1, const ScalarType betaV, const ScalarType betaS, const ScalarType dt)
{
  // Implements eq. 6 from the paper

  const auto gVectorGroupJointIdxs = [this, groupIdx]()
  {
    const Joint& gJ0 = hierarchy_.groups[groupIdx].gVector.first;
    const Joint& gJ1 = hierarchy_.groups[groupIdx].gVector.second;
    std::pair<std::pair<int, int>, std::pair<int, int>> ret = { {-1, -1}, {-1,-1} };
    for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
    {
      for (unsigned int j = 0; j < hierarchy_.groups[g].joints.size(); ++j)
      {
        if (hierarchy_.groups[g].joints[j].name == gJ0.name)
          ret.first = { g, j };
        if (hierarchy_.groups[g].joints[j].name == gJ1.name)
          ret.second = { g, j };
      }
    }
    return ret;
  };

  // Pojection vector g for this group
  const auto gVectorGJIdxs = gVectorGroupJointIdxs();
  const auto& gSpline0 = trajectorySplines_[gVectorGJIdxs.first.first][gVectorGJIdxs.first.second];
  const auto& gSpline1 = trajectorySplines_[gVectorGJIdxs.second.first][gVectorGJIdxs.second.second];
  const Vector3 g = gSpline1(0) - gSpline0(0);

  // Terms: MotionFlux (mF), velocityEndpointRegularization (vE), trajectorySegmentLenghtRegularization (sL)
  ScalarType mF = 0, vE = 0, sL = 0;
  for (unsigned int j = 0; j < trajectorySplines_[groupIdx].size(); ++j)
  {
    const auto& spline = trajectorySplines_[groupIdx][j];
    const Vector3& velt0 = spline.derivatives(t0, 1).col(1);
    const Vector3& velt1 = spline.derivatives(t1, 1).col(1);

    // Evaluate mF and sL terms by integrating (trapezoidal rule) from t0 to t1
    unsigned int steps = std::ceil((t1 - t0) / dt);
    for (unsigned int s = 0; s < steps; ++s)
    {
      const ScalarType a = t0 + s * dt;
      const ScalarType b = a + dt;
      const ScalarType h = 0.5 * dt;

      const Matrix& derivsa = spline.derivatives(a, 2);
      const Matrix& derivsb = spline.derivatives(b, 2);
      const Vector3& vela = derivsa.col(1);
      const Vector3& velb = derivsb.col(1);
      const Vector3& acca = derivsa.col(2);
      const Vector3& accb = derivsb.col(2);

      // mF term
      mF += h * (std::abs(acca.dot(g)) + std::abs(accb.dot(g)));

      // sL term (re-parametrized integral difference)
      sL += h * (vela.norm() + velb.norm());
    }

    // Evaluate vE term
    vE += (velt0.squaredNorm() + velt1.squaredNorm());
  }

  // Combine all terms weighted by the hyperparameters
  return mF - 0.5 * betaV * vE - betaS * sL;
}

ScalarType MotionFlux::objective(const std::string& groupName, ScalarType t0, const ScalarType t1, const ScalarType betaV, const ScalarType betaS, const ScalarType dt)
{
  unsigned int groupIdx = 0;
  for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
  {
    if (hierarchy_.groups[g].name == groupName)
    {
      groupIdx = g;
      break;
    }
  }

  return objective(groupIdx, t0, t1, betaV, betaS, dt);
}

ScalarType MotionFlux::objectiveDeriv(const unsigned int groupIdx, ScalarType t, const ScalarType betaV, const ScalarType betaS)
{
  // Implements eq. 7 from the paper

  const auto gVectorGroupJointIdxs = [this, groupIdx]()
  {
    const Joint& gJ0 = hierarchy_.groups[groupIdx].gVector.first;
    const Joint& gJ1 = hierarchy_.groups[groupIdx].gVector.second;
    std::pair<std::pair<int, int>, std::pair<int, int>> ret = { {-1, -1}, {-1,-1} };
    for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
    {
      for (unsigned int j = 0; j < hierarchy_.groups[g].joints.size(); ++j)
      {
        if (hierarchy_.groups[g].joints[j].name == gJ0.name)
          ret.first = { g, j };
        if (hierarchy_.groups[g].joints[j].name == gJ1.name)
          ret.second = { g, j };
      }
    }
    return ret;
  };

  // Pojection vector g for this group
  const auto gVectorGJIdxs = gVectorGroupJointIdxs();
  const auto& gSpline0 = trajectorySplines_[gVectorGJIdxs.first.first][gVectorGJIdxs.first.second];
  const auto& gSpline1 = trajectorySplines_[gVectorGJIdxs.second.first][gVectorGJIdxs.second.second];
  const Vector3 g = gSpline1(0) - gSpline0(0);

  ScalarType mFDeriv = 0, vEDeriv = 0, sLDeriv = 0;
  for (unsigned int j = 0; j < trajectorySplines_[groupIdx].size(); ++j)
  {
    const auto& spline = trajectorySplines_[groupIdx][j];
    const Matrix& derivsa = spline.derivatives(t, 2);
    const Vector3& velt = derivsa.col(1);
    const Vector3& acct = derivsa.col(2);

    mFDeriv += std::abs(acct.dot(g));
    vEDeriv += velt.dot(acct) / velt.norm();
    sLDeriv += velt.norm();
  }

  return mFDeriv - betaV * vEDeriv - betaS * sLDeriv;
}

ScalarType MotionFlux::objectiveDeriv(const std::string& groupName, ScalarType t, const ScalarType betaV, const ScalarType betaS)
{
  unsigned int groupIdx = 0;
  for (unsigned int g = 0; g < hierarchy_.groups.size(); ++g)
  {
    if (hierarchy_.groups[g].name == groupName)
    {
      groupIdx = g;
      break;
    }
  }

  return objectiveDeriv(groupIdx, t, betaV, betaS);
}