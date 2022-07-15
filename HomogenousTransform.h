#ifndef _HOMOGENOUS_TRANSFORM_H_
#define _HOMOGENOUS_TRANSFORM_H_

#include "EigenTypes.h"
#include "Transform.h"

class HomogenousTransform : public Matrix4x4
{
public:
  HomogenousTransform();

  HomogenousTransform(const HomogenousTransform& h);

  HomogenousTransform(const Matrix4x4& m);

  HomogenousTransform(const Matrix3x3& m, const Vector3& v);

  HomogenousTransform(const Transform& t);

  Matrix3x3 rot() const;

  void rot(const Matrix3x3 m);

  Quaternion quat() const;

  void quat(const Quaternion& q);

  Vector3 pos() const;

  void pos(const Vector3& v);

  Transform transform() const;

  void transform(const Transform& t);

  HomogenousTransform inverse() const;
};

#endif