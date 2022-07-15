#include "HomogenousTransform.h"

#include "QuaternionMath.h"

HomogenousTransform::HomogenousTransform()
{
  block(0, 0, 4, 4) = Matrix4x4::Zero();
  block(0, 0, 3, 3) = Matrix3x3::Identity();
  (*this)(3, 3) = 1.0;
}

HomogenousTransform::HomogenousTransform(const HomogenousTransform& h)
  : HomogenousTransform()
{
  this->block(0, 0, 4, 4) = h;
}

HomogenousTransform::HomogenousTransform(const Matrix4x4& m)
  : HomogenousTransform()
{
  this->block(0, 0, 4, 4) = m;
}

HomogenousTransform::HomogenousTransform(const Matrix3x3& m, const Vector3& v)
  : HomogenousTransform()
{
  rot(m);
  pos(v);
}

HomogenousTransform::HomogenousTransform(const Transform& t)
  : HomogenousTransform()
{
  quat(t.quat);
  pos(t.pos);
}

Matrix3x3 HomogenousTransform::rot() const
{
  return block(0, 0, 3, 3);
}

void HomogenousTransform::rot(const Matrix3x3 m)
{
  block(0, 0, 3, 3) = m;
}

Quaternion HomogenousTransform::quat() const
{
  return Quaternion(rot());
}

void HomogenousTransform::quat(const Quaternion& q)
{
  rot(QuaternionMath::R(q));
}

Vector3 HomogenousTransform::pos() const
{
  return block(0, 3, 3, 1);
}

void HomogenousTransform::pos(const Vector3& v)
{
  block(0, 3, 3, 1) = v;
};

Transform HomogenousTransform::transform() const
{
  return Transform(Quaternion(rot()), pos());
}

void HomogenousTransform::transform(const Transform& t)
{
  quat(t.quat);
  pos(t.pos);
}

HomogenousTransform HomogenousTransform::inverse() const
{
  const Matrix3x3 rotT = rot().transpose();
  return HomogenousTransform(rotT, -rotT * pos());
}