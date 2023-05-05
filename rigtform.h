#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    t_ = t;
    r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
    t_ = t;
    r_ = Quat();
  }

  explicit RigTForm(const Quat& r) {
    r_ = r;
    t_ = Cvec3();
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    // RBT = TR
    // RBT * cvec = TR * Cvec
    Cvec3 temp;
    temp[0] = a[0];
    temp[1] = a[1];
    temp[2] = a[2];
    temp = r_ * temp;

    if (a[3] > 0.5) {
      temp = t_ + temp;
    }

    Cvec4 sol;
    sol[0] = temp[0];
    sol[1] = temp[1];
    sol[2] = temp[2];
    sol[3] = a[3];

    return sol;
  }

  RigTForm operator * (const RigTForm& a) const {
    return RigTForm(t_ + r_ * a.getTranslation(), r_ * a.getRotation());
  }
};

inline RigTForm inv(const RigTForm& tform) {
  return RigTForm(inv(tform.getRotation()) * tform.getTranslation() * -1, inv(tform.getRotation()));
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  Matrix4 T = Matrix4::makeTranslation(tform.getTranslation());
  Matrix4 R = quatToMatrix(tform.getRotation());
  return T * R;
}

inline RigTForm rbtMtoOwrtA(const RigTForm& m, const RigTForm& a) {
  RigTForm a_inv = inv(a);
  return a * m * a_inv;
}

#endif
