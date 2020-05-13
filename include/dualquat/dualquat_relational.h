#pragma once

#include "quat_relational.h"

namespace eigen_ext
{

template<typename T>
bool almost_equal(const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs, T rtol, T atol)
{
    return almost_equal(lhs.real().coeffs(), rhs.real().coeffs(), rtol, atol)
        && almost_equal(lhs.dual().coeffs(), rhs.dual().coeffs(), rtol, atol);
}

template<typename T>
bool almost_equal(const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs, T tol)
{
    return almost_equal(lhs.real().coeffs(), rhs.real().coeffs(), tol)
        && almost_equal(lhs.dual().coeffs(), rhs.dual().coeffs(), tol);
}

template<typename T>
bool almost_zero(const DualQuaternion<T>& dq, T tol)
{
    return almost_zero(dq.real().coeffs(), tol)
        && almost_zero(dq.dual().coeffs(), tol);
}

}   // namespace eigen_ext
