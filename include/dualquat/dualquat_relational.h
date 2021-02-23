/**
 * @file dualquat/dualquat_relational.h
 * @brief This file provides relational functions for dual quaternion types.
 */
#pragma once

#include <cassert>
#include "quat_relational.h"
#include "dualquat_query.h"

namespace dualquat
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

/**
 * Returns true if the two dual quaternions represent the same transformation.
 * Note that every transformation is represented by two values, dq and -dq.
 */
template<typename T>
bool same_transformation(const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs, T tol)
{
    assert(is_unit(lhs, tol));
    assert(is_unit(rhs, tol));

    return almost_equal(lhs, rhs, tol)
        || almost_equal(lhs, -rhs, tol);
}

}   // namespace dualquat
