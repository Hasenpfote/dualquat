/**
 * @file dualquat/quat_relational.h
 * @brief This file provides relational functions for quaternion types.
 */
#pragma once

#include <cassert>
#include "relational.h"

namespace eigen_ext
{

template<typename T>
bool almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T rtol, T atol)
{
    return almost_equal(lhs.coeffs(), rhs.coeffs(), rtol, atol);
}

template<typename T>
bool almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tol)
{
    return almost_equal(lhs.coeffs(), rhs.coeffs(), tol);
}

template<typename T>
bool almost_zero(const Eigen::Quaternion<T>& q, T tol)
{
    return almost_zero(q.coeffs(), tol);
}

/**
 * Returns true if the two quaternions represent the same rotation.
 * Note that every rotation is represented by two values, q and -q.
 */
template<typename T>
bool same_rotation(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tol)
{
#ifndef NDEBUG
    using Matrix1 = Eigen::Matrix<T, 1, 1>;
#endif
    assert(almost_equal(Matrix1(lhs.squaredNorm()), Matrix1(T(1)), tol));
    assert(almost_equal(Matrix1(rhs.squaredNorm()), Matrix1(T(1)), tol));

    using Coefficients = typename Eigen::Quaternion<T>::Coefficients;

    return almost_equal(lhs.coeffs(), rhs.coeffs(), tol)
        || almost_equal(lhs.coeffs(), Coefficients(-rhs.coeffs()), tol);
}

}   // namespace eigen_ext
