#pragma once

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

}   // namespace eigen_ext
