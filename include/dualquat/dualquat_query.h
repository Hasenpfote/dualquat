/**
 * @file dualquat/dualquat_query.h
 * @brief This file provides query functions for dual quaternion types.
 */
#pragma once

#include "dualquat_relational.h"

namespace dualquat
{

template<typename T>
bool
is_zero(const DualQuaternion<T>& dq, T tol)
{
    return almost_zero(dq, tol);
}

template<typename T>
bool
is_real(const DualQuaternion<T>& dq, T tol)
{
    return almost_zero(dq.real().vec(), tol)
        && almost_zero(dq.dual().vec(), tol);
}

template<typename T>
bool
is_pure(const DualQuaternion<T>& dq, T tol)
{
    using Matrix1 = Eigen::Matrix<T, 1, 1>;

    return almost_zero(Matrix1(dq.real().w()), tol)
        && almost_zero(Matrix1(dq.dual().w()), tol);
}

template<typename T>
bool
is_unit(const DualQuaternion<T>& dq, T tol)
{
    using Matrix1 = Eigen::Matrix<T, 1, 1>;

    return almost_equal(Matrix1(dq.real().squaredNorm()), Matrix1(T(1)), tol)
        && almost_equal(Matrix1(dq.real().dot(dq.dual())), Matrix1(T(0)), tol);
}

}   // namespace dualquat
