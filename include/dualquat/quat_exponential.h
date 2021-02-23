/**
 * @file dualquat/quat_exponential.h
 * @brief This file provides exponential functions for quaternion types.
 */
#pragma once

#include <cmath>

namespace dualquat
{

/**
 * Returns a exponential of a quaternion.
 */
template<typename T>
Quaternion<T>
exp(const Quaternion<T>& q)
{
    const auto vn = q.vec().norm();
    const auto sinc = (vn > T(0)) ? std::sin(vn) / vn : T(1);

    Quaternion<T> temp;
    temp.w() = std::cos(vn);
    temp.vec() = sinc * q.vec();
    temp.coeffs() *= std::exp(q.w());
    return temp;
}

/**
 * Returns a logarithm of a quaternion.
 */
template<typename T>
Quaternion<T>
log(const Quaternion<T>& q)
{
    const auto vn = q.vec().norm();
    const auto phi = std::atan2(vn, q.w());
    const auto qn = q.norm();
    const auto phi_over_vn = (vn > T(0)) ? phi / vn : T(1) / qn;

    Quaternion<T> temp;
    temp.w() = std::log(qn);
    temp.vec() = phi_over_vn * q.vec();
    return temp;
}

}   // namespace dualquat
