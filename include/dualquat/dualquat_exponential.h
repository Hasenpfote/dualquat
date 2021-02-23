/**
 * @file dualquat/dualquat_exponential.h
 * @brief This file provides exponential functions for dual quaternion types.
 */
#pragma once

#include <cmath>
#include "quat_exponential.h"

namespace dualquat
{

/**
 * Returns a exponential of a dual quaternion.
 */
template<typename T>
DualQuaternion<T>
exp(const DualQuaternion<T>& dq)
{
    DualQuaternion<T> temp;
    temp.real() = dualquat::exp(dq.real());

    T sinc, alpha;
    const auto vn = dq.real().vec().norm();
    if(vn > T(0))
    {
        sinc = std::sin(vn) / vn;
        alpha = (std::cos(vn) - sinc) / (vn * vn);
    }
    else
    {
        sinc = T(1);
        alpha = - T(1) / T(3);
    }

    const auto vdot = dq.real().vec().dot(dq.dual().vec());

    temp.dual().w() = - vdot * sinc;
    temp.dual().vec() = sinc * dq.dual().vec() + vdot * alpha * dq.real().vec();
    temp.dual().coeffs() = std::exp(dq.real().w()) * temp.dual().coeffs() + dq.dual().w() * temp.real().coeffs();

    return temp;
}

/**
 * Returns a logarithm of a dual quaternion.
 */
template<typename T>
DualQuaternion<T>
log(const DualQuaternion<T>& dq)
{
    DualQuaternion<T> temp;
    temp.real() = dualquat::log(dq.real());

    T phi_over_vn, alpha;
    const auto vn = dq.real().vec().norm();
    const auto phi = std::atan2(vn, dq.real().w());
    const auto qn = dq.real().norm();
    if(vn > T(0))
    {
        phi_over_vn = phi / vn;
        alpha = (dq.real().w() - phi_over_vn * qn * qn) / (vn * vn);
    }
    else
    {
        phi_over_vn = T(1) / qn;
        alpha = (- T(2) / T(3)) / qn;
    }

    temp.dual().w() = dq.real().dot(dq.dual()) / (qn * qn);
    temp.dual().vec() = phi_over_vn * dq.dual().vec()
        + ((dq.real().vec().dot(dq.dual().vec()) * alpha - dq.dual().w()) / (qn * qn)) * dq.real().vec();

    return temp;
}

/**
 * Returns a dual quaternion raised to a power.
 */
template<typename T>
DualQuaternion<T>
pow(const DualQuaternion<T>& dq, T t)
{
    return exp(t * log(dq));
}

}   // namespace dualquat
