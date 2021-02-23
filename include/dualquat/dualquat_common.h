/**
 * @file dualquat/dualquat_common.h
 * @brief This file provides common functions for dual quaternion types.
 */
#pragma once

#include <utility>

namespace dualquat
{

/* Dual-valued functions */

template<typename T>
std::pair<T, T>
norm(const DualQuaternion<T>& dq)
{
    const auto n = dq.real().norm();
    return std::make_pair(n, dq.real().dot(dq.dual()) / n);
}

template<typename T>
std::pair<T, T>
squared_norm(const DualQuaternion<T>& dq)
{
    return std::make_pair(dq.real().squaredNorm(), T(2) * dq.real().dot(dq.dual()));
}

/* DualQuaternion-valued functions */

template<typename T>
DualQuaternion<T>
identity()
{
    return DualQuaternion<T>(
        Quaternion<T>::Identity(),
        Quaternion<T>(Quaternion<T>::Coefficients::Zero()));
}

template<typename T>
DualQuaternion<T>
inverse(const DualQuaternion<T>& dq)
{
    const auto inv_real = dq.real().inverse();

    DualQuaternion<T> temp;
    temp.real() = inv_real;
    temp.dual() = -(inv_real * dq.dual() * inv_real).coeffs();
    return temp;
}

template<typename T>
DualQuaternion<T>
normalize(const DualQuaternion<T>& dq)
{
    const auto r = dq.real().normalized();
    const auto d = dq.dual().coeffs() / dq.real().norm();

    DualQuaternion<T> temp;
    temp.real() = r;
    temp.dual() = d - r.coeffs().dot(d) * r.coeffs();
    return temp;
}

/**
 * Dual-conjugate.
 */
template<typename T>
DualQuaternion<T>
dual_conjugate(const DualQuaternion<T>& dq)
{
    DualQuaternion<T> temp(dq);
    temp.dual().coeffs() = -temp.dual().coeffs();
    return temp;
}

/**
 * Quaternion-conjugate.
 */
template<typename T>
DualQuaternion<T>
quaternion_conjugate(const DualQuaternion<T>& dq)
{
    return DualQuaternion<T>(dq.real().conjugate(), dq.dual().conjugate());
}

/**
 * Total-conjugate.
 */
template<typename T>
DualQuaternion<T>
total_conjugate(const DualQuaternion<T>& dq)
{
    DualQuaternion<T> temp(dq.real().conjugate(), dq.dual().conjugate());
    temp.dual().coeffs() = -temp.dual().coeffs();
    return temp;
}

/**
 * Returns the difference between the two dual quaternions.
 */
template<typename T>
DualQuaternion<T>
difference(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2)
{
    return inverse(dq1) * dq2;
}

}   // namespace dualquat
