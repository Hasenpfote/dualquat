/**
 * @file dualquat/dualquat_helper.h
 * @brief This file provides helper functions for dual quaternion types.
 */
#pragma once

#include <cmath>
#include <tuple>
#include "dualquat_common.h"
#include "dualquat_exponential.h"

namespace dualquat
{

/**
 * Conversion from screw parameters to a dual quaternion.
 * @param [in] l        The direction vector of the screw axis.
 * @param [in] m        The moment vector of the screw axis.
 * @param [in] theta    The angle of rotation in radians around the screw axis.
 * @param [in] d        Displacement along the screw axis.
 */
template<typename T>
DualQuaternion<T>
convert_to_dualquat(const Vector3<T>& l, const Vector3<T>& m, T theta, T d)
{
    const auto half_theta = T(0.5) * theta;
    const auto half_d = T(0.5) * d;
    const auto sine = std::sin(half_theta);

    DualQuaternion<T> temp;
    temp.real() = Eigen::AngleAxis<T>(theta, l);
    temp.dual().w() = -half_d * sine;
    temp.dual().vec() = sine * m + half_d * std::cos(half_theta) * l;
    return temp;
}

/**
 * Conversion from a dual quaternion to screw parameters.
 * @param [in] dq   A dual quaternion representing a transformation.
 */
template<typename T>
std::tuple<Vector3<T>, Vector3<T>, T, T>
convert_to_screw(const DualQuaternion<T>& dq)
{
    const auto half_theta = std::acos(dq.real().w());
    const auto theta = T(2) * half_theta;
    const Vector3<T> l = dq.real().vec() / std::sin(half_theta);
    const Vector3<T> t = T(2) * (dq.dual() * dq.real().conjugate()).vec();
    const auto d = t.dot(l);
    const Vector3<T> m = T(0.5) * (t.cross(l) + (t - d * l) * std::tan(half_theta));
    return std::make_tuple(l, m, theta, d);
}

/**
 * Screw linear interpolation.
 * @param [in] dq1  Unit Dual Quaternion as a start point for interpolation.
 * @param [in] dq2  Unit Dual Quaternion as an end point for interpolation.
 * @param [in] t    Value varies between 0 and 1.
 */
template<typename T>
DualQuaternion<T>
sclerp(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2, T t)
{
    return dq1 * pow(quaternion_conjugate(dq1) * dq2, t);
}

/**
 * Screw linear interpolation.
 * @param [in] dq1  Unit Dual Quaternion as a start point for interpolation.
 * @param [in] dq2  Unit Dual Quaternion as an end point for interpolation.
 * @param [in] t    Value varies between 0 and 1.
 */
template<typename T>
DualQuaternion<T>
sclerp_shortestpath(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2, T t)
{
    const auto cos_half_angle = dq1.real().dot(dq2.real());
    const auto diff = quaternion_conjugate(dq1) * (cos_half_angle < T(0) ? -dq2 : dq2);
    return dq1 * pow(diff, t);
}

}   // namespace dualquat
