#pragma once

#include <cmath>
#include <tuple>
#include "dualquat_common.h"
#include "dualquat_exponential.h"

namespace eigen_ext
{

/**
 * Transforms a vector with a dual quaternion.
 */
template<typename T>
Eigen::Matrix<T, 3, 1>
transform(const DualQuaternion<T>& dq, const Eigen::Matrix<T, 3, 1>& v)
{
    return (dq * DualQuaternion<T>(v) * total_conjugate(dq)).dual().vec();
}

template<typename T>
DualQuaternion<T>
convert_to_dualquat(const Eigen::Matrix<T, 3, 1>& l, const Eigen::Matrix<T, 3, 1>& m, T theta, T d)
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

template<typename T>
std::tuple<Eigen::Matrix<T, 3, 1>, Eigen::Matrix<T, 3, 1>, T, T>
convert_to_screw(const DualQuaternion<T>& dq)
{
    using Vector3 = Eigen::Matrix<T, 3, 1>;

    const auto half_theta = std::acos(dq.real().w());
    const auto theta = T(2) * half_theta;
    const Vector3 l = dq.real().vec() / std::sin(half_theta);
    const Vector3 t = T(2) * (dq.dual() * dq.real().conjugate()).vec();
    const auto d = t.dot(l);
    const Vector3 m = T(0.5) * (t.cross(l) + (t - d * l) * std::tan(half_theta));
    return std::make_tuple(l, m, theta, d);
}

template<typename T>
DualQuaternion<T>
sclerp(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2, T t)
{
    return dq1 * pow(quaternion_conjugate(dq1) * dq2, t);
}

template<typename T>
DualQuaternion<T>
sclerp_shortestpath(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2, T t)
{
    const auto cos_half_angle = dq1.real().dot(dq2.real());
    const auto diff = quaternion_conjugate(dq1) * (cos_half_angle < T(0) ? -dq2 : dq2);
    return dq1 * pow(diff, t);
}

}   // namespace eigen_ext
