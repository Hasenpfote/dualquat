/**
 * @file dualquat/dualquat_transformation.h
 * @brief This file provides transformation functions for dual quaternion types.
 */
#pragma once

#include "dualquat_common.h"

namespace eigen_ext
{

/**
 * This transformation applies first the rotation and then the translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Quaternion<T>& r, const Eigen::Matrix<T, 3, 1>& t)
{
    auto dual = Eigen::Quaternion<T>(T(0), t.x(), t.y(), t.z()) * r;
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/**
 * This transformation applies first the translation and then the rotation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Matrix<T, 3, 1>& t, const Eigen::Quaternion<T>& r)
{
    auto dual = r * Eigen::Quaternion<T>(T(0), t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/**
 * This transformation applies pure rotation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Quaternion<T>& r)
{
    return DualQuaternion<T>(
        r,
        Eigen::Quaternion<T>(Eigen::Quaternion<T>::Coefficients::Zero()));
}

/**
 * This transformation applies pure translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Matrix<T, 3, 1>& t)
{
    auto dual = Eigen::Quaternion<T>(T(0), t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(Eigen::Quaternion<T>::Identity(), dual);
}

/**
 * Returns the difference between the two dual quaternions representing the transformation.
 */
template<typename T>
DualQuaternion<T>
transformational_difference(const DualQuaternion<T>& dq1, const DualQuaternion<T>& dq2)
{
    return quaternion_conjugate(dq1) * dq2;
}

}   // namespace eigen_ext
