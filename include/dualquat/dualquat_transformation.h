/**
 * @file dualquat/dualquat_transformation.h
 * @brief This file provides transformation functions for dual quaternion types.
 */
#pragma once

#include "dualquat_common.h"

namespace dualquat
{

/**
 * This transformation applies first the rotation and then the translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Quaternion<T>& r, const Vector3<T>& t)
{
    auto dual = Quaternion<T>(T(0), t.x(), t.y(), t.z()) * r;
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/**
 * This transformation applies first the translation and then the rotation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Vector3<T>& t, const Quaternion<T>& r)
{
    auto dual = r * Quaternion<T>(T(0), t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/**
 * This transformation applies pure rotation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Quaternion<T>& r)
{
    return DualQuaternion<T>(
        r,
        Quaternion<T>(Quaternion<T>::Coefficients::Zero()));
}

/**
 * This transformation applies pure translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Vector3<T>& t)
{
    auto dual = Quaternion<T>(T(0), t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(Quaternion<T>::Identity(), dual);
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

/**
 * Transforms a point with a unit dual quaternion.
 * @param [in] dq   A unit dual quaternion representing a transformation.
 * @param [in] p    A point represented by a dual quaternion.
 */
template<typename T>
DualQuaternion<T>
transform_point(const DualQuaternion<T>& dq, const DualQuaternion<T>& p)
{
    return dq * p * total_conjugate(dq);
}

/**
 * Transforms a point with a unit dual quaternion.
 * @see dualquat::transform_point()
 */
template<typename T>
DualQuaternion<T>
transform_point(const DualQuaternion<T>& dq, const Vector3<T>& p)
{
    return transform_point(dq, DualQuaternion<T>(p));
}

/**
 * Transforms a line with a unit dual quaternion.
 * @param [in] dq   A unit dual quaternion representing a transformation.
 * @param [in] l    A line in the Plucker coordinates represented by a dual quaternion.
 */
template<typename T>
DualQuaternion<T>
transform_line(const DualQuaternion<T>& dq, const DualQuaternion<T>& l)
{
    return dq * l * quaternion_conjugate(dq);
}

/**
 * Transforms a line with a unit dual quaternion.
 * @see dualquat::transform_line()
 */
template<typename T>
DualQuaternion<T>
transform_line(const DualQuaternion<T>& dq, const Vector3<T>& l, const Vector3<T>& m)
{
    return transform_line(dq, DualQuaternion<T>(l, m));
}

}   // namespace dualquat
