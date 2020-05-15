#pragma once

namespace eigen_ext
{

/*!
 * This transformation applies first the rotation and then the translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Quaternion<T>& r, const Eigen::Matrix<T, 3, 1>& t)
{
    auto dual = Eigen::Quaternion<T>(0, t.x(), t.y(), t.z()) * r;
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/*!
 * This transformation applies first the translation and then the rotation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Matrix<T, 3, 1>& t, const Eigen::Quaternion<T>& r)
{
    auto dual = r * Eigen::Quaternion<T>(0, t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(r, dual);
}

/*!
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

/*!
 * This transformation applies pure translation.
 */
template<typename T>
DualQuaternion<T>
transformation(const Eigen::Matrix<T, 3, 1>& t)
{
    auto dual = Eigen::Quaternion<T>(0, t.x(), t.y(), t.z());
    dual.coeffs() *= T(0.5);
    return DualQuaternion<T>(Eigen::Quaternion<T>::Identity(), dual);
}

}   // namespace eigen_ext