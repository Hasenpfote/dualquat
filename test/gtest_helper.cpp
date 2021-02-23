#include <Eigen/Geometry>
#include <dualquat/quat_relational.h>
#include "gtest_helper.h"

namespace gtest_helper_detail
{

template<typename T>
bool almost_equal(T lhs, T rhs, T tolerance)
{
    using Matrix1 = Eigen::Matrix<T, 1, 1>;
    return dualquat::almost_equal(Matrix1(lhs), Matrix1(rhs), tolerance);
}

template bool almost_equal(float lhs, float rhs, float tolerance);
template bool almost_equal(double lhs, double rhs, double tolerance);

template<typename T>
bool not_almost_equal(T lhs, T rhs, T tolerance)
{
    using Matrix1 = Eigen::Matrix<T, 1, 1>;
    return !dualquat::almost_equal(Matrix1(lhs), Matrix1(rhs), tolerance);
}

template bool not_almost_equal(float lhs, float rhs, float tolerance);
template bool not_almost_equal(double lhs, double rhs, double tolerance);

template<typename T>
bool quat_almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tolerance)
{
    return dualquat::almost_equal(lhs, rhs, tolerance);
}

template bool quat_almost_equal(const Eigen::Quaternion<float>& lhs, const Eigen::Quaternion<float>& rhs, float);
template bool quat_almost_equal(const Eigen::Quaternion<double>& lhs, const Eigen::Quaternion<double>& rhs, double);

template<typename T>
bool quat_not_almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tolerance)
{
    return !dualquat::almost_equal(lhs, rhs, tolerance);
}

template bool quat_not_almost_equal(const Eigen::Quaternion<float>& lhs, const Eigen::Quaternion<float>& rhs, float);
template bool quat_not_almost_equal(const Eigen::Quaternion<double>& lhs, const Eigen::Quaternion<double>& rhs, double);

}   // namespace gtest_helper_detail
