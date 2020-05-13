#pragma once

#include <gtest/gtest.h>

namespace gtest_helper_detail
{

template<typename T>
bool almost_equal(T lhs, T rhs, T tolerance);

extern template bool almost_equal(float lhs, float rhs, float tolerance);
extern template bool almost_equal(double lhs, double rhs, double tolerance);

template<typename T>
bool not_almost_equal(T lhs, T rhs, T tolerance);

extern template bool not_almost_equal(float lhs, float rhs, float tolerance);
extern template bool not_almost_equal(double lhs, double rhs, double tolerance);

template<typename T>
bool quat_almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tolerance);

extern template bool quat_almost_equal(const Eigen::Quaternion<float>& lhs, const Eigen::Quaternion<float>& rhs, float);
extern template bool quat_almost_equal(const Eigen::Quaternion<double>& lhs, const Eigen::Quaternion<double>& rhs, double);

template<typename T>
bool quat_not_almost_equal(const Eigen::Quaternion<T>& lhs, const Eigen::Quaternion<T>& rhs, T tolerance);

extern template bool quat_not_almost_equal(const Eigen::Quaternion<float>& lhs, const Eigen::Quaternion<float>& rhs, float);
extern template bool quat_not_almost_equal(const Eigen::Quaternion<double>& lhs, const Eigen::Quaternion<double>& rhs, double);

}   // gtest_helper_detail

#define EXPECT_ALMOST_EQUAL(type, val1, val2, tolerance) \
    EXPECT_PRED3(gtest_helper_detail::almost_equal<type>, val1, val2, tolerance)

#define EXPECT_NOT_ALMOST_EQUAL(type, val1, val2, tolerance) \
    EXPECT_PRED3(gtest_helper_detail::not_almost_equal<type>, val1, val2, tolerance)

#define EXPECT_QUAT_ALMOST_EQUAL(type, val1, val2, tolerance) \
    EXPECT_PRED3(gtest_helper_detail::quat_almost_equal<type>, val1, val2, tolerance)

#define EXPECT_QUAT_NOT_ALMOST_EQUAL(type, val1, val2, tolerance) \
    EXPECT_PRED3(gtest_helper_detail::quat_not_almost_equal<type>, val1, val2, tolerance)
