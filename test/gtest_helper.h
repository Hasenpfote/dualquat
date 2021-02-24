#pragma once

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtest_helper
{

namespace detail
{

template<typename T>
constexpr T abs(T x)
{
    return x < T(0.0) ? -x : x;
}

#if ((defined(_MSVC_LANG) && _MSVC_LANG < 201402L) || __cplusplus < 201402L)
template <class T>
constexpr const T& max(const T& a, const T& b)
{
    return a < b ? b : a;
}
#endif

template<typename T>
bool almost_equal(T lhs, T rhs, T rel_tolerance, T abs_tolerance)
{
#if (__cplusplus >= 201402L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201402L))
    return abs(lhs - rhs)
        <= std::max(abs_tolerance, rel_tolerance * std::max(abs(lhs), abs(rhs)));
#else
    return abs(lhs - rhs)
        <= max(abs_tolerance, rel_tolerance * max(abs(lhs), abs(rhs)));
#endif
}

template<typename T>
bool almost_equal(T lhs, T rhs, T tolerance)
{
#if (__cplusplus >= 201402L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201402L))
    return abs(lhs - rhs)
        <= tolerance * std::max(T(1), std::max(abs(lhs), abs(rhs)));
#else
    return abs(lhs - rhs)
        <= tolerance * max(T(1), max(abs(lhs), abs(rhs)));
#endif
}

template<typename T>
bool not_almost_equal(T lhs, T rhs, T tolerance)
{
    return !almost_equal(lhs, rhs, tolerance);
}

template<typename T>
::testing::AssertionResult
assert_almost_equal_pred3_format(
    const char* e1,
    const char* e2,
    const char* e3,
    T v1,
    T v2,
    T v3)
{
    if(almost_equal(v1, v2, v3))
        return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
        << "pred_text" << "(" << e1 << ", " << e2 << ", " << e3
        << ") evaluates to false, where"
        << "\n"
        << e1 << " evaluates to " << ::testing::PrintToString(v1) << "\n"
        << e2 << " evaluates to " << ::testing::PrintToString(v2) << "\n"
        << e3 << " evaluates to " << ::testing::PrintToString(v3);
}

template<typename T>
::testing::AssertionResult
assert_not_almost_equal_pred3_format(
    const char* e1,
    const char* e2,
    const char* e3,
    T v1,
    T v2,
    T v3)
{
    if(!almost_equal(v1, v2, v3))
        return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
        << "pred_text" << "(" << e1 << ", " << e2 << ", " << e3
        << ") evaluates to false, where"
        << "\n"
        << e1 << " evaluates to " << ::testing::PrintToString(v1) << "\n"
        << e2 << " evaluates to " << ::testing::PrintToString(v2) << "\n"
        << e3 << " evaluates to " << ::testing::PrintToString(v3);
}

template<typename DerivedA, typename DerivedB>
bool almost_equal(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const typename DerivedA::RealScalar& tolerance = Eigen::NumTraits<typename DerivedA::Scalar>::dummy_precision())
{
    return ((lhs - rhs).array().abs()
        <= tolerance * Eigen::MatrixBase<DerivedA>::Ones().array().max(lhs.array().abs().max(rhs.array().abs()))).all();
}

template<typename DerivedA, typename DerivedB>
bool not_almost_equal(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const typename DerivedA::RealScalar& tolerance = Eigen::NumTraits<typename DerivedA::Scalar>::dummy_precision())
{
    return !almost_equal(lhs, rhs, tolerance);
}

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
assert_mat_almost_equal_pred3_format(
    const char* e1,
    const char* e2,
    const char* e3,
    const Eigen::MatrixBase<DerivedA>& v1,
    const Eigen::MatrixBase<DerivedB>& v2,
    const typename Eigen::MatrixBase<DerivedA>::RealScalar& v3)
{
    if(almost_equal(v1, v2, v3))
        return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
        << "pred_text" << "(" << e1 << ", " << e2 << ", " << e3
        << ") evaluates to false, where"
        << "\n"
        << e1 << " evaluates to " << ::testing::PrintToString(v1) << "\n"
        << e2 << " evaluates to " << ::testing::PrintToString(v2) << "\n"
        << e3 << " evaluates to " << ::testing::PrintToString(v3);
}

template<typename DerivedA, typename DerivedB>
::testing::AssertionResult
assert_mat_not_almost_equal_pred3_format(
    const char* e1,
    const char* e2,
    const char* e3,
    const Eigen::MatrixBase<DerivedA>& v1,
    const Eigen::MatrixBase<DerivedB>& v2,
    const typename Eigen::MatrixBase<DerivedA>::RealScalar& v3)
{
    if(!almost_equal(v1, v2, v3))
        return ::testing::AssertionSuccess();

    return ::testing::AssertionFailure()
        << "pred_text" << "(" << e1 << ", " << e2 << ", " << e3
        << ") evaluates to false, where"
        << "\n"
        << e1 << " evaluates to " << ::testing::PrintToString(v1) << "\n"
        << e2 << " evaluates to " << ::testing::PrintToString(v2) << "\n"
        << e3 << " evaluates to " << ::testing::PrintToString(v3);
}

}   // namespace detail

}   // namespace gtest_helper

#define EXPECT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_PRED_FORMAT3(gtest_helper::detail::assert_almost_equal_pred3_format, lhs, rhs, tolerance)

#define EXPECT_NOT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_PRED_FORMAT3(gtest_helper::detail::assert_not_almost_equal_pred3_format, lhs, rhs, tolerance)

#define EXPECT_MAT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_PRED_FORMAT3(gtest_helper::detail::assert_mat_almost_equal_pred3_format, lhs, rhs, tolerance)

#define EXPECT_MAT_NOT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_PRED_FORMAT3(gtest_helper::detail::assert_mat_not_almost_equal_pred3_format, lhs, rhs, tolerance)

#define EXPECT_QUAT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_MAT_ALMOST_EQUAL(lhs.coeffs(), rhs.coeffs(), tolerance)

#define EXPECT_QUAT_NOT_ALMOST_EQUAL(lhs, rhs, tolerance) \
    EXPECT_MAT_NOT_ALMOST_EQUAL(lhs.coeffs(), rhs.coeffs(), tolerance)
