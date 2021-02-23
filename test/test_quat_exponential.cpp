#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <dualquat/quat_exponential.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class QuatExponentialTest
    : public ::testing::Test
{
protected:
    static const T PI;

    template<typename U = T>
    static constexpr typename std::enable_if<std::is_same<U, float>::value, U>::type
    absolute_tolerance(){ return 1e-4f; }

    template<typename U = T>
    static constexpr typename std::enable_if<std::is_same<U, double>::value, U>::type
    absolute_tolerance(){ return 1e-8; }

    template<typename U = T>
    static constexpr typename std::enable_if<std::is_same<U, float>::value, U>::type
    relative_tolerance(){ return 1e-5f; }

    template<typename U = T>
    static constexpr typename std::enable_if<std::is_same<U, double>::value, U>::type
    relative_tolerance(){ return 1e-5; }
};

template<typename T>
const T
QuatExponentialTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QuatExponentialTest, MyTypes);

TYPED_TEST(QuatExponentialTest, exp)
{
    using Quat = Eigen::Quaternion<TypeParam>;

    constexpr auto atol = QuatExponentialTest<TypeParam>::absolute_tolerance();

    // Non-singularity.
    {
        const auto q = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));
        const auto vn = q.vec().norm();
        const auto sinc = std::sin(vn) / vn;

        Quat exp_q;
        exp_q.w() = std::cos(vn);
        exp_q.vec() = sinc * q.vec();
        exp_q.coeffs() *= std::exp(q.w());

        auto res = dualquat::exp(q);

        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.z(), res.z(), atol);
    }
    // Singularity.
    {
        const auto q = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto vn = q.vec().norm();

        Quat exp_q;
        exp_q.w() = std::cos(vn);
        exp_q.vec() = q.vec();
        exp_q.coeffs() *= std::exp(q.w());

        auto res = dualquat::exp(q);

        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, exp_q.z(), res.z(), atol);
    }
}

TYPED_TEST(QuatExponentialTest, log)
{
    using Quat = Eigen::Quaternion<TypeParam>;

    constexpr auto atol = QuatExponentialTest<TypeParam>::absolute_tolerance();

    // Non-singularity.
    {
        const auto q = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));
        const auto vn = q.vec().norm();
        const auto phi = std::atan2(vn, q.w());
        const auto qn = q.norm();
        const auto phi_over_vn = phi / vn;

        Quat log_q;
        log_q.w() = std::log(qn);
        log_q.vec() = phi_over_vn * q.vec();

        auto res = dualquat::log(q);

        EXPECT_ALMOST_EQUAL(TypeParam, log_q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.z(), res.z(), atol);
    }
    // Singularity.
    {
        const auto q = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto qn = q.norm();
        const auto phi_over_vn = TypeParam(1) / qn;

        Quat log_q;
        log_q.w() = std::log(qn);
        log_q.vec() = phi_over_vn * q.vec();

        auto res = dualquat::log(q);

        EXPECT_ALMOST_EQUAL(TypeParam, log_q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, log_q.z(), res.z(), atol);
    }
}

TYPED_TEST(QuatExponentialTest, explog)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = QuatExponentialTest<TypeParam>::absolute_tolerance();

    {
        const auto angle = TypeParam(0);
        const auto q = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));

        auto res = dualquat::log(dualquat::exp(q));

        EXPECT_ALMOST_EQUAL(TypeParam, q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.z(), res.z(), atol);

        res = dualquat::exp(dualquat::log(q));

        EXPECT_ALMOST_EQUAL(TypeParam, q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.z(), res.z(), atol);
    }
    {
        const auto angle = TypeParam(2) * QuatExponentialTest<TypeParam>::PI;
        const auto q = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));

        auto res = dualquat::log(dualquat::exp(q));

        EXPECT_ALMOST_EQUAL(TypeParam, q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.z(), res.z(), atol);

        res = dualquat::exp(dualquat::log(q));

        EXPECT_ALMOST_EQUAL(TypeParam, q.w(), res.w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.x(), res.x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.y(), res.y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, q.z(), res.z(), atol);
    }
}

}   // namespace
