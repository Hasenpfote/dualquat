#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_common.h>
#include <dualquat/dualquat_exponential.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class DualQuatExponentialTest
    : public ::testing::Test
{
protected:
    static const T PI;

    template<typename U = T>
    static constexpr typename std::enable_if<std::is_same<U, float>::value, U>::type
    absolute_tolerance(){ return 1e-3f; }

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
DualQuatExponentialTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DualQuatExponentialTest, MyTypes);

TYPED_TEST(DualQuatExponentialTest, exp)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatExponentialTest<TypeParam>::absolute_tolerance();

    // Non-singularity.
    {
        const auto a = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto vn = a.vec().norm();
        const auto sinc = std::sin(vn) / vn;
        const auto alpha = (std::cos(vn) - sinc) / (vn * vn);
        const auto vdot = a.vec().dot(b.vec());

        auto real = dualquat::exp(a);
        Quat dual;
        dual.w() = - vdot * sinc;
        dual.vec() = sinc * b.vec() + vdot * alpha * a.vec();
        dual.coeffs() = std::exp(a.w()) * dual.coeffs() + b.w() * real.coeffs();

        auto res = exp(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dual, res.dual(), atol);
    }
    // Singularity.
    {
        const auto a = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto alpha = - TypeParam(1) / TypeParam(3);
        const auto vdot = a.vec().dot(b.vec());

        auto real = dualquat::exp(a);
        Quat dual;
        dual.w() = -vdot;
        dual.vec() = b.vec() + vdot * alpha * a.vec();
        dual.coeffs() = std::exp(a.w()) * dual.coeffs() + b.w() * real.coeffs();

        auto res = exp(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dual, res.dual(), atol);
    }
}

TYPED_TEST(DualQuatExponentialTest, log)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatExponentialTest<TypeParam>::absolute_tolerance();

    // Non-singularity.
    {
        const auto a = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto vn = a.vec().norm();
        const auto phi = std::atan2(vn, a.w());
        const auto qn = a.norm();
        const auto phi_over_vn = phi / vn;
        const auto alpha = (a.w() - phi_over_vn * qn * qn) / (vn * vn);

        auto real = dualquat::log(a);
        Quat dual;
        dual.w() = a.dot(b) / (qn * qn);
        dual.vec() = phi_over_vn * b.vec() + ((a.vec().dot(b.vec()) * alpha - b.w()) / (qn * qn)) * a.vec();

        auto res = log(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dual, res.dual(), atol);
    }
    // Singularity.
    {
        const auto a = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto qn = a.norm();
        const auto phi_over_vn = TypeParam(1) / qn;
        const auto alpha = (-TypeParam(2) / TypeParam(3)) / qn;

        auto real = dualquat::log(a);
        Quat dual;
        dual.w() = a.dot(b) / (qn * qn);
        dual.vec() = phi_over_vn * b.vec() + ((a.vec().dot(b.vec()) * alpha - b.w()) / (qn * qn)) * a.vec();

        auto res = log(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dual, res.dual(), atol);
    }
}

TYPED_TEST(DualQuatExponentialTest, explog)
{
    using Vec3 = dualquat::Vector3<TypeParam>;
    using Quat = dualquat::Quaternion<TypeParam>;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatExponentialTest<TypeParam>::absolute_tolerance();

    {
        const auto angle = TypeParam(0);
        const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

        const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

        auto res = log(exp(dq));

        EXPECT_QUAT_ALMOST_EQUAL(dq.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dq.dual(), res.dual(), atol);

        res = exp(log(dq));

        EXPECT_QUAT_ALMOST_EQUAL(dq.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dq.dual(), res.dual(), atol);
    }
    {
        const auto angle = (TypeParam(2) - atol) * DualQuatExponentialTest<TypeParam>::PI;
        const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

        const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

        auto res = log(exp(dq));

        EXPECT_QUAT_ALMOST_EQUAL(dq.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dq.dual(), res.dual(), atol);

        res = exp(log(dq));

        EXPECT_QUAT_ALMOST_EQUAL(dq.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(dq.dual(), res.dual(), atol);
    }
}

TYPED_TEST(DualQuatExponentialTest, pow)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatExponentialTest<TypeParam>::absolute_tolerance();
    const auto Q_IDENTITY = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto Q_ZERO = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));

    {
        const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto s = TypeParam(0);

        auto res = pow(DualQuat(a, b), s);

        EXPECT_QUAT_ALMOST_EQUAL(Q_IDENTITY, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(Q_ZERO, res.dual(), atol);
    }
    {
        const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
        const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
        const auto s = TypeParam(2);

        const auto pow_dq = exp(s * log(DualQuat(a, b)));

        auto res = pow(DualQuat(a, b), s);

        EXPECT_QUAT_ALMOST_EQUAL(pow_dq.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(pow_dq.dual(), res.dual(), atol);
    }
}

}   // namespace
