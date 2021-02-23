#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_common.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class DualQuatCommonTest
    : public ::testing::Test
{
protected:
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

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DualQuatCommonTest, MyTypes);

TYPED_TEST(DualQuatCommonTest, norm)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a.norm();
    const auto dual = a.dot(b) / real;

    auto res = norm(DualQuat(a, b));

    EXPECT_ALMOST_EQUAL(TypeParam, real, res.first, atol);
    EXPECT_ALMOST_EQUAL(TypeParam, dual, res.second, atol);
}

TYPED_TEST(DualQuatCommonTest, squared_norm)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a.squaredNorm();
    const auto dual = TypeParam(2) * a.dot(b);

    auto res = squared_norm(DualQuat(a, b));

    EXPECT_ALMOST_EQUAL(TypeParam, real, res.first, atol);
    EXPECT_ALMOST_EQUAL(TypeParam, dual, res.second, atol);
}

TYPED_TEST(DualQuatCommonTest, identity)
{
    using Quat = Eigen::Quaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();
    const auto Q_IDENTITY = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto Q_ZERO = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));

    auto res = dualquat::identity<TypeParam>();

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_IDENTITY, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_ZERO, res.dual(), atol);
}

TYPED_TEST(DualQuatCommonTest, inverse)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();
    const auto Q_IDENTITY = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto Q_ZERO = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a.inverse();
    const auto dual = Quat(-(real * b * real).coeffs());
    const auto dq = DualQuat(a, b);
    const auto inv = inverse(dq);

    {
        auto res = inv;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // dq * inv
    {
        auto res = dq * inv;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_IDENTITY, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_ZERO, res.dual(), atol);
    }
    // inv * dq
    {
        auto res = inv * dq;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_IDENTITY, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Q_ZERO, res.dual(), atol);
    }
}

TYPED_TEST(DualQuatCommonTest, normalize)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto r = a.normalized();
    const auto d = b.coeffs() / a.norm();
    const auto real = r;
    const auto dual = Quat(d - r.coeffs().dot(d) * r.coeffs());

    auto res = normalize(DualQuat(a, b));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatCommonTest, dual_conjugate)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a;
    const auto dual = Quat(-b.coeffs());

    {
        auto res = dual_conjugate(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // (dq * conj) != (conj * dq)
    {
        const auto dq = DualQuat(a, b);
        const auto conj = dual_conjugate(dq);

        auto res1 = dq * conj;
        auto res2 = conj * dq;

        // Real(dq * conj) == Real(conj * dq)
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, res1.real(), res2.real(), atol);
        // Dual(dq * conj) != Dual(conj * dq)
        EXPECT_QUAT_NOT_ALMOST_EQUAL(TypeParam, res1.dual(), res2.dual(), atol);
    }
}

TYPED_TEST(DualQuatCommonTest, quaternion_conjugate)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a.conjugate();
    const auto dual = b.conjugate();

    {
        auto res = quaternion_conjugate(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // (dq * conj) == (conj * dq)
    {
        const auto dq = DualQuat(a, b);
        const auto conj = quaternion_conjugate(dq);

        auto res1 = dq * conj;
        auto res2 = conj * dq;

        // Real(dq * conj) == Real(conj * dq)
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, res1.real(), res2.real(), atol);
        // Dual(dq * conj) == Dual(conj * dq)
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, res1.dual(), res2.dual(), atol);
    }
}

TYPED_TEST(DualQuatCommonTest, total_conjugate)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a.conjugate();
    const auto dual = Quat(-b.conjugate().coeffs());

    {
        auto res = total_conjugate(DualQuat(a, b));

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // (dq * conj) != (conj * dq)
    {
        const auto dq = DualQuat(a, b);
        const auto conj = total_conjugate(dq);

        auto res1 = dq * conj;
        auto res2 = conj * dq;

        // Real(dq * conj) == Real(conj * dq)
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, res1.real(), res2.real(), atol);
        // Dual(dq * conj) != Dual(conj * dq)
        EXPECT_QUAT_NOT_ALMOST_EQUAL(TypeParam, res1.dual(), res2.dual(), atol);
    }
}

TYPED_TEST(DualQuatCommonTest, difference)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatCommonTest<TypeParam>::absolute_tolerance();

    const auto dq1 = DualQuat(
        Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4)),
        Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8))
    );
    const auto dq2 = DualQuat(
        Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1)),
        Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5))
    );

    const auto diff = difference(dq1, dq2);
    const auto res = dq1 * diff;

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.real(), res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.dual(), res.dual(), atol);
}

}   // namespace
