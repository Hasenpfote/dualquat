#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class DualQuatBaseTest
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
TYPED_TEST_SUITE(DualQuatBaseTest, MyTypes);

TYPED_TEST(DualQuatBaseTest, Constructor)
{
    using Vec3 = dualquat::Vector3<TypeParam>;
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto real = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto dual = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));

    const DualQuat res1(real, dual);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res1.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res1.dual(), atol);

    const DualQuat res2(dual.coeffs().head(3));
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0)), res2.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Quat(TypeParam(0), TypeParam(6), TypeParam(7), TypeParam(8)), res2.dual(), atol);

    const auto from = Vec3(TypeParam(0), TypeParam(2), TypeParam(6));
    const auto to = Vec3(TypeParam(0), TypeParam(2), TypeParam(4));
    const auto l = to - from;
    const auto m = from.cross(l);
    const DualQuat res3(l, m);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Quat((Quat().coeffs() << l, TypeParam(0)).finished()), res3.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, Quat((Quat().coeffs() << m, TypeParam(0)).finished()), res3.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, Accessor)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto real = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto dual = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));

    {
        const DualQuat res1(real, dual);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res1.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res1.dual(), atol);

        DualQuat res2;
        res2.real() = real;
        res2.dual() = dual;
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res2.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res2.dual(), atol);
    }
}

TYPED_TEST(DualQuatBaseTest, AdditionAssignment)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
    const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
    const auto real = Quat(a.coeffs() + c.coeffs());
    const auto dual = Quat(b.coeffs() + d.coeffs());

    DualQuat lhs(a, b);
    DualQuat rhs(c, d);
    lhs += rhs;

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, lhs.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, lhs.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, SubtractionAssignment)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
    const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
    const auto real = Quat(a.coeffs() - c.coeffs());
    const auto dual = Quat(b.coeffs() - d.coeffs());

    DualQuat lhs(a, b);
    DualQuat rhs(c, d);
    lhs -= rhs;

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, lhs.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, lhs.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, MultiplicationAssignment)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));

    // dq * dq
    {
        const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
        const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
        const auto real = a * c;
        const auto dual = Quat((a * d).coeffs() + (b * c).coeffs());

        DualQuat lhs(a, b);
        DualQuat rhs(c, d);
        lhs *= rhs;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, lhs.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, lhs.dual(), atol);
    }
    // dq * scalar
    {
        const auto s = TypeParam(2);
        const auto real = Quat(a.coeffs() * s);
        const auto dual = Quat(b.coeffs() * s);

        DualQuat lhs(a, b);
        lhs *= s;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, lhs.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, lhs.dual(), atol);
    }
}

TYPED_TEST(DualQuatBaseTest, UnaryPlus)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();
    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = a;
    const auto dual = b;

    auto res = +DualQuat(a, b);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, UnaryMinus)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();
    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto real = Quat(-a.coeffs());
    const auto dual = Quat(-b.coeffs());

    auto res = -DualQuat(a, b);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, Addition)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
    const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
    const auto real = Quat(a.coeffs() + c.coeffs());
    const auto dual = Quat(b.coeffs() + d.coeffs());

    auto res = DualQuat(a, b) + DualQuat(c, d);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, Subtraction)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));
    const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
    const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
    const auto real = Quat(a.coeffs() - c.coeffs());
    const auto dual = Quat(b.coeffs() - d.coeffs());

    auto res = DualQuat(a, b) - DualQuat(c, d);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatBaseTest, Multiplication)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatBaseTest<TypeParam>::absolute_tolerance();

    const auto a = Quat(TypeParam(1), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto b = Quat(TypeParam(5), TypeParam(6), TypeParam(7), TypeParam(8));

    // dq * dq
    {
        const auto c = Quat(TypeParam(8), TypeParam(7), TypeParam(6), TypeParam(5));
        const auto d = Quat(TypeParam(4), TypeParam(3), TypeParam(2), TypeParam(1));
        const auto real = a * c;
        const auto dual = Quat((a * d).coeffs() + (b * c).coeffs());

        auto res = DualQuat(a, b) * DualQuat(c, d);

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // dq * scalar
    {
        const auto s = TypeParam(2);
        const auto real = Quat(a.coeffs() * s);
        const auto dual = Quat(b.coeffs() * s);

        auto res = DualQuat(a, b) * s;

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
    // scalar * dq
    {
        const auto s = TypeParam(2);
        const auto real = Quat(a.coeffs() * s);
        const auto dual = Quat(b.coeffs() * s);

        auto res = s * DualQuat(a, b);

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
    }
}

TYPED_TEST(DualQuatBaseTest, Alignment)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    if((sizeof(Quat) % 16) != 0)
        GTEST_SKIP_("It does not need to be aligned.");

    {
        DualQuat* p = new DualQuat;
        const auto addr = reinterpret_cast<std::uintptr_t>(&(p->real()));
        EXPECT_TRUE((addr % 16) == 0);
        delete p;
    }
    {
        DualQuat* p = new DualQuat[2];
        const auto addr0 = reinterpret_cast<std::uintptr_t>(&(p[0].real()));
        const auto addr1 = reinterpret_cast<std::uintptr_t>(&(p[1].real()));
        EXPECT_TRUE((addr0 % 16) == 0);
        EXPECT_TRUE((addr1 % 16) == 0);
        delete[] p;
    }
}

}   // namespace
