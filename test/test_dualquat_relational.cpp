#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_relational.h>

namespace
{

template<typename T>
class DualQuatRelationalTest
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
DualQuatRelationalTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DualQuatRelationalTest, MyTypes);

TYPED_TEST(DualQuatRelationalTest, almost_equal)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val1 = atol;
        constexpr auto val2 = TypeParam(0);
        const auto dq1 = DualQuat(
            Quat(val1, val1, val1, val1),
            Quat(val1, val1, val1, val1));
        const auto dq2 = DualQuat(
            Quat(val2, val2, val2, val2),
            Quat(val2, val2, val2, val2));

        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(dq1, dq2, atol));
    }
    {
        constexpr auto val1 = 10 * atol;
        constexpr auto val2 = TypeParam(0);
        const auto dq1 = DualQuat(
            Quat(val1, val1, val1, val1),
            Quat(val1, val1, val1, val1));
        const auto dq2 = DualQuat(
            Quat(val2, val2, val2, val2),
            Quat(val2, val2, val2, val2));

        EXPECT_FALSE(dualquat::almost_equal<TypeParam>(dq1, dq2, atol));
    }
    {
        constexpr auto val1 = TypeParam(1000);
        constexpr auto val2 = TypeParam(999);
        const auto dq1 = DualQuat(
            Quat(val1, val1, val1, val1),
            Quat(val1, val1, val1, val1));
        const auto dq2 = DualQuat(
            Quat(val2, val2, val2, val2),
            Quat(val2, val2, val2, val2));

        EXPECT_FALSE(dualquat::almost_equal<TypeParam>(dq1, dq2, TypeParam(1e-5)));
        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(dq1, dq2, TypeParam(1e-3)));
        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(dq1, dq2, TypeParam(1e-3), TypeParam(1e-5)));
    }
}

TYPED_TEST(DualQuatRelationalTest, almost_zero)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val = atol;
        const auto dq = DualQuat(
            Quat(val, val, val, val),
            Quat(val, val, val, val));

        EXPECT_TRUE(dualquat::almost_zero<TypeParam>(dq, atol));
    }
    {
        constexpr auto val = 10 * atol;
        const auto dq = DualQuat(
            Quat(val, val, val, val),
            Quat(val, val, val, val));

        EXPECT_FALSE(dualquat::almost_zero<TypeParam>(dq, atol));
    }
}

TYPED_TEST(DualQuatRelationalTest, same_transformation)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatRelationalTest<TypeParam>::absolute_tolerance();

#ifndef NDEBUG
    // When not a unit dual quaternion.
    {
        const auto r = Quat(TypeParam(2), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));
        const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

        EXPECT_DEATH({ dualquat::same_transformation(dq, dq, atol); }, "");
    }
#endif
    // dq == dq
    {
        const auto angle = DualQuatRelationalTest<TypeParam>::PI;
        const auto axis = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
        const auto r = Quat(AngleAxis(angle, axis));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));
        const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

        EXPECT_TRUE(dualquat::same_transformation(dq, dq, atol));
    }
    // When the two axes of rotation are not parallel.
    {
        const auto angle = DualQuatRelationalTest<TypeParam>::PI;
        const auto axis1 = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
        const auto axis2 = Vec3(TypeParam(0), TypeParam(1), TypeParam(0));
        const auto r1 = Quat(AngleAxis(angle, axis1));
        const auto r2 = Quat(AngleAxis(angle, axis2));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

        const auto dq1 = DualQuat(r1, Quat(TypeParam(0.5) * (t * r1).coeffs()));
        const auto dq2 = DualQuat(r2, Quat(TypeParam(0.5) * (t * r2).coeffs()));

        EXPECT_FALSE(dualquat::same_transformation(dq1, dq2, atol));
        EXPECT_FALSE(dualquat::same_transformation(dq2, dq1, atol));
    }
    // dq == -dq
    {
        const auto angle = DualQuatRelationalTest<TypeParam>::PI;
        const auto axis = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
        const auto r1 = Quat(AngleAxis(angle, axis));
        const auto r2 = Quat(AngleAxis(TypeParam(2) * DualQuatRelationalTest<TypeParam>::PI - angle, -axis));
        const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));
        const auto dq1_real = r1;
        const auto dq1_dual = Quat(TypeParam(0.5) * (t * r1).coeffs());
        const auto dq2_real = r2;
        const auto dq2_dual = Quat(TypeParam(0.5) * (t * r2).coeffs());
        const auto dq1 = DualQuat(dq1_real, dq1_dual);
        const auto dq2 = DualQuat(dq2_real, dq2_dual);
        const auto minus_dq1 = DualQuat(Quat(-dq1_real.coeffs()), Quat(-dq1_dual.coeffs()));

        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(dq2, minus_dq1, atol));
        EXPECT_TRUE(dualquat::same_transformation(dq1, dq2, atol));
        EXPECT_TRUE(dualquat::same_transformation(dq2, dq1, atol));
    }
}

}   // namespace
