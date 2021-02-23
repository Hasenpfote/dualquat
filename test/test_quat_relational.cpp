#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/quat_relational.h>

namespace
{

template<typename T>
class QuatRelationalTest
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
QuatRelationalTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QuatRelationalTest, MyTypes);

TYPED_TEST(QuatRelationalTest, almost_equal)
{
    using Quat = dualquat::Quaternion<TypeParam>;

    constexpr auto atol = QuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val1 = atol;
        constexpr auto val2 = TypeParam(0);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(q1, q2, atol));
        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(q2, q1, atol));
    }
    {
        constexpr auto val1 = 10 * atol;
        constexpr auto val2 = TypeParam(0);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_FALSE(dualquat::almost_equal<TypeParam>(q1, q2, atol));
        EXPECT_FALSE(dualquat::almost_equal<TypeParam>(q2, q1, atol));
    }
    {
        constexpr auto val1 = TypeParam(1000);
        constexpr auto val2 = TypeParam(999);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_FALSE(dualquat::almost_equal<TypeParam>(q1, q2, TypeParam(1e-5)));
        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(q1, q2, TypeParam(1e-3)));
        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(q1, q2, TypeParam(1e-3), TypeParam(1e-5)));
    }
}

TYPED_TEST(QuatRelationalTest, almost_zero)
{
    using Quat = dualquat::Quaternion<TypeParam>;

    constexpr auto atol = QuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val = atol;
        const auto q = Quat(val, val, val, val);

        EXPECT_TRUE(dualquat::almost_zero<TypeParam>(q, atol));
    }
    {
        constexpr auto val = 10 * atol;
        const auto q = Quat(val, val, val, val);

        EXPECT_FALSE(dualquat::almost_zero<TypeParam>(q, atol));
    }
}

TYPED_TEST(QuatRelationalTest, same_rotation)
{
    using Quat = dualquat::Quaternion<TypeParam>;
    using Vec3 = dualquat::Vector3<TypeParam>;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = QuatRelationalTest<TypeParam>::absolute_tolerance();

#ifndef NDEBUG
    // |q| != 1
    {
        const auto q = Quat(TypeParam(2), TypeParam(0), TypeParam(0), TypeParam(0));

        EXPECT_DEATH({ dualquat::same_rotation(q, q, atol); }, "");
    }
#endif
    // q == q
    {
        const auto angle = QuatRelationalTest<TypeParam>::PI / TypeParam(3);    // 60 [deg]
        const auto q = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));

        EXPECT_TRUE(dualquat::same_rotation(q, q, atol));
    }
    // When the two axes of rotation are not parallel.
    {
        const auto angle = QuatRelationalTest<TypeParam>::PI / TypeParam(3);    // 60 [deg]
        const auto q1 = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
        const auto q2 = Quat(AngleAxis(angle, Vec3(TypeParam(0), TypeParam(1), TypeParam(0))));

        EXPECT_FALSE(dualquat::same_rotation(q1, q2, atol));
        EXPECT_FALSE(dualquat::same_rotation(q2, q1, atol));
    }
    // q == -q
    {
        const auto angle = QuatRelationalTest<TypeParam>::PI / TypeParam(3);    // 60 [deg]
        const auto q1 = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
        const auto q2 = Quat(AngleAxis(TypeParam(2) * QuatRelationalTest<TypeParam>::PI - angle, -Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
        const auto minus_q1 = Quat(-q1.coeffs());

        EXPECT_TRUE(dualquat::almost_equal<TypeParam>(q2, minus_q1, atol));
        EXPECT_TRUE(dualquat::same_rotation(q1, q2, atol));
        EXPECT_TRUE(dualquat::same_rotation(q2, q1, atol));
    }
}

}   // namespace
