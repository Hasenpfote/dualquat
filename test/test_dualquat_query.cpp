#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_query.h>

namespace
{

template<typename T>
class DualQuatQueryTest
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

typedef ::testing::Types<float, double> MyTypes;
TYPED_TEST_SUITE(DualQuatQueryTest, MyTypes);

TYPED_TEST(DualQuatQueryTest, is_zero)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatQueryTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val = atol;
        const auto dq = DualQuat(
            Quat(val, val, val, val),
            Quat(val, val, val, val)
            );

        EXPECT_TRUE(is_zero(dq, atol));
    }
    {
        constexpr auto val = 10 * atol;
        const auto dq = DualQuat(
            Quat(val, val, val, val),
            Quat(val, val, val, val)
        );

        EXPECT_FALSE(is_zero(dq, atol));
    }
}

TYPED_TEST(DualQuatQueryTest, is_real)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatQueryTest<TypeParam>::absolute_tolerance();

    const auto zero_q = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto real_q = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto pure_q = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));

    EXPECT_TRUE(is_real(DualQuat(zero_q, zero_q), atol));

    EXPECT_TRUE(is_real(DualQuat(real_q, real_q), atol));
    EXPECT_TRUE(is_real(DualQuat(real_q, zero_q), atol));
    EXPECT_TRUE(is_real(DualQuat(zero_q, real_q), atol));

    EXPECT_FALSE(is_real(DualQuat(pure_q, pure_q), atol));
    EXPECT_FALSE(is_real(DualQuat(pure_q, zero_q), atol));
    EXPECT_FALSE(is_real(DualQuat(zero_q, pure_q), atol));

    EXPECT_FALSE(is_real(DualQuat(real_q, pure_q), atol));
    EXPECT_FALSE(is_real(DualQuat(pure_q, real_q), atol));
}

TYPED_TEST(DualQuatQueryTest, is_pure)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatQueryTest<TypeParam>::absolute_tolerance();

    const auto zero_q = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto real_q = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto pure_q = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));

    EXPECT_TRUE(is_pure(DualQuat(zero_q, zero_q), atol));

    EXPECT_FALSE(is_pure(DualQuat(real_q, real_q), atol));
    EXPECT_FALSE(is_pure(DualQuat(real_q, zero_q), atol));
    EXPECT_FALSE(is_pure(DualQuat(zero_q, real_q), atol));

    EXPECT_TRUE(is_pure(DualQuat(pure_q, pure_q), atol));
    EXPECT_TRUE(is_pure(DualQuat(pure_q, zero_q), atol));
    EXPECT_TRUE(is_pure(DualQuat(zero_q, pure_q), atol));

    EXPECT_FALSE(is_pure(DualQuat(real_q, pure_q), atol));
    EXPECT_FALSE(is_pure(DualQuat(pure_q, real_q), atol));
}

TYPED_TEST(DualQuatQueryTest, is_unit)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatQueryTest<TypeParam>::absolute_tolerance();

    const auto zero_q = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));

    {
        const auto real_q = Quat(TypeParam(1), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto pure_q = Quat(TypeParam(0), TypeParam(1), TypeParam(0), TypeParam(0));

        EXPECT_FALSE(is_unit(DualQuat(zero_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(zero_q, real_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(zero_q, pure_q), atol));

        EXPECT_TRUE(is_unit(DualQuat(real_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(real_q, real_q), atol));
        EXPECT_TRUE(is_unit(DualQuat(real_q, pure_q), atol));

        EXPECT_TRUE(is_unit(DualQuat(pure_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(pure_q, pure_q), atol));
        EXPECT_TRUE(is_unit(DualQuat(pure_q, real_q), atol));
    }
    {
        const auto real_q = Quat(TypeParam(2), TypeParam(0), TypeParam(0), TypeParam(0));
        const auto pure_q = Quat(TypeParam(0), TypeParam(2), TypeParam(0), TypeParam(0));

        EXPECT_FALSE(is_unit(DualQuat(zero_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(zero_q, real_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(zero_q, pure_q), atol));

        EXPECT_FALSE(is_unit(DualQuat(real_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(real_q, real_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(real_q, pure_q), atol));

        EXPECT_FALSE(is_unit(DualQuat(pure_q, zero_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(pure_q, pure_q), atol));
        EXPECT_FALSE(is_unit(DualQuat(pure_q, real_q), atol));
    }
}

}   // namespace
