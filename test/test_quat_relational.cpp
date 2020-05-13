#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <dualquat/quat_relational.h>

namespace
{

template<typename T>
class QuatRelationalTest
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
TYPED_TEST_SUITE(QuatRelationalTest, MyTypes);

TYPED_TEST(QuatRelationalTest, almost_equal)
{
    using Quat = Eigen::Quaternion<TypeParam>;

    constexpr auto atol = QuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val1 = atol;
        constexpr auto val2 = TypeParam(0);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_TRUE(eigen_ext::almost_equal<TypeParam>(q1, q2, atol));
        EXPECT_TRUE(eigen_ext::almost_equal<TypeParam>(q2, q1, atol));
    }
    {
        constexpr auto val1 = 10 * atol;
        constexpr auto val2 = TypeParam(0);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_FALSE(eigen_ext::almost_equal<TypeParam>(q1, q2, atol));
        EXPECT_FALSE(eigen_ext::almost_equal<TypeParam>(q2, q1, atol));
    }
    {
        constexpr auto val1 = TypeParam(1000);
        constexpr auto val2 = TypeParam(999);
        const auto q1 = Quat(val1, val1, val1, val1);
        const auto q2 = Quat(val2, val2, val2, val2);

        EXPECT_FALSE(eigen_ext::almost_equal<TypeParam>(q1, q2, TypeParam(1e-5)));
        EXPECT_TRUE(eigen_ext::almost_equal<TypeParam>(q1, q2, TypeParam(1e-3)));
        EXPECT_TRUE(eigen_ext::almost_equal<TypeParam>(q1, q2, TypeParam(1e-3), TypeParam(1e-5)));
    }
}

TYPED_TEST(QuatRelationalTest, almost_zero)
{
    using Quat = Eigen::Quaternion<TypeParam>;

    constexpr auto atol = QuatRelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val = atol;
        const auto q = Quat(val, val, val, val);

        EXPECT_TRUE(eigen_ext::almost_zero<TypeParam>(q, atol));
    }
    {
        constexpr auto val = 10 * atol;
        const auto q = Quat(val, val, val, val);

        EXPECT_FALSE(eigen_ext::almost_zero<TypeParam>(q, atol));
    }
}

}   // namespace
