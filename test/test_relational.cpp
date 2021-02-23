#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <dualquat/relational.h>

namespace
{

template<typename T>
class RelationalTest
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
TYPED_TEST_SUITE(RelationalTest, MyTypes);

TYPED_TEST(RelationalTest, almost_equal)
{
    using Matrix1 = Eigen::Matrix<TypeParam, 1, 1>;

    constexpr auto atol = RelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val1 = atol;
        constexpr auto val2 = TypeParam(0);
        const auto s1 = Matrix1(val1);
        const auto s2 = Matrix1(val2);

        EXPECT_TRUE(dualquat::almost_equal(s1, s2, atol));
        EXPECT_TRUE(dualquat::almost_equal(s2, s1, atol));
    }
    {
        constexpr auto val1 = 10 * atol;
        constexpr auto val2 = TypeParam(0);
        const auto s1 = Matrix1(val1);
        const auto s2 = Matrix1(val2);

        EXPECT_FALSE(dualquat::almost_equal(s1, s2, atol));
        EXPECT_FALSE(dualquat::almost_equal(s2, s1, atol));
    }
    {
        constexpr auto val1 = TypeParam(1000);
        constexpr auto val2 = TypeParam(999);
        const auto s1 = Matrix1(val1);
        const auto s2 = Matrix1(val2);

        EXPECT_FALSE(dualquat::almost_equal(s1, s2, TypeParam(1e-5)));
        EXPECT_TRUE(dualquat::almost_equal(s1, s2, TypeParam(1e-3)));
        EXPECT_TRUE(dualquat::almost_equal(s1, s2, TypeParam(1e-3), TypeParam(1e-5)));
    }
}

TYPED_TEST(RelationalTest, almost_zero)
{
    using Matrix1 = Eigen::Matrix<TypeParam, 1, 1>;

    constexpr auto atol = RelationalTest<TypeParam>::absolute_tolerance();

    {
        constexpr auto val = atol;
        const auto s = Matrix1(val);

        EXPECT_TRUE(dualquat::almost_zero(s, atol));
    }
    {
        constexpr auto val = 10 * atol;
        const auto s = Matrix1(val);

        EXPECT_FALSE(dualquat::almost_zero(s, atol));
    }
}

}   // namespace
