#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_transformation.h>
#include <dualquat/dualquat_helper.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class DualQuatHelperTest
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
DualQuatHelperTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DualQuatHelperTest, MyTypes);

TYPED_TEST(DualQuatHelperTest, transform)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatHelperTest<TypeParam>::PI;
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
    const auto t = Vec3(TypeParam(5), TypeParam(6), TypeParam(7));
    const auto dq = eigen_ext::transformation(r, t);

    const auto src = Vec3(TypeParam(1), TypeParam(2), TypeParam(3));
    const auto dst = Vec3((dq * DualQuat(src) * total_conjugate(dq)).dual().vec());

    auto res = eigen_ext::transform(dq, src);

    EXPECT_ALMOST_EQUAL(TypeParam, dst.x(), res.x(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, dst.y(), res.y(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, dst.z(), res.z(), atol);
}

TYPED_TEST(DualQuatHelperTest, screw)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto l = Vec3(TypeParam(0), TypeParam(0), TypeParam(1));
    const auto m = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
    const auto theta = TypeParam(0.5) * DualQuatHelperTest<TypeParam>::PI;
    const auto d = TypeParam(2);
    const auto dq = eigen_ext::convert_to_dualquat(l, m, theta, d);
    const auto sc = eigen_ext::convert_to_screw(dq);

    EXPECT_ALMOST_EQUAL(TypeParam, l.x(), std::get<0>(sc).x(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, l.y(), std::get<0>(sc).y(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, l.z(), std::get<0>(sc).z(), atol);

    EXPECT_ALMOST_EQUAL(TypeParam, m.x(), std::get<1>(sc).x(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, m.y(), std::get<1>(sc).y(), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, m.z(), std::get<1>(sc).z(), atol);

    EXPECT_ALMOST_EQUAL(TypeParam, theta, std::get<2>(sc), atol);
    EXPECT_ALMOST_EQUAL(TypeParam, d, std::get<3>(sc), atol);
}

TYPED_TEST(DualQuatHelperTest, sclerp)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto dq1 = eigen_ext::transformation(
        Quat(AngleAxis(TypeParam(0.25) * DualQuatHelperTest<TypeParam>::PI, Vec3(TypeParam(1), TypeParam(0), TypeParam(0)))),
        Vec3(TypeParam(2), TypeParam(3), TypeParam(4))
    );
    const auto dq2 = eigen_ext::transformation(
        Quat(AngleAxis(TypeParam(0.5) * DualQuatHelperTest<TypeParam>::PI, Vec3(TypeParam(0), TypeParam(1), TypeParam(0)))),
        Vec3(TypeParam(5), TypeParam(6), TypeParam(7))
    );
    // t = 0
    {
        auto t = TypeParam(0);
        auto res = sclerp(dq1, dq2, t);

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.dual(), res.dual(), atol);
    }
    // t = 1
    {
        auto t = TypeParam(1);
        auto res = sclerp(dq1, dq2, t);

        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.real(), res.real(), atol);
        EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.dual(), res.dual(), atol);
    }
}

}   // namespace
