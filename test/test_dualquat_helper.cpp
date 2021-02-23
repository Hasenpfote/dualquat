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

TYPED_TEST(DualQuatHelperTest, screw)
{
    using Vec3 = dualquat::Vector3<TypeParam>;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto l = Vec3(TypeParam(0), TypeParam(0), TypeParam(1));
    const auto m = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
    const auto theta = TypeParam(0.5) * DualQuatHelperTest<TypeParam>::PI;
    const auto d = TypeParam(2);
    const auto dq = dualquat::convert_to_dualquat(l, m, theta, d);
    const auto sc = dualquat::convert_to_screw(dq);

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
    using Vec3 = dualquat::Vector3<TypeParam>;
    using Quat = dualquat::Quaternion<TypeParam>;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatHelperTest<TypeParam>::PI / TypeParam(6);
    const auto axis = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
    const auto r1 = Quat(AngleAxis(angle, axis));
    const auto r2 = Quat(AngleAxis(TypeParam(2) * DualQuatHelperTest<TypeParam>::PI - angle, -axis));
    const auto t = Quat(TypeParam(0), TypeParam(2), TypeParam(3), TypeParam(4));

    const auto dq1 = DualQuat(r1, Quat(TypeParam(0.5) * (t * r1).coeffs()));
    const auto dq2 = DualQuat(r2, Quat(TypeParam(0.5) * (t * r2).coeffs()));

    // t == 0
    const auto res0 = sclerp(dq1, dq2, TypeParam(0));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.real(), res0.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.dual(), res0.dual(), atol);

    // t == 1
    const auto res1 = sclerp(dq1, dq2, TypeParam(1));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.real(), res1.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.dual(), res1.dual(), atol);
}

TYPED_TEST(DualQuatHelperTest, sclerp_shortestpath)
{
    using Vec3 = dualquat::Vector3<TypeParam>;
    using Quat = dualquat::Quaternion<TypeParam>;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = dualquat::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatHelperTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatHelperTest<TypeParam>::PI / TypeParam(6);
    const auto axis = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
    const auto r1 = Quat(AngleAxis(angle, axis));
    const auto r2 = Quat(AngleAxis(TypeParam(2) * DualQuatHelperTest<TypeParam>::PI - angle, -axis));
    const auto t = Quat(TypeParam(0), TypeParam(2), TypeParam(3), TypeParam(4));

    const auto dq1 = DualQuat(r1, Quat(TypeParam(0.5) * (t * r1).coeffs()));
    const auto dq2 = DualQuat(r2, Quat(TypeParam(0.5) * (t * r2).coeffs()));

    // t == 0
    const auto res0 = sclerp_shortestpath(dq1, dq2, TypeParam(0));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.real(), res0.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq1.dual(), res0.dual(), atol);

    // t == 1
    const auto res1 = sclerp_shortestpath(dq1, dq2, TypeParam(1));
    const auto minus_dq2 = DualQuat(Quat(-dq2.real().coeffs()), Quat(-dq2.dual().coeffs()));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, minus_dq2.real(), res1.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, minus_dq2.dual(), res1.dual(), atol);
}

}   // namespace
