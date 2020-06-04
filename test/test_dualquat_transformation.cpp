#include <gtest/gtest.h>
#include <dualquat/dualquat_base.h>
#include <dualquat/dualquat_transformation.h>
#include "gtest_helper.h"

namespace
{

template<typename T>
class DualQuatTransformationTest
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
DualQuatTransformationTest<T>::PI = std::acos(-T(1));

using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DualQuatTransformationTest, MyTypes);

TYPED_TEST(DualQuatTransformationTest, transformation_r_t)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatTransformationTest<TypeParam>::PI;
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
    const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

    const auto real = r;
    const auto dual = Quat(TypeParam(0.5) * (t * r).coeffs());

    auto res = eigen_ext::transformation(r, Vec3(t.vec()));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatTransformationTest, transformation_t_r)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatTransformationTest<TypeParam>::PI;
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
    const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

    const auto real = r;
    const auto dual = Quat(TypeParam(0.5) * (r * t).coeffs());

    auto res = eigen_ext::transformation(Vec3(t.vec()), r);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatTransformationTest, transformation_r)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatTransformationTest<TypeParam>::PI;
    const auto zero = Quat(TypeParam(0), TypeParam(0), TypeParam(0), TypeParam(0));
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));

    const auto real = r;
    const auto dual = zero;

    auto res = eigen_ext::transformation(r);

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatTransformationTest, transformation_t)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

    const auto real = Quat::Identity();
    const auto dual = Quat(TypeParam(0.5) * t.coeffs());

    auto res = eigen_ext::transformation(Vec3(t.vec()));

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, real, res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dual, res.dual(), atol);
}

TYPED_TEST(DualQuatTransformationTest, transformational_difference)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto r1 = Quat(
        AngleAxis(
            DualQuatTransformationTest<TypeParam>::PI / TypeParam(6),
            Vec3(TypeParam(1), TypeParam(0), TypeParam(0))
        ));
    const auto r2 = Quat(
        AngleAxis(
            DualQuatTransformationTest<TypeParam>::PI / TypeParam(3),
            Vec3(TypeParam(0), TypeParam(1), TypeParam(0))
        ));
    const auto t1 = Quat(TypeParam(0), TypeParam(2), TypeParam(3), TypeParam(4));
    const auto t2 = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));

    const auto dq1 = DualQuat(r1, Quat(TypeParam(0.5) * (t1 * r1).coeffs()));
    const auto dq2 = DualQuat(r2, Quat(TypeParam(0.5) * (t2 * r2).coeffs()));

    const auto diff = eigen_ext::transformational_difference(dq1, dq2);
    const auto res = dq1 * diff;

    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.real(), res.real(), atol);
    EXPECT_QUAT_ALMOST_EQUAL(TypeParam, dq2.dual(), res.dual(), atol);
}

TYPED_TEST(DualQuatTransformationTest, transform_point)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatTransformationTest<TypeParam>::PI;
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
    const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));
    const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

    const auto src = Vec3(TypeParam(1), TypeParam(2), TypeParam(3));
    const auto dst = dq * DualQuat(src) * total_conjugate(dq);

    {
        auto res = eigen_ext::transform_point(dq, DualQuat(src));

        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().w(), res.real().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().x(), res.real().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().y(), res.real().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().z(), res.real().z(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().w(), res.dual().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().x(), res.dual().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().y(), res.dual().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().z(), res.dual().z(), atol);
    }
    {
        auto res = eigen_ext::transform_point(dq, src);

        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().w(), res.real().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().x(), res.real().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().y(), res.real().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().z(), res.real().z(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().w(), res.dual().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().x(), res.dual().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().y(), res.dual().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().z(), res.dual().z(), atol);
    }
}

TYPED_TEST(DualQuatTransformationTest, transform_line)
{
    using Quat = Eigen::Quaternion<TypeParam>;
    using Vec3 = typename Quat::Vector3;
    using AngleAxis = typename Quat::AngleAxisType;
    using DualQuat = eigen_ext::DualQuaternion<TypeParam>;

    constexpr auto atol = DualQuatTransformationTest<TypeParam>::absolute_tolerance();

    const auto angle = DualQuatTransformationTest<TypeParam>::PI;
    const auto r = Quat(AngleAxis(angle, Vec3(TypeParam(1), TypeParam(0), TypeParam(0))));
    const auto t = Quat(TypeParam(0), TypeParam(5), TypeParam(6), TypeParam(7));
    const auto dq = DualQuat(r, Quat(TypeParam(0.5) * (t * r).coeffs()));

    const auto l = Vec3(TypeParam(1), TypeParam(0), TypeParam(0));
    const auto m = Vec3(TypeParam(0), TypeParam(2), TypeParam(0));
    const auto dst = dq * DualQuat(l, m) * quaternion_conjugate(dq);

    {
        auto res = eigen_ext::transform_line(dq, DualQuat(l, m));

        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().w(), res.real().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().x(), res.real().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().y(), res.real().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().z(), res.real().z(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().w(), res.dual().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().x(), res.dual().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().y(), res.dual().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().z(), res.dual().z(), atol);
    }
    {
        auto res = eigen_ext::transform_line(dq, l, m);

        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().w(), res.real().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().x(), res.real().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().y(), res.real().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.real().z(), res.real().z(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().w(), res.dual().w(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().x(), res.dual().x(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().y(), res.dual().y(), atol);
        EXPECT_ALMOST_EQUAL(TypeParam, dst.dual().z(), res.dual().z(), atol);
    }
}

}   // namespace
