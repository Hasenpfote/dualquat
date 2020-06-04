/**
 * @file dualquat/dualquat_base.h
 */
#pragma once

#include <Eigen/Geometry>

namespace eigen_ext
{

template<typename T>
class DualQuaternion;

using DualQuaternionf = DualQuaternion<float>;
using DualQuaterniond = DualQuaternion<double>;

template<typename T>
class DualQuaternion
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");
public:
    using value_type = T;

/* Constructors */
    DualQuaternion()
    {}

    explicit DualQuaternion(
        const Eigen::Quaternion<T>& real,
        const Eigen::Quaternion<T>& dual)
        : real_(real),
          dual_(dual)
    {}

    /**
     * Creates a new dual quaternion with the given vector.
     */
    explicit DualQuaternion(const Eigen::Matrix<T, 3, 1>& v)
        : DualQuaternion(
            Eigen::Quaternion<T>::Identity(),
            Eigen::Quaternion<T>(T(0), v.x(), v.y(), v.z()))
    {}

    /**
     * Creates a new dual quaternion with the given line (in Plucker coordinates).
     */
    DualQuaternion(const Eigen::Matrix<T, 3, 1>& l, const Eigen::Matrix<T, 3, 1>& m)
        : DualQuaternion(
            Eigen::Quaternion<T>(T(0), l.x(), l.y(), l.z()),
            Eigen::Quaternion<T>(T(0), m.x(), m.y(), m.z()))
    {}

/* Accessors */
    const Eigen::Quaternion<T>& real() const noexcept { return real_; }
    const Eigen::Quaternion<T>& dual() const noexcept { return dual_; }

    Eigen::Quaternion<T>& real() noexcept { return real_; }
    Eigen::Quaternion<T>& dual() noexcept { return dual_; }

/* Assignment operators */
    DualQuaternion& operator += (const DualQuaternion&);
    DualQuaternion& operator -= (const DualQuaternion&);
    DualQuaternion& operator *= (const DualQuaternion&);
    DualQuaternion& operator *= (T);

private:
    Eigen::Quaternion<T> real_;
    Eigen::Quaternion<T> dual_;
};

/* Assignment operators */

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator += (const DualQuaternion& rhs)
{
    real_ = real_.coeffs() + rhs.real().coeffs();
    dual_ = dual_.coeffs() + rhs.dual().coeffs();
    return *this;
}

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator -= (const DualQuaternion& rhs)
{
    real_ = real_.coeffs() - rhs.real().coeffs();
    dual_ = dual_.coeffs() - rhs.dual().coeffs();
    return *this;
}

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator *= (const DualQuaternion& rhs)
{
    auto temp = real_;
    real_ = temp * rhs.real();
    dual_ = (temp * rhs.dual()).coeffs() + (dual_ * rhs.real()).coeffs();
    return *this;
}

template<typename T>
DualQuaternion<T>&
DualQuaternion<T>::operator *= (T rhs)
{
    real_.coeffs() *= rhs;
    dual_.coeffs() *= rhs;
    return *this;
}

/* Unary operators */

template<typename T>
DualQuaternion<T>
operator + (const DualQuaternion<T>& rhs)
{
    return rhs;
}

template<typename T>
DualQuaternion<T>
operator - (const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp;
    temp.real() = -rhs.real().coeffs();
    temp.dual() = -rhs.dual().coeffs();
    return temp;
}

/* Binary operators */

template<typename T>
DualQuaternion<T>
operator + (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp += rhs;
}

template<typename T>
DualQuaternion<T>
operator - (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp -= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (const DualQuaternion<T>& lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp *= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (const DualQuaternion<T>& lhs, T rhs)
{
    DualQuaternion<T> temp(lhs);
    return temp *= rhs;
}

template<typename T>
DualQuaternion<T>
operator * (T lhs, const DualQuaternion<T>& rhs)
{
    DualQuaternion<T> temp(rhs);
    return temp *= lhs;
}

}   // namespace eigen_ext
