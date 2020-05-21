/**
 * @file dualquat/relational.h
 * @brief This file provides relational functions for DenseBase types.
 */
#pragma once

namespace eigen_ext
{

/**
 * Returns true if two matrices are element-wise equal within tolerance.
 */
template<typename Derived>
bool almost_equal(
    const Eigen::DenseBase<Derived>& lhs,
    const Eigen::DenseBase<Derived>& rhs,
    const typename Derived::RealScalar& rtol,
    const typename Derived::RealScalar& atol)
{
    return ((lhs.derived() - rhs.derived()).array().abs()
        <= Eigen::DenseBase<Derived>::Constant(atol).array().max(rtol * lhs.derived().array().abs().max(rhs.derived().array().abs()))).all();
}

/**
 * Returns true if two matrices are element-wise equal within tolerance.
 */
template<typename Derived>
bool almost_equal(
    const Eigen::DenseBase<Derived>& lhs,
    const Eigen::DenseBase<Derived>& rhs,
    const typename Derived::RealScalar& tol)
{
    return ((lhs.derived() - rhs.derived()).array().abs()
        <= tol * Eigen::DenseBase<Derived>::Ones().array().max(lhs.derived().array().abs().max(rhs.derived().array().abs()))).all();
}

/**
 * Returns true if a matrix is element-wise equal to zero within tolerance.
 */
template<typename Derived>
bool almost_zero(
    const Eigen::DenseBase<Derived>& x,
    const typename Derived::RealScalar& tol)
{
    return (x.derived().array().abs() <= tol).all();
}

}   // namespace eigen_ext
