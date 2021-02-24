/**
 * @file dualquat/relational.h
 * @brief This file provides relational functions for DenseBase types.
 */
#pragma once

#include <Eigen/Core>

namespace dualquat
{

/**
 * Returns true if two matrices are element-wise equal within tolerance.
 */
template<typename DerivedA, typename DerivedB>
bool almost_equal(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const typename DerivedA::RealScalar& rel_tolerance,
    const typename DerivedA::RealScalar& abs_tolerance)
{
    return ((lhs - rhs).array().abs()
        <= Eigen::MatrixBase<DerivedA>::Constant(abs_tolerance).array().max(rel_tolerance * lhs.array().abs().max(rhs.array().abs()))).all();
}

/**
 * Returns true if two matrices are element-wise equal within tolerance.
 */
template<typename DerivedA, typename DerivedB>
bool almost_equal(
    const Eigen::MatrixBase<DerivedA>& lhs,
    const Eigen::MatrixBase<DerivedB>& rhs,
    const typename DerivedA::RealScalar& tolerance)
{
    return ((lhs - rhs).array().abs()
        <= tolerance * Eigen::MatrixBase<DerivedA>::Ones().array().max(lhs.array().abs().max(rhs.array().abs()))).all();
}

/**
 * Returns true if a matrix is element-wise equal to zero within tolerance.
 */
template<typename Derived>
bool almost_zero(
    const Eigen::MatrixBase<Derived>& x,
    const typename Derived::RealScalar& tolerance)
{
    return (x.array().abs() <= tolerance).all();
}

}   // namespace dualquat
