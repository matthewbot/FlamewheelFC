#ifndef FC_MATRIX_H
#define FC_MATRIX_H

#include <initializer_list>
#include <stdint.h>
#include <math.h>
#include "type_traits.h"

template <typename Self>
class MatrixExpr {
public:
    using Derived = Self;
    Derived &derived() { return static_cast<Derived &>(*this); }
    const Derived &derived() const { return static_cast<const Derived &>(*this); }
};

template <typename T, int M, int N>
class Matrix;

template <typename T, int M>
class IdentityMatrix : public MatrixExpr<IdentityMatrix<T, M>> {
public:
    static constexpr int Rows = M;
    static constexpr int Cols = M;
    using Element = T;

    IdentityMatrix() { }

    Element operator()(int r, int c) const __attribute__((always_inline)) { return (r == c) ? 1 : 0; }

    using Reduced = IdentityMatrix<T, M>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename T, int M, int N, int SM, int SN>
class MatrixSlice : public MatrixExpr<MatrixSlice<T, M, N, SM, SN>> {
    Matrix<T, M, N> &mat;
    int offset_row;
    int offset_col;
public:
    static constexpr int Rows = SM;
    static constexpr int Cols = SN;
    using Element = T;

    MatrixSlice(Matrix<T, M, N> &mat, int offset_row, int offset_col) :
        mat(mat), offset_row(offset_row), offset_col(offset_col) { }

    Element &operator()(int r, int c) __attribute__((always_inline)) { return mat(r+offset_row,c+offset_col); }
    Element operator()(int r, int c) const __attribute__((always_inline)) { return mat(r+offset_row,c+offset_col); }
    Element operator[](int i) const __attribute__((always_inline)) { return mat[i+offset_row]; }

    template <typename Expr>
        MatrixSlice<T, M, N, SM, SN> &operator=(const MatrixExpr<Expr> &expr) {
        static_assert(Expr::Rows == Rows && Expr::Cols == Cols, "Dimensions mismatch");

        const Expr &derived = expr.derived();
        for (int r=0; r<Rows; r++)
            for (int c=0; c<Cols; c++)
                (*this)(r, c) = derived(r, c);
        return *this;
    }

    using Reduced = MatrixSlice<T, M, N, SM, SN>;
    using ReducedStorage = const Reduced &;
    const Reduced &reduced() const { return *this; }

};

template <typename T, int M, int N>
class Matrix : public MatrixExpr<Matrix<T, M, N>> {
    T data[M*N];

public:
    static constexpr int Rows = M;
    static constexpr int Cols = N;
    using Element = T;

    Matrix() { }

    Matrix(std::initializer_list<T> l) {
        auto i = l.begin();
        for (int r=0; r<Rows; r++)
            for (int c=0; c<Cols; c++)
                (*this)(r, c) = *i++;
    }

    Matrix(const Matrix &mat) {
        asm("bad Copy constructor");
    }

    template <typename Expr>
    Matrix(const MatrixExpr<Expr> &expr) {
        *this = expr;
    }

    int rows() const { return Rows; }
    int cols() const { return Cols; }

    Element &operator()(int r, int c) __attribute__((always_inline)) { return data[r+c*M]; }
    Element operator()(int r, int c) const __attribute__((always_inline)) { return data[r+c*M]; }
    Element &operator[](int i)  __attribute__((always_inline)) { static_assert(Cols == 1, "index access only valid on Vectors!"); return data[i]; }
    Element operator[](int i) const __attribute__((always_inline)) { static_assert(Cols == 1, "index access only valid on Vectors!"); return data[i]; }

    template <int SM, int SN>
    MatrixSlice<T, M, N, SM, SN> slice(int row, int col) { return MatrixSlice<T, M, N, SM, SN>(*this, row, col); }

    template <typename Expr>
    Matrix<T, M, N> &operator=(const MatrixExpr<Expr> &expr) {
        static_assert(Expr::Rows == Rows && Expr::Cols == Cols, "Dimensions mismatch");

        const Expr &derived = expr.derived();
        for (int r=0; r<Rows; r++)
            for (int c=0; c<Cols; c++)
                (*this)(r, c) = derived(r, c);
        return *this;
    }

    using Reduced = Matrix<T, M, N>;
    using ReducedStorage = const Reduced &;
    const Reduced &reduced() const { return *this; }
};

template <typename T, int N>
using Vector = Matrix<T, N, 1>;

template <int M, int N>
using MatrixF = Matrix<float, M, N>;

template <int N>
using VectorF = Vector<float, N>;

template <typename OP, typename LHS, typename RHS>
class BinaryMatrixOp : public MatrixExpr<BinaryMatrixOp<OP, LHS, RHS>> {
    const OP &op;
    const LHS &lhs;
    const RHS &rhs;

public:
    static_assert(LHS::Rows == RHS::Rows && LHS::Cols == RHS::Cols, "Dimensions mismatch");

    static constexpr int Rows = LHS::Rows;
    static constexpr int Cols = RHS::Cols;
    using Element = decltype(op(lhs(0, 0), rhs(0, 0)));

    BinaryMatrixOp(const LHS &lhs, const RHS &rhs, const OP &op=OP()) : op(op), lhs(lhs), rhs(rhs) { }

    Element operator()(int r, int c) const __attribute__((always_inline)) { return op(lhs(r, c), rhs(r, c)); }

    using Reduced = BinaryMatrixOp<OP, typename LHS::Reduced, typename RHS::Reduced>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { lhs.reduced(), rhs.reduced(), op }; }
};

template <typename OP, typename Expr, typename Scalar>
class ScalarMatrixOp : public MatrixExpr<ScalarMatrixOp<OP, Expr, Scalar>> {
    const OP &op;
    const Expr &expr;
    Scalar scalar;

public:
    static constexpr int Rows = Expr::Rows;
    static constexpr int Cols = Expr::Cols;
    using Element = decltype(op(expr.derived()(0, 0), scalar));

    ScalarMatrixOp(const Expr &expr, Scalar scalar, const OP &op=OP()) : op(op), expr(expr), scalar(scalar) { }

    Element operator()(int r, int c) const __attribute__((always_inline)) { return op(expr(r, c), scalar); }

    using Reduced = ScalarMatrixOp<OP, typename Expr::Reduced, Scalar>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { expr.reduced(), scalar, op }; }
};

template <int i, typename LHS, typename RHS>
struct recdot {
    static auto eval(int r, int c, const LHS &lhs, const RHS &rhs) ->
        decltype(lhs(0, 0)*rhs(0, 0)) {
        float val = lhs(r, i)*rhs(i, c);
        return val + recdot<i-1, LHS, RHS>::eval(r, c, lhs, rhs);
    }
};

template <typename LHS, typename RHS>
struct recdot<0, LHS, RHS> {
   static auto eval(int r, int c, const LHS &lhs, const RHS &rhs) ->
       decltype(lhs(0, 0)*rhs(0, 0)) {
       float val = lhs(r, 0)*rhs(0, c);
       return val;
   }
};

template <typename LHS, typename RHS>
class MatrixMultiplyOp : public MatrixExpr<MatrixMultiplyOp<LHS, RHS>> {
    typename LHS::ReducedStorage lhs_reduced;
    typename RHS::ReducedStorage rhs_reduced;

    static_assert(LHS::Cols == RHS::Rows, "Dimensions mismatch");

public:
    static constexpr int Rows = LHS::Rows;
    static constexpr int Cols = RHS::Cols;
    using Element = decltype(lhs_reduced.derived()(0, 0) * rhs_reduced.derived()(0, 0));

    MatrixMultiplyOp(const LHS &lhs, const RHS &rhs) : lhs_reduced(lhs.reduced()), rhs_reduced(rhs.reduced()) { }

#ifndef NDEBUG
    Element operator()(int r, int c) const {
        Element accum=0;
        for (int i=0; i < LHS::Derived::Cols; i++)
            accum += lhs_reduced(r, i)*rhs_reduced(i, c);
        return accum;
    }
#else
    Element operator()(int r, int c) const { return recdot<LHS::Derived::Cols-1, typename LHS::Reduced, typename RHS::Reduced>::eval(r, c, lhs_reduced, rhs_reduced); }
#endif

    using Reduced = Matrix<Element, Rows, Cols>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename LHS, typename RHS>
class VectorCrossOp : public MatrixExpr<VectorCrossOp<LHS, RHS>> {
    typename LHS::ReducedStorage lhs_reduced;
    typename RHS::ReducedStorage rhs_reduced;

    static_assert(LHS::Rows == 3 && RHS::Rows == 3 && LHS::Cols == 1 && RHS::Cols == 1, "InvAlid dimensions for cross product");

public:
    static constexpr int Rows = 3;
    static constexpr int Cols = 1;
    using Element = decltype(lhs_reduced.derived()(0, 0) * rhs_reduced.derived()(0, 0));

    VectorCrossOp(const LHS &lhs, const RHS &rhs) : lhs_reduced(lhs.reduced()), rhs_reduced(rhs.reduced()) { }

    Element operator()(int r, int c) const {
        switch (r) {
        case 0:
            return lhs_reduced(1,0)*rhs_reduced(2,0) - lhs_reduced(2,0)*rhs_reduced(1,0);
        case 1:
            return lhs_reduced(2,0)*rhs_reduced(0,0) - lhs_reduced(0,0)*rhs_reduced(2,0);
        case 2:
            return lhs_reduced(0,0)*rhs_reduced(1,0) - lhs_reduced(1,0)*rhs_reduced(0,0);
        default:
            return 0;
        }
    }

    using Reduced = Matrix<Element, 3, 1>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename Expr>
class TransposeMatrixOp : public MatrixExpr<TransposeMatrixOp<Expr>> {
    const Expr &expr;

public:
    static constexpr int Rows = Expr::Cols;
    static constexpr int Cols = Expr::Rows;
    using Element = typename Expr::Element;

    TransposeMatrixOp(const Expr &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return expr(c, r); }

    using Reduced = TransposeMatrixOp<typename Expr::Reduced>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { expr.reduced() }; }
};

namespace ops {
    template <typename A, typename B>
    struct add {
        auto operator()(A a, B b) const -> decltype(a+b) { return a + b; }
    };

    template <typename A, typename B>
    struct sub {
        auto operator()(A a, B b) const -> decltype(a-b) { return a - b; }
    };

    template <typename A, typename B>
    struct mul {
        auto operator()(A a, B b) const -> decltype(a*b) { return a * b; }
    };

    template <typename A, typename B>
    struct div {
        auto operator()(A a, B b) const -> decltype(a/b) { return a / b; }
    };
}

template <typename LHS, typename RHS>
auto operator+(const MatrixExpr<LHS> &lhs, const MatrixExpr<RHS> &rhs) ->
    BinaryMatrixOp<ops::add<typename LHS::Element, typename RHS::Element>, LHS, RHS> { return { lhs.derived(), rhs.derived() }; }

template <typename LHS, typename RHS>
auto operator-(const MatrixExpr<LHS> &lhs, const MatrixExpr<RHS> &rhs) ->
    BinaryMatrixOp<ops::sub<typename LHS::Element, typename RHS::Element>, LHS, RHS> { return { lhs.derived(), rhs.derived() }; }

template <typename LHS, typename RHS>
auto operator*(const MatrixExpr<LHS> &lhs, const MatrixExpr<RHS> &rhs) ->
    MatrixMultiplyOp<LHS, RHS> { return { lhs.derived(), rhs.derived() }; }

template <typename Expr, typename Scalar>
auto operator*(Scalar scalar, const MatrixExpr<Expr> &expr) ->
    typename enable_if<is_arithmetic<Scalar>::value,
                       ScalarMatrixOp<ops::mul<typename Expr::Element, Scalar>, Expr, Scalar>
                       >::type
    { return { expr.derived(), scalar }; }

template <typename Expr, typename Scalar>
auto operator/(const MatrixExpr<Expr> &expr, Scalar scalar) ->
    ScalarMatrixOp<ops::div<typename Expr::Element, Scalar>, Expr, Scalar> { return { expr.derived(), scalar }; }

template <typename Expr>
auto tr(const MatrixExpr<Expr> &expr) ->
    TransposeMatrixOp<Expr> { return { expr.derived() }; }

template <typename LHS, typename RHS>
auto dot(const MatrixExpr<LHS> &lhs, const MatrixExpr<RHS> &rhs) ->
    typename MatrixMultiplyOp<TransposeMatrixOp<LHS>, RHS>::Element {
    return MatrixMultiplyOp<TransposeMatrixOp<LHS>, RHS>({ lhs.derived() }, rhs.derived())(0, 0);
}

template <typename LHS, typename RHS>
auto cross(const MatrixExpr<LHS> &lhs, const MatrixExpr<RHS> &rhs) ->
    VectorCrossOp<LHS, RHS> { return { lhs.derived(), rhs.derived() }; }

template <typename Expr>
auto norm(const MatrixExpr<Expr> &expr) -> typename Expr::Element {
    typename Expr::Element accum=0;
    for (int r=0; r<Expr::Rows; r++) {
        for (int c=0; c<Expr::Cols; c++) {
            float val = expr.derived()(r, c);
            accum += val*val;
        }
    }

    return sqrtf(accum);
}

#endif
