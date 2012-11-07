#ifndef FC_MATRIX_H
#define FC_MATRIX_H

#include <initializer_list>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "type_traits.h"

template <typename Self>
class MatrixExpr {
public:
    using Derived = Self;
    Derived &derived() { return static_cast<Derived &>(*this); }
    const Derived &derived() const { return static_cast<const Derived &>(*this); }

    int rows() const { return Self::Rows; }
    int cols() const { return Self::Cols; }
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

    Element operator()(int r, int c) const { return (r == c) ? 1 : 0; }

    using Reduced = IdentityMatrix<T, M>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename T, int M, int N>
class ZeroMatrix : public MatrixExpr<ZeroMatrix<T, M, N>> {
public:
    static constexpr int Rows = M;
    static constexpr int Cols = N;
    using Element = T;

    ZeroMatrix() { }

    Element operator()(int r, int c) const { return 0; }

    using Reduced = ZeroMatrix<T, M, N>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename T, int M, int N>
class ConstMatrix : public MatrixExpr<ConstMatrix<T, M, N>> {
    const T *const data;
public:
    static constexpr int Rows = M;
    static constexpr int Cols = N;
    using Element = T;

    ConstMatrix(const T *data) : data(data) { }

    Element operator()(int r, int c) const { return data[c+r*N]; }

    using Reduced = ConstMatrix<T, M, N>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename Expr>
class DiagMatrixReduced;

template <typename Expr>
class DiagMatrix : public MatrixExpr<DiagMatrix<Expr>> {
    static_assert(Expr::Cols == 1, "DiagMatrix must be created from a vector");
    const Expr &expr;

public:
    static constexpr int Rows = Expr::Rows;
    static constexpr int Cols = Expr::Rows;
    using Element = typename Expr::Element;

    DiagMatrix(const Expr &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return (r == c) ? expr(r, 0) : 0; }

    using Reduced = DiagMatrixReduced<Expr>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { expr.reduced() }; }
};

template <typename Expr>
class DiagMatrixReduced : public MatrixExpr<DiagMatrixReduced<Expr>> {
    static_assert(Expr::Cols == 1, "DiagMatrixReduced must be created from a vector");
    typename Expr::ReducedStorage expr;

public:
    static constexpr int Rows = Expr::Rows;
    static constexpr int Cols = Expr::Rows;
    using Element = typename Expr::Element;

    DiagMatrixReduced(const typename Expr::Reduced &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return (r == c) ? expr(r, 0) : 0; }

    using Reduced = DiagMatrixReduced<Expr>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { expr.reduced() }; }
};

template <typename T, int M, int N, int SM, int SN>
class ConstMatrixSlice : public MatrixExpr<ConstMatrixSlice<T, M, N, SM, SN>> {
    const Matrix<T, M, N> &mat;
    const int offset_row;
    const int offset_col;

public:
    static constexpr int Rows = SM;
    static constexpr int Cols = SN;
    using Element = T;

    ConstMatrixSlice(const Matrix<T, M, N> &mat, int offset_row, int offset_col) :
        mat(mat), offset_row(offset_row), offset_col(offset_col) { }

    Element operator()(int r, int c) const { static_assert(SN == 1, "Index access only valid on vector slice"); return mat(r+offset_row,c+offset_col); }
    Element operator[](int i) const { static_assert(SN == 1, "Index access only valid on vector slice"); return mat(i+offset_row, offset_col); }

    template <int SM2, int SN2>
    ConstMatrixSlice<T, M, N, SM2, SN2> slice(int row, int col) const { return ConstMatrixSlice<T, M, N, SM2, SN2>(mat, offset_row+row, offset_col+col); }

    using Reduced = ConstMatrixSlice<T, M, N, SM, SN>;
    using ReducedStorage = const Reduced &;
    const Reduced &reduced() const { return *this; }
};

template <typename T, int M, int N, int SM, int SN>
class MatrixSlice : public MatrixExpr<MatrixSlice<T, M, N, SM, SN>> {
    Matrix<T, M, N> &mat;
    const int offset_row;
    const int offset_col;

public:
    static constexpr int Rows = SM;
    static constexpr int Cols = SN;
    using Element = T;

    MatrixSlice(Matrix<T, M, N> &mat, int offset_row, int offset_col) :
        mat(mat), offset_row(offset_row), offset_col(offset_col) { }

    Element &operator()(int r, int c) { return mat(r+offset_row,c+offset_col); }
    Element operator()(int r, int c) const { return mat(r+offset_row,c+offset_col); }
    Element &operator[](int i) { static_assert(SN == 1, "Index access only valid on vector slice"); return mat(i+offset_row, offset_col); }
    Element operator[](int i) const { static_assert(SN == 1, "Index access only valid on vector slice"); return mat(i+offset_row, offset_col); }

    template <typename Expr>
    MatrixSlice<T, M, N, SM, SN> &operator=(const MatrixExpr<Expr> &expr) {
        static_assert(Expr::Rows == Rows && Expr::Cols == Cols, "Dimensions mismatch");

        const Expr &derived = expr.derived();
        for (int r=0; r<Rows; r++)
            for (int c=0; c<Cols; c++)
                (*this)(r, c) = derived(r, c);
        return *this;
    }

    template <int SM2, int SN2>
    ConstMatrixSlice<T, M, N, SM2, SN2> slice(int row, int col) const { return ConstMatrixSlice<T, M, N, SM2, SN2>(mat, offset_row+row, offset_col+col); }
    template <int SM2, int SN2>
    MatrixSlice<T, M, N, SM2, SN2> slice(int row, int col) { return MatrixSlice<T, M, N, SM2, SN2>(mat, offset_row+row, offset_col+col); }

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

    template <typename Expr>
    Matrix(const Expr &expr) {
        *this = expr;
    }

    Element &operator()(int r, int c) { return data[r+c*M]; }
    Element operator()(int r, int c) const { return data[r+c*M]; }
    Element &operator[](int i)  { static_assert(Cols == 1, "index access only valid on Vectors!"); return data[i]; }
    Element operator[](int i) const { static_assert(Cols == 1, "index access only valid on Vectors!"); return data[i]; }

    template <int SM, int SN>
    MatrixSlice<T, M, N, SM, SN> slice(int row, int col) { return MatrixSlice<T, M, N, SM, SN>(*this, row, col); }
    template <int SM, int SN>
    ConstMatrixSlice<T, M, N, SM, SN> slice(int row, int col) const { return ConstMatrixSlice<T, M, N, SM, SN>(*this, row, col); }

    Matrix<T, M, N> &operator=(const Matrix<T, M, N> &mat) {
        memcpy(data, mat.data, sizeof(T)*M*N);
        return *this;
    }

    Matrix<T, M, N> &operator=(const ZeroMatrix<T, M, N> &mat) {
        memset(data, '\0', sizeof(T)*M*N);
        return *this;
    }

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
    const OP op;
    const LHS lhs;
    const RHS rhs;

public:
    static_assert(LHS::Rows == RHS::Rows && LHS::Cols == RHS::Cols, "Dimensions mismatch");

    static constexpr int Rows = LHS::Rows;
    static constexpr int Cols = RHS::Cols;
    using Element = decltype(op(lhs(0, 0), rhs(0, 0)));

    BinaryMatrixOp(const LHS &lhs, const RHS &rhs, const OP &op=OP()) : op(op), lhs(lhs), rhs(rhs) { }

    Element operator()(int r, int c) const { return op(lhs(r, c), rhs(r, c)); }

    using Reduced = Matrix<Element, Rows, Cols>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename OP, typename Expr, typename Scalar>
class ScalarMatrixOp : public MatrixExpr<ScalarMatrixOp<OP, Expr, Scalar>> {
    const OP op;
    const Expr expr;
    Scalar scalar;

public:
    static constexpr int Rows = Expr::Rows;
    static constexpr int Cols = Expr::Cols;
    using Element = decltype(op(expr.derived()(0, 0), scalar));

    ScalarMatrixOp(const Expr &expr, Scalar scalar, const OP &op=OP()) : op(op), expr(expr), scalar(scalar) { }

    Element operator()(int r, int c) const { return op(expr(r, c), scalar); }

    using Reduced = Matrix<Element, Rows, Cols>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
};

template <typename Expr>
class NegateMatrixOp : public MatrixExpr<NegateMatrixOp<Expr>> {
    const Expr expr;

public:
    static constexpr int Rows = Expr::Rows;
    static constexpr int Cols = Expr::Cols;
    using Element = typename Expr::Element;

    NegateMatrixOp(const Expr &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return -expr(r, c); }

    using Reduced = Matrix<Element, Rows, Cols>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return *this; }
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

    static_assert(LHS::Rows == 3 && RHS::Rows == 3 && LHS::Cols == 1 && RHS::Cols == 1, "Invalid dimensions for cross product");

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
class TransposeReducedExprOp;

template <typename Expr>
class TransposeMatrixOp : public MatrixExpr<TransposeMatrixOp<Expr>> {
    const Expr expr;

public:
    static constexpr int Rows = Expr::Cols;
    static constexpr int Cols = Expr::Rows;
    using Element = typename Expr::Element;

    TransposeMatrixOp(const Expr &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return expr(c, r); }

    using Reduced = TransposeReducedExprOp<Expr>;
    using ReducedStorage = Reduced;
    Reduced reduced() const { return { expr.reduced() }; }
};

template <typename Expr>
class TransposeReducedExprOp : public MatrixExpr<TransposeReducedExprOp<Expr>> {
    typename Expr::ReducedStorage expr;

public:
    static constexpr int Rows = Expr::Cols;
    static constexpr int Cols = Expr::Rows;
    using Element = typename Expr::Element;

    TransposeReducedExprOp(const typename Expr::Reduced &expr) : expr(expr) { }

    Element operator()(int r, int c) const { return expr(c, r); }
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

template <typename Expr>
auto operator-(const MatrixExpr<Expr> &expr) ->
    NegateMatrixOp<Expr> { return { expr.derived() }; }

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
auto norm(const MatrixExpr<Expr> &expr_m) -> typename Expr::Element {
    auto expr = expr_m.derived();
    typename Expr::Element accum=0;
    for (int r=0; r<Expr::Rows; r++) {
        for (int c=0; c<Expr::Cols; c++) {
            float val = expr(r, c);
            accum += val*val;
        }
    }

    return sqrtf(accum);
}

template <typename Expr>
auto diag(const MatrixExpr<Expr> &expr) -> DiagMatrix<Expr> { return { expr.derived() }; }

template <typename Expr>
auto cross_mat(const MatrixExpr<Expr> &x_m) -> Matrix<typename Expr::Element, 3, 3> {
    static_assert(Expr::Rows == 3 && Expr::Cols == 1, "cross_mat only defined on 3 vectors");
    typename Expr::ReducedStorage x = x_m.derived().reduced();
    return { 0,    -x[2], x[1],
            x[2],  0,     -x[0],
            -x[1], x[0],  0 };
}

template <int N, typename In, typename Out>
struct choleskyrec {
    static void eval(In &in, Out &out) {
        float l11 = sqrtf(in(0, 0));
        out(0, 0) = l11;

        auto L21 = out.template slice<N-1, 1>(1, 0);
        auto A21 = in.template slice<N-1, 1>(1, 0);
        L21 = (1/l11)*A21;

        auto L22 = out.template slice<N-1, N-1>(1, 1);
        auto A22 = in.template slice<N-1, N-1>(1, 1);
        A22 = A22 - L21*tr(L21);

        choleskyrec<N-1, decltype(A22), decltype(L22)>::eval(A22, L22);
    }
};

template <typename In, typename Out>
struct choleskyrec<1, In, Out> {
    static void eval(const In &in, Out &out) {
        out(0, 0) = sqrtf(in(0, 0));
    }
};

template <typename Expr>
auto chol(const MatrixExpr<Expr> &in_m) -> Matrix<typename Expr::Element, Expr::Rows, Expr::Rows> {
    using T = typename Expr::Element;
    constexpr int N = Expr::Rows;

    Matrix<T, N, N> in = in_m.derived();
    Matrix<T, N, N> out = ZeroMatrix<T, N, N>();
    choleskyrec<N, Matrix<T, N, N>, Matrix<T, N, N> >::eval(in, out);
    return out;
}

template <typename LExpr, typename BExpr>
Vector<typename LExpr::Element, LExpr::Cols> utsolve(const MatrixExpr<LExpr> &lexpr_m, const MatrixExpr<BExpr> &bexpr_m) {
    const LExpr &lexpr = lexpr_m.derived();
    typename BExpr::ReducedStorage bexpr = bexpr_m.derived().reduced();

    Vector<typename LExpr::Element, LExpr::Cols> x;
    for (int r=LExpr::Cols-1; r>=0; r--) {
        float val = bexpr[r];
        for (int c=r+1; c<LExpr::Cols; c++) {
            val -= lexpr(r, c)*x[c];
        }
        x[r] = val / lexpr(r, r);
    }
    return x;
}

template <typename LExpr, typename BExpr>
Vector<typename LExpr::Element, LExpr::Cols> ltsolve(const MatrixExpr<LExpr> &lexpr_m, const MatrixExpr<BExpr> &bexpr_m) {
    const LExpr &lexpr = lexpr_m.derived();
    typename BExpr::ReducedStorage bexpr = bexpr_m.derived().reduced();

    Vector<typename LExpr::Element, LExpr::Cols> x;
    for (int r=0; r<LExpr::Cols; r++) {
        float val = bexpr[r];
        for (int c=0; c<r; c++) {
            val -= lexpr(r, c)*x[c];
        }
        x[r] = val / lexpr(r, r);
    }
    return x;
}

template <typename AExpr, typename BExpr>
auto cholsolve(const MatrixExpr<AExpr> &aexpr_m, const MatrixExpr<BExpr> &bexpr_m) ->
Matrix<decltype(aexpr_m.derived()(0,0)*bexpr_m.derived()(0,0)), AExpr::Rows, BExpr::Cols> {
    using T = decltype(aexpr_m.derived()(0,0)*bexpr_m.derived()(0,0));
    auto aexpr = aexpr_m.derived();
    typename BExpr::ReducedStorage bexpr = bexpr_m.derived().reduced();

    auto lt = chol(aexpr);
    Matrix<T, AExpr::Rows, BExpr::Cols> out;

    for (int c=0; c<BExpr::Cols; c++) {
        auto b = bexpr.template slice<BExpr::Rows, 1>(0, c);
        auto x = out.template slice<AExpr::Rows, 1>(0, c);

        auto y = ltsolve(lt, b);
        x = utsolve(tr(lt), y);
    }

    return out;
}

#endif
