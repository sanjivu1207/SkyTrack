#pragma once


#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

template <std::size_t R, std::size_t C>
class Mat {
public:

    Mat() : d_{} {}

    explicit Mat(const std::array<double, R * C>& values) : d_(values) {}

    static constexpr std::size_t rows() { return R; }
    static constexpr std::size_t cols() { return C; }

    double& operator()(std::size_t r, std::size_t c) { return d_[r * C + c]; }
    double operator()(std::size_t r, std::size_t c) const { return d_[r * C + c]; }


    double& operator[](std::size_t i) { return d_[i]; }
    double operator[](std::size_t i) const { return d_[i]; }

    static Mat identity() {
        static_assert(R == C, "identity() requires a square matrix");
        Mat m;
        for (std::size_t i = 0; i < R; ++i) m(i, i) = 1.0;
        return m;
    }

    Mat<C, R> transpose() const {
        Mat<C, R> t;
        for (std::size_t r = 0; r < R; ++r)
            for (std::size_t c = 0; c < C; ++c) t(c, r) = (*this)(r, c);
        return t;
    }

    Mat& operator+=(const Mat& o) {
        for (std::size_t i = 0; i < R * C; ++i) d_[i] += o.d_[i];
        return *this;
    }
    Mat& operator-=(const Mat& o) {
        for (std::size_t i = 0; i < R * C; ++i) d_[i] -= o.d_[i];
        return *this;
    }
    Mat& operator*=(double s) {
        for (double& v : d_) v *= s;
        return *this;
    }

private:
    std::array<double, R * C> d_;
};

template <std::size_t R, std::size_t C>
Mat<R, C> operator+(Mat<R, C> a, const Mat<R, C>& b) { return a += b; }

template <std::size_t R, std::size_t C>
Mat<R, C> operator-(Mat<R, C> a, const Mat<R, C>& b) { return a -= b; }

template <std::size_t R, std::size_t C>
Mat<R, C> operator*(Mat<R, C> a, double s) { return a *= s; }

template <std::size_t R, std::size_t C>
Mat<R, C> operator*(double s, Mat<R, C> a) { return a *= s; }


template <std::size_t R, std::size_t K, std::size_t C>
Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) {
    Mat<R, C> out;
    for (std::size_t r = 0; r < R; ++r)
        for (std::size_t c = 0; c < C; ++c) {
            double sum = 0.0;
            for (std::size_t k = 0; k < K; ++k) sum += a(r, k) * b(k, c);
            out(r, c) = sum;
        }
    return out;
}

using Vec2 = Mat<2, 1>;
using Vec4 = Mat<4, 1>;
using Mat2 = Mat<2, 2>;
using Mat4 = Mat<4, 4>;
using Mat24 = Mat<2, 4>;
using Mat42 = Mat<4, 2>;

inline double determinant(const Mat2& m) {
    return m(0, 0) * m(1, 1) - m(0, 1) * m(1, 0);
}

inline Mat2 inverse(const Mat2& m) {
    const double det = determinant(m);
    if (std::abs(det) < 1e-12) throw std::runtime_error("inverse: singular 2x2 matrix");
    const double inv = 1.0 / det;
    return Mat2{{m(1, 1) * inv, -m(0, 1) * inv,
                 -m(1, 0) * inv, m(0, 0) * inv}};
}
