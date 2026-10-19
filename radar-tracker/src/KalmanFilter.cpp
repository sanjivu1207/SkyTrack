#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(double processNoise) : q_(processNoise) {}

Mat24 KalmanFilter::H() {
    Mat24 h;
    h(0, 0) = 1.0;
    h(1, 1) = 1.0;
    return h;
}

void KalmanFilter::init(const Vec2& z, const Mat2& R, double maxSpeed) {
    x_ = Vec4{{z[0], z[1], 0.0, 0.0}};
    P_ = Mat4{};
    for (std::size_t r = 0; r < 2; ++r)
        for (std::size_t c = 0; c < 2; ++c) P_(r, c) = R(r, c);
    P_(2, 2) = maxSpeed * maxSpeed;
    P_(3, 3) = maxSpeed * maxSpeed;
}

void KalmanFilter::predict(double dt) {
    Mat4 F = Mat4::identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    const double dt2 = dt * dt, dt3 = dt2 * dt;
    Mat4 Q;
    Q(0, 0) = Q(1, 1) = q_ * dt3 / 3.0;
    Q(0, 2) = Q(2, 0) = Q(1, 3) = Q(3, 1) = q_ * dt2 / 2.0;
    Q(2, 2) = Q(3, 3) = q_ * dt;

    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
}

Mat2 KalmanFilter::innovationCov(const Mat2& R) const {
    const Mat24 h = H();
    return h * P_ * h.transpose() + R;
}

double KalmanFilter::mahalanobis2(const Vec2& z, const Mat2& R) const {
    const Vec2 y = z - H() * x_;
    const Mat<1, 1> d2 = y.transpose() * inverse(innovationCov(R)) * y;
    return d2(0, 0);
}

void KalmanFilter::update(const Vec2& z, const Mat2& R) {
    const Mat24 h = H();
    const Vec2 y = z - h * x_;
    const Mat42 K = P_ * h.transpose() * inverse(innovationCov(R));

    x_ += K * y;

    const Mat4 IKH = Mat4::identity() - K * h;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
}
