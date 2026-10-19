#pragma once

#include "Matrix.hpp"

class KalmanFilter {
public:
    explicit KalmanFilter(double processNoise = 1.0);

    void init(const Vec2& z, const Mat2& R, double maxSpeed = 300.0);

    void predict(double dt);
    void update(const Vec2& z, const Mat2& R);

    Mat2 innovationCov(const Mat2& R) const;
    double mahalanobis2(const Vec2& z, const Mat2& R) const;

    const Vec4& state() const { return x_; }
    const Mat4& covariance() const { return P_; }

    static Mat24 H();

private:
    double q_;
    Vec4 x_;
    Mat4 P_;
};
