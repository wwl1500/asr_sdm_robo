#ifndef ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MATH_TYPES_HPP_
#define ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MATH_TYPES_HPP_

#include <cstddef>

namespace asr_sdm_kinematic_dynamic_model
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Mat3
{
  double v[3][3]{};
};

Vec3 operator+(const Vec3 & a, const Vec3 & b);
Vec3 operator-(const Vec3 & a, const Vec3 & b);
Vec3 operator*(double s, const Vec3 & a);
Vec3 operator*(const Vec3 & a, double s);
Vec3 operator/(const Vec3 & a, double s);

double dot(const Vec3 & a, const Vec3 & b);
Vec3 cross(const Vec3 & a, const Vec3 & b);
double norm(const Vec3 & a);
Vec3 normalize(const Vec3 & a);
Vec3 projectPerpendicular(const Vec3 & value, const Vec3 & axis);
Vec3 column(const Mat3 & m, std::size_t j);
Mat3 fromColumns(const Vec3 & c0, const Vec3 & c1, const Vec3 & c2);
Mat3 identityFrame();
Mat3 multiply(const Mat3 & a, const Mat3 & b);
Vec3 multiply(const Mat3 & m, const Vec3 & a);
Mat3 rotationY(double angle);
Mat3 rotationZ(double angle);
Mat3 frameFromAxis(const Vec3 & axis);
Mat3 orthonormalize(const Mat3 & frame);

}  // namespace asr_sdm_kinematic_dynamic_model

#endif  // ASR_SDM_KINEMATIC_DYNAMIC_MODEL_MATH_TYPES_HPP_
