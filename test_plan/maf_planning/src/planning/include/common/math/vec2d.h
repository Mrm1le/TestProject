#ifndef MODULES_PLANNING_MATH_VEC2D_H_
#define MODULES_PLANNING_MATH_VEC2D_H_

#include <cmath>
#include <string>

#include "nlohmann/json.hpp"

namespace msquare {
namespace planning_math {

constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2d {
public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y) {}

  constexpr Vec2d(const double x, const double y, const int id) noexcept
      : x_(x), y_(y), id_(id) {}

  //! Constructor returning the zero vector.
  constexpr Vec2d() noexcept : Vec2d(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2d CreateUnitVec2d(const double angle);

  //! Getter for id component
  int Id() const { return id_; }

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Setter for x & y component
  void set_point(const double x, const double y) {
    x_ = x;
    y_ = y;
  }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2d &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2d &other) const;

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double CrossProd(const Vec2d &other) const;

  //! Returns the inner product between these two Vec2d.
  double InnerProd(const Vec2d &other) const;

  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle);

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d &other) const;

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d &other) const;

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(const double ratio) const;

  //! Divides Vec2d by a scalar
  Vec2d operator/(const double ratio) const;

  //! Sums another Vec2d to the current one
  Vec2d &operator+=(const Vec2d &other);

  //! Subtracts another Vec2d to the current one
  Vec2d &operator-=(const Vec2d &other);

  //! Multiplies this Vec2d by a scalar
  Vec2d &operator*=(const double ratio);

  //! Divides this Vec2d by a scalar
  Vec2d &operator/=(const double ratio);

  //! Compares two Vec2d
  bool operator==(const Vec2d &other) const;
  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Vec2d, x_, y_, id_)
protected:
  double x_ = 0.0;
  double y_ = 0.0;
  int id_ = 0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d &vec);

} // namespace planning_math
} // namespace msquare

#endif /* MODULES_PLANNING_MATH_VEC2D_H_ */
