
#include "planner/behavior_planner/lateral_behavior_data_manager.h"

namespace msquare {

template <typename T2>
BasicCell<T2> operator+(BasicCell<T2> &a, BasicCell<T2> &b) {
  if (a.type() == b.type() && a.type() <= TensorType::SCALAR)
    return BasicCell<T2>(a.value_ + b.value_, a.type(), a.type());
  else
    return BasicCell<T2>(a.value_, TensorType::ERROR, a.type());
}
template <typename T2>
BasicCell<T2> operator-(BasicCell<T2> &a, BasicCell<T2> &b) {
  if (a.type() == b.type() && a.type() <= TensorType::SCALAR)
    return BasicCell<T2>(a.value_ - b.value_, a.type(), a.type());
  else
    return BasicCell<T2>(a.value_, TensorType::ERROR, a.type());
}

template <typename T2>
BasicCell<T2> operator*(BasicCell<T2> &a, BasicCell<T2> &b) {
  if (a.type() == b.type() && a.type() <= TensorType::SCALAR)
    return BasicCell<T2>(a.value_ * b.value_, a.type(), a.type());
  else
    return BasicCell<T2>(a.value_, TensorType::ERROR, a.type());
}

template <typename T2>
BasicCell<T2> operator/(BasicCell<T2> &a, BasicCell<T2> &b) {
  if (a.type() == b.type() && a.type() <= TensorType::SCALAR)
    return BasicCell<T2>(a.value_ / b.value_, a.type(), a.type());
  else
    return BasicCell<T2>(a.value_, TensorType::ERROR, a.type());
}

template <typename T1> BasicCell<T1> operator+(BasicCell<T1> &a, T1 &b) {
  if (a.type() == b.type && a.type() <= TensorType::SCALAR)
    return BasicCell<T1>(a.value_ + b, a.type(), a.type());
  else
    return BasicCell<T1>(a.value_, TensorType::ERROR, a.type());
}

template <typename T1> BasicCell<T1> operator-(BasicCell<T1> &a, T1 &b) {
  if (a.type() == b.type && a.type() <= TensorType::SCALAR)
    return BasicCell<T1>(a.value_ - b, a.type(), a.type());
  else
    return BasicCell<T1>(a.value_, TensorType::ERROR, a.type());
}

template <typename T1> BasicCell<T1> operator*(BasicCell<T1> &a, T1 &b) {
  if (a.type() == b.type && a.type() <= TensorType::SCALAR)
    return BasicCell<T1>(a.value_ * b, a.type(), a.type());
  else
    return BasicCell<T1>(a.value_, TensorType::ERROR, a.type());
}

template <typename T1> BasicCell<T1> operator/(BasicCell<T1> &a, T1 &b) {
  if (a.type() == b.type && a.type() <= TensorType::SCALAR)
    return BasicCell<T1>(a.value_ * b, a.type(), a.type());
  else
    return BasicCell<T1>(a.value_, TensorType::ERROR, a.type());
}

} // namespace msquare
