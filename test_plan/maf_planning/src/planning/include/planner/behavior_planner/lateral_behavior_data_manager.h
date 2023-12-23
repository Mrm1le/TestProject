#pragma once
#include <nlohmann/json.hpp>
// #include "common/world_model.h"
// #include "planner/tasks/deciders/decider.h"
namespace msquare {

typedef enum {
  ERROR = -1,
  LOGIC = 0,
  SCALAR = 1,
  VECTOR = 2,
  MATRIX = 3,
} TensorType;

typedef enum {
  INPUT = 0,
  OUTPUT = 1,
} LayerType;

template <typename T1> class BasicCell {
public:
  explicit BasicCell(T1 value, TensorType type, LayerType layer)
      : value_(value), type_(type), layer_(layer){};
  virtual ~BasicCell() = default;
  void update_value(T1 value) { value_ = value; };
  void update_layer(int layer) { layer_ = (LayerType)layer; };
  T1 value() const { return value_; };
  TensorType type() const { return type_; };
  LayerType layer() const { return layer_; };

  template <typename T2>
  friend BasicCell<T2> operator+(BasicCell<T2> &a, BasicCell<T2> &b);

  template <typename T2>
  friend BasicCell<T2> operator-(BasicCell<T2> &a, BasicCell<T2> &b);

  template <typename T2>
  friend BasicCell<T2> operator*(BasicCell<T2> &a, BasicCell<T2> &b);
  template <typename T2>
  friend BasicCell<T2> operator/(BasicCell<T2> &a, BasicCell<T2> &b);

  template <typename T2>
  friend BasicCell<T2> operator+(BasicCell<T2> &a, T2 &b);
  template <typename T2>
  friend BasicCell<T2> operator-(BasicCell<T2> &a, T2 &b);
  template <typename T2>
  friend BasicCell<T2> operator*(BasicCell<T2> &a, T2 &b);
  template <typename T2>
  friend BasicCell<T2> operator/(BasicCell<T2> &a, T2 &b);

private:
  T1 value_;
  TensorType type_;
  LayerType layer_ = INPUT;
};

class LateralDecisionDataManager {
public:
  explicit LateralDecisionDataManager() = default;
  virtual ~LateralDecisionDataManager(){};
  BasicCell<bool> gap_valid_cell = BasicCell<bool>(false, LOGIC, OUTPUT);
  BasicCell<bool> gap_inserable_cell = BasicCell<bool>(false, LOGIC, OUTPUT);
  BasicCell<int> gap_first_id_cell = BasicCell<int>(-1, SCALAR, INPUT);
  BasicCell<int> gap_second_id_cell = BasicCell<int>(-1, SCALAR, INPUT);
  BasicCell<std::vector<double>> gap_first_obj_cell =
      BasicCell<std::vector<double>>({}, VECTOR, INPUT);
  BasicCell<std::vector<double>> gap_second_obj_cell =
      BasicCell<std::vector<double>>({}, VECTOR, INPUT);

  // NLOHMANN_DEFINE_TYPE_INTRUSIVE(LateralDecisionDataManager, gap_valid_logic)
private:
};
} // namespace msquare
