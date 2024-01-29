#ifndef MSQUARE_DECISION_PLANNING_PARKING_STATEMACHINE_H_
#define MSQUARE_DECISION_PLANNING_PARKING_STATEMACHINE_H_

#include "common/hfsm/machine_single.hpp"
#include <functional>
#include <map>

// simpify the definition of State with callback
#define DEFINE_STATE(classname, baseclassname)                                 \
  struct classname : baseclassname {                                           \
    void enter(const Context &context) {                                       \
      context.callback_on_entry<classname>();                                  \
    }                                                                          \
    void transition(Control &control, Context &context) {                      \
      context.callback_on_transition<classname>(control);                      \
    }                                                                          \
    void update(const Context &context) {                                      \
      context.callback_on_update<classname>();                                 \
    }                                                                          \
    void leave(const Context &context) {                                       \
      context.callback_on_leave<classname>();                                  \
    }                                                                          \
  };

namespace msquare {
namespace parking_statemachine {

class Context {
  std::map<std::string, std::function<void()>> entry_callback_map;
  std::map<std::string, std::function<void()>> update_callback_map;
  std::map<std::string, std::function<void(hfsm::Machine<Context>::Control &)>>
      transition_callback_map;
  std::map<std::string, std::function<void()>> leave_callback_map;

public:
  template <typename T>
  void register_entry_callback(std::function<void()> func) {
    entry_callback_map[typeid(T).name()] = func;
  }

  template <typename T>
  void register_update_callback(std::function<void()> func) {
    update_callback_map[typeid(T).name()] = func;
  }

  template <typename T>
  void register_transition_callback(
      std::function<void(hfsm::Machine<Context>::Control &)> func) {
    transition_callback_map[typeid(T).name()] = func;
  }

  template <typename T>
  void register_leave_callback(std::function<void()> func) {
    leave_callback_map[typeid(T).name()] = func;
  }

  template <typename T> void callback_on_entry() const {
    if (entry_callback_map.find(typeid(T).name()) != entry_callback_map.end()) {
      entry_callback_map.at(typeid(T).name())();
    } else {
      // std::cout << "entry callback for " << typeid(T).name() << " not found"
      // << std::endl;
    }
  }

  template <typename T> void callback_on_update() const {
    if (update_callback_map.find(typeid(T).name()) !=
        update_callback_map.end()) {
      update_callback_map.at(typeid(T).name())();
    } else {
      // std::cout << "update callback for " << typeid(T).name() << " not found"
      // << std::endl;
    }
  }

  template <typename T>
  void callback_on_transition(hfsm::Machine<Context>::Control &control) {
    if (transition_callback_map.find(typeid(T).name()) !=
        transition_callback_map.end()) {
      transition_callback_map.at(typeid(T).name())(control);
    } else {
      // std::cout << "transition callback for " << typeid(T).name() << " not
      // found" << std::endl;
    }
  }

  template <typename T> void callback_on_leave() const {
    if (leave_callback_map.find(typeid(T).name()) != leave_callback_map.end()) {
      leave_callback_map.at(typeid(T).name())();
    } else {
      // std::cout << "leave callback for " << typeid(T).name() << " not found"
      // << std::endl;
    }
  }
};

namespace apa {

// forward declare the structure of states
#define S(s) struct s

using COMPOSITE = hfsm::Machine<Context>::CompositePeers<S(SimpleScene)>;

#undef S

DEFINE_STATE(SimpleScene, hfsm::Machine<Context>::Base)
} // namespace apa

// forward declare the structure of states
#define S(s) struct s

using COMPOSITE = hfsm::Machine<Context>::CompositePeers<
    S(WAIT),
    hfsm::Machine<Context>::Composite<S(APA),
                                      parking_statemachine::apa::COMPOSITE>,
    S(APOA), S(RPA_STRAIGHT_STANDBY), S(RPA_STRAIGHT)>;

#undef S

DEFINE_STATE(WAIT, hfsm::Machine<Context>::Base)
DEFINE_STATE(APA, hfsm::Machine<Context>::Base)
DEFINE_STATE(APOA, hfsm::Machine<Context>::Base)
DEFINE_STATE(RPA_STRAIGHT_STANDBY, hfsm::Machine<Context>::Base)
DEFINE_STATE(RPA_STRAIGHT, hfsm::Machine<Context>::Base)

} // namespace parking_statemachine
} // namespace msquare
#endif
