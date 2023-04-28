#include <hfsm/machine_single.hpp>
#include <iostream>

class MyStateMachine : public hfsm::Machine {
public:
    // 状态A
    struct StateA : hfsm::State {
        void enter(Control& control) override {}
        void update(FullControl& control) override {}
        void exit(Control& control) override {}
    };

    // 状态B
    struct StateB : hfsm::State {
        void enter(Control& control) override {}
        void update(FullControl& control) override {}
        void exit(Control& control) override {}
    };

    // 跳转条件
    struct EventA {
        int a;
    };

    // 状态机的起始状态为A
    using InitialState = StateA;

    // 状态机的定义
    MyStateMachine() {
        // 如果a等于1，从状态A跳转到状态B
        hfsm::transition<StateA, StateB, EventA>([](const Control& control, const EventA& event) {
            return event.a == 1;
        });
    }
};

class MyCallback {
public:
    void onEnterStateA() {
        std::cout << "Entering state A" << std::endl;
    }

    void onExitStateA() {
        std::cout << "Exiting state A" << std::endl;
    }

    void onEnterStateB() {
        std::cout << "Entering state B" << std::endl;
    }

    void onExitStateB() {
        std::cout << "Exiting state B" << std::endl;
    }
};

class MyStateMachine : public hfsm::Machine {
    // 状态和事件的定义
};

class MyCallback {
    //
public:
void onEnterStateA() {
std::cout << "Entering state A" << std::endl;
}
void onExitStateA() {
    std::cout << "Exiting state A" << std::endl;
}

void onEnterStateB() {
    std::cout << "Entering state B" << std::endl;
}

void onExitStateB() {
    std::cout << "Exiting state B" << std::endl;
}
};

int main() {
MyStateMachine stateMachine;
MyCallback callback;
// 将callback类实例传递给状态机
stateMachine.setContext(callback);

// 在状态机中处理事件
stateMachine.update();
stateMachine.raise(EventA{1});
stateMachine.update();

return 0;
}
