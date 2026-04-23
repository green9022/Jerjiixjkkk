#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class BaseState
{
public:
    BaseState(int state, std::string state_string)
        : state(state), state_string(std::move(state_string))
    {}

    virtual ~BaseState() = default;

    virtual void enter() {}
    virtual void run() {}
    virtual void exit() {}
    virtual void pre_run() {}
    virtual void post_run() {}

    bool isState(int s) const { return state == s; }
    int getState() const { return state; }
    std::string getStateString() const { return state_string; }

    // 原有静态检查
    std::vector<std::pair<std::function<bool()>, int>> registered_checks;

    // ===== 新增：动态状态切换 =====
    std::atomic<bool> request_dynamic_transition{false};
    int dynamic_next_state_mode = 0;

protected:
    int state;
    std::string state_string;
};




void run_()
{
    currentState->pre_run();
    currentState->run();
    currentState->post_run();
    
    int nextStateMode = 0;

    // 1. 优先处理动态切换请求
    if (currentState->request_dynamic_transition.load())
    {
        nextStateMode = currentState->dynamic_next_state_mode;
        currentState->request_dynamic_transition = false;
    }
    else
    {
        // 2. 再处理原有静态 registered_checks
        for (int i = 0; i < currentState->registered_checks.size(); i++)
        {
            if (currentState->registered_checks[i].first())
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }
    }

    if (nextStateMode != 0 && !currentState->isState(nextStateMode))
    {
        for (auto& state : states)
        {
            if (state->isState(nextStateMode))
            {
                spdlog::info("FSM: Change state from {} to {}",
                             currentState->getStateString(),
                             state->getStateString());
                currentState->exit();
                currentState = state;
                currentState->enter();
                break;
            }
        }
    }
}



