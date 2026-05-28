#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // Optional keyboard_transitions: parallel to transitions but triggered by a
        // single key string instead of the joystick DSL. Used when no gamepad is connected.
        // Example in config.yaml:
        //   keyboard_transitions:
        //     FixStand: "\r"    # Enter
        //     Passive:  "\033"  # Esc
        auto kb_transitions = param::config["FSM"][state_string]["keyboard_transitions"];
        if(kb_transitions && keyboard)
        {
            auto kb_map = kb_transitions.as<std::map<std::string, std::string>>();
            for(auto it = kb_map.begin(); it != kb_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap (keyboard_transitions)!", target_fsm);
                    continue;
                }
                int fsm_id = FSMStringMap.right.at(target_fsm);
                std::string trigger_key = it->second;
                registered_checks.emplace_back(
                    std::make_pair(
                        [trigger_key]()->bool{
                            return keyboard && keyboard->on_pressed
                                   && keyboard->key() == trigger_key;
                        },
                        fsm_id
                    )
                );
                spdlog::info("  Keyboard shortcut: '{}' → {}", trigger_key, target_fsm);
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};