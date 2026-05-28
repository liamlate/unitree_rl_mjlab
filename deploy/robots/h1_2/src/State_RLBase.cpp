#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// Keyboard velocity commands — copied from deploy/robots/g1/src/State_RLBase.cpp.
// To activate: change "velocity_commands" → "keyboard_velocity_commands" in
// config/policy/velocity/v0/params/deploy.yaml (observations section).
//
// Key mapping (same as G1 and h1v2-Isaac convention):
//   w/s = forward/backward   a/d = strafe left/right   q/e = turn left/right
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", { 1.0f,  0.0f,  0.0f}},
        {"s", {-1.0f,  0.0f,  0.0f}},
        {"a", { 0.0f,  1.0f,  0.0f}},
        {"d", { 0.0f, -1.0f,  0.0f}},
        {"q", { 0.0f,  0.0f,  1.0f}},
        {"e", { 0.0f,  0.0f, -1.0f}},
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    auto it = key_commands.find(key);
    if (it != key_commands.end()) cmd = it->second;
    return cmd;
}
} // namespace isaaclab

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    // H1_2_DEPLOY_CFG selects sim (keyboard) vs real (gamepad) params.
    // Set automatically by setup_all_mujoco.sh / setup_all_robot.sh.
    const char* cfg_env = std::getenv("H1_2_DEPLOY_CFG");
    std::string deploy_cfg = cfg_env ? cfg_env : "deploy.yaml";
    spdlog::info("Using deploy config: {}", deploy_cfg);
    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / deploy_cfg),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for(int i(0); i < env->robot->data.joint_ids_map.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }
}