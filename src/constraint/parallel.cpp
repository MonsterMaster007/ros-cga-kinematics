#include "constraint/parallel.h"

loop::Parallel::Parallel(const urdf::Model &model, const std::map<std::string, std::string> &joint_names)
{
    joint_alpha = joint_names.at("alpha");
    joint_L_beta_top = joint_names.at("L_beta_top");
    joint_L_beta_bot = joint_names.at("L_beta_bot");
    joint_R_beta_top = joint_names.at("R_beta_top");
    joint_R_beta_bot = joint_names.at("R_beta_bot");
    joint_L_gamma = joint_names.at("L_gamma");
}

bool loop::Parallel::apply_fk(std::map<std::string, double> &positions)const
{
    positions[joint_L_beta_bot] = -positions[joint_L_beta_top];
    positions[joint_R_beta_top] = positions[joint_L_beta_top];
    positions[joint_R_beta_bot] = -positions[joint_L_beta_top];
    return true;
}

bool loop::Parallel::apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions)const
{
    // TODO
    positions[joint_alpha] = 0;
    positions[joint_L_beta_top] = 0;
    positions[joint_L_gamma] = 0;
    return true;
}
