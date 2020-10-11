#include "constraint/parallel.h"

loop::Parallel::Parallel(const urdf::Model &model, const std::map<std::string, std::string> &joints):Constraint(joints)
{
    // TODO: Verify joints are present in model
    // No dimensions to find
}

bool loop::Parallel::apply_fk(std::map<std::string, double> &positions)const
{
    positions[joints.at("L_beta_bot")] = -positions[joints.at("L_beta_top")];
    positions[joints.at("R_beta_top")] = positions[joints.at("L_beta_top")];
    return true;
}

bool loop::Parallel::apply_ik(const geometry_msgs::Pose &pose, std::map<std::string, double> &positions)const
{
    // TODO
    positions[joints.at("alpha")] = 0;
    positions[joints.at("L_beta_top")] = 0;
    positions[joints.at("gamma")] = 0;
    return true;
}
