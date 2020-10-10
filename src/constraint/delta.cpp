#include "constraint/delta.h"

loop::Delta::Delta(const urdf::Model &model, const std::map<std::string, std::string> &joint_names)
{

}

bool loop::Delta::apply_fk(std::map<std::string, double> &positions)const
{
    return true;
}

bool loop::Delta::apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions)const
{
    return true;
}
