#include "constraint/parallel.h"

loop::Parallel::Parallel(const urdf::Model &model, const std::map<std::string, std::string> &joint_names)
{

}

bool loop::Parallel::apply_fk(std::map<std::string, double> &positions){
    return true;
}

bool loop::Parallel::apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions)
{
    return true;
}
