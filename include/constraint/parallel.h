#ifndef CONSTRAINT_PARALLEL_H
#define CONSTRAINT_PARALLEL_H

#include "constraint/constraint.h"

namespace loop {

class Parallel: public Constraint {
public:
    Parallel(const urdf::Model &model, const std::map<std::string, std::string> &joint_names);
    virtual ~Parallel() {}
    bool apply_fk(std::map<std::string, double> &positions);
    bool apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions);
private:
    std::string joint_alpha;
    std::string joint_L_beta_top, joint_L_beta_bot, joint_L_gamma;
    std::string joint_R_beta_top, joint_R_beta_bot;
    // No dimensions needed
};

} // namespace loop

#endif
