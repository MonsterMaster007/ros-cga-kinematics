#ifndef CONSTRAINT_DELTA_H
#define CONSTRAINT_DELTA_H

#include "constraint/constraint.h"

namespace loop {

class Delta: public Constraint {
public:
    Delta(const urdf::Model &model, const std::map<std::string, std::string> &joint_names);
    virtual ~Delta() {}
    bool apply_fk(std::map<std::string, double> &positions);
    bool apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions);
private:
    // TODO
};

} // namespace loop

#endif
