#ifndef CONSTRAINT_CONSTRAINT_H
#define CONSTRAINT_CONSTRAINT_H

#include <urdf/model.h>

namespace loop {

class Constraint {
public:
    virtual ~Constraint();
    virtual bool construct(const urdf::Model &model, std::string param)=0;
    virtual bool apply_fk(std::map<std::string, double> &positions)=0;
    virtual bool apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions)=0;
};

} // namespace loop

#endif