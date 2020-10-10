#ifndef CONSTRAINT_CONSTRAINT_H
#define CONSTRAINT_CONSTRAINT_H

#include <urdf/model.h>
#include <geometry_msgs/Pose.h>

namespace loop {

class Constraint {
public:
    Constraint(const std::map<std::string, std::string> joints):joints(joints){}
    virtual ~Constraint() {}
    virtual bool apply_fk(std::map<std::string, double> &positions)const =0;
    virtual bool apply_ik(const geometry_msgs::Pose &pose, std::map<std::string, double> &positions)const =0;
protected:
    std::map<std::string, std::string> joints;
};

} // namespace loop

#endif
