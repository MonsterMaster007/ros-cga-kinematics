#ifndef CONSTRAINT_DELTA_H
#define CONSTRAINT_DELTA_H

#include "constraint/constraint.h"
#include "cga/grades.h"

namespace loop {

class Delta: public Constraint {
public:
    Delta(const urdf::Model &model, const std::map<std::string, std::string> &joints);
    virtual ~Delta() {}
    bool apply_fk(std::map<std::string, double> &positions)const;
    bool apply_ik(const geometry_msgs::Pose &pose, std::map<std::string, double> &positions)const;
private:
    std::vector<cga::Vector> s;
    std::vector<cga::Vector> s_perp;
    std::vector<double> base_radius;
    std::vector<double> ee_radius;
    std::vector<double> upper_length;
    std::vector<double> lower_length;
};

} // namespace loop

#endif
