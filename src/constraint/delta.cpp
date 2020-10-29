#include "constraint/delta.h"
#include "cga/transformations.h"
#include "cga/constants.h"
#include "cga/multivector.h"

const cga::Vector to_cga_vector(const urdf::Vector3 &vec)
{
    return cga::Vector(vec.x, vec.y, vec.z, 0, 0);
}

const cga::Vector to_cga_vector(const geometry_msgs::Pose::_position_type &vec)
{
    return cga::Vector(vec.x, vec.y, vec.z, 0, 0);
}

const double vector_norm(const urdf::Vector3 &vec)
{
    return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

loop::Delta::Delta(
    const urdf::Model &model,
    const std::map<std::string, std::string> &joints):
    Constraint(joints),
    s(3), s_perp(3), base_radius(3), ee_radius(3),
    upper_length(3), lower_length(3)
{
    s[0] = to_cga_vector(
        model.getJoint(joints.at("theta_1"))->
            parent_to_joint_origin_transform.position);
    base_radius[0] = cga::Multivector(s[0]).norm();
    s[0] = cga::Multivector(s[0]).normalized();
    s_perp[0] = -cga::I3*wedge(cga::e3, s[0]);

    s[1] = to_cga_vector(
        model.getJoint(joints.at("theta_2"))->
            parent_to_joint_origin_transform.position);
    base_radius[1] = cga::Multivector(s[1]).norm();
    s[1] = s[1].normalized();
    s_perp[1] = -cga::I3*wedge(cga::e3, s[1]);

    s[2] = to_cga_vector(
        model.getJoint(joints.at("theta_3"))->
            parent_to_joint_origin_transform.position);
    base_radius[2] = cga::Multivector(s[2]).norm();
    s[2] = s[2].normalized();
    s_perp[2] = -cga::I3*wedge(cga::e3, s[2]);

    ee_radius[0] = vector_norm(
        model.getJoint(joints.at("to_end_effector_1"))->
            parent_to_joint_origin_transform.position);
    ee_radius[1] = vector_norm(
        model.getJoint(joints.at("to_end_effector_2"))->
            parent_to_joint_origin_transform.position);
    ee_radius[2] = vector_norm(
        model.getJoint(joints.at("to_end_effector_3"))->
            parent_to_joint_origin_transform.position);

    upper_length[0] = vector_norm(
        model.getJoint(joints.at("to_lower_1"))->
            parent_to_joint_origin_transform.position);
    upper_length[1] = vector_norm(
        model.getJoint(joints.at("to_lower_2"))->
            parent_to_joint_origin_transform.position);
    upper_length[2] = vector_norm(
        model.getJoint(joints.at("to_lower_3"))->
            parent_to_joint_origin_transform.position);

    lower_length[0] = vector_norm(
        model.getJoint("lower_1_L_beta_bot")->
            parent_to_joint_origin_transform.position);
    lower_length[1] = vector_norm(
        model.getJoint("lower_2_L_beta_bot")->
            parent_to_joint_origin_transform.position);
    lower_length[2] = vector_norm(
        model.getJoint("lower_3_L_beta_bot")->
            parent_to_joint_origin_transform.position);
}

bool loop::Delta::apply_fk(std::map<std::string, double> &positions)const
{
    std::vector<double> theta(3);
    std::vector<double> alpha(3);
    std::vector<double> beta(3);

    theta[0] = positions[joints.at("theta_1")];
    theta[1] = positions[joints.at("theta_2")];
    theta[2] = positions[joints.at("theta_3")];

    // Calculate the necessary alpha_i and beta_i

    std::vector<cga::Vector> a(3);
    std::vector<cga::Vector> A(3);
    // Dual spheres = vectors
    std::vector<cga::Vector> PiA(3);

    for (std::size_t i = 0; i < 3; i++) {
        a[i] = (base_radius[i] - ee_radius[i]
                + upper_length[i] * cos(theta[i])) * s[i]
            - upper_length[i] * sin(theta[i]) * cga::e3;
        A[i] = cga::up(a[i]);
        PiA[i] = A[i] - 0.5*pow(lower_length[i], 2)*cga::ni;
    }

    cga::Multivector T, P, Y, y;
    T = cga::Multivector(cga::I5)*wedge(PiA[0], wedge(PiA[1], PiA[2]));
    if ((T*T).scalar() < 0) return false;

    P = 1 + T.normalized();
    Y = -(P.reverse() * dot(T, cga::ni) * P);
    y = cga::down(Y.vector());

    cga::Vector lower_disp;
    for (std::size_t i = 0; i < 3; i++) {
        lower_disp = (y - a[i]).vector();
        alpha[i] = atan2(
            -dot(lower_disp, cga::e3).scalar(),
            -dot(lower_disp, s[i]).scalar()
        );
        beta[i] = atan2(
            dot(lower_disp, s_perp[i]).scalar(),
            -dot(lower_disp, cga::e3).scalar()
        );
    }

    // Put angles in joint_state message

    positions[joints.at("lower_1_alpha")] = M_PI - theta[0] - alpha[0];
    positions[joints.at("lower_2_alpha")] = M_PI - theta[1] - alpha[1];
    positions[joints.at("lower_3_alpha")] = M_PI - theta[2] - alpha[2];

    positions[joints.at("lower_1_L_beta_top")] = beta[0];
    positions[joints.at("lower_2_L_beta_top")] = beta[1];
    positions[joints.at("lower_3_L_beta_top")] = beta[2];

    positions[joints.at("lower_1_L_gamma")] = alpha[0];
    positions[joints.at("lower_2_L_gamma")] = alpha[1];
    positions[joints.at("lower_3_L_gamma")] = alpha[2];

    return true;
}

bool loop::Delta::apply_ik(const geometry_msgs::Pose &pose, std::map<std::string, double> &positions)const
{
    cga::Vector y = to_cga_vector(pose.position);

    std::vector<cga::Vector> C(3);
    // Dual spheres, so vectors
    std::vector<cga::Vector> Sigma(3);

    for (std::size_t i = 0; i < 3; i++) {
        C[i] = wedge(
            (cga::up((base_radius[i]*s[i]).vector()) - 0.5*pow(upper_length[i], 2)*cga::ni),
            (cga::I3*wedge(s[i], cga::e3))
        ).vector();
        Sigma[i] = (cga::up((y + ee_radius[i]*s[i]).vector()) - 0.5*pow(lower_length[i], 2)*cga::ni).vector();
    }

    std::vector<cga::Multivector> T(3), P(3), A(3), a(3), z(3);
    for (std::size_t i = 0; i < 3; i++) {
        T[i] = -wedge(C[i], Sigma[i])*cga::Multivector(cga::I5);
        // Check valid:
        if ((T[i]*T[i]).scalar() < 0) return false;

        P[i] = 0.5*(1 + T[i]*(1 / sqrt((T[i]*T[i]).scalar())) );
        A[i] = -P[i].reverse()*dot(T[i], cga::ni)*P[i];
        a[i] = cga::down(A[i].vector());
        z[i] = a[i] - ee_radius[i]*s[i];
    }

    positions[joints.at("theta_1")] = atan2(
        dot(z[0], cga::e3).scalar(),
        dot(z[0], s[0]).scalar()
    );
    positions[joints.at("theta_2")] = atan2(
        dot(z[1], cga::e3).scalar(),
        dot(z[1], s[1]).scalar()
    );
    positions[joints.at("theta_3")] = atan2(
        dot(z[2], cga::e3).scalar(),
        dot(z[2], s[2]).scalar()
    );

    return true;
}
