#include "constraint/delta.h"
#include "cga/operations.h"
#include "cga/geometry.h"
#include "cga/printing.h"

const cga::Vector to_cga_vector(const urdf::Vector3 &vec)
{
    cga::Vector result;
    result.e1(vec.x);
    result.e2(vec.y);
    result.e3(vec.z);
    return result;
}

const cga::Vector to_cga_vector(const geometry_msgs::Pose::_position_type &vec)
{
    cga::Vector result;
    result.e1(vec.x);
    result.e2(vec.y);
    result.e3(vec.z);
    return result;
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
    cga::Vector asdf2 = normalized(s[0]);
    base_radius[0] = norm(s[0]);
    s[0] = normalized(s[0]);
    s_perp[0] = -(cga::I3*outer(cga::e3, s[0])).vector();

    s[1] = to_cga_vector(
        model.getJoint(joints.at("theta_2"))->
            parent_to_joint_origin_transform.position);
    base_radius[1] = norm(s[1]);
    s[1] = normalized(s[1]);
    s_perp[1] = -(cga::I3*outer(cga::e3, s[1])).vector();

    s[2] = to_cga_vector(
        model.getJoint(joints.at("theta_3"))->
            parent_to_joint_origin_transform.position);
    base_radius[2] = norm(s[2]);
    s[2] = normalized(s[2]);
    s_perp[2] = -(cga::I3*outer(cga::e3, s[2])).vector();

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
                + upper_length[i]*cos(theta[i])) * s[i]
            - upper_length[i] * sin(theta[i]) * cga::e3;
        A[i] = cga::up(a[i]);
        PiA[i] = A[i] - 0.5*pow(lower_length[i], 2)*cga::ni;
    }

    cga::Bivector T = cga::I5*(PiA[0]^PiA[1]^PiA[2]);
    if ((T*T).scalar() < 0) return false;

    cga::Versor P;
    P.scalar(1);
    P.bivector(T/sqrt((T*T).scalar()));

    cga::Vector Y = -(reverse(P) * inner(T, cga::ni) * P).vector();

    cga::Vector y = -cga::down(Y);

    cga::Vector lower_disp;
    for (std::size_t i = 0; i < 3; i++) {
        lower_disp = y - a[i];
        alpha[i] = atan2(
            -inner(lower_disp, cga::e3),
            -inner(lower_disp, s[i])
        );
        beta[i] = atan2(
            inner(lower_disp, s_perp[i]),
            inner(lower_disp, -cos(alpha[i])*s[i]-sin(alpha[i])*cga::e3)
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

    std::vector<cga::Trivector> C(3);
    // Dual spheres, so vectors
    std::vector<cga::Vector> Sigma(3);

    for (std::size_t i = 0; i < 3; i++) {
        C[i] = outer(
            cga::up(base_radius[i]*s[i] - 0.5*pow(upper_length[i], 2)*cga::ni),
            (cga::I3*(s[i]^cga::e3)).bivector()
        );
        Sigma[i] = cga::up(y + ee_radius[i]*s[i]) - 0.5*pow(lower_length[i], 2)*cga::ni;
    }

    std::vector<cga::Multivector> P(3);
    std::vector<cga::Vector> T(3), A(3), a(3), z(3);
    for (std::size_t i = 0; i < 3; i++) {
        T[i] = -dual(outer(C[i], Sigma[i]));
        // Check valid:
        if ((T[i]*T[i]).scalar() < 0) return false;

        P[i] = 0.5*(1 + T[i]*(1 / sqrt((T[i]*T[i]).scalar())) );
        A[i] = -(reverse(P[i])*inner(T[i], cga::ni)*P[i]).vector();
        a[i] = cga::down(A[i]);
        z[i] = a[i] - ee_radius[i]*s[i];
    }

    positions[joints.at("theta_1")] = atan2(
        inner(z[0], cga::e3),
        inner(z[0], s[0])
    );
    positions[joints.at("theta_2")] = atan2(
        inner(z[1], cga::e3),
        inner(z[1], s[1])
    );
    positions[joints.at("theta_3")] = atan2(
        inner(z[2], cga::e3),
        inner(z[2], s[2])
    );

    return true;
}
