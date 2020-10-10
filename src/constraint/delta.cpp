#include "constraint/delta.h"

const cga::CGA vector_to_cga(const urdf::Vector3 &vec)
{
    cga::CGA result;
    result[cga::E1] = vec.x;
    result[cga::E2] = vec.y;
    result[cga::E3] = vec.z;
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
    s[0] = vector_to_cga(
        model.getJoint(joints.at("theta_1"))->
            parent_to_joint_origin_transform.position);
    base_radius[0] = s[0].norm();
    s[0] = s[0].normalized();
    s_perp[0] = -1 * cga::I3*(cga::e3^s[0]);


    s[1] = vector_to_cga(
        model.getJoint(joints.at("theta_2"))->
            parent_to_joint_origin_transform.position);
    base_radius[1] = s[1].norm();
    s[1] = s[1].normalized();
    s_perp[1] = -1 * cga::I3*(cga::e3^s[1]);

    s[2] = vector_to_cga(
        model.getJoint(joints.at("theta_3"))->
            parent_to_joint_origin_transform.position);
    base_radius[2] = s[2].norm();
    s[2] = s[2].normalized();
    s_perp[2] = -1 * cga::I3*(cga::e3^s[2]);

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

    std::vector<cga::CGA> a(3);
    std::vector<cga::CGA> A(3);
    std::vector<cga::CGA> PiA(3);

    for (std::size_t i = 0; i < 3; i++) {
        a[i] = (base_radius[i] - ee_radius[i]
                + upper_length[i] * cos(theta[i])) * s[i]
            - upper_length[i] * sin(theta[i]) * cga::e3;
        A[i] = cga::up(a[i]);
        PiA[i] = A[i] - 0.5*pow(lower_length[i], 2)*cga::ninf;
    }

    cga::CGA T, P, Y, y;
    T = cga::I5*(PiA[0]^PiA[1]^PiA[2]);
    P = cga::CGA(1.0f, cga::SCALAR) + T.normalized();
    Y = -1 * (~P * (T|cga::ninf) * P);
    y = cga::down(Y);

    // TODO: Check there is a possible solution
    // If not, return false and don't update joint_values

    cga::CGA lower_disp;
    for (std::size_t i = 0; i < 3; i++) {
        lower_disp = y - a[i];
        alpha[i] = atan2(
            -(lower_disp|cga::e3)[0],
            -(lower_disp|s[i])[0]
        );
        beta[i] = atan2(
            (lower_disp|s_perp[i])[0],
            -(lower_disp|cga::e3)[0]
        );
    }

    // Put angles in joint_state message

    positions[joints.at("lower_1_alpha")] = PI - theta[0] - alpha[0];
    positions[joints.at("lower_2_alpha")] = PI - theta[1] - alpha[1];
    positions[joints.at("lower_3_alpha")] = PI - theta[2] - alpha[2];

    positions[joints.at("lower_1_L_beta_top")] = beta[0];
    positions[joints.at("lower_2_L_beta_top")] = beta[1];
    positions[joints.at("lower_3_L_beta_top")] = beta[2];

    positions[joints.at("lower_1_L_gamma")] = alpha[0];
    positions[joints.at("lower_2_L_gamma")] = alpha[1];
    positions[joints.at("lower_3_L_gamma")] = alpha[2];

    return true;
}

bool loop::Delta::apply_ik(const urdf::Pose &pose, std::map<std::string, double> &positions)const
{
    // TODO
    return true;
}
