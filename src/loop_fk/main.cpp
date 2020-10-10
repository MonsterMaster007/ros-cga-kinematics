#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>

#include <algorithm>
#include <stack>

#include "cga/cga.h"

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

void log_cga(const cga::CGA &mvec)
{
    printf("[");
    for (std::size_t i = 0; i < cga::COUNT; i++) {
        printf("%f ", mvec[i]);
    }
    printf("]\n");
}

void log_cga3(const cga::CGA &mvec)
{
    ROS_INFO("[%f, %f, %f]", mvec[cga::E1], mvec[cga::E2], mvec[cga::E3]);
}

struct DeltaDimensions {
    std::vector<cga::CGA> s;
    std::vector<cga::CGA> s_perp;
    std::vector<double> base_radius;
    std::vector<double> ee_radius;
    std::vector<double> upper_length;
    std::vector<double> lower_length;

    DeltaDimensions():
        s(3), s_perp(3), base_radius(3), ee_radius(3),
        upper_length(3), lower_length(3) {}

    void load_model(const urdf::Model &model)
    {
        s[0] = vector_to_cga(
            model.getJoint("base_to_upper_1")->
                parent_to_joint_origin_transform.position);
        base_radius[0] = s[0].norm();
        s[0] = s[0].normalized();
        s_perp[0] = -1 * cga::I3*(cga::e3^s[0]);

        s[1] = vector_to_cga(
            model.getJoint("base_to_upper_2")->
                parent_to_joint_origin_transform.position);
        base_radius[1] = s[1].norm();
        s[1] = s[1].normalized();
        s_perp[1] = -1 * cga::I3*(cga::e3^s[1]);

        s[2] = vector_to_cga(
            model.getJoint("base_to_upper_3")->
                parent_to_joint_origin_transform.position);
        base_radius[2] = s[2].norm();
        s[2] = s[2].normalized();
        s_perp[2] = -1 * cga::I3*(cga::e3^s[2]);

        ee_radius[0] = vector_norm(
            model.getJoint("pseudo_elbow_offset_to_pseudo_elbow_1")->
                parent_to_joint_origin_transform.position);
        ee_radius[1] = vector_norm(
            model.getJoint("pseudo_elbow_offset_to_pseudo_elbow_2")->
                parent_to_joint_origin_transform.position);
        ee_radius[2] = vector_norm(
            model.getJoint("pseudo_elbow_offset_to_pseudo_elbow_3")->
                parent_to_joint_origin_transform.position);

        upper_length[0] = vector_norm(
            model.getJoint("upper_to_lower_top_1")->
                parent_to_joint_origin_transform.position);
        upper_length[1] = vector_norm(
            model.getJoint("upper_to_lower_top_2")->
                parent_to_joint_origin_transform.position);
        upper_length[2] = vector_norm(
            model.getJoint("upper_to_lower_top_3")->
                parent_to_joint_origin_transform.position);

        lower_length[0] = vector_norm(
            model.getJoint("virtual_lower_to_virtual_end_effector_1")->
                parent_to_joint_origin_transform.position);
        lower_length[1] = vector_norm(
            model.getJoint("virtual_lower_to_virtual_end_effector_2")->
                parent_to_joint_origin_transform.position);
        lower_length[2] = vector_norm(
            model.getJoint("virtual_lower_to_virtual_end_effector_3")->
                parent_to_joint_origin_transform.position);
    }
};

bool compute_fk(
    const DeltaDimensions &dim,
    std::map<std::string, double> &joint_values)
{
    std::vector<double> theta(3);
    std::vector<double> alpha(3);
    std::vector<double> beta(3);

    theta[0] = joint_values["base_to_upper_1"];
    theta[1] = joint_values["base_to_upper_2"];
    theta[2] = joint_values["base_to_upper_3"];

    // Calculate the necessary alpha_i and beta_i

    std::vector<cga::CGA> a(3);
    std::vector<cga::CGA> A(3);
    std::vector<cga::CGA> PiA(3);

    for (std::size_t i = 0; i < 3; i++) {
        a[i] = (dim.base_radius[i] - dim.ee_radius[i]
                + dim.upper_length[i] * cos(theta[i])) * dim.s[i]
            - dim.upper_length[i] * sin(theta[i]) * cga::e3;
        A[i] = cga::up(a[i]);
        PiA[i] = A[i] - 0.5*pow(dim.lower_length[i], 2)*cga::ninf;
    }

    cga::CGA T, P, Y, y;
    T = cga::I5*(PiA[0]^PiA[1]^PiA[2]);
    P = cga::CGA(1.0f, cga::SCALAR) + T.normalized();
    Y = -1 * (~P * (T|cga::ninf) * P);
    y = down(Y);

    // TODO: Check there is a possible solution
    // If not, return false and don't update joint_values

    cga::CGA lower_disp;
    for (std::size_t i = 0; i < 3; i++) {
        lower_disp = y - a[i];
        alpha[i] = atan2(
            -(lower_disp|cga::e3)[0],
            -(lower_disp|dim.s[i])[0]
        );
        beta[i] = -atan2(
            (lower_disp|dim.s_perp[i])[0],
            -(lower_disp|cga::e3)[0]
        );
    }

    // Put angles in joint_state message

    joint_values["upper_to_lower_top_1"] = theta[0] + alpha[0];
    joint_values["upper_to_lower_top_2"] = theta[1] + alpha[1];
    joint_values["upper_to_lower_top_3"] = theta[2] + alpha[2];

    joint_values["upper_to_pseudo_elbow_offset_1"] = theta[0];
    joint_values["upper_to_pseudo_elbow_offset_2"] = theta[1];
    joint_values["upper_to_pseudo_elbow_offset_3"] = theta[2];

    joint_values["pseudo_elbow_offset_to_pseudo_elbow_1"] = alpha[0];
    joint_values["pseudo_elbow_offset_to_pseudo_elbow_2"] = alpha[1];
    joint_values["pseudo_elbow_offset_to_pseudo_elbow_3"] = alpha[2];

    joint_values["pseudo_elbow_to_virtual_lower_1"] = beta[0];
    joint_values["pseudo_elbow_to_virtual_lower_2"] = beta[1];
    joint_values["pseudo_elbow_to_virtual_lower_3"] = beta[2];

    joint_values["lower_top_to_lower_left_1"] = beta[0];
    joint_values["lower_top_to_lower_left_2"] = beta[1];
    joint_values["lower_top_to_lower_left_3"] = beta[2];

    joint_values["lower_top_to_lower_right_1"] = beta[0];
    joint_values["lower_top_to_lower_right_2"] = beta[1];
    joint_values["lower_top_to_lower_right_3"] = beta[2];

    joint_values["lower_left_to_lower_bot_1"] = beta[0];
    joint_values["lower_left_to_lower_bot_2"] = beta[1];
    joint_values["lower_left_to_lower_bot_3"] = beta[2];

    joint_values["lower_bot_1_to_virtual_end_effector_offset"] = alpha[0];
    
    return true;
}

class Node {
public:
    Node(ros::NodeHandle &n)
    {
        joint_state_sub = n.subscribe(
            "joint_states_in", 1, &Node::joint_states_callback, this
        );

        joint_state_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out", 1
        );

        // Parse urdf
        if (!model.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
        }
        dim.load_model(model);

        // Create joint_values map, and joint_state_msg

        std::stack<urdf::LinkConstSharedPtr> links;
        links.push(model.getRoot());
        urdf::LinkConstSharedPtr it;
        while (!links.empty()) {
            it = links.top();
            links.pop();
            for (std::size_t joint_i = 0; joint_i < it->child_joints.size(); joint_i++) {
                joint_state_msg.name.push_back(it->child_joints[joint_i]->name);
                joint_values[it->child_joints[joint_i]->name] = 0;
                links.push(it->child_links[joint_i]);
            }
        }
        joint_state_msg.position.resize(joint_state_msg.name.size());
        joint_state_msg.velocity.resize(joint_state_msg.name.size());
        joint_state_msg.effort.resize(joint_state_msg.name.size());
    }

    void joint_states_callback(sensor_msgs::JointState joint_state_msg_in)
    {
        // Copy independent joint values to map
        for (std::size_t i = 0; i < joint_state_msg_in.name.size(); i++) {
            joint_values[joint_state_msg_in.name[i]] = joint_state_msg_in.position[i];
        }

        // Complete remaining joint values by satisfying constraints
        compute_fk(dim, joint_values);

        for (std::size_t i = 0; i < joint_state_msg.name.size(); i++) {
            joint_state_msg.position[i] = joint_values[joint_state_msg.name[i]];
        }

        joint_state_msg.header.stamp = joint_state_msg_in.header.stamp;
        joint_state_pub.publish(joint_state_msg);
    }

private:
    std::map<std::string, double> joint_values;
    ros::Subscriber joint_state_sub;

    sensor_msgs::JointState joint_state_msg;
    ros::Publisher joint_state_pub;

    urdf::Model model;
    DeltaDimensions dim;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;
    Node node(n);
    ros::spin();
}
