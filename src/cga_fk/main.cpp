#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <algorithm>
#include <stack>

#include "cga.h"

namespace cga {

CGA up(const KDL::Vector &vector)
{
    return up(vector.x(), vector.y(), vector.z());
}

const KDL::Vector down(const CGA &mvec)
{
    float normalisation = -(mvec|cga::ninf)[cga::SCALAR];
    return KDL::Vector(
        mvec[E1] / normalisation,
        mvec[E2] / normalisation,
        mvec[E3] / normalisation
    );
}

} // namespace cga

void compute_fk(
    const KDL::Tree &tree,
    std::map<std::string, double> &joint_values)
{
    std::vector<double> theta(3);
    std::vector<double> alpha(3);
    std::vector<double> beta(3);

    theta[0] = joint_values["base_to_upper_1"];
    theta[1] = joint_values["base_to_upper_2"];
    theta[2] = joint_values["base_to_upper_3"];

    // Calculate the necessary alpha_i and beta_i

    // Get s(i) and a(i)
    std::vector<KDL::Vector> s(3);
    std::vector<KDL::Vector> a(3);
    std::vector<cga::CGA> A(3);

    KDL::Chain a_chain;
    KDL::JntArray a_joints = KDL::JntArray(3);
    KDL::Frame a_frame;

    static const std::string pseudo_elbow_names[] = {
        "pseudo_elbow_1", "pseudo_elbow_2", "pseudo_elbow_3"
    };
    static const std::string upper_names[] = {
        "upper_1", "upper_2", "upper_3"
    };

    for (std::size_t i = 0; i < 3; i++) {
        tree.getChain("base", pseudo_elbow_names[i], a_chain);
        KDL::ChainFkSolverPos_recursive fk_solver(a_chain);
        a_joints(0) = theta[i];
        a_joints(1) = theta[i];
        a_joints(2) = 0;
        fk_solver.JntToCart(a_joints, a_frame);
        a[i] = a_frame.p;
        A[i] = cga::up(a[i]);

        s[i] = tree.getSegment(upper_names[i])->second.segment.pose(0).p;
        s[i].Normalize();
    }

    static const std::string virtual_end_effector_names[] = {
        "virtual_end_effector_1", "virtual_end_effector_2", "virtual_end_effector_3"
    };

    std::vector<cga::CGA> PiA(3);
    for (std::size_t i = 0; i < 3; i++) {
        float rho = tree.getSegment(virtual_end_effector_names[i])->second.segment.pose(0).p.Norm();
        PiA[i] = A[i] - 0.5*pow(rho, 2)*cga::ninf;
    }


    cga::CGA(1, cga::E12345);

    cga::CGA T, P, Y;
    T = cga::I5*(PiA[0]^PiA[1]^PiA[2]);
    P = cga::CGA(1.0f, cga::SCALAR) + T.normalized();
    Y = -1 * (~P * (T|cga::ninf) * P);

    KDL::Vector y = down(Y);

    std::vector<KDL::Vector> lower_disp(3);
    for (std::size_t i = 0; i < 3; i++) {
        lower_disp[i] = y - a[i];

        alpha[i] = atan2(
            lower_disp[i].z(),
            lower_disp[i].x()*s[i].x()
            + lower_disp[i].y()*s[i].y()
        );

        beta[i] = atan2(
            - lower_disp[i].y()*s[i].x()
            + lower_disp[i].x()*s[i].y(),
            lower_disp[i].z()
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
}

class Node {
public:
    Node(ros::NodeHandle &n)
    {
        joint_states_sub = n.subscribe(
            "joint_states_in",
            1,
            &Node::joint_states_callback,
            this
        );

        // Parse urdf
        if (!model.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
        }

        // Add joints to joint_values map
        std::stack<urdf::LinkConstSharedPtr> links;
        links.push(model.getRoot());
        urdf::LinkConstSharedPtr it;
        while (!links.empty()) {
            it = links.top();
            links.pop();
            for (std::size_t joint_i = 0; joint_i < it->child_joints.size(); joint_i++) {
                joint_values[it->child_joints[joint_i]->name] = 0;
                links.push(it->child_links[joint_i]);
            }
        }

        // Create KDL tree from URDF.
        // In future, create CGA kinematic tree from urdf
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
            ROS_ERROR("Failed to construct kdl tree");
        }
    }

    void joint_states_callback(sensor_msgs::JointState joint_state)
    {
        // Copy independent joint values to map
        for (std::size_t i = 0; i < joint_state.name.size(); i++) {
            joint_values[joint_state.name[i]] = joint_state.position[i];
        }

        // Complete remaining joint values by satisfying constraints
        compute_fk(tree, joint_values);

        // Traverse tree and publish relative transforms

        tf_msg.header.stamp = ros::Time::now();
        std::stack<KDL::SegmentMap::const_iterator> segments;
        segments.push(tree.getRootSegment());
        KDL::SegmentMap::const_iterator it;

        while (!segments.empty()) {
            it = segments.top();
            segments.pop();

            tf_msg.header.frame_id = it->first;
            for (std::size_t child_i = 0; child_i < it->second.children.size(); child_i++) {
                tf_msg.child_frame_id = it->second.children[child_i]->first;
                std::string joint_name = it->second.children[child_i]->second.segment.getJoint().getName();
                KDL::Frame pose = it->second.children[child_i]->second.segment.pose(joint_values[joint_name]);
                tf_msg.transform.translation.x = pose.p.x();
                tf_msg.transform.translation.y = pose.p.y();
                tf_msg.transform.translation.z = pose.p.z();
                pose.M.GetQuaternion(
                    tf_msg.transform.rotation.x,
                    tf_msg.transform.rotation.y,
                    tf_msg.transform.rotation.z,
                    tf_msg.transform.rotation.w
                );
                tf_broadcaster.sendTransform(tf_msg);
                segments.push(it->second.children[child_i]);
            }
        }
    }

private:
    std::map<std::string, double> joint_values;
    ros::Subscriber joint_states_sub;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped tf_msg;

    urdf::Model model;
    KDL::Tree tree;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;
    Node node(n);
    ros::spin();
}
