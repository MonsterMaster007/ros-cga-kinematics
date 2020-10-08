#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

#include <algorithm>
#include <stack>

#include "cga.h"

namespace cga {

CGA up(const geometry_msgs::Vector3 &vector)
{
    return up(vector.x, vector.y, vector.z);
}

const geometry_msgs::Vector3 down(const CGA &mvec)
{
    geometry_msgs::Vector3 result;
    float normalisation = -(mvec|cga::ei)[cga::SCALAR];
    result.x = mvec[E1] / normalisation;
    result.y = mvec[E2] / normalisation;
    result.z = mvec[E3] / normalisation;
    return result;
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

    // ...
    /*
    def get_si(i):
        return np.cos(i*np.pi*2/3)*e1 + np.sin(i*np.pi*2/3)*e2

    def get_w(i, theta_i):
        return (rb + l*np.cos(theta_i))*get_si(i) + l*np.sin(theta_i)*e3

    def get_a(i, theta_i):
        return get_w(i, theta_i) - re*get_si(i)

    # Arbitrary angles

    theta = [np.pi*0.3, np.pi*0.2, np.pi*0.5]

    # Calculate joint positions

    W = [up(get_w(i, theta[i])) for i in range(3)]
    A = [up(get_a(i, theta[i])) for i in range(3)]
    PiA = [Ai - 0.5*rho**2*einf for Ai in A]

    T = I5*(PiA[0]^PiA[1]^PiA[2])
    P = 1 + T.normal()
    Y = -~P * (T|ninf) * P
    Ys = [float(Y|ei)*ei for ei in [e1, e2, e3, einf, eo]]
    Y = Ys[0] + Ys[1] + Ys[2] + Ys[3] + Ys[4]
    y = down(Y)

    Z = [up(y + re*get_si(i)) for i in range(3)]
    z = [down(Zi) for Zi in Z]
    */

    // Calculate angles

    alpha[0] = 1.2;
    alpha[1] = 1.4;
    alpha[2] = 1.3;

    beta[0] = 0.2;
    beta[1] = -0.3;
    beta[2] = 0.5;

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
