#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>

#include <algorithm>

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

enum class DeltaJoint {
    BASE_TO_UPPER_1,
    BASE_TO_UPPER_2,
    BASE_TO_UPPER_3,
    UPPER_TO_ELBOW_1,
    UPPER_TO_ELBOW_2,
    UPPER_TO_ELBOW_3,
    ELBOW_TO_LOWER_LEFT_1,
    ELBOW_TO_LOWER_RIGHT_1,
    ELBOW_TO_LOWER_LEFT_2,
    ELBOW_TO_LOWER_RIGHT_2,
    ELBOW_TO_LOWER_LEFT_3,
    ELBOW_TO_LOWER_RIGHT_3,
    LOWER_LEFT_TO_BOT_1,
    LOWER_LEFT_TO_BOT_2,
    LOWER_LEFT_TO_BOT_3,
    LOWER_BOT_TO_END_EFFECTOR_1,
    LOWER_BOT_TO_END_EFFECTOR_2,
    LOWER_BOT_TO_END_EFFECTOR_3
};

const std::vector<std::string> delta_joint_names = {
    "base_to_upper_1",
    "base_to_upper_2",
    "base_to_upper_3",
    "upper_to_elbow_1",
    "upper_to_elbow_2",
    "upper_to_elbow_3",
    "elbow_to_lower_left_1",
    "elbow_to_lower_right_1",
    "elbow_to_lower_left_2",
    "elbow_to_lower_right_2",
    "elbow_to_lower_left_3",
    "elbow_to_lower_right_3",
    "lower_left_to_lower_bot_1",
    "lower_left_to_lower_bot_2",
    "lower_left_to_lower_bot_3",
    "lower_bot_to_end_effector_1",
    "lower_bot_to_end_effector_2",
    "lower_bot_to_end_effector_3"
};

void apply_simple_contraints(sensor_msgs::JointState &joint_state)
{
    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_RIGHT_1]
        = joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_1];

    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_RIGHT_2]
        = joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_2];

    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_RIGHT_3]
        = joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_3];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_LEFT_TO_BOT_1]
        = - joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_1];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_LEFT_TO_BOT_2]
        = - joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_2];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_LEFT_TO_BOT_3]
        = - joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_3];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_BOT_TO_END_EFFECTOR_1]
        = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_1]
        + joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_1];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_BOT_TO_END_EFFECTOR_2]
        = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_2]
        + joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_2];

    joint_state.position[(std::size_t)DeltaJoint::LOWER_BOT_TO_END_EFFECTOR_3]
        = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_3]
        + joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_3];
}

void compute_fk(
    sensor_msgs::JointState &joint_state,
    geometry_msgs::Vector3 &pos)

{
    std::vector<double> theta;
    std::vector<double> alpha;
    std::vector<double> beta;

    theta[0] = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_1];
    theta[1] = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_1];
    theta[2] = joint_state.position[(std::size_t)DeltaJoint::BASE_TO_UPPER_1];

    // Calculate pos

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

    pos.x = 0;
    pos.y = 0;
    pos.z = -0.6;

    // Calculate angles

    // Put angles in joint_state message

    joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_1] = alpha[0];
    joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_2] = alpha[1];
    joint_state.position[(std::size_t)DeltaJoint::UPPER_TO_ELBOW_3] = alpha[2];

    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_1] = beta[0];
    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_2] = beta[1];
    joint_state.position[(std::size_t)DeltaJoint::ELBOW_TO_LOWER_LEFT_3] = beta[2];
}

struct Loop {
    std::vector<std::string> parents;
    std::string child;
    std::vector<std::string> dependent_joints;
    std::vector<std::string> independent_joint;
};

class Node {
public:
    Node(ros::NodeHandle &n)
    {
        joint_states_in_sub = n.subscribe(
            "joint_states_in",
            1,
            &Node::joint_states_callback,
            this
        );
        joint_states_out_pub = n.advertise<sensor_msgs::JointState>(
            "joint_states_out",
            100
        );
        msg_out.name = delta_joint_names;
        msg_out.position.resize(msg_out.name.size());

        end_effector_transform.header.frame_id = "base";
        end_effector_transform.child_frame_id = "end_effector";
        end_effector_transform.transform.rotation.w = 1;

        // Parse urdf
        urdf::Model model;
        if (!model.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
        }

        // Extract loops parameter
        XmlRpc::XmlRpcValue loops;
        if (n.getParam("loops", loops)) {
            assert(loops.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (std::size_t i = 0; i < loops.size(); i++) {
                ROS_INFO("Loop %lu", i);
                XmlRpc::XmlRpcValue &parents = loops[i]["parents"];
                assert(parents.getType() == XmlRpc::XmlRpcValue::TypeArray);
                for (std::size_t j = 0; j < parents.size(); j++) {
                    std::string parent_name(parents[j]);
                    ROS_INFO("Parent %lu: %s", j, parent_name.c_str());
                }
                XmlRpc::XmlRpcValue &child = loops[i]["child"];
                std::string child_name(child);
                ROS_INFO("Child: %s", child_name.c_str());
            }
        } else {
            ROS_INFO("Failed to read loops parameter");
        }

        // TODO: Use the "loops" parameter to get a list of loops which
        // need closing.
    }

    void joint_states_callback(sensor_msgs::JointState msg_in)
    {
        // Copy the joint positions from msg_in to msg_out
        auto name_in_it = msg_in.name.cbegin();
        auto pos_in_it = msg_in.position.cbegin();
        while (name_in_it != msg_in.name.cend()) {
            auto name_out_it = msg_out.name.begin();
            auto pos_out_it = msg_out.position.begin();
            while (name_out_it != msg_out.name.end()) {
                if (*name_in_it == *name_out_it) {
                    *pos_out_it = *pos_in_it;
                    break;
                }
                name_out_it++;
                pos_out_it++;
            }
            name_in_it++;
            pos_in_it++;
        }

        // Compute the forward kinematics, which requires finding
        // the correct joint angles to joint the 3 kinematic chains
        // Additionally, provide a vector for the end effector position
        geometry_msgs::Vector3 end_effector_pos;
        compute_fk(msg_out, end_effector_pos);

        // The remaining joints are constrained in simple ways
        apply_simple_contraints(msg_out);

        msg_out.header.stamp = ros::Time::now();
        joint_states_out_pub.publish(msg_out);

        end_effector_transform.transform.translation.x = end_effector_pos.x;
        end_effector_transform.transform.translation.y = end_effector_pos.y;
        end_effector_transform.transform.translation.z = end_effector_pos.z;
        end_effector_transform.header.stamp = ros::Time::now();
        tf_broadcaster.sendTransform(end_effector_transform);
    }

private:
    ros::Subscriber joint_states_in_sub;
    ros::Publisher joint_states_out_pub;
    sensor_msgs::JointState msg_out;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped end_effector_transform;

    std::vector<Loop> loops;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;
    Node node(n);
    ros::spin();
}
