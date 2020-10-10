#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <urdf/model.h>

#include <algorithm>
#include <stack>

#include "constraint/constraint.h"
#include "constraint/parallel.h"
#include "constraint/delta.h"


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
        urdf::Model model;
        if (!model.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
        }

        // Create joint_values map, and joint_state_msg

        std::stack<urdf::LinkConstSharedPtr> links;
        links.push(model.getRoot());
        urdf::LinkConstSharedPtr it;
        while (!links.empty()) {
            it = links.top();
            links.pop();
            for (std::size_t joint_i = 0; joint_i < it->child_joints.size(); joint_i++) {
                if (it->child_joints[joint_i]->type != urdf::Joint::FIXED &&
                    it->child_joints[joint_i]->type != urdf::Joint::FLOATING)
                {
                    joint_state_msg.name.push_back(it->child_joints[joint_i]->name);
                    joint_values[it->child_joints[joint_i]->name] = 0;
                }
                links.push(it->child_links[joint_i]);
            }
        }
        joint_state_msg.position.resize(joint_state_msg.name.size());
        joint_state_msg.velocity.resize(joint_state_msg.name.size());
        joint_state_msg.effort.resize(joint_state_msg.name.size());

        // Extract loops parameter

        std::map<std::string, std::string> loop_joints;

        XmlRpc::XmlRpcValue loops;
        if (n.getParam("loops", loops)) {
            assert(loops.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (std::size_t i = 0; i < loops.size(); i++) {
                std::string class_name(loops[i]["class"]);
                XmlRpc::XmlRpcValue &joints = loops[i]["joints"];
                for (auto it = joints.begin(); it != joints.end(); it++) {
                    loop_joints[it->first] = std::string(it->second);
                }
                if (class_name == "delta") {
                    loop_constraints.push_back(std::unique_ptr<loop::Constraint>(
                        new loop::Delta(model, loop_joints)
                    ));
                } else if (class_name == "parallel") {
                    loop_constraints.push_back(std::unique_ptr<loop::Constraint>(
                        new loop::Parallel(model, loop_joints)
                    ));
                } else {
                    ROS_INFO("Unknown loop class %s", class_name.c_str());
                }
                loop_joints.clear();
            }
        } else {
            ROS_INFO("Failed to read loops parameter");
        }
    }

    void joint_states_callback(sensor_msgs::JointState joint_state_msg_in)
    {
        // Copy independent joint values to map
        for (std::size_t i = 0; i < joint_state_msg_in.name.size(); i++) {
            joint_values[joint_state_msg_in.name[i]] = joint_state_msg_in.position[i];
        }

        for (auto it = loop_constraints.cbegin(); it != loop_constraints.cend(); it++) {
            (*it)->apply_fk(joint_values);
        }

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

    std::vector<std::unique_ptr<loop::Constraint>> loop_constraints;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_fk");
    ros::NodeHandle n;
    Node node(n);
    ros::spin();
}
