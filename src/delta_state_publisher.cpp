#include "tf2/LinearMath/Vector3.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>

void set_transform(
    geometry_msgs::TransformStamped &transform_stamped,
    urdf::JointConstSharedPtr joint,
    double angle)
{
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = joint->parent_link_name;
    transform_stamped.child_frame_id = joint->child_link_name;

    static urdf::Vector3 pos;
    static urdf::Rotation rot;
    static tf2::Vector3 axis;
    static tf2::Quaternion quat1, quat2, quat3;
    pos = joint->parent_to_joint_origin_transform.position;
    rot = joint->parent_to_joint_origin_transform.rotation;
    quat1 = tf2::Quaternion(rot.x, rot.y, rot.z, rot.w);
    axis = tf2::Vector3(joint->axis.x, joint->axis.y, joint->axis.z);
    quat2.setRotation(axis, angle);
    quat3 = quat2 * quat1;

    transform_stamped.transform.translation.x = pos.x;
    transform_stamped.transform.translation.y = pos.y;
    transform_stamped.transform.translation.z = pos.z;
    transform_stamped.transform.rotation.x = quat3.x();
    transform_stamped.transform.rotation.y = quat3.y();
    transform_stamped.transform.rotation.z = quat3.z();
    transform_stamped.transform.rotation.w = quat3.w();
}

void set_transform(
    geometry_msgs::TransformStamped &transform_stamped,
    const std::string &parent_id,
    const std::string &child_id,
    const urdf::Vector3 &translation,
    const urdf::Rotation rotation)
{
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = parent_id;
    transform_stamped.child_frame_id = child_id;
    transform_stamped.transform.translation.x = translation.x;
    transform_stamped.transform.translation.y = translation.y;
    transform_stamped.transform.translation.z = translation.z;
    transform_stamped.transform.rotation.x = rotation.x;
    transform_stamped.transform.rotation.y = rotation.y;
    transform_stamped.transform.rotation.z = rotation.z;
    transform_stamped.transform.rotation.w = rotation.w;
}

void set_transform(
    geometry_msgs::TransformStamped &transform_stamped,
    const std::string &parent_id,
    const std::string &child_id,
    const tf2::Vector3 &translation,
    const tf2::Quaternion rotation)
{
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = parent_id;
    transform_stamped.child_frame_id = child_id;
    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();
    transform_stamped.transform.rotation.w = rotation.w();
}

void describe_model(urdf::LinkConstSharedPtr link, int depth=0)
{
    urdf::Vector3 joint_position;
    urdf::Rotation joint_rotation;

    ROS_INFO("[%d] Link: %s", depth, link->name.c_str());
    for (size_t i = 0; i < link->child_joints.size(); i++) {
        ROS_INFO(
            "- %lu: joint=%s, link=%s",
            i,
            link->child_joints[i]->name.c_str(),
            link->child_links[i]->name.c_str()
        );
        ROS_INFO(
            "-    axis=(%f, %f, %f)",
            link->child_joints[i]->axis.x,
            link->child_joints[i]->axis.y,
            link->child_joints[i]->axis.z
        );

        joint_position = link->child_joints[i]->
            parent_to_joint_origin_transform.position;
        joint_rotation = link->child_joints[i]->
            parent_to_joint_origin_transform.rotation;

        ROS_INFO("-    joint_pos=(%f, %f, %f)",
            joint_position.x,
            joint_position.y,
            joint_position.z
        );
        ROS_INFO("-    joint_rot=(%f, %f, %f, %f)",
            joint_rotation.x,
            joint_rotation.y,
            joint_rotation.z,
            joint_rotation.w
        );
    }
    for (size_t i = 0; i < link->child_joints.size(); i++) {
        describe_model(link->child_links[i], depth+1);
    }
}

class ForwardKinematicsNode {
public:
    ForwardKinematicsNode()
    {
        if (!model.initParam("robot_description")) {
            ROS_ERROR("Failed to parse urdf file");
        } else {
            ROS_INFO("Successfully parsed urdf file");
        }


        angles_sub = n.subscribe(
            "/delta_angles", 1,
            &ForwardKinematicsNode::angles_callback, this
        );
    }

    void angles_callback(const sensor_msgs::JointState &msg)
    {
        // Set base transform
        set_transform(
            transform_stamped,
            "world",
            "base",
            tf2::Vector3(0, 0, 2),
            tf2::Quaternion(0, 0, 0, 1)
        );
        tf_broadcaster.sendTransform(transform_stamped);

        // for (size_t i = 0; i < msg.name.size(); i++) {
        //     set_transform(
        //         transform_stamped,
        //         model.getJoint(msg.name[i]),
        //         msg.position[i]
        //     );
        //     tf_broadcaster.sendTransform(transform_stamped);
        // }

        // set_transform(
        //     transform_stamped,
        //     "base",
        //     "end_effector",
        //     tf2::Vector3(0, 0, -2),
        //     tf2::Quaternion(0, 0, 0, 1)
        // );
        // tf_broadcaster.sendTransform(transform_stamped);
    }

private:
    ros::NodeHandle n;
    ros::Subscriber angles_sub;
    urdf::Model model;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    geometry_msgs::TransformStamped transform_stamped;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delta_state_broadcaster");
    ROS_INFO("Spinning until killed, publishing delta transforms to world");
    ForwardKinematicsNode node;
    ros::spin();
    return 0;
}
