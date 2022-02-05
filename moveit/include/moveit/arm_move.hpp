#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <math.h>


class Arm_Move
{
public:
    Arm_Move();
    void arm_register(std::string);
    void hand_register(std::string);
    void show_arm_joint();
    void show_hand_joint();
    void hand_open();
    void hand_close();
    static tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
    {
        tf2::Quaternion q_after, q_final;
        q_after = q_moto * q_ori * q_moto.inverse();
        tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
        q_final.setRotation(vec, angle);
        return q_final;
    }
    void set_close_range(double);
    void tf_get(std::string, std::string, geometry_msgs::TransformStamped&);
    void show_tf_value(std::string, std::string);
    void move_end_effector(double, double, double, double);
    void move_end_effector_set_tf(double, double, double, double, double, double, double);
    void move_end_effector_set_tf(double, double, double, tf2::Quaternion, double);
    void move_end_effector_set_tf(geometry_msgs::TransformStamped, double);
    void move_end_effector_set_tf(geometry_msgs::TransformStamped, double, double, double);
    void move_end_effector_set_tf(geometry_msgs::Transform, double);
    void move_place();
    void return_home();
    void Move_Down();
    geometry_msgs::Point get_pose_tf(std::string, std::string);
    geometry_msgs::TransformStamped get_pose_tf(std::string, std::string, double);
    geometry_msgs::Point transform_to_target_point(geometry_msgs::TransformStamped);
    geometry_msgs::Transform transform_to_target_point(geometry_msgs::Transform);
    
private:
    moveit::planning_interface::MoveGroupInterface *arm_group_;
    moveit::planning_interface::MoveGroupInterface *hand_group_;
    ros::AsyncSpinner spinner;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::Buffer tfBuffer_;
    geometry_msgs::TransformStamped transform_;
    geometry_msgs::Point point_;
    double dulation_;
    double close_range_;
    template <typename T>
    void show_value(std::vector<T> value){
        for (int i = 0; i < value.size(); i++) {
            std::cout << value[i] << " ";
        }
        std::cout << std::endl;
    }
};