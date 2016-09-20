#include "ros/ros.h"
#include "std_msgs/String.h"
#include <baxter_gripper_manager/GripperCommand.h>

#include <baxter_core_msgs/EndEffectorCommand.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ontoreef_msgs/PerceptionExchange.h>

baxter_core_msgs::EndEffectorCommand comm_right;
baxter_core_msgs::EndEffectorCommand comm_left;

ros::Publisher* pub_right_ptr;
ros::Publisher* pub_left_ptr;

void gripperAction(const baxter_gripper_manager::GripperCommand::ConstPtr& msg)
{
    if (msg->arm == "right"){

        if (msg->command == "close") {
            comm_right.command = "grip";
            pub_right_ptr->publish(comm_right);
            return; }

        if (msg->command == "open") {
            comm_right.command = "release";
            pub_right_ptr->publish(comm_right);
            return;}
    }

    if (msg->arm == "left") {

        if (msg->command == "close") {
            comm_left.command = "grip";
            pub_left_ptr->publish(comm_left);
            return; }

        else if (msg->command == "open") {
            comm_left.command = "release";
            pub_left_ptr->publish(comm_left);
            return; }
    }

    if (msg->arm == "both") {

        if (msg->command == "close") {
            comm_left.command = "grip";
            comm_right.command = "grip";
            pub_left_ptr->publish(comm_left);
            pub_right_ptr->publish(comm_right);
            return; }

        else if (msg->command == "open") {
            comm_left.command = "release";
            comm_right.command = "release";
            pub_left_ptr->publish(comm_left);
            pub_right_ptr->publish(comm_right);
            return; }
    }
        
    ROS_ERROR("Invalid command. Valid commands are:\ncommand: open/close\narm: left/right");
    return;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "baxter_gripper_control");
  ros::NodeHandle n;

  ros::Publisher pub_right = n.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/right_gripper/command", 1);
  ros::Publisher pub_left = n.advertise<baxter_core_msgs::EndEffectorCommand>("robot/end_effector/left_gripper/command", 1);

  pub_right_ptr = &pub_right;
  pub_left_ptr = &pub_left;

  ros::Subscriber sub = n.subscribe("baxter_gripper_control", 100, gripperAction);

  // Initialize calibration command

  comm_right.id = 65538;
  comm_right.command = "calibrate";
  comm_right.sender = "move_robot";
  comm_right.sequence = 0;

  comm_left.id = 65538;
  comm_left.command = "calibrate";
  comm_left.sender = "move_robot";
  comm_left.sequence = 0;

  ROS_INFO("Ready to control grippers.");

  ros::spin();

  return 0;
}
