#include <vector>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gripper_action_controller/gripper_action_controller.h>
#include <geometry_msgs/Pose.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <std_msgs/Float64.h>
#include "baxterGripper.h"

using namespace std;

// tracking object
bool firstSet = false;
void v4rCallback(geometry_msgs::Transform msg);
geometry_msgs::Pose objectPose;

int main(int argc, char** args) {

    ros::init(argc, args, "move_Baxter"); ros::NodeHandle* node_handle = new ros::NodeHandle(); usleep(1e6);
    BaxterGripper gripper("left");

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate lrate(20.0);

    sleep(5.0);

    //object tracking interface
    //uncomment next line to get object pose
    //first you have to start object tracking as in vision exercise
    //ros::Subscriber v4rSubscriber = node_handle->subscribe("object_tracker/object_pose", 1, v4rCallback);


    //    //move_it interface
    moveit::planning_interface::MoveGroup group("left_arm");

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    ros::Publisher display_publisher = node_handle->advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::Pose newPose;
    newPose.position.x = 0.5180;
    newPose.position.y = 0.8464;
    newPose.position.z = -0.215;
    newPose.orientation.x = -0.3843;
    newPose.orientation.y = 0.923;
    newPose.orientation.z = 0.0035;
    newPose.orientation.w = -0.0012;
    group.setPoseTarget(newPose);

    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    if (1)
    {
        ROS_INFO("Visualizing plan 1 (again)");
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        /* Sleep to give Rviz time to visualize the plan. */
        sleep(5.0);
    }

    //open gripper
    gripper.open();


    //prove the plan in rviz before executing
    cout<<"Approve path was OK (type y or n) "<<endl;
    char approved = 0;
    cin >> approved;
    if(approved == 'y') group.move();

    //give time robot to move
    sleep (5.0);

    //prove the plan in rviz before executing
    cout<<"Approve gripper closes.(type y or n) "<<endl;
    approved = 0;
    cin >> approved;
    if(approved == 'y') gripper.close();

    return 0;

}

void v4rCallback(geometry_msgs::Transform msg) {

    firstSet = true;
    objectPose.position.x = msg.translation.x;
    objectPose.position.y = msg.translation.y;
    objectPose.position.z = msg.translation.z;
    objectPose.orientation.x = msg.rotation.x;
    objectPose.orientation.y = msg.rotation.y;
    objectPose.orientation.z = msg.rotation.z;
    objectPose.orientation.w = msg.rotation.w;

}
