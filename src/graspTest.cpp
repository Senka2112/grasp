#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectoryRequest.h>
#include <moveit_msgs/GetMotionPlan.h>

using namespace std;

string groupPrefix = "left_arm";
ros::ServiceClient planning_client_;

bool plan(const geometry_msgs::Pose &goal,
                             moveit_msgs::MotionPlanResponse &solution,
                             const sensor_msgs::JointState &start_state) {

    if (!planning_client_.exists()) {
        ROS_ERROR_STREAM("Unable to connect to planning service - ensure that MoveIt is launched!");

        return false;
    }

    moveit_msgs::GetMotionPlanRequest get_mp_request;
    moveit_msgs::MotionPlanRequest &request = get_mp_request.motion_plan_request;

    request.group_name = groupPrefix;
    request.num_planning_attempts = 5;
    request.allowed_planning_time = 5.0;
    request.start_state.joint_state = start_state;

    ROS_INFO("Computing possible IK solutions for goal pose");

    moveit_msgs::Constraints c;

    for(int i = 0; i < 5; ++i) {
        stringstream s;
        s << "joint" << (i + 1);
        moveit_msgs::JointConstraint js;
        js.position = start_state.position.at(i);
        if(i == 0) js.position += 0.4;
        js.joint_name = s.str();
        js.tolerance_above = 1e-4;
        js.tolerance_below = 1e-4;
        js.weight = 1.0;
        c.joint_constraints.push_back(js);
    }

    request.goal_constraints.push_back(c);

   
    if(request.goal_constraints.size() == 0) {
        ROS_WARN("No valid IK solution found for given pose goal - planning failed!");
        return false;
    }

    ROS_DEBUG("Found %d valid IK solutions for given pose goal", (int)request.goal_constraints.size());
    ROS_DEBUG("Calling planning service...");

    moveit_msgs::GetMotionPlanResponse get_mp_response;

    bool success = planning_client_.call(get_mp_request, get_mp_response);
    solution = get_mp_response.motion_plan_response;
    int error_code = solution.error_code.val;

    if(success) {

        int pts_count = (int) solution.trajectory.joint_trajectory.points.size();

        if(error_code != moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Planning failed");
            cout << " " << error_code;
            return false;
        }

        ROS_INFO("Solution found for planning problem .");
        return true;

    } else {
        ROS_INFO("Planning failed");
        cout << " " << error_code;
        return false;
    }

}

bool executePlan(moveit_msgs::RobotTrajectory& trajectory, ros::ServiceClient& execution_client) {

    moveit_msgs::ExecuteKnownTrajectory msg;
    moveit_msgs::ExecuteKnownTrajectoryRequest &request = msg.request;
    request.wait_for_execution = true;
    request.trajectory = trajectory;
    bool success = execution_client.call(msg);
    if (success) {
        moveit_msgs::MoveItErrorCodes &code = msg.response.error_code;
        if (code.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
            ROS_INFO("Execution finished successfully.");
        } else {
            ROS_ERROR("Execution finished with error_code '%d'", code.val);
            return false;
        }
    } else {
        ROS_ERROR("Execution failed!");
        return false;
    }
    return true;

}


int main(int argc, char** args) {

    ros::init(argc, args, "demo"); ros::NodeHandle* node = new ros::NodeHandle(); usleep(1e6);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Retrieving joint state...");
    moveit::planning_interface::MoveGroup group(groupPrefix);
    group.setEndEffectorLink("left_hand");

    vector<double> jointVals = group.getCurrentJointValues();
   // cout<<"
    for(int i = 0; i < jointVals.size(); ++i) {
        cout << jointVals.at(i) << " ";
    }

    ROS_INFO("Connecting to planning service...");

    string topic = "plan_kinematic_path";
    planning_client_ = node->serviceClient<moveit_msgs::GetMotionPlan>(topic);

    geometry_msgs::Pose newPose;
    // 0 0 0 0 position
    newPose.position.x = 0.5180;
    newPose.position.y = 0.8464;
    newPose.position.z = -0.215;
    newPose.orientation.x = -0.3843;
    newPose.orientation.y = 0.923;
    newPose.orientation.z = 0.0035;
    newPose.orientation.w = -0.0012;

    robot_state::RobotState start_state(*group.getCurrentState());
    group.setStartState(start_state);

   
    sensor_msgs::JointState js;
    js.position = jointVals;

    moveit_msgs::MotionPlanResponse p;
    ROS_INFO("Starting to plan...");
    plan(newPose, p, js);
    moveit_msgs::RobotTrajectory trajectory = p.trajectory;

    ros::ServiceClient execution_client = node->serviceClient<moveit_msgs::ExecuteKnownTrajectory>("execute_kinematic_path");
    // cout << trajectory << endl;
    cout << "success: " << executePlan(trajectory, execution_client) << endl;

    getchar();

    return 0;

}
