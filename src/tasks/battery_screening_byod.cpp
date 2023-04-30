#include <ros/ros.h>
#include "hrii_robothon_msgs/BatteryScreening.h"
#include "hrii_robothon_msgs/ScreenVoltage.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include "hrii_robothon_byod/utils/ControllerUtils.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class BatteryScreeningBYOD
{
    public:
        BatteryScreeningBYOD(ros::NodeHandle& nh) : 
            nh_(nh),default_closing_gripper_speed_(10.0),
            default_grasping_gripper_force_(5.0),
            desired_contact_force_(2.0),
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose"),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
        {
            activation_server_ = nh_.advertiseService("activate", &BatteryScreeningBYOD::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");

            read_voltage_service_name_ = "/robothon/read_voltage";
            read_voltage_client_ = nh_.serviceClient<hrii_robothon_msgs::ScreenVoltage>(read_voltage_service_name_);
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_left_;
        GripperInterfaceClientHelper::Ptr gripper_right_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;
        double desired_contact_force_;

        std::string read_voltage_service_name_;
        ros::ServiceClient read_voltage_client_;

        HRII::TrajectoryHelper::Ptr traj_helper_left_;
        HRII::TrajectoryHelper::Ptr traj_helper_right_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_right_;
        ros::ServiceClient controller_set_EE_T_task_frame_client_left_;
        std::string controller_set_EE_T_task_frame_service_name_;

        std::string controller_desired_pose_topic_name_;

        ros::ServiceServer activation_server_;

        bool activationCallback(hrii_robothon_msgs::BatteryScreening::Request& req,
                                hrii_robothon_msgs::BatteryScreening::Response& res)
        {
            ROS_INFO_STREAM("Activate read voltage interface for robot: " << req.left_robot_id);
            //Services for setting the end effector task frame 
            //FRANKA LEFT
            controller_set_EE_T_task_frame_client_left_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_left_.waitForExistence();
            //FRANKA RIGHT
            controller_set_EE_T_task_frame_client_right_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.right_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_right_.waitForExistence();

            // Trajectory helper declaration and initialization LEFT
            traj_helper_left_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.left_robot_id+"/trajectory_handler");
            if (!traj_helper_left_->init()) return false;
            traj_helper_left_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Left trajectory handler client initialized.");
            // Trajectory helper declaration and initialization RIGHT
            traj_helper_right_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.right_robot_id+"/trajectory_handler");
            if (!traj_helper_right_->init()) return false;
            traj_helper_right_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Right trajectory handler client initialized.");

            // Initialize LEFT gripper 
            gripper_left_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.left_robot_id+"/gripper");
            ROS_INFO("Left gripper client initialized.");
            // Initialize RIGHT gripper
            gripper_right_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.right_robot_id+"/gripper");
            ROS_INFO("Right gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 3.0;

            ROS_INFO("SCREENING OF BATTERY STARTED!");
            // Calling read voltage service
            // hrii_robothon_msgs::ScreenVoltage screen_voltage_srv;
            // ROS_INFO_STREAM("Waiting for " << nh_.resolveName(read_voltage_service_name_) << " ROS service...");
            // read_voltage_client_.waitForExistence();

            // if (!read_voltage_client_.call(screen_voltage_srv))
            // {
            //     ROS_ERROR("Error calling read screen service.");
            //     return false;
            // }
            // float voltage = screen_voltage_srv.response.voltage;
            // ROS_INFO_STREAM("Received voltage from " << nh_.resolveName(read_voltage_service_name_) << " ROS service is" << voltage << "Voltage");
            float voltage = 3.0; //to simulate the perception message 

            ros::Duration(5.0).sleep();
            
            // Store pose of probe handle, battery and boxes for batteries
            geometry_msgs::Pose approach_red_probe_handle_pose, red_probe_handle_pose, approach_black_probe_handle_pose, 
                                        black_probe_handle_pose;

            red_probe_handle_pose = req.red_probe_handle_pose.pose;
            approach_red_probe_handle_pose = red_probe_handle_pose;
            black_probe_handle_pose = req.black_probe_handle_pose.pose;
            approach_black_probe_handle_pose = black_probe_handle_pose; 

            // Move the franka LEFT back to the RED probe approach handle pose
            approach_red_probe_handle_pose.position.z += 0.15;
            if(!traj_helper_left_->moveToTargetPose(approach_red_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to come back to approach red probe pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            // Move the franka RIGHT back to the BLACK probe approach handle pose
            approach_black_probe_handle_pose.position.z += 0.15;
            if(!traj_helper_right_->moveToTargetPoseAndWait(approach_black_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.right_robot_id+" failed to come back to approach black probe pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move franka LEFT down to RED probe handle pose
            execution_time = 4.0;
            if(!traj_helper_left_->moveToTargetPose(red_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to come back to red probe handle pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move franka RIGHT back to BLACK probe handle pose
            if(!traj_helper_right_->moveToTargetPoseAndWait(black_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.right_robot_id+" failed to come back to black probe handle pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            ROS_INFO("Screening of the battery succeded.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // BatteryScreeningBYOD

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_screening_byod");
    ros::NodeHandle nh("~");

    BatteryScreeningBYOD byod(nh);

    ros::spin();

    return 0;
}

