#include <ros/ros.h>
#include "hrii_robothon_msgs/ProbeCircuitByod.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include <eigen_conversions/eigen_msg.h>
#include "hrii_robothon_byod/utils/ControllerUtils.h"

class ProbeCircuitBYOD
{
    public:
        ProbeCircuitBYOD(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10.0),
            default_grasping_gripper_force_(5.0),
            desired_contact_force_(1), //1 Newton in simulation, 2 N with real robot
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose"),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
        {
            activation_server_ = nh_.advertiseService("activate", &ProbeCircuitBYOD::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_left_;
        GripperInterfaceClientHelper::Ptr gripper_right_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;
        double desired_contact_force_;

        HRII::TrajectoryHelper::Ptr traj_helper_left_;
        HRII::TrajectoryHelper::Ptr traj_helper_right_;

        ros::ServiceServer activation_server_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_right_;
        ros::ServiceClient controller_set_EE_T_task_frame_client_left_;
        std::string controller_set_EE_T_task_frame_service_name_;

        ros::Publisher desired_pose_pub_;
        std::string controller_desired_pose_topic_name_;

        bool activationCallback(hrii_robothon_msgs::ProbeCircuitByod::Request& req,
                                hrii_robothon_msgs::ProbeCircuitByod::Response& res)
        {
            ROS_INFO_STREAM("Activate circuit probing for robot: " << req.left_robot_id);

            //FRANKA LEFT task frame initialization
            controller_set_EE_T_task_frame_client_left_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_left_.waitForExistence();
            //FRANKA RIGHT task frame initialization
            controller_set_EE_T_task_frame_client_right_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.right_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.left_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_right_.waitForExistence();

            // Set the new task frame for th LEFT FRANKA to probe hole in gripper fingertips
            // The task frame is rotated by 45° wrt EE orientation
            hrii_robot_msgs::SetPose set_EE_T_task_frame_srv_left;
            set_EE_T_task_frame_srv_left.request.pose_stamped.pose.position.z = -0.01;
            set_EE_T_task_frame_srv_left.request.pose_stamped.pose.orientation.y = -0.3826834; 
            set_EE_T_task_frame_srv_left.request.pose_stamped.pose.orientation.w = 0.9238795; 

            // Set the new task frame for th RIGHT FRANKA to probe hole in gripper fingertips
            // The task frame is rotated by 45° wrt EE orientation
            hrii_robot_msgs::SetPose set_EE_T_task_frame_srv_right;
            set_EE_T_task_frame_srv_right.request.pose_stamped.pose.position.z = -0.01;
            set_EE_T_task_frame_srv_right.request.pose_stamped.pose.orientation.y = 0.3826834; //The opposite sign wrt to left franka is fundamental to not crash!!
            set_EE_T_task_frame_srv_right.request.pose_stamped.pose.orientation.w = 0.9238795;

            //FRANKA LEFT
            if (!controller_set_EE_T_task_frame_client_left_.call(set_EE_T_task_frame_srv_left))
            {
                ROS_ERROR("Error calling set_EE_T_task_frame_left ROS service.");
                return false;
            }
            else if (!set_EE_T_task_frame_srv_left.response.success)
            {
                ROS_ERROR("Failure setting EE_T_task_frame_left. Exiting.");
                return false;
            }

            //FRANKA RIGHT
            if (!controller_set_EE_T_task_frame_client_right_.call(set_EE_T_task_frame_srv_right))
            {
                ROS_ERROR("Error calling set_EE_T_task_frame_right ROS service.");
                return false;
            }
            else if (!set_EE_T_task_frame_srv_right.response.success)
            {
                ROS_ERROR("Failure setting EE_T_task_frame_right. Exiting.");
                return false;
            }

            // Trajectory helper declaration and initialization FRANKA LEFT
            traj_helper_left_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.left_robot_id+"/trajectory_handler");
            if (!traj_helper_left_->init()) return false;
            traj_helper_left_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Left trajectory handler client initialized.");
            // Trajectory helper declaration and initialization FRANKA RIGHT
            traj_helper_right_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.right_robot_id+"/trajectory_handler");
            if (!traj_helper_right_->init()) return false;
            traj_helper_right_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Right trajectory handler client initialized.");

            // Initialize gripper and open it FRANKA LEFT
            gripper_left_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.left_robot_id+"/gripper");
            if (!gripper_left_->init()) return false;
            if (!gripper_left_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Left gripper client initialized.");
            // Initialize gripper and open it FRANKA RIGHT
            gripper_right_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.right_robot_id+"/gripper");
            if (!gripper_right_->init()) return false;
            if (!gripper_right_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Right gripper client initialized.");
            
            double execution_time = 5.0;

            // Store probes handle pose
            geometry_msgs::Pose red_probe_handle_pose, approach_red_probe_handle_pose, black_probe_handle_pose, approach_black_probe_handle_pose;
            red_probe_handle_pose = req.red_probe_handle_pose.pose;
            approach_red_probe_handle_pose = red_probe_handle_pose;
            black_probe_handle_pose = req.black_probe_handle_pose.pose;
            approach_black_probe_handle_pose = black_probe_handle_pose;

            //---------------------------------------------------- APPROACHING THE TWO PROBES ----------------------------------------------------
            // Move the franka LEFT above the red probe handle pose
            approach_red_probe_handle_pose.position.z += 0.15;
            if(!traj_helper_left_->moveToTargetPose(approach_red_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to reach the approach red probe pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            // Move the franka RIGHT above the black probe handle pose
            approach_black_probe_handle_pose.position.z += 0.15;
            if(!traj_helper_right_->moveToTargetPoseAndWait(approach_black_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to reach the approach black probe pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            //---------------------------------------------------- GRASPING THE TWO PROBES ----------------------------------------------------
            // Move franka LEFT to red probe handle pose
            execution_time = 4.0;
            if(!traj_helper_left_->moveToTargetPose(red_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to reach the red probe handle pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            // Move franka RIGHT to black probe handle pose
            if(!traj_helper_right_->moveToTargetPoseAndWait(black_probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.right_robot_id+" failed to reach the red probe handle pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Grasp the probes
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false; //real robot
            if (!gripper_left_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper left: grasp from outside failed..."); //simulation
            // if (!gripper_right->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false; //real robot
            if (!gripper_right_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper right: grasp from outside failed..."); //simulation
            
            // Extract the probes
            //Left
            Eigen::Affine3d robot_link0_T_red_probe_handle;
            tf::poseMsgToEigen(red_probe_handle_pose, robot_link0_T_red_probe_handle);
            Eigen::Affine3d red_probe_handle_T_probe_extraction_pose = Eigen::Affine3d::Identity();
            red_probe_handle_T_probe_extraction_pose.translation() << 0, 0.0, -0.06;
            geometry_msgs::Pose desired_red_pose_msg;
            tf::poseEigenToMsg(robot_link0_T_red_probe_handle * red_probe_handle_T_probe_extraction_pose, desired_red_pose_msg);
            execution_time = 5.0;

            //Right
            Eigen::Affine3d robot_link0_T_black_probe_handle;
            tf::poseMsgToEigen(black_probe_handle_pose, robot_link0_T_black_probe_handle);
            Eigen::Affine3d black_probe_handle_T_probe_extraction_pose = Eigen::Affine3d::Identity();
            black_probe_handle_T_probe_extraction_pose.translation() << 0, 0.0, -0.06;
            geometry_msgs::Pose desired_black_pose_msg;
            tf::poseEigenToMsg(robot_link0_T_black_probe_handle * black_probe_handle_T_probe_extraction_pose, desired_black_pose_msg);
            execution_time = 5.0;

            if(!traj_helper_left_->moveToTargetPose(desired_red_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to extract the red probe handle.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            if(!traj_helper_right_->moveToTargetPoseAndWait(desired_black_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.right_robot_id+" failed to extract the red probe handle.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            //---------------------------------------------------- PROBING THE CIRCUITS ----------------------------------------------------
            // Approaching franka left circuit to probe pose
            desired_red_pose_msg = req.red_circuit_pose.pose;
            desired_red_pose_msg.position.z += 0.1;
            execution_time = 5.0;
            if(!traj_helper_left_->moveToTargetPose(desired_red_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.left_robot_id+" failed to approach the left circuit pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            // Approaching franka right circuit to probe pose
            desired_black_pose_msg = req.black_circuit_pose.pose;
            desired_black_pose_msg.position.z += 0.1;
            execution_time = 5.0;
            if(!traj_helper_right_->moveToTargetPoseAndWait(desired_black_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.right_robot_id+" failed to approach the right circuit pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            else
            {
                ROS_INFO_STREAM("Right circuit approached");
            }

            // Switch to task force in Z-axis
            geometry_msgs::WrenchStamped desired_wrench;
            desired_wrench.header.stamp = ros::Time::now();
            desired_wrench.wrench.force.z = desired_contact_force_;
            // desired_wrench.wrench.force.z = 0.0;

            //Franka left probing
            if (!applyContactForce(nh_, req.left_robot_id,
                            "cart_hybrid_motion_force_controller",
                            hrii_robot_msgs::TaskSelection::Request::Z_LIN,
                            desired_wrench))
            {
                ROS_ERROR_STREAM("Contact force application failed.");
                return false;
            }

            //Franka right probing
            if (!applyContactForce(nh_, req.right_robot_id,
                            "cart_hybrid_motion_force_controller",
                            hrii_robot_msgs::TaskSelection::Request::Z_LIN,
                            desired_wrench))
            {
                ROS_ERROR_STREAM("Contact force application failed.");
                return false;
            }
            else
            {
                ROS_INFO_STREAM("Right contact forces applied");
            }

            ROS_INFO("Circuit probed.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // ProbeCircuitBYOD

int main(int argc, char **argv)
{
    ros::init(argc, argv, "probe_circuit_byod");
    ros::NodeHandle nh("~");

    ProbeCircuitBYOD byod(nh);

    ros::spin();

    return 0;
}
