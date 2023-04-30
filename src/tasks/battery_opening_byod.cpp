#include <ros/ros.h>
#include "hrii_robothon_msgs/OpenBattery.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

class BatteryOpeningBYOD
{
    public:
        BatteryOpeningBYOD(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_force_(0.2),
            default_closing_gripper_speed_(0.02)
        {
            activation_server_ = nh_.advertiseService("activate", &BatteryOpeningBYOD::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_closing_gripper_force_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_robothon_msgs::OpenBattery::Request& req,
                                hrii_robothon_msgs::OpenBattery::Response& res)
        {
            ROS_INFO_STREAM("Activate open battery interface for robot: " << req.robot_id);

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize gripper and open it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            double opening_gri_speed = 1.0;
            if (!gripper_->open(opening_gri_speed)) return false;
            ROS_INFO("Gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 3.0;

            ROS_INFO("OPEN BATTERY BYOD STARTED!");
            geometry_msgs::Pose battery_pose, approach_pose, release_pose, release_approaching_pose;

            // Move to an approach pose upon the battery 
            battery_pose = req.battery_pose.pose;
            approach_pose = battery_pose;
            approach_pose.position.z += 0.02;
            //Compute the pose to release the cover of the battery
            release_pose = battery_pose;
            Eigen::Quaternion<double> Q(battery_pose.orientation.w, battery_pose.orientation.x, battery_pose.orientation.y, battery_pose.orientation.z);
            Eigen::Matrix3d displacement_transformation_rot_matrix = Q.toRotationMatrix();
            Eigen::Vector3d displacement_vector(-0.08, 0, 0);
            // Displacement vector in robot_base RF
            displacement_vector =  displacement_transformation_rot_matrix * displacement_vector;

            // Pose of the point where we have to move the cover of the battery
            release_pose.position.x += displacement_vector(0);      //we add the computed displacement along each axes wrt robot RF
            release_pose.position.y += displacement_vector(1);
            release_pose.position.z += displacement_vector(2);          
            release_approaching_pose = release_pose;
            release_approaching_pose.position.z += 0.02; 
            
            waypoints.push_back(approach_pose);
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the approaching battery pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            //Open the gripper
            if (!gripper_->open(opening_gri_speed)) return false;

            // Move to the cover battery pose
            battery_pose.position.z += 0.003;
            waypoints.push_back(battery_pose);
            execution_time = 2;

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the cover of the battery.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Closing the gripper to grab the cover of the battery
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) return false;  //with real robot
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed..."); //for simulation

            //Move upon the release approaching pose 
            waypoints.push_back(release_approaching_pose);
            execution_time = 1.5;
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the cover of the battery.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            //Move down to the release pose 
            waypoints.push_back(release_pose);
            execution_time = 1.5;
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the door of the battery.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());
            
            //Open the gripper to release the cover of the battery 
            if (!gripper_->open(opening_gri_speed)) return false;

            ROS_INFO("Open battery task accomplished.");

            res.success = true;
            res.message = "";
            return true;
        }

}; // BatteryOpeningBYOD

int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_opening_byod");
    ros::NodeHandle nh("~");

    BatteryOpeningBYOD byod(nh);

    ros::spin();

    return 0;
}
