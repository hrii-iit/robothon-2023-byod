#include <ros/ros.h>
#include "hrii_robothon_msgs/PowerOnVoltmeter.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class VoltmeterPowerOnBYOD
{
    public:
        VoltmeterPowerOnBYOD(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_force_(0.2),
            default_closing_gripper_speed_(0.02)
        {
            activation_server_ = nh_.advertiseService("activate", &VoltmeterPowerOnBYOD::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_closing_gripper_force_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_robothon_msgs::PowerOnVoltmeter::Request& req,
                                hrii_robothon_msgs::PowerOnVoltmeter::Response& res)
        {
            ROS_INFO_STREAM("Activate voltmeter power on interface for robot: " << req.robot_id);

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

            ROS_INFO("VOLTMETER POWER ON BYOD STARTED!");
            geometry_msgs::Pose power_on_pose, power_on_pose_V, approach_pose;
            tf2::Quaternion q_orig, q_rot, q_new;

            power_on_pose = req.power_on_pose.pose;
            // Get the original orientation of 'power_on_pose' 
            tf2::convert(power_on_pose.orientation, q_orig);
            double r=0, p=0, y=0.7853982;  // Rotate the previous pose by 45 degrees about Z to align with power on button direction
            q_rot.setRPY(r, p, y);
            q_new = q_rot*q_orig;  // Calculate the new orientation
            q_new.normalize();
            // Stuff the new rotation back into the pose. This requires conversion into a msg type
            tf2::convert(q_new, power_on_pose.orientation);

            // Move to an approach pose upon the power on circle 
            approach_pose = power_on_pose;
            approach_pose.position.z += 0.02;
            waypoints.push_back(approach_pose);
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the approaching power on pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            //Open the gripper 
            if (!gripper_->open(opening_gri_speed)) return false;

            // Move to the power on pose
            power_on_pose.position.z += 0.005; 
            waypoints.push_back(power_on_pose);
            execution_time = 1.5;

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the door of the battery.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Closing the gripper to grab the power on button with the default force 
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) return false; //real robot
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed..."); //simulation

            //Calculation of the pose to power on the voltmeter --> rotation of 22.5 degrees wrt to the previous pose
            power_on_pose_V = power_on_pose;
            // Get the original orientation of 'power_on_pose'
            tf2::convert(power_on_pose.orientation, q_orig);
            r=0, p=0, y=-0.3926991;  // Rotate the previous pose by 22.5 degrees about Z
            q_rot.setRPY(r, p, y);
            q_new = q_rot*q_orig;  // Calculate the new orientation
            q_new.normalize();
            // Stuff the new rotation back into the pose. This requires conversion into a msg type
            tf2::convert(q_new, power_on_pose_V.orientation);

            waypoints.push_back(power_on_pose_V);
            execution_time = 1.5;
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the door of the battery.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());
            
            //Open the gripper to release the button
            if (!gripper_->open(opening_gri_speed)) return false;

            ROS_INFO("Power on voltmeter task accomplished.");

            res.success = true;
            res.message = "";
            return true;
        }

}; // VoltmeterPowerOnBYOD

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voltmeter_power_on_byod");
    ros::NodeHandle nh("~");

    VoltmeterPowerOnBYOD byod(nh);

    ros::spin();

    return 0;
}
