#include "ros/ros.h"
#include "hrii_robothon_msgs/Homing.h"
#include "hrii_robothon_msgs/OpenBattery.h"
#include "hrii_robothon_msgs/PowerOnVoltmeter.h"
#include "hrii_robothon_msgs/ProbeCircuitByod.h"
#include "hrii_robothon_msgs/BatteryScreening.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "hrii_robothon_byod/utils/ControllerUtils.h"
#include "hrii_robothon_byod/utils/GeometryMsgs.h"

class MainBYOD
{
    public:
        MainBYOD() : nh_priv_("~"), tf_listener_{tf_buffer_}
        {
            homing_service_name_ = "homing_byod/activate";
            homing_client_ = nh_.serviceClient<hrii_robothon_msgs::Homing>(homing_service_name_);

            battery_opening_activation_service_name_ = "battery_opening_byod/activate";
            battery_opening_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::OpenBattery>(battery_opening_activation_service_name_);

            voltmeter_power_on_activation_service_name_ = "power_on_voltmeter_byod/activate";
            voltmeter_power_on_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::PowerOnVoltmeter>(voltmeter_power_on_activation_service_name_);

            probe_circuit_activation_service_name_ = "probe_circuit_byod/activate";
            probe_circuit_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::ProbeCircuitByod>(probe_circuit_activation_service_name_);

            battery_screening_activation_service_name_ = "battery_screening_byod/activate";
            battery_screening_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::BatteryScreening>(battery_screening_activation_service_name_);

        }

        bool init()
        {
            if(!nh_priv_.getParam("left_robot_id", left_robot_id_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("left_robot_id") <<" ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("right_robot_id", right_robot_id_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("right_robot_id") << " ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("task_order", task_order_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("task_order") << " ROS param found");
                return false;
            }
            for (int cnt = 0; cnt < task_order_.size(); cnt++)
            {
                ROS_INFO_STREAM(cnt << ": " << task_order_[cnt]);
            }

            if(!nh_priv_.getParam("left_robot_launch", left_robot_launch_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("left_robot_launch") << " ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("right_robot_launch", right_robot_launch_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("right_robot_launch") << " ROS param found");
                return false;
            }
            
            if(left_robot_launch_)
            {
                left_robot_controller_manager_status_service_name_ = "/" + left_robot_id_ + "/controller_manager/list_controllers";
                if (!waitForRunningController(nh_, 
                                            left_robot_controller_manager_status_service_name_,
                                            "cart_hybrid_motion_force_controller")) return false;
                ROS_INFO("Left robot controller running.");
            }else
                ROS_INFO("Left robot controller not activated.");

            if(right_robot_launch_)
            {
                right_robot_controller_manager_status_service_name_ = "/" + right_robot_id_ + "/controller_manager/list_controllers";
                if (!waitForRunningController(nh_, 
                                            right_robot_controller_manager_status_service_name_,
                                            "cart_hybrid_motion_force_controller")) return false;
                ROS_INFO("Right robot controller running.");
            }else
                ROS_INFO("Right robot controller not activated.");

            
            return true;
        }

        void spin()
        {
            task_cnt_ = 0;
            state_ = resolveStateString(task_order_[task_cnt_]);
            while (ros::ok() &&
                   task_cnt_ < task_order_.size() &&
                   state_ != MainBYOD::States::EXIT &&
                   state_ != MainBYOD::States::ERROR)
            {
                switch (state_)
                {
                case MainBYOD::States::HOMING:
                {
                    // Move to homing pose and wait for start
                    ROS_INFO("- - - HOMING STATE - - -");
                    if (!homing())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }

                case MainBYOD::States::OTHER_HOMING:
                {
                    // Move to other pose and wait for start
                    ROS_INFO("- - - OTHER HOMING STATE - - -");
                    if (!otherHoming())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }

                case MainBYOD::States::BATTERY_OPENING:
                {
                    // Battery opening 
                    ROS_INFO("- - - BETTERY OPENING STATE - - -");
                    if (!batteryOpening())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }
                    
                case MainBYOD::States::VOLTMETER_POWER_ON:
                {
                    // Board detection
                    ROS_INFO("- - - VOLTMETER POWER ON STATE - - -");
                    if (!voltmeterPowerOn())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }
                    
                case MainBYOD::States::PROBE:
                {
                    ROS_INFO("- - - PROBE STATE - - -");
                    if (!probeCircuit())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }

                case MainBYOD::States::BATTERY_SCREENING:
                {
                    ROS_INFO("- - - BATTERY SCREENING STATE - - -");
                    if (!batteryScreening())
                        state_ = MainBYOD::States::ERROR;
                    break;
                }

                default:
                {
                    state_ = MainBYOD::States::ERROR;
                    ROS_INFO("State not implemented. Error.");
                    break;
                }
                }
                if (state_ != MainBYOD::States::EXIT && state_ != MainBYOD::States::ERROR)
                {
                    task_cnt_++;
                    state_ = resolveStateString(task_order_[task_cnt_]);
                }
            }

            if (state_ == MainBYOD::States::EXIT)
            {
                ROS_INFO("Tasks completed. Congratulations!");
            }
            else if (state_ == MainBYOD::States::ERROR)
            {
                ROS_ERROR("Main FSM finished with error.");
            }
        }
    
    private:
        // ROS attributes
        ros::NodeHandle nh_, nh_priv_;

        std::vector<std::string> task_order_;
        int task_cnt_;
        
        std::string left_robot_controller_manager_status_service_name_;
        ros::ServiceClient left_robot_controller_manager_status_client_;

        std::string right_robot_controller_manager_status_service_name_;
        ros::ServiceClient right_robot_controller_manager_status_client_;

        std::string homing_service_name_;
        ros::ServiceClient homing_client_;

        std::string battery_opening_activation_service_name_;
        ros::ServiceClient battery_opening_activation_client_;

        std::string voltmeter_power_on_activation_service_name_;
        ros::ServiceClient voltmeter_power_on_activation_client_;

        std::string probe_circuit_activation_service_name_;
        ros::ServiceClient probe_circuit_activation_client_;

        std::string battery_screening_activation_service_name_;
        ros::ServiceClient battery_screening_activation_client_;
        
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Robots attributs
        std::string left_robot_id_, right_robot_id_;
        bool left_robot_launch_, right_robot_launch_;


        // FSM states declaration
        enum class States {HOMING, 
                            OTHER_HOMING,
                            BATTERY_OPENING,
                            VOLTMETER_POWER_ON, 
                            PROBE,
                            BATTERY_SCREENING,
                            EXIT,
                            ERROR} state_;

        bool homing()
        {
            ROS_INFO("Go to home position for left franka...");

            // Wait for task services activation
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(homing_service_name_) << " ROS service...");
            homing_client_.waitForExistence();

            hrii_robothon_msgs::Homing homing_srv;
            homing_srv.request.robot_id = left_robot_id_;

            // Home pose
            geometry_msgs::Pose home_pose;
            home_pose.position.x = 0.351;
            home_pose.position.y = -0.233;
            home_pose.position.z = 0.441;
            home_pose.orientation.x = -0.693;
            home_pose.orientation.y = 0.706;
            home_pose.orientation.z = -0.104;
            home_pose.orientation.w = -0.104;
            homing_srv.request.home_pose.pose = home_pose;

            if (!homing_client_.call(homing_srv))
            {
                ROS_ERROR("Error calling homing service.");
                return false;
            }
            else if (!homing_srv.response.success)
            {
                ROS_ERROR("Failure going home. Exiting.");
                return false;
            }
            ROS_INFO("Franka left homing succeded.");
            return true;
        }

        bool otherHoming()
        {
            ROS_INFO("Go to home position for right franka...");

            // Wait for task services activation
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(homing_service_name_) << " ROS service...");
            homing_client_.waitForExistence();

            geometry_msgs::Pose other_homing_pose;

            // Define homing pose
            other_homing_pose.position.x = 0.113;
            other_homing_pose.position.y = -0.284;
            other_homing_pose.position.z = 0.488;
            other_homing_pose.orientation.x = 1.000;
            other_homing_pose.orientation.y = 0.000;
            other_homing_pose.orientation.z = 0.000;
            other_homing_pose.orientation.w = 0.000;

            hrii_robothon_msgs::Homing other_homing_srv;
            other_homing_srv.request.robot_id = right_robot_id_;
            other_homing_srv.request.home_pose.pose = other_homing_pose;

            if (!homing_client_.call(other_homing_srv))
            {
                ROS_ERROR("Error calling homing service.");
                return false;
            }
            else if (!other_homing_srv.response.success)
            {
                ROS_ERROR("Failure going home. Exiting.");
                return false;
            }
            ROS_INFO("Franka right homing succeded.");
            return true;
        }

        bool batteryOpening()
        {
            ROS_INFO("Opening of the battery...");

            hrii_robothon_msgs::OpenBattery opening_battery_srv;
            opening_battery_srv.request.robot_id = left_robot_id_;
            
            // Battery pose wrt franka left arm 
            geometry_msgs::TransformStamped batteryTransform;
            try{
                batteryTransform = tf_buffer_.lookupTransform(opening_battery_srv.request.robot_id+"_link0", "voltmeter_circuit_battery_box_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << opening_battery_srv.request.robot_id << "_link0 and voltmeter_circuit_battery_box_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR_STREAM("Tranform btw " << opening_battery_srv.request.robot_id << "_link0 and voltmeter_circuit_battery_box_link found!");
                return false;
            }

            geometry_msgs::Pose battery_pose;
            battery_pose.orientation = batteryTransform.transform.rotation;
            battery_pose.position.x = batteryTransform.transform.translation.x;
            battery_pose.position.y = batteryTransform.transform.translation.y;
            battery_pose.position.z = batteryTransform.transform.translation.z;
            opening_battery_srv.request.battery_pose.pose = battery_pose;

            if (!battery_opening_activation_client_.call(opening_battery_srv))
            {
                ROS_ERROR("Error calling opening battery activation service.");
                return false;
            }
            else if (!opening_battery_srv.response.success)
            {
                ROS_ERROR("Failure opneing the battery. Exiting.");
                return false;
            }
            ROS_INFO("BATTERY OPENED!.");

            return true;
            
        }

        bool voltmeterPowerOn()
        {
            ROS_INFO("Power on the voltmeter...");

            hrii_robothon_msgs::PowerOnVoltmeter power_on_srv;
            power_on_srv.request.robot_id = left_robot_id_;
            
            // Power on button pose wrt franka left  
            geometry_msgs::TransformStamped powerOnTransform;
            try{
                powerOnTransform = tf_buffer_.lookupTransform(power_on_srv.request.robot_id+"_link0", "voltmeter_circuit_power_on_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << power_on_srv.request.robot_id << "_link0 and voltmeter_circuit_power_on_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR_STREAM("Tranform btw " << power_on_srv.request.robot_id << "_link0 and voltmeter_circuit_power_on_link found!");
                return false;
            }

            geometry_msgs::Pose power_on_pose;
            power_on_pose.orientation = powerOnTransform.transform.rotation;
            power_on_pose.position.x = powerOnTransform.transform.translation.x;
            power_on_pose.position.y = powerOnTransform.transform.translation.y;
            power_on_pose.position.z = powerOnTransform.transform.translation.z;
            power_on_srv.request.power_on_pose.pose = power_on_pose;

            if (!voltmeter_power_on_activation_client_.call(power_on_srv))
            {
                ROS_ERROR("Error calling power on voltmeter activation service.");
                return false;
            }
            else if (!power_on_srv.response.success)
            {
                ROS_ERROR("Failure power on voltmeter. Exiting.");
                return false;
            }
            ROS_INFO("BATTERY OPENED!.");
            return true;
        }

        bool probeCircuit()
        {
            ROS_INFO("Probing the desired circuit...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(probe_circuit_activation_service_name_) << " ROS service...");
            probe_circuit_activation_client_.waitForExistence();

            hrii_robothon_msgs::ProbeCircuitByod probe_circuit_srv;

            //-------------------------------------------------- FRANKA LEFT PROBING CIRCUIT--------------------------------------------------
            //We use the left franka to grab the red probe 
            probe_circuit_srv.request.left_robot_id = left_robot_id_;
            // Get red probe pose wrt to left franka 
            geometry_msgs::TransformStamped red_probe_handle_transform;
            try
            {
                red_probe_handle_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.left_robot_id+"_link0", "voltmeter_circuit_red_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.left_robot_id << "_link0 and voltmeter_circuit_red_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.left_robot_id << "_link0 and voltmeter_circuit_red_probe_link NOT found!");
                return false;
            }
            // With the franka left we probe the gold hole in the battery. Computation of the gold hole pose wrt to left franka pose
            geometry_msgs::TransformStamped hole_to_probe_transform;
            try
            {
                hole_to_probe_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.left_robot_id+"_link0", "voltmeter_circuit_battery_gold_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.left_robot_id << "_link0 and voltmeter_circuit_battery_gold_hole_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.left_robot_id << "_link0 and voltmeter_circuit_battery_gold_hole_link NOT found!");
                return false;
            }
                        
            probe_circuit_srv.request.red_probe_handle_pose.pose = geometry_msgs::toPose(red_probe_handle_transform.transform);
            probe_circuit_srv.request.red_circuit_pose.pose = geometry_msgs::toPose(hole_to_probe_transform.transform);

            //-------------------------------------------------- FRANKA RIGHT PROBING CIRCUIT--------------------------------------------------
            //We use the right franka to grab the black probe 
            probe_circuit_srv.request.right_robot_id = right_robot_id_;
            // Get black probe pose wrt franka right
            geometry_msgs::TransformStamped black_probe_handle_transform;
            try
            {
                black_probe_handle_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.right_robot_id+"_link0", "voltmeter_circuit_black_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.right_robot_id << "_link0 and voltmeter_circuit_black_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.right_robot_id << "_link0 and voltmeter_circuit_black_probe_link NOT found!");
                return false;
            }
            // With the franka right we probe the battery. Computation of the battery pose wrt to right franka pose
            geometry_msgs::TransformStamped battery_to_probe_transform;
            try
            {
                battery_to_probe_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.right_robot_id+"_link0", "voltmeter_circuit_battery_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.right_robot_id << "_link0 and voltmeter_circuit_battery_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.right_robot_id << "_link0 and voltmeter_circuit_battery_link NOT found!");
                return false;
            }

            probe_circuit_srv.request.black_probe_handle_pose.pose = geometry_msgs::toPose(black_probe_handle_transform.transform);
            probe_circuit_srv.request.black_circuit_pose.pose = geometry_msgs::toPose(battery_to_probe_transform.transform);

            if (!probe_circuit_activation_client_.call(probe_circuit_srv))
            {
                ROS_ERROR("Error calling probing activation service.");
                return false;
            }
            else if (!probe_circuit_srv.response.success)
            {
                ROS_ERROR("Failure probing the battery. Exiting.");
                return false;
            }
            ROS_INFO("Battery probed pressed.");

            return true;
        }

        bool batteryScreening()
        {
            ROS_INFO("Battery screening...");

            hrii_robothon_msgs::BatteryScreening battery_screening_srv;
            battery_screening_srv.request.left_robot_id = left_robot_id_;
            battery_screening_srv.request.right_robot_id = right_robot_id_;
            
            // Get red probe pose
            geometry_msgs::TransformStamped red_probe_handle_transform;
            try
            {
                red_probe_handle_transform = tf_buffer_.lookupTransform(battery_screening_srv.request.left_robot_id+"_link0", "voltmeter_circuit_red_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << battery_screening_srv.request.left_robot_id << "_link0 and voltmeter_circuit_red_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << battery_screening_srv.request.left_robot_id << "_link0 and voltmeter_circuit_red_probe_link NOT found!");
                return false;
            }
            // Get black probe pose
            geometry_msgs::TransformStamped black_probe_handle_transform;
            try
            {
                black_probe_handle_transform = tf_buffer_.lookupTransform(battery_screening_srv.request.right_robot_id+"_link0", "voltmeter_circuit_black_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << battery_screening_srv.request.right_robot_id << "_link0 and voltmeter_circuit_black_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << battery_screening_srv.request.right_robot_id << "_link0 and voltmeter_circuit_black_probe_link NOT found!");
                return false;
            }

            battery_screening_srv.request.red_probe_handle_pose.pose = geometry_msgs::toPose(red_probe_handle_transform.transform);
            battery_screening_srv.request.black_probe_handle_pose.pose = geometry_msgs::toPose(black_probe_handle_transform.transform);

            if (!battery_screening_activation_client_.call(battery_screening_srv))
            {
                ROS_ERROR("Error calling screening battery activation service.");
                return false;
            }
            else if (!battery_screening_srv.response.success)
            {
                ROS_ERROR("Failure screening battery. Exiting.");
                return false;
            }
            ROS_INFO("SLIDER MOVED!.");

            return true;
        }


        States resolveStateString(const std::string& state_str)
        {
            if (state_str.compare("homing") == 0 || state_str.compare("HOMING") == 0) return States::HOMING;
            if (state_str.compare("other_homing") == 0 || state_str.compare("OTHER_HOMING") == 0) return States::OTHER_HOMING;
            if (state_str.compare("battery_opening") == 0 || state_str.compare("BATTERY_OPENING") == 0) return States::BATTERY_OPENING;
            if (state_str.compare("voltmeter_power_on") == 0 || state_str.compare("VOLTMETER_POWER_ON") == 0) return States::VOLTMETER_POWER_ON;
            if (state_str.compare("probe") == 0 || state_str.compare("PROBE") == 0) return States::PROBE;
            if (state_str.compare("battery_screening") == 0 || state_str.compare("BATTERY_SCREENING") == 0) return States::BATTERY_SCREENING;
            if (state_str.compare("exit") == 0 || state_str.compare("EXIT") == 0) return States::EXIT;
            ROS_ERROR("State string not recognired. Return ERROR state.");
            return States::ERROR;
        }

}; // class MainBYOD

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_byod");
    
    MainBYOD byod;

    if (!byod.init()) return -1;
    byod.spin();

    return 0;
}