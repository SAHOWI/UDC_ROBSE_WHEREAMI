#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

/* Begin of DriveBot Class definition */


class DriveBot
{
private:
    ros::Publisher motor_command_publisher__;
    ros::ServiceServer service__; 
    ros::NodeHandle n__;
public:
    DriveBot()
    {
        /* inform ROS of what commands will be published by publisher */
        motor_command_publisher__ = n__.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
     
        /* create a service called command_bot service with handle_drive call back */
        service__ = n__.advertiseService("/ball_chaser/command_bot", &DriveBot::handle_drive, this);

        ROS_INFO("Ready to send velocity commands (/cmd_vel)");
    }

    bool handle_drive(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
    {

        ROS_INFO("Drive command request received - linear_X:%1.2f, angular_Z:%1.2f", (float)req.linear_x, (float)req.angular_z);

        /* create Twist velocity command */
        geometry_msgs::Twist cmd;

        /* copy first */
        cmd.linear.x = req.linear_x;
        cmd.angular.z = req.angular_z;

        /* publish the command */
        motor_command_publisher__.publish(cmd);

        res.msg_feedback = "Linear X is set to: " + std::to_string(cmd.linear.x) + ", Angular Z is set to: " + std::to_string(cmd.angular.z);

        ROS_INFO_STREAM(res.msg_feedback);

        return true;
    }
};

/* End of DriveBot Class definition */


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", DriveBot::handle_drive);
    

    // Handle ROS communication events
    ROS_INFO("Ready to send joint commands");
    ros::spin();

    return 0;
}