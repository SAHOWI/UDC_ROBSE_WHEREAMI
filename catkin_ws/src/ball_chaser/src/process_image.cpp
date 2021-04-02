#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget drive_cmd;
    drive_cmd.request.linear_x = lin_x;
    drive_cmd.request.angular_z = ang_z;
    
    if (!client.call(drive_cmd)){
        ROS_ERROR("Failed to execute drive command");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    // Definitions
    int white_pixel_val = 255;
    int white_moment = 0;
    int white_pixels = 0;
    int total_pixels = 0;
    float awm;
    float x, z;
 
    for(int i=0; i<img.height; ++i)
    {
        for (int j=0; j<img.step; ++j)
        {
            ++total_pixels;
            
            if (img.data[i * img.step + j] == white_pixel_val)
            {
                ++white_pixels;
                white_moment = white_moment + j - img.step/2;
                awm = white_moment/white_pixels;
            }
        }
    }
    
    ROS_INFO("%d white pixels found - awm is:  %1.2f ", (int)white_pixels,awm);

    if (white_pixels == 0) 
    { // no pixels found turn until you find some!
        x = 0.0;
        z = -.5;
    }  
    else 
    {
        x = 0.1;
        z=  -0.5*awm/600;
    }
    
    // next move
    drive_robot(x,z);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}