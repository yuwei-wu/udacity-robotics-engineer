#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    ROS_INFO_STREAM(lin_x);
    ROS_INFO_STREAM(ang_z);

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot ");



}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    ROS_INFO("Process images");
    int white_pixel = 255;
    // Loop through each pixel in the image and check if there's a bright white one



    // revise : we need to check three times 

    int locationy = -1;
    for (int i = 0; i<  img.height * img.step  ; i+=3) {
        if (img.data[i] == white_pixel &&  img.data[i+1] == white_pixel && img.data[i+2] == white_pixel  ) {
            locationy = i % img.step;
            break;
        }
   
   }
   ROS_INFO_STREAM(locationy);
   ROS_INFO_STREAM(img.step);
   ROS_INFO("Process images");
    // Then, identify if this pixel falls in the left, mid, or right side of the image

    // Depending on the white ball position, call the drive_bot function and pass velocities to it


    double lin_x = 0;
    double ang_z = 0; // pase info to the robot


    if ( locationy <= img.step/3 ) {ang_z = 0.5;

   }else if ( locationy >= img.step *2.0/3 ) {ang_z = -0.5;}

    else if( locationy != -1){
      lin_x = 0.5;
    }


     
    // Request a stop when there's no white ball seen by the camera
    drive_robot(lin_x, ang_z);   //0 0 

}


int main(int argc, char** argv)
{
  // Initialize the process_image node and create a NodeHandle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  // Define a service client capabale of requesting services from "/ball_chaser/command_robot"
  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // Subscribe to /camera/rgb/image_raw topic to read the image data inside
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

  // Handle ROS communication events
  ros::spin();

  return 0;

}

