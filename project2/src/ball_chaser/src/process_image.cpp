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

    int white_pixel = 255;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int move_left = 0;
    int move_right = 0;
    int move_forward = 0;


    for (int i = 0; i < img.height * img.step; i+=3) {
        int location = i % (img.width * 3) / 3;
    	if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel){
          if(location <= 300){
            move_left +=1;
          }else if(location >= 600){ move_right +=1;
          }else{
            move_forward +=1;
          }
        }
  }


    int lin_x = 0;
    int ang_z = 0;
    if(move_left > move_right && move_left > move_forward){
      ang_z = -0.5;
    }else if(move_right > move_left && move_right > move_forward){
      ang_z = 0.5;
    }else if(move_forward > move_left && move_forward > move_right){
      lin_x = 0.5;
    }




    drive_robot(lin_x, ang_z);

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

