/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



// I modify this code to fix udacity project
// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// %EndTag(INCLUDES)%



double positionx;
double positiony;

double positionx1 = 2.0, positiony1 = 0;
double positionx2 = -1.0, positiony2 = 0;
bool pickup = false;
bool dropoff = false;
bool wait = false;

void odom_callback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
    positionx = pose_msg.pose.pose.position.x;
    positiony = pose_msg.pose.pose.position.y;
}



// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
// %EndTag(INIT)%

  
  
  ros::Subscriber sub = n.subscribe("/amcl_pose", 1, odom_callback);

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
  marker.ns = "add_markers";
  marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
  marker.type = shape;
// %EndTag(TYPE)%

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
// %Tag(ACTION)%
  marker.action = visualization_msgs::Marker::ADD;
// %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
  marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

   
// our task is to subscribe to the odometry values




 // Publish the marker
    
    
  while (ros::ok())
  {
    // Calculate the distance
    double pickup_dist = (positionx-positionx1)*(positionx-positionx1) + (positiony-positiony1)*(positiony-positiony1);

    if (pickup_dist>0.01 && !pickup)
    {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = positionx1;
        marker.pose.position.y = positiony1;
    }else{
      marker.action = visualization_msgs::Marker::DELETE;
      pickup = true;
    
    }
      

    if ( pickup && ! wait )
    {
       ROS_INFO("wait for 5 seconds");
       ros::Duration(5.0).sleep();
       wait = true;  
      
    }
    
      
    if ( pickup && wait &&  !dropoff ){
      
      double dropoff_dist = (positionx-positionx2)*(positionx-positionx2) + (positiony-positiony2)*(positiony-positiony2);
      
      if (dropoff_dist>0.01)
      {
        marker.action = visualization_msgs::Marker::DELETE;
      }
      else
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = positionx2;
        marker.pose.position.y = positiony2;
        dropoff = true;
      }
      
    }
     
    
    marker_pub.publish(marker);
    
    sleep(1);
  }

  
}
// %EndTag(FULLTEXT)%





