// #include <respawn_robot.h>

#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <nav_msgs/Path.h>


// Callback function for the robot's odometry or state topic
void robotStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // Check if the robot has fallen down or gone out of bounds
    if (msg->data[0] < -1)
    {
        // Call the SetModelState service to respawn the robot
        gazebo_msgs::ModelState modelState;
        modelState.model_name = "aidin81";  // Replace with your robot's name
        modelState.pose.position.x = 0.0;   // Replace with your robot's starting position
        modelState.pose.position.y = 0.0;
        modelState.pose.position.z = 0.0;
        modelState.pose.orientation.x = 0.0;  // Replace with your robot's starting orientation
        modelState.pose.orientation.y = 0.0;
        modelState.pose.orientation.z = 0.0;
        modelState.pose.orientation.w = 1.0;
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        gazebo_msgs::SetModelState srv;
        srv.request.model_state = modelState;
        if (client.call(srv))
        {
            ROS_INFO("Robot respawned successfully");
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/set_model_state");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "respawn_robot");

    // Subscribe to the robot's odometry or state topic
    ros::NodeHandle nh;
    ros::Subscriber sub_bodypose = nh.subscribe("/aidin81/BodyPose_sim", 10, robotStateCallback);
    ros::Publisher pub_tarpos = nh.advertise<nav_msgs::Path>("/aidin81/targetpos", 100);
    

    ros::spin();

    return 0;
}
