// Include the ROS headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// Define a class to control the joint
class VelodyneController
{
public:
  // Constructor
  VelodyneController()
  {
    // Create a publisher to the joint controller topic
    pub_ = nh_.advertise<std_msgs::Float64>("/joint/command", 1);
    // Create a subscriber to the teleop twist keyboard topic
    sub_ = nh_.subscribe("/cmd_vel", 1, &VelodyneController::twistCallback, this);
  }

  // A callback function to receive Twist messages
  void twistCallback(const geometry_msgs::Twist& msg)
  {
    // Get the angular velocity around the y axis
    double vel = msg.angular.y;
    // Create a Float64 message
    std_msgs::Float64 joint_msg;
    // Set the data field to the velocity
    joint_msg.data = vel;
    // Publish the message to the joint controller
    pub_.publish(joint_msg);
  }

private:
  // A node handle
  ros::NodeHandle nh_;
  // A publisher
  ros::Publisher pub_;
  // A subscriber
  ros::Subscriber sub_;
};

// The main function
int main(int argc, char** argv)
{
  // Initialize the node
  ros::init(argc, argv, "velodyne_controller");
  // Create an object of the class
  VelodyneController vc;
  // Spin the node
  ros::spin();
  // Return 0
  return 0;
}

