#include <ros/ros.h>     
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;
class VelToOdom
{
  private:
  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub; 
  double x;
  double y;
  double th;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time;
  ros::Time last_time;
  public:
  VelToOdom() 
  { 
    x=0.0;y=0.0;th=0.0;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    sub = n.subscribe("new_car", 2, &VelToOdom::callback, this);//cmd_vel_mux/input/teleop;new_car
  }
  void callback(const geometry_msgs::Twist& msg)
  {
      ros::spinOnce();                 // check for incoming messages
      current_time = ros::Time::now();
      double dt = (current_time - last_time).toSec();
      double delta_x = 0;
      double delta_y = 0;
      double delta_th = 0;
      if (msg.angular.z==0)
      {
        delta_x = (msg.linear.x * cos(th)-msg.linear.y * sin(th)) * dt;
        delta_y = (msg.linear.x * sin(th)+msg.linear.y * cos(th)) * dt;
        delta_th = msg.angular.z * dt;
      }
      else if (msg.angular.z!=0)
      {
        delta_x = msg.linear.x / msg.angular.z * sin(th + msg.angular.z * dt) - msg.linear.x / msg.angular.z * sin(th);
        delta_y = msg.linear.x / msg.angular.z * cos(th) - msg.linear.x / msg.angular.z * cos(th + msg.angular.z * dt);
        delta_th = msg.angular.z * dt;
      }
      x += delta_x;
      y += delta_y;
      th += delta_th;
      //cout<<th<<endl;
      //cout<<x<<endl;
      //cout<<y<<endl<<endl;
      //since all odometry is 6DOF we'll need a quaternion created from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      
      odom_broadcaster.sendTransform(odom_trans);

      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = msg.linear.x * cos(th);
      odom.twist.twist.linear.y = msg.linear.x * sin(th);
      odom.twist.twist.angular.z = msg.angular.z;

      //publish the message
      pub.publish(odom);

      last_time = current_time; 
  }
};

int main(int argc, char **argv)
{ 
//Initiate ROS 
ros::init(argc, argv, "VelToOdom");
//Create an object of class SubscribeAndPublish that will take care of everything 
VelToOdom VTO;
ros::spin();
return 0;
}

