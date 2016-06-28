#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <rosserial_msgs/UltraSonic.h>

#define PI 3.14159265
#define DEG_TO_RAD(x) (x * PI / 180.0)
#define MAX_RANGE 100.0 // meters
#define MIN_ANGLE -45 // degrees
#define MAX_ANGLE 45 // degrees
#define DEG_INCREMENT 7.5 // degrees


class SubscribeAndPublish 
{ 
  private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_; 
  public:
  SubscribeAndPublish() 
  { 
    //Topic you want to publish 
    ros::Rate r(0.2);
    pub_ = n_.advertise<sensor_msgs::LaserScan>("scan",100);
    //Topic you want to subscribe 
    sub_ = n_.subscribe("UltraSonic", 100, &SubscribeAndPublish::callback, this);
  }
  void callback(const rosserial_msgs::UltraSonic& input) 
  { 
    int id=0;
    sensor_msgs::LaserScan     output; 
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "/scan";
    output.angle_min=DEG_TO_RAD(MIN_ANGLE);
    output.angle_max=DEG_TO_RAD(MAX_ANGLE);
    output.angle_increment=DEG_TO_RAD(DEG_INCREMENT);
    output.time_increment=0.1;
    output.scan_time=0.1;
    output.range_min=0;
    output.range_max=100;

    output.ranges.push_back(input.sonar1/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar8/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar2/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar9/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar3/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar10/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar4/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar11/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar5/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar12/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar6/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar13/100.0);
    output.intensities.push_back(0.5);
    output.ranges.push_back(input.sonar7/100.0);
    output.intensities.push_back(0.5);
    //output.ranges.push_back(input.sonar14/100.0);
    //output.intensities.push_back(0.5);
    //output.ranges.push_back(input.sonar15/100.0);
    //output.intensities.push_back(0.5);
    //output.ranges.push_back(input.sonar16/100.0);
    //output.intensities.push_back(0.5);
    //output.ranges.push_back(input.sonar17/100.0);
    //output.intensities.push_back(0.5);
    //output.ranges.push_back(input.sonar18/100.0);
    //output.intensities.push_back(0.5);
    pub_.publish(output);
  } 
};
//End of class SubscribeAndPublish
int main(int argc, char **argv)
{ 
//Initiate ROS 
ros::init(argc, argv, "subscribe_and_publish");
//Create an object of class SubscribeAndPublish that will take care of everything 
SubscribeAndPublish SAPObject;
ros::spin();
return 0;
}
