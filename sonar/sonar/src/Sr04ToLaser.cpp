#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <rosserial_msgs/UltraSonicR.h>

#define PI 3.14159265
#define DEG_TO_RAD(x) (x * PI / 180.0)
#define MAX_RANGE 1.0 // meters
#define MIN_ANGLE -90 // degrees
#define MAX_ANGLE 80 // degrees
#define DEG_INCREMENT 10 // degrees


class SubscribeAndPublish 
{ 
  private:
  ros::NodeHandle n;
  ros::Publisher  pub;
  ros::Subscriber sub; 
  public:
  SubscribeAndPublish() 
  { 
    //Topic you want to publish 
    pub = n.advertise<sensor_msgs::LaserScan>("laser_sr04",100);
    //Topic you want to subscribe 
    sub = n.subscribe("UltraSonicR", 100, &SubscribeAndPublish::callback, this);
  }
  void callback(const rosserial_msgs::UltraSonicR& input) 
  { 
    sensor_msgs::LaserScan     output; 
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "/laser_sr04";
    output.angle_min=DEG_TO_RAD(MIN_ANGLE);
    output.angle_max=DEG_TO_RAD(MAX_ANGLE);
    output.angle_increment=DEG_TO_RAD(DEG_INCREMENT);
    output.time_increment=0.1;
    output.scan_time=0.1;
    output.range_min=0;
    output.range_max=1;

    if (input.sonar1==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar1/100.0);
    if (input.sonar2==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar2/100.0);
    if (input.sonar3==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar3/100.0);
    if (input.sonar4==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar4/100.0);
    if (input.sonar5==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar5/100.0);
    if (input.sonar6==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar6/100.0);
    if (input.sonar7==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar7/100.0);
    if (input.sonar8==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar8/100.0);
    if (input.sonar9==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar9/100.0);
    if (input.sonar10==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar10/100.0);
    if (input.sonar11==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar11/100.0);
    if (input.sonar12==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar12/100.0);
    if (input.sonar13==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar13/100.0);
    if (input.sonar14==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar14/100.0);
    if (input.sonar15==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar15/100.0);
    if (input.sonar16==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar16/100.0);
    if (input.sonar17==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar17/100.0);
    if (input.sonar18==6) output.ranges.push_back(300);else output.ranges.push_back(input.sonar18/100.0);

    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);
    output.intensities.push_back(0.5);


    pub.publish(output);
  } 
};
//End of class SubscribeAndPublish
int main(int argc, char **argv)
{ 
//Initiate ROS 
ros::init(argc, argv, "laser_sr04");
//Create an object of class SubscribeAndPublish that will take care of everything 
SubscribeAndPublish SAPObject;
ros::spin();
return 0;
}
