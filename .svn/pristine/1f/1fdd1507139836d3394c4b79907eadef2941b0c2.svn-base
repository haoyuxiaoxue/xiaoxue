#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
using namespace std;

#define key_ESC 27

void init_keyboard();

void close_keyboard();

int kbhit();

int readch(); /* 相关函数声明 */

static struct termios initial_settings, new_settings;

static int peek_character = -1;         /* 用于测试一个按键是否被按下 */

/* 检测键盘按键的函数 */

int kbhit()

{
    char ch;

    int nread;

    if ( peek_character != -1 )

        return(1);

    new_settings.c_cc[VMIN] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );

    nread = read( 0, &ch, 1 );

    new_settings.c_cc[VMIN] = 1;

    tcsetattr( 0, TCSANOW, &new_settings );

    if ( nread == 1 )
    {
        peek_character = ch;

        return(1);
    }

    return(0);
}

/* 用来接收按下的按键，并peek_character = -1恢复状态 */

int readch()

{
    char ch;

    if ( peek_character != -1 )
    {
        ch = peek_character;

        peek_character = -1;

        return(ch);
    }

    read( 0, &ch, 1 );

    return(ch);
}

/* 配置终端函数 */

void init_keyboard()

{
    tcgetattr( 0, &initial_settings );

    new_settings = initial_settings;

    new_settings.c_lflag &= ~ICANON;

    new_settings.c_lflag &= ~ECHO;

    new_settings.c_lflag &= ~ISIG;

    new_settings.c_cc[VMIN] = 1;

    new_settings.c_cc[VTIME] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );
}

/* 关闭键盘 */
void close_keyboard()

{
    tcsetattr( 0, TCSANOW, &initial_settings );
}

int main(int argc, char **argv)
{
  float speed=0.0;
  float angular=0.0;
  int count = 0;
  int ch = 0;
  geometry_msgs::Twist vel;
  ros::init(argc, argv, "pub_cmdvel");
  ros::NodeHandle n;
  ros::Publisher pub;
  pub=n.advertise<geometry_msgs::Twist>("cmd_vel",100);//cmd_vel_mux/input/teleop
  cout<<"按asdw控制小车行动"<<endl;
  ros::Rate ros_loop(3);
  
  init_keyboard();//初始化键盘
  
  while(ros::ok()&&ch!=27)
  {
    if ( kbhit() )
    {
      ch = readch();
      if ( ch != 27 )
      //printf( "You put %c ! Only put ESC can quit! \n", ch );
      {
        if(ch==119)//w向前
        {
          speed=speed+0.3;
          cout<<"小车以"<<speed<<"速度行驶"<<endl;
        }
        else if(ch==115)//s向后
        {
          speed=speed-0.3;
          cout<<"小车以"<<speed<<"速度行驶"<<endl;
        }
        else if(ch==97)//a左转
        {
          angular=angular+0.4;
          cout<<"小车以"<<angular<<"速度旋转"<<endl;
        }
        else if(ch==100)//d右转
        {
          angular=angular-0.4;
          cout<<"小车以"<<angular<<"速度旋转"<<endl;
        }
        else if(ch==113)//q停止
        {
          angular=0.0;
          speed=0.0;
          cout<<"小车停止"<<endl;
        }
      }
    }
    vel.linear.x=speed;
    vel.linear.y=0;
    vel.linear.z=0;
    vel.angular.x=0;
    vel.angular.y=0;
    vel.angular.z=angular;
    pub.publish(vel);
    ros::spinOnce();
    ros_loop.sleep();
    count++;
  }
  close_keyboard();
}
