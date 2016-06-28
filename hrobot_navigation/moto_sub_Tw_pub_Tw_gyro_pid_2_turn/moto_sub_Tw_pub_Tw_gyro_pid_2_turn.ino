#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "GY_85.h"
#include <Wire.h>
#include <MsTimer2.h>
//pid参数设置
unsigned long lastTime, lastTime_left, Input_left_gz, lastTime_right, Input_right_gz;
float Input, Input_left, Input_right, Output, Output_left, Output_right, Setpoint, Setpoint_left, Setpoint_left_az, Setpoint_right, Setpoint_right_az;
float ITerm, ITerm_left, ITerm_right, lastInput, lastInput_left, lastInput_left_gz, lastInput_right, lastInput_right_gz;
float kp, ki, kd, kp_, ki_, kd_;
int SampleTime = 100; //0.1sec
float outMax, outMin, outMax_a, outMin_a;
float lx = 0;
float az = 0;
//驱动参数设置
int ena = 2;
int motoIn1 = 3;
int motoIn2 = 4;
int motoIn3 = 5;
int motoIn4 = 6;
int enb = 7;
float count = 0;
float count_right = 0;
float count_left = 0;
int judgepin = 18;
float v = 0;
GY_85 GY85;
bool GB = true;
bool tn = false;
float car_band = 0.7;
int ctrl = 0;
int ctrl_0 = 0;
int ctrl_1 = 0;
int ctrl_2 = 0;
int ctrl_left = 0;
int ctrl_right = 0;
float gz = 0;
int punish = 0; //控制机器人做直线,如果不是直线,则加入惩罚因数进行调整.

ros::NodeHandle  nh;
geometry_msgs::Twist vel;
ros::Publisher pub("new_car", &vel);

void Compute_left()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime_left);//第10行
  Setpoint_left = (lx - az * car_band) / 0.5 * 193 + punish;
  Input_left = count_left;
  if (timeChange >= SampleTime)
  {
    float error = Setpoint_left - Input_left;
    ITerm_left = ITerm_left + ki * error;
    if (ITerm_left > outMax)ITerm_left = outMax;
    else if (ITerm_left < outMin)ITerm_left = outMin;
    float dInput = (Input_left - lastInput_left);
    Output_left = kp * error + ITerm_left - kd * dInput;
    if (Output_left > outMax)Output_left = outMax;
    else if (Output_left < outMin)Output_left = outMin;
    lastInput_left = Input_left;
    lastTime_left = now;
    //Input = Output; //测试加上的代码
    //ctrl_2 = ctrl_1;
    //ctrl_1 = ctrl_0;
    //ctrl_0 = Output;
    ctrl_left = Output_left;
  }
}
void Compute_left_turn()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime_left);//第10行
  Setpoint_left_az = -az * 150 ;
  if (Setpoint_left_az > outMax_a)Setpoint_left_az = outMax;
  else if (Setpoint_left_az < outMin_a)Setpoint_left_az = outMin;
  Input_left_gz = -gz * 150;
  if (Input_left_gz > outMax_a)Input_left_gz = outMax;
  else if (Input_left_gz < outMin_a)Input_left_gz = outMin;
  if (timeChange >= SampleTime)
  {
    float error = Setpoint_left_az - Input_left_gz;
    ITerm_left = ITerm_left + ki * error;
    if (ITerm_left > outMax)ITerm_left = outMax;
    else if (ITerm_left < outMin)ITerm_left = outMin;
    float dInput = (Input_left_gz - lastInput_left_gz);
    Output_left = kp * error + ITerm_left - kd * dInput;
    if (Output_left > outMax)Output_left = outMax;
    else if (Output_left < outMin)Output_left = outMin;
    lastInput_left_gz = Input_left_gz;
    lastTime_left = now;
    //Input = Output; //测试加上的代码
    //ctrl_2 = ctrl_1;
    //ctrl_1 = ctrl_0;
    //ctrl_0 = Output;
    ctrl_left = Output_left;
  }
}
void Compute_right_turn()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime_right);//第10行
  Setpoint_right_az = az * 150 ;
  if (Setpoint_right_az > outMax_a)Setpoint_right_az = outMax;
  else if (Setpoint_right_az < outMin_a)Setpoint_right_az = outMin;
  Input_right_gz = gz * 150;
  if (Input_right_gz > outMax_a)Input_right_gz = outMax;
  else if (Input_right_gz < outMin_a)Input_right_gz = outMin;
  if (timeChange >= SampleTime)
  {
    float error = Setpoint_right_az - Input_right_gz;
    ITerm_right = ITerm_right + ki * error;
    if (ITerm_right > outMax)ITerm_right = outMax;
    else if (ITerm_right < outMin)ITerm_right = outMin;
    float dInput = (Input_right_gz - lastInput_right_gz);
    Output_right = kp * error + ITerm_right - kd * dInput;
    if (Output_right > outMax)Output_right = outMax;
    else if (Output_right < outMin)Output_right = outMin;
    lastInput_right_gz = Input_right_gz;
    lastTime_right = now;
    //Input = Output; //测试加上的代码
    //ctrl_2 = ctrl_1;
    //ctrl_1 = ctrl_0;
    //ctrl_0 = Output;
    ctrl_right = Output_right;
  }
}
void Compute_right()
{
  unsigned long now = millis();
  int timeChange = (now - lastTime_right);//第10行
  Setpoint_right = (lx + az * car_band) / 0.5 * 193 - punish;
  Input_right = count_right;
  if (timeChange >= SampleTime)
  {
    float error = Setpoint_right - Input_right;
    ITerm_right = ITerm_right + ki * error;
    if (ITerm_right > outMax)ITerm_right = outMax;
    else if (ITerm_right < outMin)ITerm_right = outMin;
    float dInput = (Input_right - lastInput_right);
    Output_right = kp * error + ITerm_right - kd * dInput;
    if (Output_right > outMax)Output_right = outMax;
    else if (Output_right < outMin)Output_right = outMin;
    lastInput_right = Input_right;
    lastTime_right = now;
    //Input = Output; //测试加上的代码
    //ctrl_2 = ctrl_1;
    //ctrl_1 = ctrl_0;
    //ctrl_0 = Output;
    ctrl_right = Output_right;
  }
}
void SetTunings(float Kp, float Ki, float Kd)
{
  float SampleTimeInSec = ((double)SampleTime) / 1000; // millis()函数的时间是ms所以/1000可以转化成秒
  kp = Kp;
  ki = Ki * SampleTimeInSec;//31行
  kd = Kd / SampleTimeInSec;
}

void messageCb( const geometry_msgs::Twist& vel_msg) {
  lx = vel_msg.linear.x;
  az = vel_msg.angular.z;
  if (vel_msg.angular.z > 0.01)
  {
    if (vel_msg.linear.x > 0.01)
    {
      tn = false;
      GB = true;
      digitalWrite(motoIn1, LOW); //设置方向
      digitalWrite(motoIn2, HIGH); //设置方向
      analogWrite(ena, ctrl_right ); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, LOW); //设置方向
      digitalWrite(motoIn4, HIGH); //设置方向
      analogWrite(enb, ctrl_left); //i为转速信息,i越大速度越快,最大255.
    }
    else
    {
      tn = true;
      GB = true;
      digitalWrite(motoIn1, LOW); //设置方向
      digitalWrite(motoIn2, HIGH); //设置方向
      analogWrite(ena, ctrl_right); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, HIGH); //设置方向
      digitalWrite(motoIn4, LOW); //设置方向
      analogWrite(enb, ctrl_right); //i为转速信息,i越大速度越快,最大255.
    }
  }
  else if (vel_msg.angular.z < -0.01)
  {
    if (vel_msg.linear.x > 0.01)
    {
      tn = false;
      GB = true;
      digitalWrite(motoIn1, LOW); //设置方向
      digitalWrite(motoIn2, HIGH); //设置方向
      analogWrite(ena, ctrl_right); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, LOW); //设置方向
      digitalWrite(motoIn4, HIGH); //设置方向
      analogWrite(enb, ctrl_left); //i为转速信息,i越大速度越快,最大255.
    }
    else
    {
      tn = true;
      GB = true;
      digitalWrite(motoIn1, HIGH); //设置方向
      digitalWrite(motoIn2, LOW); //设置方向
      analogWrite(ena, ctrl_left); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, LOW); //设置方向
      digitalWrite(motoIn4, HIGH); //设置方向
      analogWrite(enb, ctrl_left); //i为转速信息,i越大速度越快,最大255.
    }
  }
  else
  {
    if (vel_msg.linear.x > 0.01)
    {
      tn = false;
      GB = true;
      digitalWrite(motoIn1, LOW); //设置方向
      digitalWrite(motoIn2, HIGH); //设置方向
      analogWrite(ena, ctrl_right); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, LOW); //设置方向
      digitalWrite(motoIn4, HIGH); //设置方向
      analogWrite(enb, ctrl_left); //i为转速信息,i越大速度越快,最大255.
    }
    else
    {
      //tn = false;
      //GB = false;
      digitalWrite(motoIn1, LOW); //设置方向
      digitalWrite(motoIn2, HIGH); //设置方向
      analogWrite(ena, 0); //i为转速信息,i越大速度越快,最大255.
      digitalWrite(motoIn3, LOW); //设置方向
      digitalWrite(motoIn4, HIGH); //设置方向
      analogWrite(enb, 0); //i为转速信息,i越大速度越快,最大255.
    }
  }
}

void jishu_right()//中断函数
{
  delayMicroseconds(30);
  count_right = count_right + 1;
}
void jishu_left()//中断函数
{
  delayMicroseconds(30);
  count_left = count_left + 1;
}

//geometry_msgs::Twist vel;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb );//cmd_vel_mux/input/teleop,cmd_vel

void setup() {
  kp_ = 0.2;
  ki_ = 1.5;
  kd_ = 0.01;
  SetTunings(kp_, ki_, kd_);
  Input = 0;
  Output = 0;
  outMax = 255;
  outMin = 15;
  outMax_a = 255;
  outMin_a = 0;
  lastTime = millis();
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  pinMode(motoIn1, OUTPUT);
  pinMode(motoIn2, OUTPUT);
  pinMode(motoIn3, OUTPUT);
  pinMode(motoIn4, OUTPUT);
  pinMode(judgepin, INPUT);
  Wire.begin();
  GY85.init();
  attachInterrupt(4, jishu_right, FALLING);//当int.0电平改变时,触发中断函数blink
  attachInterrupt(5, jishu_left, FALLING);//当int.0电平改变时,触发中断函数blink
  MsTimer2::set(102, pbh); // 500ms period
  MsTimer2::start();
}
void pbh()
{

}
void loop() {

  if (az > 0.01)
  {
    if (lx > 0.01)
    {
      Compute_left();
      Compute_right_turn();
    }
    else
    {
      Compute_left_turn();
      Compute_right_turn();
    }
  }
  else if (az < -0.01)
  {
    if (lx > 0.01)
    {
      Compute_right();
      Compute_left_turn();
    }
    else
    {
      Compute_left_turn();
      Compute_right_turn();
    }
  }
  else
  {
    Compute_left();
    Compute_right();
    //if (gz > 0.01) punish = punish + 5;
    //if (gz < -0.01)punish = punish - 5;
  }
  count = (count_left + count_right) / 2;
  gz = 0.252 * GY85.gyro_z( GY85.readGyro() ) / 14.375;
  v = (count / 80) * 33.18 * 5 / 100 ;
  if (!tn)
  {
    if (GB)
    {
      vel.linear.x = v / 8;
    }
    else
    {
      vel.linear.x = -v / 8;
    }
  }
  else
  {
    vel.linear.x = 0;
  }
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  if (gz < 0.03 && gz > (-0.03))gz = 0;
  vel.angular.z = gz;
  pub.publish(&vel);
  count = 0;
  count_right = 0;
  count_left = 0;
  nh.spinOnce();
  delay(200);
}

