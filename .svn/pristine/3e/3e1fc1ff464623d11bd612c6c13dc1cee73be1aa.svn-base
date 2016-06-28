
#include <ros.h>
#include <rosserial_msgs/UltraSonicR.h>
#include <Servo.h>

int inputPin1=4;//定义超声波信号接受口
int outputPin1=5;//定义超声波信号发出口
int inputPin2=6;//定义超声波信号接受口
int outputPin2=7;//定义超声波信号发出口
int inputPin3=8;//定义超声波信号接受口
int outputPin3=9;//定义超声波信号发出口
int dis_max=58*80;

Servo servo1;
int v=0;
float distance=0;
float dis[2]={0.0};
ros::NodeHandle  nh;
rosserial_msgs::UltraSonicR output;
ros::Publisher us("UltraSonicR", &output);//这里output前加& 一定要注意!!

float sonar(int inputPin,int outputPin)
{
  //产生10us的脉冲
  digitalWrite(outputPin,LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin,HIGH);
  delayMicroseconds(10);
  //的量脉冲宽度
  distance=pulseIn(inputPin,HIGH,dis_max)/58.8;
  //声速一般书29cm/us,因为是来回,所以乘以2
  return distance;
  }

void process(int p)
{
  servo1.write(p);
  delay(70);
  //Serial.print("degree = ");
  //Serial.println(p);
}

void come()
{
  process(v);
  output.sonar1=sonar(inputPin1,outputPin1)+6;
  output.sonar7=sonar(inputPin2,outputPin2)+6;
  output.sonar13=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  v=10;
  process(v);
  output.sonar2=sonar(inputPin1,outputPin1)+6;
  output.sonar8=sonar(inputPin2,outputPin2)+6;
  output.sonar14=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  v=20;
  process(v);
  output.sonar3=sonar(inputPin1,outputPin1)+6;
  output.sonar9=sonar(inputPin2,outputPin2)+6;
  output.sonar15=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  v=30;
  process(v);
  output.sonar4=sonar(inputPin1,outputPin1)+6;
  output.sonar10=sonar(inputPin2,outputPin2)+6;
  output.sonar16=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  v=40;
  process(v);
  output.sonar5=sonar(inputPin1,outputPin1)+6;
  output.sonar11=sonar(inputPin2,outputPin2)+6;
  output.sonar17=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  v=50;
  process(v);
  output.sonar6=sonar(inputPin1,outputPin1)+6;
  output.sonar12=sonar(inputPin2,outputPin2)+6;
  output.sonar18=sonar(inputPin3,outputPin3)+6;
  
  delay(45);
  us.publish(&output);//这里output前加& 一定要注意!!
  }
void back()
{
  v=50;
  process(v);
  output.sonar6=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar12=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar18=sonar(inputPin3,outputPin3)+6;
  delay(10);
  v=40;
  process(v);
  output.sonar5=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar11=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar17=sonar(inputPin3,outputPin3)+6;
  delay(10);
  v=30;
  process(v);
  output.sonar4=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar10=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar16=sonar(inputPin3,outputPin3)+6;
  delay(10);
  v=20;
  process(v);
  output.sonar3=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar9=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar15=sonar(inputPin3,outputPin3)+6;
  delay(10);
  v=10;
  process(v);
  output.sonar2=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar8=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar14=sonar(inputPin3,outputPin3)+6;
  delay(10);
  v=0;
  process(v);
  output.sonar1=sonar(inputPin1,outputPin1)+6;
  delay(10);
  output.sonar7=sonar(inputPin2,outputPin2)+6;
  delay(10);
  output.sonar13=sonar(inputPin3,outputPin3)+6;
  delay(10);
  us.publish(&output);//这里output前加& 一定要注意!! 
  }

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(us);
  servo1.attach(10);//舵机连接到pin10.
  pinMode(inputPin1,INPUT);
  pinMode(outputPin1,OUTPUT);
  pinMode(inputPin2,INPUT);
  pinMode(outputPin2,OUTPUT);
  pinMode(inputPin3,INPUT);
  pinMode(outputPin3,OUTPUT);
  Serial.begin(9600);
  process(v);
  delay(500);
}
  
void loop() {
  come();
  back();
  nh.spinOnce();
}
