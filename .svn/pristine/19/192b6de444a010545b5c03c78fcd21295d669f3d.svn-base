
#include <ros.h>
#include <rosserial_msgs/UltraSonic.h>

int inputPin1 = 22; //定义超声波信号接受口
int outputPin1 = 23; //定义超声波信号发出口,对超声波模块发指令
int inputPin2 = 24; //定义超声波信号接受口
int outputPin2 = 25; //定义超声波信号发出口,对超声波模块发指令
int inputPin3 = 26; //定义超声波信号接受口
int outputPin3 = 27; //定义超声波信号发出口,对超声波模块发指令
int inputPin4 = 28; //定义超声波信号接受口
int outputPin4 = 29; //定义超声波信号发出口,对超声波模块发指令
int inputPin5 = 30; //定义超声波信号接受口
int outputPin5 = 31; //定义超声波信号发出口,对超声波模块发指令
int inputPin6 = 32; //定义超声波信号接受口
int outputPin6 = 33; //定义超声波信号发出口,对超声波模块发指令
int inputPin7 = 34; //定义超声波信号接受口
int outputPin7 = 35; //定义超声波信号发出口,对超声波模块发指令
int inputPin8 = 36; //定义超声波信号接受口
int outputPin8 = 37; //定义超声波信号发出口,对超声波模块发指令
int inputPin9 = 38; //定义超声波信号接受口
int outputPin9 = 39; //定义超声波信号发出口,对超声波模块发指令
int inputPin10 = 40; //定义超声波信号接受口
int outputPin10 = 41; //定义超声波信号发出口,对超声波模块发指令
int inputPin11 = 42; //定义超声波信号接受口
int outputPin11 = 43; //定义超声波信号发出口,对超声波模块发指令
int inputPin12 = 44; //定义超声波信号接受口
int outputPin12 = 45; //定义超声波信号发出口,对超声波模块发指令
int inputPin13 = 46; //定义超声波信号接受口
int outputPin13 = 47; //定义超声波信号发出口,对超声波模块发指令

int16_t distance[13] = {0};
int index=0;
int dis_max=58*80;
ros::NodeHandle  nh;
rosserial_msgs::UltraSonic output;
ros::Publisher us("UltraSonic", &output);//这里output前加& 一定要注意!!

void sonar()
{
  //产生10us的脉冲
  digitalWrite(outputPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin1, HIGH);
  delayMicroseconds(10);
  //的量脉冲宽度
  distance[0] = pulseIn(inputPin1, HIGH,dis_max) / 58.8;
  //声速一般书29cm/us,因为是来回,所以乘以2\
  delay(5);
  digitalWrite(outputPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin2, HIGH);
  delayMicroseconds(10);
  distance[1] = pulseIn(inputPin2, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin3, HIGH);
  delayMicroseconds(10);
  distance[2] = pulseIn(inputPin3, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin4, HIGH);
  delayMicroseconds(10);
  distance[3] = pulseIn(inputPin4, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin5, HIGH);
  delayMicroseconds(10);
  distance[4] = pulseIn(inputPin5, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin6, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin6, HIGH);
  delayMicroseconds(10);
  distance[5] = pulseIn(inputPin6, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin7, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin7, HIGH);
  delayMicroseconds(10);
  distance[6] = pulseIn(inputPin7, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin8, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin8, HIGH);
  delayMicroseconds(10);
  distance[7] = pulseIn(inputPin8, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin9, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin9, HIGH);
  delayMicroseconds(10);
  distance[8] = pulseIn(inputPin9, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin10, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin10, HIGH);
  delayMicroseconds(10);
  distance[9] = pulseIn(inputPin10, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin11, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin11, HIGH);
  delayMicroseconds(10);
  distance[10] = pulseIn(inputPin11, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin12, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin12, HIGH);
  delayMicroseconds(10);
  distance[11] = pulseIn(inputPin12, HIGH,dis_max) / 58.8;
  delay(5);
  digitalWrite(outputPin13, LOW);
  delayMicroseconds(2);
  digitalWrite(outputPin13, HIGH);
  delayMicroseconds(10);
  distance[12] = pulseIn(inputPin13, HIGH,dis_max) / 58.8;
  delay(5);
}

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(us);
  pinMode(inputPin1, INPUT);
  pinMode(outputPin1, OUTPUT);
  pinMode(inputPin2, INPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(inputPin3, INPUT);
  pinMode(outputPin3, OUTPUT);
  pinMode(inputPin4, INPUT);
  pinMode(outputPin4, OUTPUT);
  pinMode(inputPin5, INPUT);
  pinMode(outputPin5, OUTPUT);
  pinMode(inputPin6, INPUT);
  pinMode(outputPin6, OUTPUT);
  pinMode(inputPin7, INPUT);
  pinMode(outputPin7, OUTPUT);
  pinMode(inputPin8, INPUT);
  pinMode(outputPin8, OUTPUT);
  pinMode(inputPin9, INPUT);
  pinMode(outputPin9, OUTPUT);
  pinMode(inputPin10, INPUT);
  pinMode(outputPin10, OUTPUT);
  pinMode(inputPin11, INPUT);
  pinMode(outputPin11, OUTPUT);
  pinMode(inputPin12, INPUT);
  pinMode(outputPin12, OUTPUT);
  pinMode(inputPin13, INPUT);
  pinMode(outputPin13, OUTPUT);
  delay(20);
}

void loop() {
  // put your main code here, to run repeatedly:
  sonar();
  for(index=0;index<13;index++)
  {
    if(distance[index]==0)distance[index]=30000; 
  }
  output.sonar1=distance[0]+19;
  output.sonar2=distance[1]+19;
  output.sonar3=distance[2]+19;
  output.sonar4=distance[3]+19;
  output.sonar5=distance[4]+19;
  output.sonar6=distance[5]+19;
  output.sonar7=distance[6]+19;
  output.sonar8=distance[7]+19;
  output.sonar9=distance[8]+19;
  output.sonar10=distance[9]+19;
  output.sonar11=distance[10]+19;
  output.sonar12=distance[11]+19;
  output.sonar13=distance[12]+19;
  us.publish(&output);//这里output前加& 一定要注意!!
  nh.spinOnce();
  //delay(50);
}
