#include <IBusBM.h>
#include <ros.h>
#include <FastLED.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

#define  PI 3.14159
#define numLeds 10
#define ledPin 42                   // 53
CRGB leds[numLeds];

IBusBM ibusRC;
HardwareSerial& ibusRcSerial = Serial1;
HardwareSerial& debugSerial = Serial;

int channel1 = 0;                   // Declare channels
int channel2 = 0;
int channel3 = 0;
int channel4 = 0;
int channel5 = 0;
int channel8 = 0;

ros::NodeHandle nh;                 // Start the node

const float d = 0.25;               // d = Wheel diameter

float velLeft = 0;                  // Variables for right and left velocities
float velRight = 0;

const int leftMotor = 6;            // Define HALL pins
const int rightMotor = 7;
const int hallRightPin1 = A8;       // 51
const int hallRightPin2 = A9;       // 49
const int hallRightPin3 = A10;      // 47
const int hallLeftPin1 = A7;
const int hallLeftPin2 = A6;
const int hallLeftPin3 = A5;

const int rightBrake = 2;           // Define braking pins
const int leftBrake = 3;

const int rightReverse = A0;        // Define reversing pins
const int leftReverse = A1;

const float maxSpeed = 0.3;         // Adding a limit to our output

bool reached = false;               // LiDAR obstacle
bool camera_reached = false;        // Camera obstacle

float left_speed = 0.0;
float right_speed = 0.0;

void LeftVel(const std_msgs::Float32& value)
{
  left_speed = value.data;
}

void RightVel(const std_msgs::Float32& value)
{
  right_speed = value.data;
}

void LidarCallback(const std_msgs::Bool& value)
{
  reached = value.data;
}

void CameraCallback(const std_msgs::Bool& value)
{
  camera_reached = value.data;
}

void CmdVelCallback(const geometry_msgs::Twist& velocity)
{
  velLeft = left_speed;
  velRight = right_speed;
  //velLeft = velocity.linear.x - d * velocity.angular.z;
  //velRight = velocity.linear.x + d * velocity.angular.z;

  // Reverse and brake logic
  if (velRight < 0.0)
  {
    velRight = velRight * (-1);
    digitalWrite(rightReverse, LOW);
  }else
  {
    digitalWrite(rightReverse, HIGH);
  }

  if (velLeft < 0.0)
  {
    velLeft = velLeft * (-1);
    digitalWrite(leftReverse, LOW);
  }else
  {
    digitalWrite(leftReverse, HIGH);
  }

  BrakeLogic(velRight,velLeft);
  // Outputing the channels to ESC
}

std_msgs::String str_msg;
// Subscribers
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);
ros::Subscriber<std_msgs::Bool> lidar("obj_detection", LidarCallback);
ros::Subscriber<std_msgs::Bool> camera("camera_obj_detection", CameraCallback);
ros::Subscriber<std_msgs::Float32> left_vel("Left_vel", LeftVel);
ros::Subscriber<std_msgs::Float32> right_vel("Right_vel", RightVel);
ros::Publisher chatter("chatter", &str_msg);

void RemoteControl(float left, float right)
{
  // Reading certain channels
  channel3 = readChannel(2, -100, 100, 0);
  channel2 = readChannel(1, -100, 100, 0);
  channel1 = readChannel(0, -100, 100, 0);
  channel4 = readChannel(3, -100, 100, 0);
  channel5 = readChannel(4, -100, 100, 0);
  channel8 = readChannel(7, -100, 100, 0);
  // 100 = Teleoperation
  if (channel5 == 100)
  {
    analogWrite(leftMotor, left * maxSpeed);
    analogWrite(rightMotor, right * maxSpeed);
    // if (reached)
    // {
    //   Leds(1);
    //   //BrakeLogic(0.0, 0.0);
    // }else
    // {
    //   BrakeLogic(0.1, 0.1);
    //   Leds(3);
    // }
    if (camera_reached)
    {
      Leds(1);
      // BrakeLogic(0.0, 0.0);
    }else
    {
      // BrakeLogic(0.1, 0.1);
      Leds(3);
    }
  // 0 = Neutral
  }else if (channel5 == 0)
  {
    Leds(2);
    BrakeLogic(0.0, 0.0);
  }
  // -100 = Remote control
  else if (channel5 == -100)
  {
    velLeft = channel2 + channel1+1;
    velRight = channel2 - channel1+1;

    if (velRight < 0.0)
    {
      velRight = velRight * (-1);
      digitalWrite(rightReverse,LOW);
    }else
    {
      digitalWrite(rightReverse,HIGH);
    }
    if (velLeft < 0.0)
    {
      velLeft = velLeft * (-1);
      digitalWrite(leftReverse,LOW);
    }else
    {
      digitalWrite(leftReverse,HIGH);
    }
    BrakeLogic(velRight,velLeft);

    // Outputing the channels to ESC
    analogWrite(leftMotor, velLeft * maxSpeed);
    analogWrite(rightMotor, velRight * maxSpeed);
    Leds(4);
  }
}
void setup()
{
  // Put your setup code here, to run once:
  pinMode(hallRightPin1, INPUT);
  pinMode(hallRightPin2, INPUT);
  pinMode(hallRightPin3, INPUT);
  pinMode(hallLeftPin1, INPUT);
  pinMode(hallLeftPin2, INPUT);
  pinMode(hallLeftPin3, INPUT);
  pinMode(rightReverse, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightBrake, OUTPUT);
  pinMode(leftBrake, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  // Adding LEDstrip
  FastLED.addLeds<WS2812B, ledPin, GRB>(leds, numLeds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  // Initializing node and subscribing to it
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(lidar);
  nh.subscribe(camera);
  nh.subscribe(left_vel);
  nh.subscribe(right_vel);
  nh.advertise(chatter);
  // Starting IBUS
  debugSerial.begin(57600);
  ibusRC.begin(ibusRcSerial);
}

// Reading the channels
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = ibusRC.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
void loop()
{
  String str = String(left_speed);
  str_msg.data = str.c_str();
  RemoteControl(left_speed, right_speed);
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(10);
}