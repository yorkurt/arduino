//Import required files
#include <ros.h>
#include <std_msgs/Float32.h>
#include <AltSoftSerial.h>
#include <AltSSPololuQik.h>
#include <math.h> 

//AltSoftSerial uses Pin 8 for Rx and Pin 9 for Tx, and that is unchangeable because hardware interrupts. PWM Pin 10 cannot be used.
//Connect Arduino Rx to driver Tx , arduino Tx to driver Rx

const byte resetPin = 4;
float base_hdg = 0.0;
const byte SPEED = 60;
const float ERR = 0.5;

PololuQik2s12v10 arm(resetPin);

//Create a ROS node-handler to handle ROS stuff
ros::NodeHandle  nh;

void moveJoint(const std_msgs::Float32 &a) {
  float rover_hdg = a.data;

  float diff = getDifference(rover_hdg, base_hdg);

  if (diff > 0) {
    while (abs(base_hdg - rover_hdg) > ERR) {
      arm.setM0Speed(SPEED); // clockwise
    }
    arm.setM0Speed(0);
  } else if (diff < 0) {
    while (abs(base_hdg - rover_hdg) > ERR) {
      arm.setM0Speed(-SPEED); // counter-clockwise
    }
    arm.setM0Speed(0);
  } else {
    arm.setM0Speed(0);
  }
}

void getAntennaHeading(const std_msgs::Float32 &i) {
  base_hdg = i.data;
}

//Setup subscribers

ros::Subscriber<std_msgs::Float32> sub_rover_hdg("base_bearing", &moveJoint);
ros::Subscriber<std_msgs::Float32> sub_base_hdg("IMU_rover", &getAntennaHeading);


void setup()
{
  //Initiate the node & subscribers
  nh.initNode();
  nh.subscribe(sub_rover_hdg);
  nh.subscribe(sub_base_hdg);
  //Initialize serial comms with the driver
  arm.init();
  delay(10);
  arm.setM0Speed(0);
  arm.setM1Speed(0);
}


void loop()
{
  nh.spinOnce();
}

float getDifference(float a1, float a2) {
  return fmod((fmod(a1 - a2, 360) + 540), 360) - 180;
}
