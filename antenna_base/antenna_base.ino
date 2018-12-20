//Import required files
#include <ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <AltSoftSerial.h>
#include <AltSSPololuQik.h>

//AltSoftSerial uses Pin 8 for Rx and Pin 9 for Tx, and that is unchangeable because hardware interrupts. PWM Pin 10 cannot be used.
//Connect Arduino Rx to driver Tx , arduino Tx to driver Rx

const byte resetPin = 4;
bool timeout = false;
float imu = 0;
const byte SPEED = 60;

PololuQik2s12v10 arm(resetPin);

//Create a ROS node-handler to handle ROS stuff
ros::NodeHandle  nh;

std_msgs::Byte error_msg;

ros::Publisher error("error", &error_msg);

//Declare a reset function
void(* resetFunc) (void) = 0;

void setTimeOut(const std_msgs::Bool &timeout_data) {
  timeout = timeout_data.data;
}

void moveJoint(const std_msgs::Float32 &a) {
  float deg = a.data;
  if (imu < deg) {
    while (imu < deg && abs(imu - deg) < 0.5) {
      arm.setM0Speed(SPEED);
    }
    arm.setM0Speed(0);
  } else if (imu > deg) {
    while (imu > deg && abs(imu - deg) < 0.5) {
      arm.setM0Speed(-SPEED);
    }
    arm.setM0Speed(0);
  } else {
    arm.setM0Speed(0);
  }
}

void getAntennaHeading(const std_msgs::Float32 &i) {
  imu = i.data;
}

//Setup subscribers

ros::Subscriber<std_msgs::Float32> sub_degrees("base_bearing", &moveJoint);
ros::Subscriber<std_msgs::Bool> sub_timeout("timeout", &setTimeOut);
ros::Subscriber<std_msgs::Float32> sub_imu("IMU_base", &getAntennaHeading);


void setup()
{
  //Initiate the node & subscribers
  nh.initNode();
  nh.subscribe(sub_degrees);
  nh.subscribe(sub_timeout);
  nh.subscribe(sub_imu);
  //Initialize serial comms with the driver
  arm.init();
  delay(10);
  arm.setM1Speed(0);
}


void loop()

{
  if (!timeout) {
    nh.spinOnce();
  } else {
    arm.setM0Speed(0);
    arm.setM1Speed(0);
    delay(1000);
    resetFunc();
  }

}
