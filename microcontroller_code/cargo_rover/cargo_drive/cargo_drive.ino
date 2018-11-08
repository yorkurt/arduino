//Import required files
#include <ros.h>
#include <joysticks/drive.h>
#include <std_msgs/Bool.h>
#include <AltSoftSerial.h>

//Create a ROS node-handler to handle ROS stuff
ros::NodeHandle  nh;

bool timeout = false;

const byte left_PWM = 3;
const byte left_DIR = 4;

AltSoftSerial right_drv;
//AltSoftSerial uses Pin 8 for Rx and Pin 9 for Tx, and that is unchangeable because hardware interrupts. PWM Pin 10 cannot be used.
//Connect Arduino Rx to driver Tx , arduino Tx to driver Rx

//Reset function
void(* resetFunc) (void) = 0;

//Code for when a drive message is received
void spinWheel(const joysticks::drive &drive) {
  
  right_drv.write(255); //As we are using the MiniSCC protocol, we transmit this byte.
  right_drv.write(1); //Next, we specify what controller we need to write to. Our right driver is 1.
  right_drv.write((drive.right / 2) + 127); //Convert our speed to controller speed 

  if(drive.left > 0) {
    digitalWrite(left_DIR, HIGH);
    analogWrite(left_PWM, drive.left);
    
  } else if (drive.left < 0) {
    
    digitalWrite(left_DIR, LOW);
    analogWrite(left_PWM, - drive.left);
  } 
  
}

void setTimeOut(const std_msgs::Bool &timeout_data) {
  timeout = timeout_data.data;
}

//Setup subscriber to joystick drive node & timeout
ros::Subscriber<joysticks::drive> sub_drive("drive", &spinWheel);
ros::Subscriber<std_msgs::Bool> sub_timeout("timeout", &setTimeOut);


void setup()
{
  pinMode(left_DIR, OUTPUT);
  pinMode(left_PWM, OUTPUT);
  //Initialize serial comms with the left driver
  right_drv.begin(38400);

  //Initiate the node & subscribers
  nh.initNode();
  nh.subscribe(sub_drive);
  nh.subscribe(sub_timeout);
  //Configure auto-baudrate
  right_drv.write(0xAA);
  delay(2); //Wait for a little bit
  right_drv.write(0x83); //Exit safe-start mode

}


void loop()

{
  if (!timeout) {
    nh.spinOnce();
  } else {
  
    digitalWrite(left_DIR, HIGH);
    digitalWrite(left_PWM, 0);
    right_drv.write(255);
    right_drv.write(1);
    right_drv.write(byte (0));
    right_drv.end();
    delay(1000);
    resetFunc(); //Commit seppuku
  }


}



