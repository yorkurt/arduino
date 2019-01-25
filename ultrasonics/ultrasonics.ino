#include <ros.h>
#include <messages/ultrasonics.h>

// defines pins numbers
const byte trigPin1 = 2;
const byte echoPin1 = 3;
const byte trigPin2 = 4;
const byte echoPin2 = 5;
const byte trigPin3 = 6;
const byte echoPin3 = 7;

// defines variables
long duration;
float distance1;
float distance2;
float distance3;

// define period
const long PERIOD = 100;

//Create a ROS node-handler to handle ROS stuff
ros::NodeHandle  nh;
messages::ultrasonics msg;
ros::Publisher pub("ultrasonics", &msg);

void setup() {
  nh.initNode();
  nh.advertise(pub);
  pinMode(trigPin1, OUTPUT); // Sets the trigPin1 as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin1 as an Input
  pinMode(trigPin2, OUTPUT); // Sets the trigPin2 as an Output
  pinMode(echoPin2, INPUT); // Sets the echoPin2 as an Input
  pinMode(trigPin3, OUTPUT); // Sets the trigPin3 as an Output
  pinMode(echoPin3, INPUT); // Sets the echoPin3 as an Input
}

void loop() {
  long time_now = millis();
  
  // Clears the trigPin1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin1 on HIGH state for 10 micro seconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  
  // Reads the echoPin2, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin1, HIGH);
  
  // Calculating the distance
  distance1 = (float) duration * 0.034 / 2.0f;

  // Clears the trigPin2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin2 on HIGH state for 10 micro seconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  
  // Reads the echoPin2, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin2, HIGH);
  
  // Calculating the distance
  distance2 = (float) duration * 0.034 / 2.0f;

   // Clears the trigPin3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin3 on HIGH state for 10 micro seconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  
  // Reads the echoPin3, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin3, HIGH);
  
  // Calculating the distance
  distance3 = (float) duration * 0.034 / 2.0f;

  
  // Publishes to ROS
  msg.dist1 = distance1;
  msg.dist2 = distance2;
  msg.dist3 = distance3;
  pub.publish(&msg);

  nh.spinOnce();
  

  // Wait to publish at 10 Hz
  while(millis() < time_now + PERIOD){
  }
}
