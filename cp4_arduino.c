#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <PID_v1.h>
#include <Servo.h>

#define TIME_DELAY (5)
#define OUTPUT_PERIOD (TIME_DELAY * 10)


/*-----Define pin(s)-----*/
// Left = motorA
const int motorA1 = 8;
const int motorA2 = 13; // 9 -> 13
const int motorAPWM = 5;
const int encoderA_A = 12;
const int encoderA_B = 3;
// Right = motorB
const int motorB1 = 10;
const int motorB2 = 11;
const int motorBPWM = 6;
const int encoderB_A = 4;
const int encoderB_B = 2;
// Light snesor
const int light = A0;
// Servo motor
const int servo_pin = 9;


/*-----Define variables-----*/
volatile long encoderAPos = 0;
volatile long encoderBPos = 0;
bool need_correction = false;
double encoderA_accumulation = 0.0;
double encoderB_accumulation = 0.0;
double distanceDifference = 0.0;
double last_distanceDifference = 0.0;
double speed_correction = 0.0;
double kc = 0.00003;
int output_counter = 0;


/*-----Define PID parameters-----*/
double Kpleft=30.0, Kileft=0.0, Kdleft=0.0;
double Kpright=32.9, Kiright=0.0, Kdright=0.0;
double Setleft=0.0, Setright=0.0, Inputleft=0.0, Inputright=0.0, Outputleft=0.0, Outputright=0.0;
PID PIDleft(&Inputleft, &Outputleft, &Setleft, Kpleft, Kileft, Kdleft, DIRECT);
PID PIDright(&Inputright, &Outputright, &Setright, Kpright, Kiright, Kdright, DIRECT);


/*-----Servo Define-----*/
// Establish Servo object
Servo myservo;


/*-----Rosserial Define-----*/
#define SUB_MAX (10) // Nodes
#define PUB_MAX (10) // Nodes
#define SUB_BUF (32) // Bytes
#define PUB_BUF (128) // Bytes
#define PUB_MSG_SIZE (3)
ros::NodeHandle_<ArduinoHardware, SUB_MAX, PUB_MAX, SUB_BUF, PUB_BUF> nh;

// control subscriber
void controlCallback(const std_msgs::Int16MultiArray& msg) {
  // Set left control
  Setleft = abs(msg.data[0]);
  // Set right control
  Setright = abs(msg.data[1]);

  // Set left motor direction
  if (msg.data[0] == 0) {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, HIGH);
  } else if (msg.data[0] > 0) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
  }
  // Set right motor direction
  if (msg.data[1] == 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, HIGH);
  } else if (msg.data[1] > 0) {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  }

  // If same speed need straight line correction
  if (msg.data[0] == msg.data[1] && msg.data[0] != 0) {
    need_correction = true;
  } else {
    need_correction = false;
  }

  // Reset cumulative distance
  encoderA_accumulation =0;
  encoderB_accumulation =0;
}
ros::Subscriber<std_msgs::Int16MultiArray> control_sub("control", controlCallback);

// servo subscriber
void servoCallback(const std_msgs::Int16MultiArray& msg) {
  if (msg.data[0]) {
    myservo.write(180);
  } else {
    myservo.write(35);
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> servo_sub("servo", servoCallback);

// output publisher
std_msgs::Int16MultiArray output_msg;
int16_t output_data[PUB_MSG_SIZE];
ros::Publisher output_pub("output", &output_msg);


void setup() {
  // Set motor as output
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorAPWM, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorBPWM, OUTPUT);

  // Set encoder pin as input
  pinMode(encoderA_A, INPUT);
  pinMode(encoderA_B, INPUT);
  pinMode(encoderB_A, INPUT);
  pinMode(encoderB_B, INPUT);

  // Set Servo pin
  myservo.attach(servo_pin);
  // Reset Servo
  myservo.write(35);

  // Set light sensor as input
  pinMode(light, INPUT);

  // Set ISR
  attachInterrupt(1, encoderAInt, CHANGE);
  attachInterrupt(0, encoderBInt, CHANGE);

  // Set PID mode
  PIDleft.SetMode(AUTOMATIC);
  PIDright.SetMode(AUTOMATIC);
  
  // Set PID limit
  PIDleft.SetOutputLimits(0, 255);
  PIDright.SetOutputLimits(0, 255);

  // Set rosserial
  nh.initNode();
  nh.subscribe(control_sub);
  nh.subscribe(servo_sub);
  output_msg.data_length = PUB_MSG_SIZE;
  nh.advertise(output_pub);
}


void loop() {
  // Read from encoder
  Inputleft = encoderAPos;
  encoderAPos = 0;
  Inputright = encoderBPos;
  encoderBPos = 0;

  // Calculate moving distance
  encoderA_accumulation += Inputleft;
  encoderB_accumulation += Inputright;

  // Straight line correction
  if (need_correction) {
    distanceDifference = abs(encoderA_accumulation - encoderB_accumulation);
    speed_correction = distanceDifference * kc;
    if (encoderA_accumulation > encoderB_accumulation) {
      if (distanceDifference > last_distanceDifference) {
        if (speed_correction > Setleft) speed_correction = Setleft;
        Setleft -= speed_correction;
        Setright += speed_correction;
      }
    } else if (encoderA_accumulation < encoderB_accumulation) {
      if (distanceDifference > last_distanceDifference) {
        if (speed_correction > Setright) speed_correction = Setright;
        Setleft += speed_correction;
        Setright -= speed_correction;
      }
    }
    last_distanceDifference = distanceDifference;
  }

  // Compute PID
  PIDleft.Compute();
  PIDright.Compute();

  // Motor PWM control
  analogWrite(motorAPWM, round(Outputleft));
  analogWrite(motorBPWM, round(Outputright));
  
  // Output system info for every OUTPUT_PERIOD of time
  if (output_counter == OUTPUT_PERIOD) {
    output_counter = 0;
    output_data[0] = analogRead(light);
    output_data[1] = encoderA_accumulation;
    output_data[2] = encoderB_accumulation;

    output_msg.data = output_data;
    output_pub.publish(&output_msg);
  }

  nh.spinOnce();

  output_counter += TIME_DELAY;
  delay(TIME_DELAY);
}

void encoderAInt()
{
  encoderAPos++;
}

void encoderBInt()
{
  encoderBPos++;
}