

#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_base/UpdateGains.h>
#include <ros_arduino_msgs/Encoders.h>
#include <ros_arduino_msgs/CmdDiffVel.h>

/********************************************************************************************
/                                                     USER CONFIG                           *
/********************************************************************************************/

// Select your baud rate here
#define BAUD 115200

// Select your motor driver here
#define Sabertooth

// Define your encoder pins here.
// Try to use pins that have interrupts
// Left side encoders pins
#define LEFT_ENCODER_A 20  // Interrupt on Mega ADK
#define LEFT_ENCODER_B 21  // Interrupt on Mega ADK
// Right side encoders pins
#define RIGHT_ENCODER_A 2  // Interrupt on Mega ADK
#define RIGHT_ENCODER_B 3  // Interrupt on Mega ADK 

/********************************************************************************************
/                                                 END OF USER CONFIG                        *
/********************************************************************************************/

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <USBSabertooth.h>

typedef struct {
  float desired_velocity;     // [m/s]
  uint32_t current_time;      // [milliseconds]
  uint32_t previous_time;     // [milliseconds]
  int32_t current_encoder;    // [counts]
  int32_t previous_encoder;   // [counts]
  float previous_error;       // 
  float total_error;          // 
  int16_t command;            // [PWM]
}
ControlData;

USBSabertoothSerial C(Serial3);    // Use SWSerial as the serial port.
USBSabertooth ST(C, 128);     // Use address 128.

// Encoder objects from PJRC encoder library.
Encoder left_encoder(LEFT_ENCODER_A,LEFT_ENCODER_B);
Encoder right_encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B);

// Vehicle characteristics
float counts_per_rev[1];
float wheel_radius[1];         // [m]
float meters_per_counts;       // [m/counts]
int pwm_range[1];

// Gains;
float pid_gains_left[3];
float pid_gains_right[3];
float Kp_l, Ki_l, Kd_l;
float Kp_r, Ki_r, Kd_r;

// Structures containing PID data
ControlData left_motor_controller;
ControlData right_motor_controller;

// Control methods prototypes
void updateControl(ControlData * ctrl, int32_t encoder_reading);
void doControl(ControlData * ctrl);
void Control();

char frame_id[] = "base_footprint";

int control_rate[1];   // [Hz]
int encoder_rate[1];   // [Hz]
int no_cmd_timeout[1]; // [seconds]

uint32_t up_time;             // [milliseconds]
uint32_t last_encoders_time;  // [milliseconds]
uint32_t last_cmd_time;       // [milliseconds]
uint32_t last_control_time;   // [milliseconds]
uint32_t last_status_time;    // [milliseconds]


// ROS node
ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> nh;

// ROS subribers/service callbacks prototye
void cmdDiffVelCallback(const ros_arduino_msgs::CmdDiffVel& diff_vel_msg); 

// ROS subsribers
ros::Subscriber<ros_arduino_msgs::CmdDiffVel> sub_diff_vel("cmd_diff_vel", cmdDiffVelCallback);

// ROS services prototype
void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res);
// ROS services
ros::ServiceServer<ros_arduino_base::UpdateGains::Request, ros_arduino_base::UpdateGains::Response> update_gains_server("update_gains", &updateGainsCb);

// ROS publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
// ROS publishers
ros::Publisher pub_encoders("encoders", &encoders_msg);


void setup() 
{ 
  // Set the node handle
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  //left_motor_controller.desired_velocity =0.005;
  //right_motor_controller.desired_velocity = 0.005;

  encoders_msg.header.frame_id = frame_id;
  // Pub/Sub
  nh.advertise(pub_encoders);
  nh.subscribe(sub_diff_vel);
  nh.advertiseService(update_gains_server);
  
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller.");
  
  // Load parameters 
  loadParams();

  // Initialize the motors
  Serial3.begin(9600);
  ST.setRamping(-16383);
} 


void loop() 
{
  if ((millis() - last_encoders_time) >= (1000 / encoder_rate[0]))
  { 
    encoders_msg.left = left_encoder.read();
    encoders_msg.right = right_encoder.read();
    encoders_msg.header.stamp = nh.now();
    pub_encoders.publish(&encoders_msg);
    last_encoders_time = millis();
  }
  /*if ((millis()) - last_control_time >= (1000 / control_rate[0]))
  {
    Control();
    last_control_time = millis();
  }*/
  int cmd1=int(left_motor_controller.desired_velocity/0.0025);
  int cmd2=int(right_motor_controller.desired_velocity/0.0025);
  ST.motor(1, cmd1);//Right
  ST.motor(2, cmd2);//Left

  // Stop motors after a period of no commands
  /*if((millis() - last_cmd_time) >= (no_cmd_timeout[0] * 1000))
  {
    left_motor_controller.desired_velocity = 0.0;
    right_motor_controller.desired_velocity = 0.0;
  }*/
  nh.spinOnce();
}


void cmdDiffVelCallback( const ros_arduino_msgs::CmdDiffVel& diff_vel_msg) 
{
  left_motor_controller.desired_velocity = diff_vel_msg.left;
  right_motor_controller.desired_velocity = diff_vel_msg.right;
  last_cmd_time = millis();
}

void updateControl(ControlData * ctrl, int32_t encoder_reading)
{
  ctrl->current_encoder = encoder_reading;
  ctrl->current_time = millis();;
}

void doControl(ControlData * ctrl, bool isLeft)
{
  float estimated_velocity = meters_per_counts * (ctrl->current_encoder - ctrl->previous_encoder) * 1000.0 / (ctrl->current_time - ctrl->previous_time);
  float error = ctrl->desired_velocity - estimated_velocity;
  float cmd;
  
  if (isLeft) 
  {
    cmd = Kp_l * error + Ki_l * (error + ctrl->total_error) + Kd_l * (error - ctrl->previous_error);
  }
  else 
  {
  cmd = Kp_r * error + Ki_r * (error + ctrl->total_error) + Kd_r * (error - ctrl->previous_error);
  }
  
  cmd += ctrl->command;
  
  if(cmd >= pwm_range[0])
  {
    cmd = pwm_range[0];
  }
  else if (cmd <= -pwm_range[0])
  {
    cmd = -pwm_range[0];
  }
  else
  { 
    ctrl->total_error += error;
  }

  ctrl->command = cmd;
  ctrl->previous_time = ctrl->current_time;
  ctrl->previous_encoder = ctrl->current_encoder;
  ctrl->previous_error = error;

}

void Control()
{
  updateControl(&left_motor_controller, left_encoder.read());
  updateControl(&right_motor_controller, right_encoder.read());

  doControl(&left_motor_controller, false);
  doControl(&right_motor_controller, true);
  
  commandLeftMotor(left_motor_controller.command);
  commandRightMotor(right_motor_controller.command);
  
}

void updateGainsCb(const ros_arduino_base::UpdateGains::Request & req, ros_arduino_base::UpdateGains::Response & res)
{
  for ( int x = 0; x < 3; x++)
  {
    pid_gains_left[x] = req.gains[x];
    pid_gains_right[x] = req.gains[x+3];
  }
  
  Kp_l = pid_gains_left[0];
  Ki_l = pid_gains_left[1] / control_rate[0];
  Kd_l = pid_gains_left[2] * control_rate[0];
  
  Kp_r = pid_gains_right[0];
  Ki_r = pid_gains_right[1] / control_rate[0];
  Kd_r = pid_gains_right[2] * control_rate[0];
}

void loadParams() {
  /*nh.getParam("control_rate", control_rate,1);
  nh.getParam("encoder_rate", encoder_rate,1);
  nh.getParam("no_cmd_timeout", no_cmd_timeout,1);;
  nh.getParam("counts_per_rev", counts_per_rev,1);
  nh.getParam("wheel_radius", wheel_radius,1);
  nh.getParam("pwm_range", pwm_range,1);*/
  wheel_radius[0] =  0.0635 ;
  counts_per_rev[0]= 2400;
  
  control_rate[0]=50;
  encoder_rate[0] =50;
  pwm_range[0] =1023;
  no_cmd_timeout[0] =1;
  
  // Compute the meters per count
  meters_per_counts = ((PI * 2 * wheel_radius[0]) / counts_per_rev[0]);
}
void commandRightMotor(int16_t cmd) {
 Serial.print(-cmd);
 Serial.print("\t");
 
  ST.motor(1, -cmd); 
  delay(500);
   
}

void commandLeftMotor(int16_t cmd) {
  Serial.println(-cmd);
  ST.motor(2, -cmd);
  delay(500);
}



