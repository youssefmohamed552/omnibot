#include <micro_ros_arduino.h>

#include <stdio.h>
#include <math.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>

#include <dc_motor.h>
#include <hall_encoder.h>
#include <pid.h>



// DCMotor(dir1, dir2, speed)
// HallEncoder(sigA, sigB)

DCMotor FR_Motor(19, 20, 18);
HallEncoder FR_Encoder(2, 3);

DCMotor FL_Motor(21, 22, 23);
HallEncoder FL_Encoder(5, 4);

DCMotor BR_Motor(11, 12, 14);
HallEncoder BR_Encoder(6, 7);

DCMotor BL_Motor(16, 17, 15);
HallEncoder BL_Encoder(9, 8);


rcl_publisher_t odom_publisher;
rcl_subscription_t twist_subscriber;
nav_msgs__msg__Odometry odom_msg;
geometry_msgs__msg__Twist twist_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t omnibot_node;
rcl_timer_t odom_timer;
rcl_timer_t twist_timer;

const unsigned int odom_timer_timeout = 200;
volatile int64_t nano_sec_elapsed = 0;
//const int MOTOR_THRESHOLD = 130;



#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define NANO_TO_SEC(t) (double)t / 1000000000.0
//#define CAP(x) (x < MOTOR_THRESHOLD)? 0 : x 

const double l = 0.094;
volatile double lx = 0.05;
volatile double ly = 0.08;
volatile const double beta[4] = {M_PI / 2.0, -M_PI / 2.0, M_PI / 2.0, -M_PI / 2.0};
volatile const double alpha[4] = {1.01, -1.01, M_PI - 1.01, -M_PI + 1.01};
volatile const double r = 0.0385;
volatile const double l_squared = l * l;
volatile const double inv_scal = 1.0 / ((l_squared) + 1.0);


double FL_w, FR_w, BL_w, BR_w;
double FL_speed, FR_speed, BL_speed, BR_speed;
double FL_w_target, FR_w_target, BL_w_target, BR_w_target;
const double Kp = 15.0;
const double Ki = 2.0;
const double Kd = 0.0;

// PID(measured_signal, control_signal, set_point, Kp, Ki, Kd)
PID FL_pid(&FL_w, &FL_speed, &FL_w_target, Kp, Ki, Kd);
PID FR_pid(&FR_w, &FR_speed, &FR_w_target, Kp, Ki, Kd);
PID BL_pid(&BL_w, &BL_speed, &BL_w_target, Kp, Ki, Kd);
PID BR_pid(&BR_w, &BR_speed, &BR_w_target, Kp, Ki, Kd);






void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

geometry_msgs__msg__Quaternion yaw_to_quaternion(const double yaw){
  geometry_msgs__msg__Quaternion q;
  q.w = cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  return q;
}

double quaternion_to_yaw(geometry_msgs__msg__Quaternion& q){
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}

double round_val(double angle, const double limit){
  while(angle > limit){
    angle -= limit;
  }
  return angle;
}

void update_odom(const double omega[4], const double dt){
//  odom_msg.twist.twist.linear.x = 0.0;
//  odom_msg.twist.twist.linear.y = 0.0;
//  odom_msg.twist.twist.angular.z = 0.0;
//  for(int i = 0; i < 4; i++){
//    odom_msg.twist.twist.linear.x += ((l_squared * sin(beta[i])) - (l_squared * sin(-beta[i] + 2 * alpha[i])) + (2 * sin(beta[i]))) * omega[i];
//    odom_msg.twist.twist.linear.y += ((l_squared * cos(beta[i])) - (l_squared * cos(-beta[i] + 2 * alpha[i])) + (2 * cos(beta[i]))) * omega[i];
//    odom_msg.twist.twist.angular.z += cos(alpha[i] - beta[i]) * l * r;
//  }
//  odom_msg.twist.twist.linear.x *= (-0.5 * r * inv_scal);
//  odom_msg.twist.twist.linear.y *= ( 0.5 * r * inv_scal);
//  odom_msg.twist.twist.angular.z *= (l * r * inv_scal);
  odom_msg.twist.twist.linear.x  = ( omega[0] + omega[1] + omega[2] + omega[3]) * r * 0.25;
  odom_msg.twist.twist.linear.y  = (-omega[0] + omega[1] + omega[2] - omega[3]) * r * 0.25;
  odom_msg.twist.twist.angular.z = (-omega[0] + omega[1] - omega[2] + omega[3]) * r / (4 * (lx + ly));
  odom_msg.pose.pose.position.x += odom_msg.twist.twist.linear.x * dt;
  odom_msg.pose.pose.position.y += odom_msg.twist.twist.linear.y * dt;
  odom_msg.twist.covariance[0] = FL_Encoder.count();
  odom_msg.twist.covariance[1] = FR_Encoder.count();
  odom_msg.twist.covariance[2] = BL_Encoder.count();
  odom_msg.twist.covariance[3] = BR_Encoder.count();
  double new_theta = round_val(quaternion_to_yaw(odom_msg.pose.pose.orientation) + odom_msg.twist.twist.angular.z, 2 * M_PI);
  odom_msg.pose.pose.orientation = yaw_to_quaternion(new_theta);
}




void odom_timer_callback(rcl_timer_t* timer, int64_t last_call_time){
  if(timer != NULL){
    FL_w = FL_Encoder.rev_per_sec(NANO_TO_SEC(last_call_time));
    FR_w = FR_Encoder.rev_per_sec(NANO_TO_SEC(last_call_time));
    BL_w = BL_Encoder.rev_per_sec(NANO_TO_SEC(last_call_time));
    BR_w = BR_Encoder.rev_per_sec(NANO_TO_SEC(last_call_time));
    double omega[4] = {FL_w, FR_w, BL_w, BR_w};
    update_odom(omega, NANO_TO_SEC(last_call_time));
    nano_sec_elapsed += last_call_time;
    odom_msg.header.stamp.sec = NANO_TO_SEC(nano_sec_elapsed);
    odom_msg.header.stamp.nanosec = nano_sec_elapsed;
    odom_msg.twist.covariance[4] = FL_speed;
    odom_msg.twist.covariance[5] = FR_speed;
    odom_msg.twist.covariance[6] = BL_speed;
    odom_msg.twist.covariance[7] = BR_speed;
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    FR_Encoder.reset();
    FL_Encoder.reset();
    BL_Encoder.reset();
    BR_Encoder.reset();
  }
}

void drive_robot(const double vx, const double vy, const double w){
  FL_w_target = (1 / r) * (vx - vy - (lx + ly) * w);
  FR_w_target = (1 / r) * (vx + vy + (lx + ly) * w);
  BL_w_target = (1 / r) * (vx + vy - (lx + ly) * w);
  BR_w_target = (1 / r) * (vx - vy + (lx + ly) * w);

  // TODO: implement the pid to control the values
  
}


void twist_subscription_callback(const void* msgin){
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  double vx = msg->linear.x;
  double vy = msg->linear.y;
  double w = msg->angular.z;
  drive_robot(vx, vy, w);
}


void FR_ISR(){
  FR_Encoder.update_count();
}

void FL_ISR(){
  FL_Encoder.update_count();
}

void BR_ISR(){
  BR_Encoder.update_count();
}

void BL_ISR(){
  BL_Encoder.update_count();
}

const double TARGET = 0.5;

void setup() {
  // put your setup code here, to run once:
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);


  FL_pid.set_signal_limits(-255, 255);
  FR_pid.set_signal_limits(-255, 255);
  BL_pid.set_signal_limits(-255, 255);
  BR_pid.set_signal_limits(-255, 255);

  FL_pid.set_signal_offsets(-100, 100);
  FR_pid.set_signal_offsets(-100, 100);
  BL_pid.set_signal_offsets(-100, 100);
  BR_pid.set_signal_offsets(-100, 100);

  FL_pid.set_signal_thresholds(-20, 20);
  FR_pid.set_signal_thresholds(-20, 20);
  BL_pid.set_signal_thresholds(-20, 20);
  BR_pid.set_signal_thresholds(-20, 20);


  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0 , NULL, &allocator));

  RCCHECK(rclc_node_init_default(&omnibot_node, "omnibot_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &omnibot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  RCCHECK(rclc_subscription_init_default(
    &twist_subscriber,
    &omnibot_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_mux/input/navi"));


  RCCHECK(rclc_timer_init_default(
    &odom_timer,
    &support,
    RCL_MS_TO_NS(odom_timer_timeout),
    odom_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, & twist_subscriber, &twist_msg, &twist_subscription_callback, ON_NEW_DATA));
  
  attachInterrupt(digitalPinToInterrupt(FR_Encoder.pinINT()), FR_ISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(FL_Encoder.pinINT()), FL_ISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(BR_Encoder.pinINT()), BR_ISR, RISING); 
  attachInterrupt(digitalPinToInterrupt(BL_Encoder.pinINT()), BL_ISR, RISING); 

  

//  FR_Motor.drive(130);
//  FL_Motor.drive(255);
//  BR_Motor.drive(255);
//  BL_Motor.drive(255);
}

void loop() {
  // put your main code here, to run repeatedly:
  FL_pid.compute();
  FR_pid.compute();
  BL_pid.compute();
  BR_pid.compute();
  FL_Motor.drive((int)FL_speed);
  FR_Motor.drive((int)FR_speed);
  BL_Motor.drive((int)BL_speed);
  BR_Motor.drive((int)BR_speed);
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
