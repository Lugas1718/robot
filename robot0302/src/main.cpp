#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <time.h>

// =======================================================
// ROS2 OBJECTS & MESSAGES
// =======================================================
rcl_publisher_t pub_odom, pub_tf, pub_left, pub_right, pub_imu;
rcl_subscription_t sub_cmd_vel;
nav_msgs__msg__Odometry msg_odom;
tf2_msgs__msg__TFMessage msg_tf;
geometry_msgs__msg__TransformStamped tf_stamped;
geometry_msgs__msg__Twist msg_cmd_vel;
geometry_msgs__msg__Vector3 msg_L, msg_R;
sensor_msgs__msg__Imu msg_imu;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =======================================================
// PIN CONFIGURATION
// =======================================================
const int L_PIN_PWM = 5;    const int L_PIN_DIR = 19;   
const int R_PIN_PWM = 23;   const int R_PIN_DIR = 18;   
const int L_ENC_A = 25;     const int L_ENC_B = 26;   
const int R_ENC_A = 13;     const int R_ENC_B = 12;   

// =======================================================
// ROBOT PARAMETERS & PID GABUNGAN
// =======================================================
const float wheel_radius = 0.0375;   
const float wheelbase    = 0.19;    
const int REAL_PPR = 44; 
const float Ts_fixed = 0.02; // 50Hz

// Parameter PID dari versi terbaru kamu
float Kp = 0.8; 
float Ki = 0.5;
float Kd = 0.06; 
float Kff_L = 0.50; 
float Kff_R = 0.45; 

float integral_L = 0, integral_R = 0;
float last_err_L = 0, last_err_R = 0;
float target_rpm_L = 0, target_rpm_R = 0;
float smoothed_target_L = 0, smoothed_target_R = 0;
const float RAMP_STEP = 2.5; 

// Variabel Odometri
double odom_x = 0.0, odom_y = 0.0, odom_th = 0.0;
char odom_name[] = "odom";
char base_name[] = "base_footprint";
char imu_name[] = "imu_link";

volatile long left_count = 0; volatile long right_count = 0;
Adafruit_MPU6050 mpu;

// =======================================================
// ISR & CALLBACKS
// =======================================================
void IRAM_ATTR leftISR() { (digitalRead(L_ENC_B) == HIGH) ? left_count-- : left_count++; }
void IRAM_ATTR rightISR() { (digitalRead(R_ENC_B) == HIGH) ? right_count++ : right_count--; }

void cmdVelCb(const void * msgin) {
  const geometry_msgs__msg__Twist * data = (const geometry_msgs__msg__Twist *)msgin;
  float v_l = data->linear.x - (data->angular.z * (wheelbase / 2.0));
  float v_r = data->linear.x + (data->angular.z * (wheelbase / 2.0));
  const float MAX_RPM = 200.0; 
  target_rpm_L = constrain((v_l * 60.0) / (2.0 * PI * wheel_radius), -MAX_RPM, MAX_RPM);
  target_rpm_R = constrain((v_r * 60.0) / (2.0 * PI * wheel_radius), -MAX_RPM, MAX_RPM);
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(921600); 
  set_microros_transports();
  Wire.begin(21, 22); 
  if (mpu.begin()) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  }

  pinMode(L_PIN_DIR, OUTPUT); pinMode(L_PIN_PWM, OUTPUT);
  pinMode(R_PIN_DIR, OUTPUT); pinMode(R_PIN_PWM, OUTPUT);
  pinMode(L_ENC_A, INPUT_PULLUP); pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP); pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightISR, RISING);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot_node", "", &support);

  // Publisher Initialization
  rclc_publisher_init_best_effort(&pub_left, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "left_encoder");
  rclc_publisher_init_best_effort(&pub_right, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "right_encoder");
  rclc_publisher_init_best_effort(&pub_imu, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");
  rclc_publisher_init_default(&pub_odom, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  rclc_publisher_init_default(&pub_tf, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), "tf");
  
  // Subscriber Initialization
  rclc_subscription_init_default(&sub_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  rclc_executor_init(&executor, &support.context, 10, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel, &cmdVelCb, ON_NEW_DATA);

  // Setup Frame IDs
  msg_odom.header.frame_id.data = odom_name;
  msg_odom.header.frame_id.size = strlen(odom_name);
  msg_odom.header.frame_id.capacity = strlen(odom_name) + 1;
  msg_odom.child_frame_id.data = base_name;
  msg_odom.child_frame_id.size = strlen(base_name);
  msg_odom.child_frame_id.capacity = strlen(base_name) + 1;

  msg_tf.transforms.capacity = 1;
  msg_tf.transforms.size = 1;
  msg_tf.transforms.data = &tf_stamped;

  tf_stamped.header.frame_id.data = odom_name;
  tf_stamped.header.frame_id.size = strlen(odom_name);
  tf_stamped.header.frame_id.capacity = strlen(odom_name) + 1;
  
  tf_stamped.child_frame_id.data = base_name;
  tf_stamped.child_frame_id.size = strlen(base_name);
  tf_stamped.child_frame_id.capacity = strlen(base_name) + 1;
  
  msg_imu.header.frame_id.data = imu_name;
  msg_imu.header.frame_id.size = strlen(imu_name);
  msg_imu.header.frame_id.capacity = strlen(imu_name) + 1;
}

// =======================================================
// LOOP
// =======================================================
unsigned long prev_ms = 0;
long last_L = 0, last_R = 0;
float filtered_rpm_L = 0, filtered_rpm_R = 0;
int loop_counter = 0;

void loop() {
  unsigned long now = millis();
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

  if (now - prev_ms >= 20) {
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);

    sensors_event_t a, g_imu, temp;
    mpu.getEvent(&a, &g_imu, &temp);

    // 1. Ramping Target (PID Terbaru Logic)
    float current_ramp_L = (abs(target_rpm_L) < 1.0) ? 10.0 : RAMP_STEP; 
    float current_ramp_R = (abs(target_rpm_R) < 1.0) ? 10.0 : RAMP_STEP;

    if (smoothed_target_L < target_rpm_L) smoothed_target_L = min(smoothed_target_L + current_ramp_L, target_rpm_L);
    else if (smoothed_target_L > target_rpm_L) smoothed_target_L = max(smoothed_target_L - current_ramp_L, target_rpm_L);

    if (smoothed_target_R < target_rpm_R) smoothed_target_R = min(smoothed_target_R + current_ramp_R, target_rpm_R);
    else if (smoothed_target_R > target_rpm_R) smoothed_target_R = max(smoothed_target_R - current_ramp_R, target_rpm_R);

    // 2. Kalkulasi RPM
    noInterrupts();
    long cur_L = left_count; long cur_R = right_count;
    interrupts();

    float raw_rpm_L = ((float)(cur_L - last_L) / (float)REAL_PPR / Ts_fixed) * 60.0;
    float raw_rpm_R = ((float)(cur_R - last_R) / (float)REAL_PPR / Ts_fixed) * 60.0;
    last_L = cur_L; last_R = cur_R;
    
    // 3. Filter RPM
    filtered_rpm_L = (0.05 * raw_rpm_L) + (0.95 * filtered_rpm_L);
    filtered_rpm_R = (0.05 * raw_rpm_R) + (0.95 * filtered_rpm_R);

    // 4. PID Logic Gabungan
    float err_L = smoothed_target_L - filtered_rpm_L;
    float err_R = smoothed_target_R - filtered_rpm_R;

    integral_L = constrain(integral_L + (err_L * Ts_fixed), -100, 100);
    integral_R = constrain(integral_R + (err_R * Ts_fixed), -100, 100);

    float d_err_L = (err_L - last_err_L) / Ts_fixed;
    float d_err_R = (err_R - last_err_R) / Ts_fixed;
    last_err_L = err_L; last_err_R = err_R;

    float pwm_L_f = (smoothed_target_L * Kff_L) + (Kp * err_L) + (Ki * integral_L) + (Kd * d_err_L);
    float pwm_R_f = (smoothed_target_R * Kff_R) + (Kp * err_R) + (Ki * integral_R) + (Kd * d_err_R);

    // Motor Boost Logic
    if (abs(pwm_L_f) < 45 && abs(target_rpm_L) > 5) pwm_L_f += (target_rpm_L > 0 ? 25 : -25);
    if (abs(pwm_R_f) < 45 && abs(target_rpm_R) > 5) pwm_R_f += (target_rpm_R > 0 ? 25 : -25);

    int pwm_L = (int)constrain(pwm_L_f, -255, 255);
    int pwm_R = (int)constrain(pwm_R_f, -255, 255);

    if (abs(target_rpm_L) < 1.0 && abs(target_rpm_R) < 1.0) {
      pwm_L = 0; pwm_R = 0; integral_L = 0; integral_R = 0;
    }

    digitalWrite(L_PIN_DIR, pwm_L >= 0 ? HIGH : LOW);
    analogWrite(L_PIN_PWM, abs(pwm_L));
    digitalWrite(R_PIN_DIR, pwm_R >= 0 ? HIGH : LOW);
    analogWrite(R_PIN_PWM, abs(pwm_R));

    // 5. Odometri & TF
    float v_L_ms = (filtered_rpm_L * 2.0 * PI * wheel_radius) / 60.0;
    float v_R_ms = (filtered_rpm_R * 2.0 * PI * wheel_radius) / 60.0;
    float v_linear = (v_R_ms + v_L_ms) / 2.0;
    
    float gz = (abs(g_imu.gyro.z) > 0.01) ? g_imu.gyro.z : 0;
    odom_th += gz * Ts_fixed; 
    odom_x += (v_linear * cos(odom_th)) * Ts_fixed;
    odom_y += (v_linear * sin(odom_th)) * Ts_fixed;
    
    // 6. Publishing (Load Balancing)
    if (loop_counter % 2 == 0) {
      msg_imu.header.stamp.sec = tv.tv_sec;
      msg_imu.header.stamp.nanosec = tv.tv_nsec;
      msg_imu.angular_velocity.x = g_imu.gyro.x;
      msg_imu.angular_velocity.y = g_imu.gyro.y;
      msg_imu.angular_velocity.z = g_imu.gyro.z;
      msg_imu.linear_acceleration.x = a.acceleration.x;
      msg_imu.linear_acceleration.y = a.acceleration.y;
      msg_imu.linear_acceleration.z = a.acceleration.z;
      rcl_publish(&pub_imu, &msg_imu, NULL);
    } else {
      msg_odom.header.stamp.sec = tv.tv_sec;
      msg_odom.header.stamp.nanosec = tv.tv_nsec;
      msg_odom.pose.pose.position.x = odom_x;
      msg_odom.pose.pose.position.y = odom_y;
      msg_odom.pose.pose.orientation.z = sin(odom_th / 2.0);
      msg_odom.pose.pose.orientation.w = cos(odom_th / 2.0);
      rcl_publish(&pub_odom, &msg_odom, NULL);

      tf_stamped.header.stamp = msg_odom.header.stamp;
      tf_stamped.transform.translation.x = odom_x;
      tf_stamped.transform.translation.y = odom_y;
      tf_stamped.transform.rotation.z = msg_odom.pose.pose.orientation.z;
      tf_stamped.transform.rotation.w = msg_odom.pose.pose.orientation.w;
      rcl_publish(&pub_tf, &msg_tf, NULL);
    }
    
    // Debugging RPM
    msg_L.x = target_rpm_L; msg_L.y = (float)pwm_L; msg_L.z = filtered_rpm_L;
    msg_R.x = target_rpm_R; msg_R.y = (float)pwm_R; msg_R.z = filtered_rpm_R;
    rcl_publish(&pub_left, &msg_L, NULL);
    rcl_publish(&pub_right, &msg_R, NULL);

    loop_counter++;
    prev_ms = now;
  }
}