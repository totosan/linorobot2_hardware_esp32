// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

#define LED_PIN 2
#define ESP32
#define SOC_PCNT_SUPPORTED
#define BAUDRATE 115200
#define LINO_BASE DIFFERENTIAL_DRIVE 

#define USE_GENERIC_2_IN_MOTOR_DRIVER 

#define USE_MPU6050_IMU
#define SDA_PIN 21 // specify I2C pins for esp32
#define SCL_PIN 22

// #define USE_LIDAR_UDP
// #define LIDAR_RXD 17 // you may use any available input pin
// // #define LIDAR_PWM 15  // do not use, the PWM control loop is not implememted yet
// #define LIDAR_SERIAL 1 // uart number, 1 or 2
// #define LIDAR_BAUDRATE 230400 // the Lidar serial buadrate
// #define LIDAR_SERVER { 192, 168, 1, 145 }  // eg your desktop IP addres
// #define LIDAR_PORT 8889 // the UDP port on server


#define K_P 3.5                             
#define K_I 2.8                            
#define K_D 3.0                            

#define MOTOR_MAX_RPM 100       
#define MAX_RPM_RATIO 0.95
#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12            
#define COUNTS_PER_REV1 1152
#define COUNTS_PER_REV2 1107
#define COUNTS_PER_REV3 620
#define COUNTS_PER_REV4 620
#define WHEEL_DIAMETER 0.0675               
#define LR_WHEELS_DISTANCE 0.237            
#define PWM_BITS 12          
#define PWM_FREQUENCY 16000

#define MOTOR2_ENCODER_INV false
#define MOTOR1_ENCODER_INV false
#define MOTOR3_ENCODER_INV false 
#define MOTOR4_ENCODER_INV false 

#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

// Fixed pin numbers for ESP32-WROOM-32D 38 PIN VERSION
/// ENCODER PINS

#define MOTOR1_ENCODER_A 35
#define MOTOR1_ENCODER_B 34 

#define MOTOR2_ENCODER_A 18 
#define MOTOR2_ENCODER_B 19 

#define MOTOR3_ENCODER_A 17
#define MOTOR3_ENCODER_B 16 

#define MOTOR4_ENCODER_A 1
#define MOTOR4_ENCODER_B 3

// Motor Pins
#define MOTOR1_PWM 12
#define MOTOR1_IN_A 26
#define MOTOR1_IN_B 25 

#define MOTOR2_PWM 13
#define MOTOR2_IN_A 14
#define MOTOR2_IN_B 27 

#define MOTOR3_PWM 32
#define MOTOR3_IN_A 33
#define MOTOR3_IN_B 15

#define MOTOR4_PWM 34
#define MOTOR4_IN_A 35
#define MOTOR4_IN_B 36

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)
//#define USE_WIFI_TRANSPORT
#define AGENT_IP { 192, 168, 0, 179 }  // eg IP of the desktop computer
#define AGENT_PORT 8888
// Enable WiFi with null terminated list of multiple APs SSID and password
#define WIFI_AP_LIST {{"UPC0678289", "x24_$5N9?QxT"}, {NULL}}
#define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
//#define USE_ARDUINO_OTA
#define USE_SYSLOG
#define SYSLOG_SERVER { 192, 168, 0, 171 }  // eg IP of the desktop computer
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "linorobot2"
#define APP_NAME "hardware"
#define USE_SHORT_BRAKE
#endif
