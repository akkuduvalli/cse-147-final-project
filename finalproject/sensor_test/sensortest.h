
#ifndef _ASSIGNMENT_BODY_
#define _ASSIGNMENT_BODY_

#include <stdint.h>

// Macros
#define TURN_ON(pin) digitalWrite(pin, 1)
#define TURN_OFF(pin) digitalWrite(pin, 0)

#define READ(pin) digitalRead(pin)
#define WRITE(pin, x) digitalWrite(pin, x)

// Constants
#define RUNNING 3
#define PAUSE 2

#define LOW 0
#define HIGH 1

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

#define DETECT_SOUND 1
#define NO_SOUND 0

// A. Pin number definitions (DO NOT MODIFY)
// We use 8 sensors.
//
// 1. Button
#define PIN_BUTTON 0



// Ultrasonic
#define PIN_TRIGGER 3
#define PIN_ECHO 2

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

// B. Shared structure
// All thread functions get a shared variable of the structure
// as the function parameter.
// If needed, you can add anything in this structure.
typedef struct shared_variable {
    int bProgramExit; // Once set to 1, the program will terminate.
    float Acc_x,Acc_y,Acc_z;
	float Ax, Ay, Az;
    int fd;
} SharedVariable;

// C. Functions
// You need to implement the following functions.
// Do not change any function name here.
void MPU6050_Init(SharedVariable* sv);
short read_raw_data(SharedVariable* sv, int addr);
void init_shared_variable(SharedVariable* sv);
void init_sensors(SharedVariable* sv);
void body_button(SharedVariable* sv);     // Button
void body_accelerometer(SharedVariable* sv);     // Accelerometer (GY-521) sensor
void body_ultrasonic(SharedVariable* sv);      // Ultrasonic sensor
void body_camera(SharedVariable* sv);    // Raspberry Pi Camera
void body_servo(SharedVariable* sv);   // Servo motor
void body_dcmotor(SharedVariable* sv);   // DC motors

#endif
