
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

// A. Pin number definitions 
// We use 8 sensors.
//
// 1. Button
#define PIN_BUTTON 0
#define PIN_DC_MOTOR1 4
#define PIN_DC_MOTOR0 5

#define PIN_DC_MOTOR1R 7
#define PIN_DC_MOTOR0R 6

#define PIN_PWM_MOTOR 1
#define PIN_AUTO_LED 29


// Ultrasonic
#define PIN_TRIGGER 3
#define PIN_ECHO 2

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/
#define Device_Address_ADC 0x48	/*Device Address/Identifier for ADS1015*/


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

#define VRx          0
#define VRy          1

#define DR_ADS101X_128      0 // data rate for ads1015

#define CONVERSION_REG   0x00
#define CONFIG_REG       0x01
#define LO_THRESH_REG    0x02
#define HI_THRESH_REG    0x03
#define CONTINUOUS_MODE_REG 0

#define	CONFIG_PGA_MASK		(0x0E00)
#define	CONFIG_PGA_2_048V	(0x0400)	// +/-2.048V range = Gain 2 (default)
#define	CONFIG_PGA_1_024V	(0x0600)	// +/-1.024V range = Gain 4

#define	CONFIG_DEFAULT		(0x8583)	// From the ADS1x15 datasheet




// B. Shared structure
// All thread functions get a shared variable of the structure
// as the function parameter.
// If needed, you can add anything in this structure.
typedef struct shared_variable {
    int bProgramExit; // Once set to 1, the program will terminate.
    float Acc_x,Acc_y,Acc_z;
	float Ax, Ay, Az;
    int fd;
    int fd2; 
    uint16_t config; // config register for ADC
    int xpos, ypos; // the joystick position
    double distance; // distance to nearest object
    int autonomous;
} SharedVariable;

// C. Functions
double measureDistance();
long getMicrotime();
void MPU6050_Init(SharedVariable* sv);
void ADC_Init(SharedVariable* sv);
short read_raw_data(SharedVariable* sv, int addr);
int read_joystick_I2C(SharedVariable *sv, int apin);

void left(int speed);
void forwards(int speed);
void backwards(int speed);
void right(int speed);

void init_shared_variable(SharedVariable* sv);
void init_sensors(SharedVariable* sv);
void body_button(SharedVariable* sv);     // Button
void body_accelerometer(SharedVariable* sv);     // Accelerometer (GY-521) sensor
void body_ultrasonic(SharedVariable* sv);      // Ultrasonic sensor
void body_camera(SharedVariable* sv);    // Raspberry Pi Camera
void body_joystick(SharedVariable* sv);   // joystick (i2C from ADS1015)
void body_dcmotor(SharedVariable* sv);   // DC motors

#endif
