#include "sensortest.h"
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>


void init_shared_variable(SharedVariable* sv) {
    sv->bProgramExit = 0;
    sv->Ax=0; sv->Ay=0; sv->Az=0; 
    sv->fd = wiringPiI2CSetup(Device_Address);
// You can initialize the shared variable if needed.
}

void ledInit(void) {
    
//......
//initialize SMD and DIP
}

void init_sensors(SharedVariable* sv) {
// .......
    ledInit();
    MPU6050_Init(sv);		                 /* Initializes MPU6050 */

    pinMode(PIN_BUTTON, INPUT);
    pullUpDnControl(PIN_BUTTON, PUD_UP);  

}

void MPU6050_Init(SharedVariable* sv){
	
	wiringPiI2CWriteReg8 (sv->fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	wiringPiI2CWriteReg8 (sv->fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	wiringPiI2CWriteReg8 (sv->fd, CONFIG, 0);		/* Write to Configuration register */
	wiringPiI2CWriteReg8 (sv->fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (sv->fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */

} 

short read_raw_data(SharedVariable* sv, int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(sv->fd, addr);
	low_byte = wiringPiI2CReadReg8(sv->fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}


// 1. Button
void body_button(SharedVariable* sv) {
    //printf("reading: %d\n",digitalRead(PIN_BUTTON));
    //delay(100);
    if (digitalRead(PIN_BUTTON) == LOW) { 
        printf("button pressed!");
        delay(300);
    }
}

// 2. Accelerometer (GY-521) Sensor
void body_accelerometer(SharedVariable* sv) {
    /*Read raw value of Accelerometer and gyroscope from MPU6050*/
		sv->Acc_x = read_raw_data(sv, ACCEL_XOUT_H);
		sv->Acc_y = read_raw_data(sv, ACCEL_YOUT_H);
		sv->Acc_z = read_raw_data(sv, ACCEL_ZOUT_H);
		
		/* Divide raw value by sensitivity scale factor */
		sv->Ax = sv->Acc_x/16384.0;
		sv->Ay = sv->Acc_y/16384.0;
		sv->Az = sv->Acc_z/16384.0;
		
		
		printf("\n Ax=%.3f g\tAy=%.3f g\tAz=%.3f g\n",sv->Ax,sv->Ay,sv->Az);
		delay(500);
    
}

// 3. Ultrasonic Sensor
void body_ultrasonic(SharedVariable* sv) {
    
}

// 4. Raspberry Pi Camera
void body_camera(SharedVariable* sv) {
    
}

// 5. Servo Motor
void body_servo(SharedVariable* sv) {
    
}

// 6. DC Motor (GPIOs to the motor driver shields)
void body_dcmotor(SharedVariable* sv) {
    
}


