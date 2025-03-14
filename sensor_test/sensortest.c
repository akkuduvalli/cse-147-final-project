#include "sensortest.h"
#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <softPwm.h>
#include <sys/time.h>
#include <microhttpd.h>


void init_shared_variable(SharedVariable* sv) {
    sv->bProgramExit = 0;
    sv->Ax=0; sv->Ay=0; sv->Az=0; 
    sv->fd = wiringPiI2CSetup(Device_Address);
    sv->fd2 = wiringPiI2CSetup(Device_Address_ADC);
    sv->xpos = 2047; // max 
    sv->ypos = 2047; // max
    sv->autonomous = 0;
// You can initialize the shared variable if needed.
}

void ledInit(void) {
    
//......
//initialize SMD and DIP
	softPwmCreate(PIN_PWM_MOTOR, 0, 0xff);

    pinMode(PIN_AUTO_LED, OUTPUT);
    digitalWrite(PIN_AUTO_LED, LOW);
}

void init_sensors(SharedVariable* sv) {
// .......
    ledInit();
    MPU6050_Init(sv);		                 /* Initializes MPU6050 */
    ADC_Init(sv);

    pinMode(PIN_BUTTON, INPUT);
    pullUpDnControl(PIN_BUTTON, PUD_UP);  

	pinMode(PIN_TRIGGER, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIGGER, LOW);

	pinMode(PIN_DC_MOTOR1, OUTPUT);
    pinMode(PIN_DC_MOTOR0, OUTPUT);

    pinMode(PIN_DC_MOTOR1R, OUTPUT);
    pinMode(PIN_DC_MOTOR0R, OUTPUT);


}

void MPU6050_Init(SharedVariable* sv){
	
	wiringPiI2CWriteReg8 (sv->fd, SMPLRT_DIV, 0x07);	/*  sample rate register */
	wiringPiI2CWriteReg8 (sv->fd, PWR_MGMT_1, 0x01);	/*  to power management register */
	wiringPiI2CWriteReg8 (sv->fd, CONFIG, 0);		/* Configuration register */
	wiringPiI2CWriteReg8 (sv->fd, GYRO_CONFIG, 24);	/*  to Gyro Configuration register */
	wiringPiI2CWriteReg8 (sv->fd, INT_ENABLE, 0x01);	/* interrupt enable register */

}

int byteswap(int x) {
    return ((x << 8) & 0xFF00) | ((x >> 8) & 0x00FF);
}

void ADC_Init(SharedVariable* sv){
    // reminder: i2cget is in big endian
    sv->config = wiringPiI2CReadReg16 (sv->fd2, CONFIG_REG); // read the current configuration register (Big endian)
    sv->config = byteswap(sv->config); // bit swap before storing (little endian)
    sv->config = 0x8583; 
    printf("register 0x%x value 0x%x \n", CONFIG_REG, sv->config);
    // set programmable gain amplifier (set default)
    int gainRegister = CONFIG_PGA_2_048V; 
    sv->config &= ~CONFIG_PGA_MASK ;
    sv->config |= gainRegister ;
    printf("register value %x \n", sv->config);
    sv->config &= 0xFEFF;  // Set MODE = 0 (continuous mode)
    printf("register value %x \n", sv->config);
    //printf("byteswap register value %x \n", byteswap(sv->config));
    wiringPiI2CWriteReg16 (sv->fd2, CONFIG_REG, byteswap(sv->config));  // write new gain value to register (bits 9-11)
    delay(100);
}

long getMicrotime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000L + tv.tv_usec;
}

double measureDistance() {
    long startTime, stopTime;
    
    // trigger pulse
    digitalWrite(PIN_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIGGER, LOW);

    // wait for ECHO to go HIGH
    while (digitalRead(PIN_ECHO) == LOW);
    startTime = getMicrotime();

    // wait for ECHO to go LOW
    while (digitalRead(PIN_ECHO) == HIGH);
    stopTime = getMicrotime();

    //  distance in cm
    double distance = (stopTime - startTime) * 0.0343 / 2;

    // maybe in autonomous mode, stops when the distance is under a certain threshold. 
    return distance;
}

short read_raw_data(SharedVariable* sv, int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(sv->fd, addr);
	low_byte = wiringPiI2CReadReg8(sv->fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

// second arg is the pin A0, A1, A2, A3
int read_joystick_I2C(SharedVariable *sv, int apin) {
    // select the input channel
    // VRx A0 to GND: (0) 100 (0x4000)
    // VRy A1 to GND: (0) 101 (0x5000)
    uint16_t select = apin ? 0x4000 : 0x5000; // 1100 or 1101
    uint16_t temp = 0x8FFF & sv->config; // clear config[14:12]
    //printf("config before: 0x%x \n", sv->config);
    sv->config = temp | select;     // just change [14:12] to select (or-ing with 1 should'mnt change anything)
    //printf("selection for %d: 0x%x \n", apin, sv->config);
    wiringPiI2CWriteReg16 (sv->fd2, CONFIG_REG, byteswap(sv->config));  // set the input register bits
    delay(100);
    int value = wiringPiI2CReadReg16(sv->fd2, 0x00);
    value = byteswap(value);
    return value >> 4;  // bit shift because adc is 12 bit 
}


// 1. Button
void body_button(SharedVariable* sv) {
    //rintf("reading: %d\n",digitalRead(PIN_BUTTON));
    //delay(50);
    if (digitalRead(PIN_BUTTON) == LOW) { 
        if (digitalRead(PIN_AUTO_LED) == LOW)
            digitalWrite(PIN_AUTO_LED, HIGH);
        else 
            digitalWrite(PIN_AUTO_LED, LOW);

        sv->autonomous = !sv->autonomous;
        printf("button pressed!");
        delay(300);
    }
}

// 2. Accelerometer (GY-521) Sensor
void body_accelerometer(SharedVariable* sv) {
    /*read raw value of Accelerometer */
		sv->Acc_x = read_raw_data(sv, ACCEL_XOUT_H);
		sv->Acc_y = read_raw_data(sv, ACCEL_YOUT_H);
		sv->Acc_z = read_raw_data(sv, ACCEL_ZOUT_H);
		
		/* divide raw value by sensitivity scale factor */
		sv->Ax = sv->Acc_x/16384.0;
		sv->Ay = sv->Acc_y/16384.0;
		sv->Az = sv->Acc_z/16384.0;
		
		
		//printf("\n Ax=%.3f g\tAy=%.3f g\tAz=%.3f g\n",sv->Ax,sv->Ay,sv->Az);
		delay(500);
    
}

// 3. Ultrasonic Sensor
void body_ultrasonic(SharedVariable* sv) {
	double distance = measureDistance();
    //printf("Distance: %.2f cm\n", distance);
    sv->distance = distance;
	//delay(500);
}

// 4. Raspberry Pi Camera
void body_camera(SharedVariable* sv) {
    
}

// 5. Joystick
void body_joystick(SharedVariable* sv) {
    float VRx_val = read_joystick_I2C(sv, 0);
    float VRy_val = read_joystick_I2C(sv, 1);
    sv->xpos = VRx_val;
    sv->ypos = VRy_val;
    //printf("X value: %.2f, Y value: %.2f \n", VRx_val, VRy_val);
    delay(100);
}

// 6. DC Motor (GPIOs to the motor driver shields)
void body_dcmotor(SharedVariable* sv) {
    if (sv->autonomous) {
        if (sv->distance > 35) {
            forwards(0x50);
        }
        else {
            softPwmWrite (PIN_PWM_MOTOR, 0x00) ;
        }
    }
    else {
        // 2047 x 2047 base value, goes all the way down to 0
        float origin = 2047;
        int maxspeed = 0x100;
        // let the threshold of movement start at like 2000 2000
        int lower_threshold = 2000; // tune based on mappings
        int upper_threshold = 3000; // tune based on mappings
        if (sv->xpos < lower_threshold | sv->ypos < lower_threshold) {
            if (sv->xpos < sv->ypos) {
                // x movement, right
                int speed = (origin-sv->xpos)/origin * maxspeed; // lower xpos = higher speed
                right(speed);
            }
            else {
                int speed = (origin-sv->ypos)/origin * maxspeed; // lower xpos = higher speed
                forwards(speed);
            }
        }
        else if (sv->xpos > upper_threshold || sv->ypos > upper_threshold) {
            if (sv->xpos > sv->ypos) {
                // x movement, right
                int speed = (origin-sv->xpos)/origin * maxspeed; // lower xpos = higher speed
                left(speed);
            }
            else {
                int speed = (sv->ypos-origin)/origin * maxspeed; // lower xpos = higher speed
                backwards(speed);
            }
        }
        else {
            softPwmWrite (PIN_PWM_MOTOR, 0x00) ; // STOP
        }
    }
}

void backwards(int speed) {
    softPwmWrite (PIN_PWM_MOTOR, speed) ;
    digitalWrite(PIN_DC_MOTOR0, HIGH);
    digitalWrite(PIN_DC_MOTOR1, LOW);
    digitalWrite(PIN_DC_MOTOR0R, HIGH);
    digitalWrite(PIN_DC_MOTOR1R, LOW);
}

void forwards(int speed) {
    softPwmWrite (PIN_PWM_MOTOR, speed) ;
    digitalWrite(PIN_DC_MOTOR1, HIGH);
    digitalWrite(PIN_DC_MOTOR0, LOW);
    digitalWrite(PIN_DC_MOTOR1R, HIGH);
    digitalWrite(PIN_DC_MOTOR0R, LOW);
}

void left(int speed) {
    softPwmWrite (PIN_PWM_MOTOR, speed) ;
    digitalWrite(PIN_DC_MOTOR0, LOW);
    digitalWrite(PIN_DC_MOTOR1, HIGH);
    digitalWrite(PIN_DC_MOTOR0R, HIGH);
    digitalWrite(PIN_DC_MOTOR1R, LOW);
}

void right(int speed) {
    softPwmWrite (PIN_PWM_MOTOR, speed) ;
    digitalWrite(PIN_DC_MOTOR0, HIGH);
    digitalWrite(PIN_DC_MOTOR1, LOW);
    digitalWrite(PIN_DC_MOTOR0R, LOW);
    digitalWrite(PIN_DC_MOTOR1R, HIGH);
}

