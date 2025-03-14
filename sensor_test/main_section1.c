// thread management for sensor interaction of 
// Mini Mars Rover 

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <wiringPi.h>
#include <softPwm.h>
#include "sensortest.h"
#include <microhttpd.h>
#include <string.h>
#include <signal.h>


// Thread declaration macros
#define thread_decl(NAME) \
void* thread_##NAME(void* param) { \
	SharedVariable* pV = (SharedVariable*) param; \
	body_##NAME(pV); \
	return NULL; }

#define PORT 8080

volatile sig_atomic_t stop_server = 0;
SharedVariable v;

void handle_sigint(int sig) {
    stop_server = 1;
}

// handle HTTP requests
static int request_handler(void *cls, struct MHD_Connection *connection, 
                           const char *url, const char *method, 
                           const char *version, const char *upload_data, 
                           size_t *upload_data_size, void **ptr) {
    static int dummy;
    if (&dummy != *ptr) {
        *ptr = &dummy;
        return MHD_YES;
    }
	char response[512];
	if (strstr(url, "/direction") != NULL) {
		// parse request for direction set and speed (speed currently not used)
		const char *direction_str = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, "dir");
		const char *speed_str = MHD_lookup_connection_value(connection, MHD_GET_ARGUMENT_KIND, "speed");
	
		int direction, speed;
	
		if (direction_str && speed_str) {
			direction = atoi(direction_str);
			//speed = atoi(speed_str);
			speed = 0x80;
			// set the speed and run the car
			// 0: forward, 1: backward, 2: left, 3: right
			switch (direction) {
				case 0: forwards(speed);
				break;
				case 1: backwards(speed);
				break;
				case 2: left(speed);
				break;
				case 3: right(speed);
			}
			delay(200);
			softPwmWrite (PIN_PWM_MOTOR, 0x00) ;
		
			snprintf(response, sizeof(response), "Direction %d set to speed %d\n", direction, speed);
		} else {
			snprintf(response, sizeof(response), "Usage: /direction?dir=<num>&speed=<num>\n");
		}
	}
	else if (strstr(url, "/status") != NULL) {
		// acceleration data for website
		float X,Y,Z;
		X = read_raw_data(&v, ACCEL_XOUT_H)/16384.0;
		Y = read_raw_data(&v, ACCEL_YOUT_H)/16384.0;
		Z = read_raw_data(&v, ACCEL_ZOUT_H)/16384.0;
		//printf("{\"X\": %f, \"Y\": %f,\"Z\": %f}\n", X, Y, Z); 
		snprintf(response, sizeof(response), "{\"X\": %f, \"Y\": %f,\"Z\": %f}", X, Y, Z); // send acceleration
	}
	else {
		snprintf(response, sizeof(response), "Invalid request\n");
	}
    
	//printf(response);
	//printf("\n\n\n");
    struct MHD_Response *mhd_response = MHD_create_response_from_buffer(strlen(response),
                                          (void *)response, MHD_RESPMEM_MUST_COPY);
	MHD_add_response_header(mhd_response, "Access-Control-Allow-Origin", "*"); // allows reading response from the web server 
    int ret = MHD_queue_response(connection, MHD_HTTP_OK, mhd_response);
    MHD_destroy_response(mhd_response);
	/*if (ret == MHD_NO) {
        printf("Failed to queue response\n");
    } else {
        printf("Response sent successfully\n");
    }*/

    return ret;
}

// Declare threads for each sensor/actuator function
thread_decl(button);
thread_decl(accelerometer);
thread_decl(ultrasonic);
thread_decl(camera);
thread_decl(joystick);
thread_decl(dcmotor);

// Thread creation and joining macros
#define thread_create(NAME) pthread_create(&t_##NAME, NULL, thread_##NAME, &v);
#define thread_join(NAME) pthread_join(t_##NAME, NULL);



int main(int argc, char* argv[]) {
	// Initialize shared variable
	

	// Initialize WiringPi library
	if (wiringPiSetup() == -1) {
		printf("Failed to setup wiringPi.\n");
		return 1; 
	}

	// Initialize shared variable and sensors
	init_shared_variable(&v);
	init_sensors(&v);

	// Thread identifiers
	pthread_t t_button,
			  t_accelerometer,
			  t_ultrasonic,
			  t_camera,
			  t_joystick,
			  t_dcmotor;

	// Main program loop
	signal(SIGINT, handle_sigint); // Capture Ctrl+C
	struct MHD_Daemon *server = MHD_start_daemon(MHD_USE_INTERNAL_POLLING_THREAD, PORT, NULL, NULL,
		&request_handler, NULL, MHD_OPTION_END);
	if (!server) {
		fprintf(stderr, "Failed to start server\n");
		return 1;
	}

	printf("Server running on port %d...\n", PORT);
	getchar();  // Wait for user input to exit

	while (v.bProgramExit != 1 && !stop_server) {
		// Create sensing threads
		thread_create(button);
		thread_create(accelerometer);
		thread_create(ultrasonic);
		thread_create(camera);
		thread_create(joystick);
		thread_create(dcmotor);

		// Wait for all threads to finish
		thread_join(button);
		thread_join(accelerometer);
		thread_join(ultrasonic);
		thread_join(camera);
		thread_join(joystick);
		thread_join(dcmotor);

		// Add a slight delay between iterations
		delay(10);
	}
	softPwmWrite (PIN_PWM_MOTOR, 0x00) ;
	MHD_stop_daemon(server); // stop server on program terminate
	printf("Program finished.\n");

	return 0;
}
