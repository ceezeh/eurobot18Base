/*
 * MD49.c
 *
 * MD49 example code for raspberry pi.
 * Runs the motors to an encoder value and back while displaying the encoder values on the screen.
 *
 * By James Henderson, 2017
 *
 */

#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
using namespace std;

int writeBytes(int descriptor, int count);
int readBytes(int descriptor, int count);
void displaySoftwareValue(int fd);
void setMode(int fd, char mode);
void driveMotors(int fd, char speed);
void resetEncoders(int fd);
void displayDecoderValues(int fd);
void displayMode(int fd);

char serialBuffer[10];					// Serial buffer sto store data for I/O

union encoder {									// To store the encoder values
	int value;
	char bytes[8];
} encoder1, encoder2;

int main(int argc, char **argv) {
	ros::init(argc, argv, "wheel_odometry");
	ros::NodeHandle nh;

	int fd;							// File descriptor of port we will talk to
	char *portName = "/dev/ttyS0";	// Name of the UART port on the Raspberry pi
	struct termios options;								// Port options

	fd = open(portName, O_RDWR | O_NOCTTY);	// Open port for read and write not making it a controlling terminal
	if (fd == -1) {
		perror("openPort: Unable to open port ");// If open() returns an error
	}
	tcgetattr(fd, &options);
	cfsetispeed(&options, B9600);						// Set baud rate
	cfsetospeed(&options, B9600);
	cfmakeraw(&options);
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &options);

	usleep(10000);				// Sleep for UART to power up and set options

	displaySoftwareValue(fd);		// Display the software version of the MD49

	setMode(fd, 1);								// Set the mode of the MD49 to 1

	displayMode(fd);

	resetEncoders(fd);							// Reset the encode values to 0

	displayDecoderValues(fd);
	ros::Rate loop_rate(5);
	do {							// Run motors forward to an encoder value
		driveMotors(fd, 100);
		displayDecoderValues(fd);
		loop_rate.sleep();
	} while (encoder1.value < 0x4000 && ros::ok());

	do {												// Run motors backwards
		driveMotors(fd, -100);
		displayDecoderValues(fd);
		loop_rate.sleep();
	} while (encoder1.value > 0 && ros::ok());

	driveMotors(fd, 0);								// Stop motors before exit

	close(fd);											// Close port
	printf("\n");
	return 0;
}

int writeBytes(int descriptor, int count) {
	int ret = write(descriptor, serialBuffer, count);
	if (ret == -1) {		// Send data out
		perror("Error writing");
		close(descriptor);					// Close port if there is an error
		exit(1);
	}
	return ret;
}

int readBytes(int descriptor, int count) {
	int ret = read(descriptor, serialBuffer, count);
	if (ret == -1) {					// Read back data into buf[]
		perror("Error reading ");
		close(descriptor);					// Close port if there is an error
		exit(1);
	}
	return ret;
}

void displaySoftwareValue(int fd) {
	serialBuffer[0] = 0;									// Sync byte of 0
	serialBuffer[1] = 0x29;					// Command to get software version

	writeBytes(fd, 2);
	readBytes(fd, 1);

//	cout << "MD49 Software v: " << serialBuffer[0] << endl;	// display the software version
	printf("MD49 Software v: %d \n", serialBuffer[0]);

}

void displayMode(int fd) {
	serialBuffer[0] = 0;									// Sync byte of 0
	serialBuffer[1] = 0x2B;					// Command to get software version

	writeBytes(fd, 2);
	readBytes(fd, 1);

	printf("MD49 Mode v: %d \n", serialBuffer[0]);// display the software version
}

void setMode(int fd, char mode) {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x34;								// Command to set mode
	serialBuffer[2] = mode; 							// Mode we wish to set

	writeBytes(fd, 3);

}

void driveMotors(int fd, char speed) {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x31;						// Command to set motor speed
	serialBuffer[2] = speed;
	writeBytes(fd, 3);										// Speed to be set
	usleep(10000);

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x32;
	serialBuffer[2] = speed;

	writeBytes(fd, 3);
}

void resetEncoders(int fd) {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x35;					// Command to reset encoder values

	writeBytes(fd, 2);
	encoder1.value = 0;
	encoder2.value = 0;
}

void displayDecoderValues(int fd) {



	serialBuffer[0] = 0;
	serialBuffer[1] = 0x25;					// Command to return encoder values

	writeBytes(fd, 2);
	readBytes(fd, 8);
	encoder1.bytes[0] = serialBuffer[3];
	encoder1.bytes[1] = serialBuffer[2];
	encoder1.bytes[2] = serialBuffer[1];
	encoder1.bytes[3] = serialBuffer[0];
//	encoder1.value = (int(serialBuffer[0]) << 3) + (int(serialBuffer[1]) << 2)
//			+(int(serialBuffer[2]) << 1)+ int(serialBuffer[3]);
//	cout << "encoder 1: " << encoder1.value << endl;				// Display it
	printf("\rencoder 1 : %08X ", encoder1.value);

	encoder2.bytes[0] = serialBuffer[7];
	encoder2.bytes[1] = serialBuffer[6];
	encoder2.bytes[2] = serialBuffer[5];
	encoder2.bytes[3] = serialBuffer[4];
//	encoder2.value = (serialBuffer[4] << 3) & (serialBuffer[5] << 2)
//			& (serialBuffer[6] << 1) & serialBuffer[7];
//	cout << "encoder 2: " << encoder2.value << endl;
	printf("encoder 2 : %08X   ", encoder2.value);
//
	fflush (stdout);// Flush output to ensure that data is displayed on the screen
}
