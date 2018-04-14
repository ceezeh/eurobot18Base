#ifndef _MD49_H_
#define _MD49_H_

#include "odometry/helper.h"



namespace md49 {
#define TWO_PI_PER_CPR 0.002369225229 // CPR = 2652

#define RIGHT_WHEEL_RADIUS 0.07080206777
#define LEFT_WHEEL_RADIUS  0.06943099181
#define PER_DISTANCE_BETWEEN_WHEELS 3.1009118899//0.2713995752 //meters

#define LINEAR_SPEED_TO_CMD 168
#define ANGULAR_SPEED_TO_CMD 168

class MD49 {
	union encoder {								// To store the encoder values
		int32_t value;
		char bytes[8];
	} right_encoder, right_encoder_t, left_encoder, left_encoder_t;

public:
	MD49();
	~MD49();
	void displaySoftwareValue();
	void displayDecoderValues();
	void setMode(char mode);
	void setAcceleration();
	void driveMotors( char v, char w);
	void resetEncoders();
	void displayMode();
	void getVelocityIntegration(float &v, float&w );
//	void turnUntil(float rad);
private:
	MyTimer enc_timer;
	char* serialBuffer;					// Serial buffer sto store data for I/O
	int fd;
};
}
#endif  /*_MD49_H_*/
