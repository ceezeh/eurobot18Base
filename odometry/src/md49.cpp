#include "odometry/wiringSerial.h"
#include "odometry/md49.h"
using namespace md49;

MD49::MD49() {
	serialBuffer = new char[10];
	char *portName = "/dev/ttyS0";	// Name of the UART port on the Raspberry pi
	fd = serialOpen(portName, 9600);

	usleep(10000);				// Sleep for UART to power up and set options

	displaySoftwareValue();		// Display the software version of the MD49

	setMode(3);								// Set the mode of the MD49 to 1

	setAcceleration();
	usleep(100);
	displayMode();

	resetEncoders();							// Reset the encode values to 0
	serialFlush (fd);
	enc_timer.start();
}

MD49::~MD49() {
	driveMotors(0, 0);
	serialClose (fd);
	delete serialBuffer;
}

void MD49::displaySoftwareValue() {
	serialBuffer[0] = 0;									// Sync byte of 0
	serialBuffer[1] = 0x29;					// Command to get software version

	writeBytes(fd, serialBuffer, 2);
	readBytes(fd, serialBuffer, 1);

//	cout << "MD49 Software v: " << serialBuffer[0] << endl;	// display the software version
	printf("MD49 Software v: %d \n", serialBuffer[0]);

}

void MD49::displayMode() {
	serialBuffer[0] = 0;									// Sync byte of 0
	serialBuffer[1] = 0x2B;					// Command to get software version

	writeBytes(fd, serialBuffer, 2);
	readBytes(fd, serialBuffer, 1);

	printf("MD49 Mode v: %d \n", serialBuffer[0]);// display the software version
}

void MD49::setMode(char mode) {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x34;								// Command to set mode
	serialBuffer[2] = mode; 							// Mode we wish to set

	writeBytes(fd, serialBuffer, 3);

}

void MD49::setAcceleration() {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x33;						// Command to set acceleration
	serialBuffer[2] = 0x05; 					// Set acceleration to 0.416s

	writeBytes(fd, serialBuffer, 3);

}

void MD49::driveMotors(char v, char w) {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x31;						// Command to set motor speed
	serialBuffer[2] = v;
	writeBytes(fd, serialBuffer, 3);						// Speed to be set
	usleep(10000);

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x32;
	serialBuffer[2] = w;
	writeBytes(fd, serialBuffer, 3);
}

void MD49::resetEncoders() {

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x35;					// Command to reset encoder values

	writeBytes(fd, serialBuffer, 2);
	right_encoder.value = 0;
	left_encoder.value = 0;
}

void MD49::displayDecoderValues() {
	serialBuffer[0] = 0;
	serialBuffer[1] = 0x25;					// Command to return encoder values
	right_encoder_t.value = 0;
	left_encoder_t.value = 0;

	writeBytes(fd, serialBuffer, 2);
	readBytes(fd, serialBuffer, 8);

	right_encoder_t.bytes[0] = serialBuffer[3];
	right_encoder_t.bytes[1] = serialBuffer[2];
	right_encoder_t.bytes[2] = serialBuffer[1];
	right_encoder_t.bytes[3] = serialBuffer[0];			// Display it
	printf("\rencoder 1 : %d ", right_encoder.value);

	left_encoder_t.bytes[0] = serialBuffer[7];
	left_encoder_t.bytes[1] = serialBuffer[6];
	left_encoder_t.bytes[2] = serialBuffer[5];
	left_encoder_t.bytes[3] = serialBuffer[4];
	printf("Left Encoder : %d   ", left_encoder.value);
}

void MD49::getVelocityIntegration(float &v, float&w) {

//	double elapsed_time = enc_timer.restart() / 1e6;			// in seconds.
	int32_t old_right_enc_value = right_encoder.value;
	int32_t old_left_enc_value = left_encoder.value;
	right_encoder.value = 0;
	left_encoder.value = 0;

	serialBuffer[0] = 0;
	serialBuffer[1] = 0x25;					// Command to return encoder values

	writeBytes(fd, serialBuffer, 2);
	readBytes(fd, serialBuffer, 8);

	right_encoder.bytes[0] = serialBuffer[3];
	right_encoder.bytes[1] = serialBuffer[2];
	right_encoder.bytes[2] = serialBuffer[1];
	right_encoder.bytes[3] = serialBuffer[0];
//	right_encoder.value = -right_encoder.value;	// Encoder hard wired in the reverse direction..
//	printf("\r[Vel Integral] Right Encoder : %d ", right_encoder.value);

	left_encoder.bytes[0] = serialBuffer[7];
	left_encoder.bytes[1] = serialBuffer[6];
	left_encoder.bytes[2] = serialBuffer[5];
	left_encoder.bytes[3] = serialBuffer[4];
//	left_encoder.value = -left_encoder.value;// Encoder hard wired in the reverse direction..
//	printf("Left Encoder : %d   ", left_encoder.value);
//
	fflush (stdout);// Flush output to ensure that data is displayed on the screen

	float wR = float(old_right_enc_value - right_encoder.value) * TWO_PI_PER_CPR;

	float wL = float(old_left_enc_value - left_encoder.value) * TWO_PI_PER_CPR;

	v = (wL * LEFT_WHEEL_RADIUS + wR * RIGHT_WHEEL_RADIUS) / 2;
	w = (-wL * LEFT_WHEEL_RADIUS + wR * RIGHT_WHEEL_RADIUS)
			* PER_DISTANCE_BETWEEN_WHEELS;
//	cout << "v: " << v << ", w: " << w << endl;
}

//// Turns by a certain angle.
//void MD49::turnUntil(float rad) {
////convert angles to encoder count.
//	float countDiff = fabs(rad * DISTANCE_BETWEEN_WHEELS);	// * CPR / 2 / M_PI;
//	cout << "Count Diff: " << countDiff << endl;
//
//	float v, w; // useless
//	displayDecoderValues (fd); // updates encoder values;
//	int8_t cmd_w = -copysign(60, rad);
//	driveMotors(fd, 0, cmd_w);
//
//	int32_t old_right_enc_value = right_encoder_t.value;
//	int32_t old_left_enc_value = left_encoder_t.value;
//	//begin driving.
//
//	int32_t Rdiff = (right_encoder_t.value - old_right_enc_value);
//	int32_t Ldiff = (left_encoder_t.value - old_left_enc_value);
//	float diff = (float(Rdiff) * RIGHT_WHEEL_RADIUS
//			- float(Ldiff) * LEFT_WHEEL_RADIUS) * 2 * M_PI / CPR;
//	cout << "Diff: " << diff << ",Rdiff: " << Rdiff << ", R enc old: "
//			<< right_encoder_t.value << endl;
//
//	while (ros::ok() && (fabs(diff) < countDiff)) {
//		displayDecoderValues(fd); // updates encoder values;
//		usleep(10000);
//		Rdiff = right_encoder_t.value - old_right_enc_value;
//		Ldiff = left_encoder_t.value - old_left_enc_value;
//		diff = (float(Rdiff) * RIGHT_WHEEL_RADIUS
//				- float(Ldiff) * LEFT_WHEEL_RADIUS) * 2 * M_PI / CPR;
//		cout << "Diff: " << diff << ",Rdiff: " << Rdiff << ", R enc old: "
//				<< old_right_enc_value << endl;
//	}
//	cout << "Turn complete with diff= " << diff << endl;
//	driveMotors(fd, 0, 0);
//}
