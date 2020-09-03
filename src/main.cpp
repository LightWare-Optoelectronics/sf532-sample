//----------------------------------------------------------------------------------------------------------------------------------
// LightWare tools.
//----------------------------------------------------------------------------------------------------------------------------------
#include "common.h"
#include "lwNx.h"

//----------------------------------------------------------------------------------------------------------------------------------
// Helper utilities.
//----------------------------------------------------------------------------------------------------------------------------------
void printHexDebug(uint8_t* Data, uint32_t Size) {
	printf("Buffer: ");

	for (int i = 0; i < Size; ++i) {
		printf("0x%02X ", Data[i]);
	}

	printf("\n");
}

void exitWithMessage(const char* Msg) {
	std::cerr << Msg << "\n";
	exit(1);
}

void exitCommandFailure() {
	exitWithMessage("No response to command, terminating.\n");
}

//----------------------------------------------------------------------------------------------------------------------------------
// Math helpers.
//----------------------------------------------------------------------------------------------------------------------------------
#define PI 3.14159265359
#define DEG_TO_RAD 0.0174533

struct lwMat4 {
	float v[16];
};

struct lwVec3 {
	float x;
	float y;
	float z;
};

void vec3Set(lwVec3* Vec, float X, float Y, float Z) {
	Vec->x = X;
	Vec->y = Y;
	Vec->z = Z;
}

// Set matrix to identity.
void mat4SetIdentity(lwMat4* Mat) {
	Mat->v[0] = 1;
	Mat->v[1] = 0;
	Mat->v[2] = 0;
	Mat->v[3] = 0;

	Mat->v[4] = 0;
	Mat->v[5] = 1;
	Mat->v[6] = 0;
	Mat->v[7] = 0;

	Mat->v[8] = 0;
	Mat->v[9] = 0;
	Mat->v[10] = 1;
	Mat->v[11] = 0;

	Mat->v[12] = 0;
	Mat->v[13] = 0;
	Mat->v[14] = 0;
	Mat->v[15] = 1;
}

// Create a rotation matrix from an angle and an axis.
void mat4FromRotationAxis(lwMat4* Mat, float X, float Y, float Z, float Angle) {
	float c = cos(Angle);
	float s = sin(Angle);
	float t = 1.0f - c;
	float tx = t * X;
	float ty = t * Y;

	Mat->v[0] = tx * X + c;
	Mat->v[4] = tx * Y - s * Z;
	Mat->v[8] = tx * Z + s * Y;
	Mat->v[12] = 0;

	Mat->v[1] = tx * Y + s * Z;
	Mat->v[5] = ty * Y + c;
	Mat->v[9] = ty * Z - s * X;
	Mat->v[13] = 0;

	Mat->v[2] = tx * Z - s * Y;
	Mat->v[6] = ty * Z + s * X;
	Mat->v[10] = t * Z * Z + c;
	Mat->v[14] = 0;

	Mat->v[3] = 0;
	Mat->v[7] = 0;
	Mat->v[11] = 0;
	Mat->v[15] = 1;
}

// Multiply matrices.
lwMat4 mat4Mul(lwMat4* A, lwMat4* B) {
	lwMat4 result;

	const float a11 = A->v[0], a12 = A->v[4], a13 = A->v[8], a14 = A->v[12];
	const float a21 = A->v[1], a22 = A->v[5], a23 = A->v[9], a24 = A->v[13];
	const float a31 = A->v[2], a32 = A->v[6], a33 = A->v[10], a34 = A->v[14];
	const float a41 = A->v[3], a42 = A->v[7], a43 = A->v[11], a44 = A->v[15];

	const float b11 = B->v[0], b12 = B->v[4], b13 = B->v[8], b14 = B->v[12];
	const float b21 = B->v[1], b22 = B->v[5], b23 = B->v[9], b24 = B->v[13];
	const float b31 = B->v[2], b32 = B->v[6], b33 = B->v[10], b34 = B->v[14];
	const float b41 = B->v[3], b42 = B->v[7], b43 = B->v[11], b44 = B->v[15];

	result.v[0] = a11 * b11 + a12 * b21 + a13 * b31 + a14 * b41;
	result.v[4] = a11 * b12 + a12 * b22 + a13 * b32 + a14 * b42;
	result.v[8] = a11 * b13 + a12 * b23 + a13 * b33 + a14 * b43;
	result.v[12] = a11 * b14 + a12 * b24 + a13 * b34 + a14 * b44;

	result.v[1] = a21 * b11 + a22 * b21 + a23 * b31 + a24 * b41;
	result.v[5] = a21 * b12 + a22 * b22 + a23 * b32 + a24 * b42;
	result.v[9] = a21 * b13 + a22 * b23 + a23 * b33 + a24 * b43;
	result.v[13] = a21 * b14 + a22 * b24 + a23 * b34 + a24 * b44;

	result.v[2] = a31 * b11 + a32 * b21 + a33 * b31 + a34 * b41;
	result.v[6] = a31 * b12 + a32 * b22 + a33 * b32 + a34 * b42;
	result.v[10] = a31 * b13 + a32 * b23 + a33 * b33 + a34 * b43;
	result.v[14] = a31 * b14 + a32 * b24 + a33 * b34 + a34 * b44;

	result.v[3] = a41 * b11 + a42 * b21 + a43 * b31 + a44 * b41;
	result.v[7] = a41 * b12 + a42 * b22 + a43 * b32 + a44 * b42;
	result.v[11] = a41 * b13 + a42 * b23 + a43 * b33 + a44 * b43;
	result.v[15] = a41 * b14 + a42 * b24 + a43 * b34 + a44 * b44;

	return result;
}

// Debug print matrix.
void mat4Print(lwMat4* Mat) {
	printf("%f %f %f %f\n", Mat->v[0], Mat->v[1], Mat->v[2], Mat->v[3]);
	printf("%f %f %f %f\n", Mat->v[4], Mat->v[5], Mat->v[6], Mat->v[7]);
	printf("%f %f %f %f\n", Mat->v[8], Mat->v[9], Mat->v[10], Mat->v[11]);
	printf("%f %f %f %f\n", Mat->v[12], Mat->v[13], Mat->v[14], Mat->v[15]);
}

// Transform a point by a matrix.
void vec3Transform(lwVec3* Vec, lwMat4* Mat) {
	const float x = Vec->x;
	const float y = Vec->y;
	const float z = Vec->z;
	const float w = 1.0 / (Mat->v[3] * x + Mat->v[7] * y + Mat->v[11] * z + Mat->v[15]);

	Vec->x = (Mat->v[0] * x + Mat->v[4] * y + Mat->v[8] * z + Mat->v[12]) * w;
	Vec->y = (Mat->v[1] * x + Mat->v[5] * y + Mat->v[9] * z + Mat->v[13]) * w;
	Vec->z = (Mat->v[2] * x + Mat->v[6] * y + Mat->v[10] * z + Mat->v[14]) * w;
}

//----------------------------------------------------------------------------------------------------------------------------------
// Packet helpers.
//----------------------------------------------------------------------------------------------------------------------------------
uint16_t readUInt16FromPacket(uint8_t* Data, int32_t Offset) {
	return (Data[Offset + 1] << 8) | Data[Offset];
}

uint32_t readUInt32FromPacket(uint8_t* Data, int32_t Offset) {
	return (Data[Offset + 3] << 24) | (Data[Offset + 2] << 16)  | (Data[Offset + 1] << 8) | Data[Offset];
}

//----------------------------------------------------------------------------------------------------------------------------------
// Application Entry.
//----------------------------------------------------------------------------------------------------------------------------------
int main(int args, char **argv) {
	platformInit();

#ifdef __linux__
	const char* portName = "/dev/ttyUSB0";
#else
	const char* portName = "\\\\.\\COM10";
#endif

	int32_t baudRate = 921600;

	lwSerialPort* serial = platformCreateSerialPort();
	if (!serial->connect(portName, baudRate)) {
		exitWithMessage("Could not establish serial connection.\n");
	};

	printf("Getting data from device.\n");

	// Read the product name. (Command 0: Product name)
	char modelName[16];
	if (!lwnxCmdReadString(serial, 0, modelName)) { exitCommandFailure(); }

	// Read the hardware version. (Command 1: Hardware version)
	uint32_t hardwareVersion;
	if (!lwnxCmdReadUInt32(serial, 1, &hardwareVersion)) { exitCommandFailure(); }

	// Read the firmware version. (Command 2: Firmware version)
	uint32_t firmwareVersion;	
	if (!lwnxCmdReadUInt32(serial, 2, &firmwareVersion)) { exitCommandFailure(); }
	char firmwareVersionStr[16];
	lwnxConvertFirmwareVersionToStr(firmwareVersion, firmwareVersionStr);

	// Read the serial number. (Command 3: Serial number)
	char serialNumber[16];
	if (!lwnxCmdReadString(serial, 3, serialNumber)) { exitCommandFailure(); }

	// Read the product type. (Command 11: Product type)
	int8_t productType;
	if (!lwnxCmdReadInt8(serial, 11, &productType)) { exitCommandFailure(); }

	printf("Model: %.16s\n", modelName);
	printf("Hardware: %d\n", hardwareVersion);
	printf("Firmware: %.16s (%d)\n", firmwareVersionStr, firmwareVersion);
	printf("Serial: %.16s\n", serialNumber);
	printf("Product type: %d\n", productType);

	// The individual lidars on the SF532 have different prodcut type codes.
	// The horizontal scanning angle offset is different for each lidar.
	float horizontalAngleOffset = 0.0f;

	if (productType == 61) {
		// Main unit.
		horizontalAngleOffset = 28.0f;

	} else if (productType == 62) {
		// Secondary unit.
		horizontalAngleOffset = 208.0f;
	}

	// Read servo angle. (Command 131: Servo angle) -2000 to 2000 (100ths of a degree).
	int32_t servoAngle = 0;
	if (!lwnxCmdReadInt32(serial, 131, &servoAngle)) { exitCommandFailure(); }

	printf("Servo angle: %f\n", (servoAngle / 100.0f));

	// Write servo angle. (Command 131: Servo angle) -2000 to 2000 (100ths of a degree).
	// NOTE: Do not set the servo angle beyond the range of -2000 to 2000.
	if (!lwnxCmdWriteInt32(serial, 131, 0)) { exitCommandFailure(); }
	
	// Enable streaming of point scan data. (Command 30: Stream)
	if (!lwnxCmdWriteUInt32(serial, 30, 30)) { exitCommandFailure(); }

	// Store some state for the lidar sweep.
	int32_t lastOpto = 0;
	float lastSweepHz = 0.0f;
	
	// Vertical transform: servo position angle.
	// Only needs to be updated when the servo position changes.
	lwMat4 verticalMat;
	float verticalAngleOffset = servoAngle / 100.0f;
	mat4FromRotationAxis(&verticalMat, 0, 0, 1, -verticalAngleOffset * DEG_TO_RAD);
	
	// Array of total possible points per sweep.
	// In normal running conditions we expect about 7000 points per sweep.
	const int32_t MAX_POINTS_PER_REV = 10000;
	lwVec3 points[MAX_POINTS_PER_REV];
	int32_t totalPoints = 0;

	// Continuously wait for and process the streamed point data packets.
	// The incoming point data packet is Command 210.
	while (1) {
		lwResponsePacket response;
		
		if (lwnxRecvPacket(serial, 210, &response, 1000)) {
			int32_t byteOffset = 4;

			// There are 25 sets of data in a single packet.
			for (int32_t s = 0; s < 25; ++s) {
			
				// Each set of data has distances for all 16 beams.
				for (int32_t b = 0; b < 16; ++b) {

					// Read distance in mm.
					uint16_t distMm = readUInt16FromPacket(response.data, byteOffset);
					byteOffset += 2;

					// Convert distance to meters.
					float distance = distMm / 1000.0f;

					// Convert beam to position relative to the laser receiver.
					// NOTE: This unwraps the position of the 16 beams, but does not account for servo angle or spin rotation.
					float p = (98.0 - b) * PI / 180.0;
					float x = distance * sin(p);
					float y = distance * cos(p);

					// Store the point.
					if (totalPoints < MAX_POINTS_PER_REV - 1) {
						vec3Set(&points[totalPoints], x, y, 0);
						++totalPoints;
					}
				}

				// Read global counter.
				uint32_t globalCounter = readUInt32FromPacket(response.data, byteOffset);
				byteOffset += 4;

				// Read photo interrupter counter.
				uint32_t globalOptoCounter = readUInt32FromPacket(response.data, byteOffset);
				byteOffset += 4;

				// If a new sweep has occured then process the data of the previous sweep.
				if (globalOptoCounter < lastOpto) {
					const float updateHz = 30000.0f;
					lastSweepHz = 1.0 / (totalPoints / updateHz);

					printf("Frequency: %f5 Hz   Point count: %d\n", lastSweepHz, totalPoints);

					// Convert all points to final cartesian positions.
					for (int32_t p = 0; p < totalPoints; ++p) {
						// Only once we have a full 360 degree sweep can we know what angle each point was taken at.
						float angle = ((PI * 2.0) / totalPoints) * p;

						// Horizontal transform: primary rotation angle.
						lwMat4 horizontalMat;
						mat4FromRotationAxis(&horizontalMat, 0, 1, 0, -angle + horizontalAngleOffset * DEG_TO_RAD);

						// Combine vertical and horizontal matrices.
						lwMat4 fullMat = mat4Mul(&horizontalMat, &verticalMat);

						// Transform the point.
						vec3Transform(&points[p], &fullMat);

						// NOTE: points[p] now holds the final lidar space position of the point.
					}

					totalPoints = 0;
				}

				lastOpto = globalOptoCounter;
			}
		}
	}

	return 0;
}