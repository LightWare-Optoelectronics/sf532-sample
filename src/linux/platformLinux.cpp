#include "platformLinux.h"
#include "lwSerialPortLinux.h"

void platformInit() { }

int64_t platformGetMicrosecond() {
	timespec time;
	clock_gettime(CLOCK_REALTIME, &time);

	return time.tv_sec * 1000000 + time.tv_nsec / 1000;
}

int32_t platformGetMillisecond() {
	return (platformGetMicrosecond() / 1000);
}

bool platformSleep(int32_t TimeMS) {
	usleep(TimeMS * 1000);
	return true;
};

lwSerialPort* platformCreateSerialPort() {
	return new lwSerialPortLinux();
}

uint8_t _startsWith(const char* Data, const char* Match) {

}

uint8_t _find(const char* Data, const char* Match) {
	int i = 0;
	int matchI = 0;

	while (Data[i] != 0) {
		if (Data[i] == Match[matchI]) {
			++matchI;

			if (Match[matchI] == 0) {
				return true;
			}
		} else {
			matchI = 0;
		}

		++i;
	}

	return false;
}

uint8_t _getPropValue(const char* Data, const char* Prop, char* Value) {
	int dataPos = 0;
	int propPos = 0;
	int state = 0;
	
	while (Data[dataPos] != 0) {

		if (state == 0) {
			if (Data[dataPos] == Prop[propPos]) {
				++propPos;

				if (Prop[propPos] == 0) {
					state = 1;
				}
			} else {
				return 0;
			}
		} else if (state == 1) {
			if (Data[dataPos] == '=') {
				state = 2;
			} else {
				return 0;
			}
		} else if (state == 2) {
			char c = Data[dataPos];

			if (c != 10 && c != 13) {
				*Value = c;
				++Value;
			}
		}

		++dataPos;
	}

	*Value = 0;

	return 1;
}

std::vector<lwSerialPortInfo> platformGetSerialPortList() {
	char line[1024];
	FILE* output = popen("udevadm info -e", "r");

	uint8_t foundPort = 0;

	lwSerialPortInfo serialPortInfo = {};
	lwSerialPortInfo* currentPortInfo = 0;
	std::vector<lwSerialPortInfo> serialPorts;

	while (fgets(line, 1024, output)) {
		if (line[0] == 'P') {
			// std::cout << "new port\n";
			foundPort = 1;
		} else if (line[0] == 'N' && foundPort == 1) {
			if (_find(line, "ttyUSB") || _find(line, "ttyACM") || _find(line, "ttyAMA")) {
				strcpy(serialPortInfo.deviceName, "Unknown");
				strcpy(serialPortInfo.vendorName, "Unknown");
				strcpy(serialPortInfo.serial, "Unknown");
				strcpy(serialPortInfo.vendorId, "Unknown");
				strcpy(serialPortInfo.productId, "Unknown");
				serialPorts.push_back(serialPortInfo);
				currentPortInfo = &serialPorts.back();
				foundPort = 2;
			} else {
				foundPort = 0;
			}
		} else if (line[0] == 'E' && foundPort == 2) {
			if (currentPortInfo) {
				_getPropValue(line + 3, "DEVNAME", currentPortInfo->deviceName);
				_getPropValue(line + 3, "ID_VENDOR_ENC", currentPortInfo->vendorName);
				_getPropValue(line + 3, "ID_SERIAL_SHORT", currentPortInfo->serial);
				_getPropValue(line + 3, "ID_VENDOR_ID", currentPortInfo->vendorId);
				_getPropValue(line + 3, "ID_MODEL_ID", currentPortInfo->productId);
			}
		}
	}

	pclose(output);

	printf("Done %lu\n", serialPorts.size());

	return serialPorts;
}