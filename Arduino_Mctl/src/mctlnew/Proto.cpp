#include "Proto.h"

Proto::Proto() {}

void Proto::setup(int baud, void (*cb)(const uint8_t*, const uint8_t)) {
	Serial.begin(115200);
  callback = cb;
  bufClear();
}

void Proto::recv() {
	uint16_t byteIn = Serial.read();
	if (byteIn != -1) {
		if (bufFull()) {
			bufClear();
		}
		
		if (!inMsg) {
			inEsc = false;
		}
		
		if (!inEsc) { 
			if (inMsg && byteIn == ESC) {
				inEsc = true;
			} else if (byteIn == STX) {
				bufClear();
				inMsg = true;
			} else if (byteIn == ETX) {
				if (inMsg) callback(buf, ptr);
				bufClear();
			} else if (inMsg) {
				bufPush(byteIn);
			}	
		} else {
			inEsc = false;
			if (inMsg) {
				bufPush(byteIn);
			}
		}
	}
}

void Proto::bufClear() {
	ptr = 0;
	inMsg = false;
	inEsc = false;
}

bool Proto::bufFull() {
	return ptr >= BUF_LEN;
}

void Proto::bufPush(const uint8_t data) {
	buf[ptr++] = data;
}
