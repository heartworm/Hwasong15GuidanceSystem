#include "Proto.h"

Proto::Proto() {}

/*
 * Setup the packet system, giving it a baud rate to use, and a callback to
 * invoke when valid packets are received.
 */
void Proto::setup(int baud, void (*cb)(const uint8_t*, const uint8_t)) {
	Serial.begin(115200);
	callback = cb;
	bufClear();
}

/*
 * Expects Packets in the from
 * 		<STX> Some Data Here <ETX>
 * However if the flags <STX> or <ETX> or <ESC> are present in the data 
 * Those bytes must be preceded by an <ESC> byte (escaped)
 * On reception of a valid packet that fits into the buffer, the "callback" function is called
 * passing the buffer and received packet length as parameters
 */
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

// Clear the buffer and set it to initial state
void Proto::bufClear() {
	ptr = 0;
	inMsg = false;
	inEsc = false;
}

// Return true if no more data can fit in the buffer
bool Proto::bufFull() {
	return ptr >= BUF_LEN;
}

// Push one byte into the buffer. 
void Proto::bufPush(const uint8_t data) {
	buf[ptr++] = data;
}
