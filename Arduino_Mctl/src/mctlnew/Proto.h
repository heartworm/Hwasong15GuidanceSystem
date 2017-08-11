#ifndef PROTO_H
#define PROTO_H

#include "Arduino.h"

//Define Start, End, and Escape flags to use 
#define STX 0xFD
#define ETX 0xFE
#define ESC 0xFF

//Specify the the maximum packet length receivable (buffer size)
#define BUF_LEN 10

class Proto {
	private: 
		int baudrate;
		uint8_t buf[BUF_LEN];
		uint8_t ptr;
		bool inEsc; 
		bool inMsg;
		void (*callback)(const uint8_t*, const uint8_t);
		
		void bufClear();
		bool bufFull();
		void bufPush(const uint8_t data);
		
	public: 
		Proto();
    void Proto::setup(int baud, void (*cb)(const uint8_t*, const uint8_t));
		void recv();
		void send(const uint8_t *buffer, const uint8_t len);
};

#endif
