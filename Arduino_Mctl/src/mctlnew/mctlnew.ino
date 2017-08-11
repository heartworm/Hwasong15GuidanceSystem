#define BAUD 115200
#define BUF_LEN 10

#define MA1 5
#define MA2 6
#define MB1 9
#define MB2 10

#include "Proto.h"

const uint8_t HDR_MOTOR_RIGHT = 0x00;
const uint8_t HDR_MOTOR_LEFT = 0x01;

//Callback passed into Proto::setup. Called when a valid packet is received. 
void onPacket(const uint8_t* buffer, const uint8_t length) {
  if (length <= 0) return;
  uint8_t hdr = buffer[0];
  switch (hdr) {
    case HDR_MOTOR_RIGHT:
      if (length != 3) return;
      setMotor(false, buffer[1], buffer[2]);
    break;
    case HDR_MOTOR_LEFT:
      if (length != 3) return;
      setMotor(true, buffer[1], buffer[2]);
    break;  
  }
}
Proto proto;

//Convenience function for setting drive motor speeds.
// Specify which motor (left/right), which direction (forward/reverse),
// And a PWM duty cycle (0-255)
void setMotor(bool left, bool reverse, uint8_t val) {
  analogWrite(left ? MA1 : MB1, reverse ? 0 : val);
  analogWrite(left ? MA2 : MB2, reverse ? val : 0);
}


void setup() {
	proto.setup(115200, onPacket);
	pinMode(MA1, OUTPUT);
	pinMode(MA2, OUTPUT);
	pinMode(MB1, OUTPUT);
	pinMode(MB2, OUTPUT);
}

void loop() {
	proto.recv();
}
