#define BAUD 115200
#define BUF_LEN 10

#define MA1 5
#define MA2 6
#define MB1 9
#define MB2 10

#include "Proto.h"

const uint8_t HDR_MOTOR_RIGHT = 0x00;
const uint8_t HDR_MOTOR_LEFT = 0x01;


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

void setMotor(bool left, bool reverse, uint8_t val) {
  analogWrite(left ? MA1 : MB1, reverse ? 0 : val);
  analogWrite(left ? MA2 : MB2, reverse ? val : 0);
}

void setup() {
	proto.setup(115200, onPacket);
  pinMode(
}

void loop() {
  proto.recv();
}
