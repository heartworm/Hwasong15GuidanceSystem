
#define LED_PIN 13

#define ma_F 5
#define ma_R 6

#define mb_F 9
#define mb_R 10

/*Motor diagram:
5(ma_F) ---\ +  1-256, Left side, move forward
	----ma (L?)
6(m1_R) ---/ -  256-512 - Left side, move reverse


9(mb_F) ---\ +  512-768 Right side, move forward
	----mb (R?)
10(mb_R) ---/ -  768-1024 Right side, move reverse
*/

int serialIN = 0; /*Value from 0-1023, used to dicate motor control, not ideal at this point
Divided into sections of 256, translated directly into pwm control
*/

void setup()
{
	pinMode(LED_PIN, OUTPUT);
	Serial.begin(115200);
}

void loop()
{
	}
//interrupt that gets called if serial event is detected
void serialEvent() {
	while (Serial.available()) {
		serialIN = Serial.parseInt();
		if (serialIN <= 510 && serialIN >= 1){   //Do a check to see which side we want controlled
			LMctl(serialIN);
		}
		else if (serialIN  >= 511){
			RMctl(serialIN - 511); //Value recieved is greater than 511, subtract 511 for simplicities sake.
		}
		serialIN = -1;
	}
}



void LMctl(int serialIN){
	if (serialIN <=256){
		analogWrite(mb_R, 0); //Set other pin to 0, to prevent clashes
		analogWrite(mb_F, serialIN - 1); //Set new value for the pin
		Serial.print("LM forward");
	}
	else {
		analogWrite(mb_F, 0); //Set other pin to 0, to prevent clashes
		analogWrite(mb_R, serialIN - 1); //Set new value for the pin
		Serial.print("LM reverse");
	}
}


void RMctl(int serialIN){
	if (serialIN <=255){
		analogWrite(ma_R, 0); //Set other pin to 0, to prevent clashes
		analogWrite(ma_F, serialIN); //Set new value for the pin
		Serial.print("RM forward");
	}
	else {
		analogWrite(ma_F, 0); //Set other pin to 0, to prevent clashes
		analogWrite(ma_R, serialIN); //Set new value for the pin
		Serial.print("RM Reverse");
	}

}


/*---------------------------------CODE DUMP-------------------------------*

  void serialEvent() {
  while (Serial.available()) {
  temp = Serial.parseInt();
  if (temp != 0 ){
  test = temp;
  }
  Serial.println(test);
  }
  }
----------------------------
//digitalWrite(LED_PIN, HIGH);
	//delay(test);
	//digitalWrite(LED_PIN, LOW);
	//delay(test);
	//temp = Serial.parseInt();
	//if (temp != 0 ){
	//	test = temp;
	//}
	//Serial.println(test);


  --------------------------------------------------------------------------*/

