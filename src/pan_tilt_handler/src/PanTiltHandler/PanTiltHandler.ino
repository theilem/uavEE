#define PAN 12
#define TILT 13
#define BASEVAL 1500
#define TIMEOUT 1000

int32_t pan = BASEVAL;
int32_t tilt = BASEVAL;

unsigned long last_packet = 0;

void setup() {
  Serial.begin(115200); 
  pinMode(PAN, OUTPUT);
  pinMode(TILT, OUTPUT);
}

void loop() {
   
   if(Serial.available()>0){
      pan = Serial.parseInt();
      tilt = Serial.parseInt();
      /*Serial.print("pan:");
      Serial.println(pan);
      Serial.print("tilt:");
      Serial.println(tilt);*/
      serialFlush();
      last_packet = millis();
   } else {
      if(millis()-last_packet > TIMEOUT){
         pan = BASEVAL;
         tilt = BASEVAL;
      }
   }
   digitalWrite(PAN, HIGH);
   delayMicroseconds(pan);
   digitalWrite(PAN, LOW);
   delayMicroseconds(100);
   digitalWrite(TILT, HIGH);
   delayMicroseconds(tilt);
   digitalWrite(TILT, LOW);
   delayMicroseconds(100);
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}
