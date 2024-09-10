#include <IRremote.h>

#define IR_RECEIVE_PIN 7

#define LED 13

unsigned long keycode;

void setup() {
  pinMode(LED, OUTPUT);
  
  digitalWrite(LED, LOW);


  Serial.begin(9600); 
  IrReceiver.begin(IR_RECEIVE_PIN);
  Serial.println("ok");
  
}


void loop() {
  if (IrReceiver.decode()) {
    keycode = IrReceiver.decodedIRData.command;
    Serial.println(keycode);
    if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
         IrReceiver.resume();
         return;
    }
    IrReceiver.resume();
  }
}

/*
 * DIGIQUEST remote controll keys:
 * - standby: 3
 * - mute: 2
 * 1 7
 * 2 5
 * 3 6
 * 4 11
 * 5 9
 * 6 10
 * 7 15
 * 8 13
 * 9 14
 * 0 17
 * epg 19
 * recall 18
 * audio 23
 * subtitle 21
 * info 22
 * menu 24
 * exit 4
 * ch+ 8
 * ch- 16
 * vol- 28
 * vol+ 0
 * ok 12
 * text 27
 * red 25
 * green 26
 * pause 31
 * yellow 29
 * blue 30
 */
