/* ########################################################################################

    Simple IDE ATAPI controller using the Arduino I2C interface and 
    3 PCF8574 I/O expanders. Release 2.0.
    Copyright (C) 2011  Carlos Durandal
  
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

  
 ##########################################################################################
 
    PCF8574   A2,A1,A0     Addr.
        #1     0  0  0     0x20    interfaces to IDE DD0-DD7
        #2     0  0  1     0x21    interfaces to IDE DD8-DD15
        #3     0  1  0     0x22    interfaces to the IDE register selection as follows:

    PCF8574#3 bit:    7     6    5    4    3   2   1   0
    IDE pin:        nDIOR nDIOW nRST nCS1 nCS0 DA2 DA1 DA0
 
     pin names with leading 'n' are active LOW

   ########################################################################################
*/

#include "U8glib.h"
#include "Wire.h"                  // I2C bus library
#include <IRremote.h>

#define IR_RECEIVE_PIN 7
unsigned long keycode;

// Start of Definitions 
// ####################

// I/O expander addresses:
const int DataL = 0x20;            // Addr. of I/O expander connected to IDE DD0-DD7
const int DataH = 0x21;            // Addr. of I/O expander connected to IDE DD8-DD15
const int RegSel = 0x22;           // Addr. of I/O expander connected to IDE register

// IDE Register addresses
const byte DataReg = 0xF0;         // Addr. Data register of IDE device.
const byte ErrFReg = 0xF1;         // Addr. Error/Feature (rd/wr) register of IDE device.
const byte SecCReg = 0xF2;         // Addr. Sector Count register of IDE device.
const byte SecNReg = 0xF3;         // Addr. Sector Number register of IDE device.
const byte CylLReg = 0xF4;         // Addr. Cylinder Low register of IDE device.
const byte CylHReg = 0xF5;         // Addr. Cylinder High register of IDE device.
const byte HeadReg = 0xF6;         // Addr. Device/Head register of IDE device.
const byte ComSReg = 0xF7;         // Addr. Command/Status (wr/rd) register of IDE device.
const byte AStCReg = 0xEE;         // Addr. Alternate Status/Device Control (rd/wr) register of IDE device.

// Program Variables
byte dataLval;                     // dataLval and dataHval hold data from/to 
byte dataHval;                     // D0-D15 of IDE
byte regval;                       // regval holds addr. of reg. to be addressed on IDE
byte reg;                          // Holds the addr. of the IDE register with adapted
                                   // nDIOR/nDIOW/nRST values to suit purpose.
byte cnt;                          // packet byte counter
byte idx = 0;                      // index used as pointer within packet array
byte paclen = 12;                  // Default packet length
byte s_trck;                       // Holds start track
byte e_trck;                       // Holds end track
byte c_trck;                       // Follows current track while reading TOC
byte c_trck_m;                     // MSF values for current track
byte c_trck_s;
byte c_trck_f;
byte a_trck = 1;                   // Holds actual track from reading subchannel data
byte d_trck;                       // Destination track
byte d_trck_m;                     // MSF values for destination track
byte d_trck_s;
byte d_trck_f;
byte p_trck = 0;                   // Stores track currently played
byte aud_stat = 0xFF;              // Derived  Audio Status from reading subchannel data
byte sw = 0;                       // switch to control pause and resume operations.
boolean paused = false;            // flags play operation paused. Combined with sw
                                   // makes one button play/pause operation possible.
boolean playing = false;           // flags play is ongoing or stopped.                     
boolean pktlen = true;             // Used in ident_device to get packet length data


// Array containing sets of 16 byte packets corresponding to part of the CD-ROM 
// function set. If IDE device only supports packets with 12 byte length the last 
// 4 bytes are not sent.

byte fnc[]= {
  0x1B,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=0 Open tray
  0x1B,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=16 Close tray
  0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=32 Stop disk
  0x47,0x00,0x00,0x10,0x28,0x05,0x4C,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=48 Start PLAY
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=64 PAUSE play
  0x4B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=80 RESUME play
  0x43,0x02,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=96 Read TOC
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=112 unit ready
  0x5A,0x00,0x01,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // idx=128 mode sense
  0x42,0x02,0x40,0x01,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00  // idx=144 rd subch.  
};

// Arduino pin assignments:
const byte LED = 13;
const byte EJCT = 12;                   // EJECT button, open/close CD-ROM tray
const byte STOP = 11;                   // STOP button
const byte NEXT = 10;                   // NEXT button
const byte PLAY = 9;                    // PLAY button
const byte PREV = 8;                    // PREV button

// End of Definitions ########################################################################

//char comando;

char text[16];
char text1[16];
char text2[20];

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI

void setup(){

  u8g.begin();
  u8g.setRot180();
  u8g.setFont(u8g_font_profont12);
  
  // Arduino Part
  // #############

  // start I2C interface as Master
  Wire.begin();

  // Set all pins of all PCF8574 to high impedance inputs.
  highZ();

  // Start Serial Interface
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN);
  

  // initialize the push button pins as inputs with pullup:
  pinMode(NEXT, INPUT_PULLUP);
  pinMode(PREV, INPUT_PULLUP);
  pinMode(EJCT, INPUT_PULLUP);
  pinMode(STOP, INPUT_PULLUP);
  pinMode(PLAY, INPUT_PULLUP);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

// IDE Initialisation Part
// ########################

  Serial.println("ATAPIduino Oled+IR");
  u8g.firstPage();  
  do {
    u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
  } while( u8g.nextPage() );
  reset_IDE();                              // Do hard reset
  delay(3000);
  BSY_clear_wait();
  DRY_set_wait();
  
  readIDE(CylLReg);                         // Check device signature for ATAPI capability
  if(dataLval == 0x14){
    readIDE(CylHReg);
    if(dataLval == 0xEB){
         Serial.println("Found ATAPI Device");         
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
  } while( u8g.nextPage() );
    }
  }else{
         Serial.println("No ATAPI Device !!!");
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "No ATAPI Device !!!" );
  } while( u8g.nextPage() );
  }

  Serial.println("Set IDE to Master");      // Set Device to Master (Device 0)
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
  } while( u8g.nextPage() );
  dataLval = 0;
  writeIDE(HeadReg, dataLval, 0xFF);
  delay(1000);

// Initialise task file
// ####################
  BSY_clear_wait();
  DRQ_clear_wait();
  dataLval=0;                             // Set Feature register = 0
  writeIDE(ErrFReg, dataLval, 0xFF);
  BSY_clear_wait();
  dataLval=2;                             // Set PIO buffer to max. transfer length (= 200h)
  writeIDE(CylHReg, dataLval, 0xFF);        // Set byte count high
  dataLval=0;
  writeIDE(CylLReg, dataLval, 0xFF);        // Set byte count low
  BSY_clear_wait();
  BSY_clear_wait();

// Run Self Diagnostic
// ###################
  Serial.print("Self diagnostic:");
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
         u8g.drawStr( 0, 40, "Self diagnostic:" );
  } while( u8g.nextPage() );
  writeIDE(ComSReg, 0x90, 0xFF);            // Issue Run Self Diagnostic Command
  readIDE(ErrFReg);
  if(dataLval == 0x01){
    Serial.println(" OK");
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
         u8g.drawStr( 0, 40, "Self diagnostic: OK" );
  } while( u8g.nextPage() );
  }else{
        Serial.println(" Fail");            // Units failing this may still work fine
         u8g.firstPage();  
         do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
         u8g.drawStr( 0, 40, "Self diagnostic: Fail" );
  } while( u8g.nextPage() );
  }
    
// Identify Device
// ###############
  writeIDE (ComSReg, 0xA1, 0xFF);           // Issue Identify Device Command
  delay(1000);                              // Instead of wait for IRQ. Needed by some dev.
  DRQ_set_wait();                           // If no valid CD in tray some dev. hang here.
  do{
    readIDE(DataReg);
    if (pktlen == true){                          // Get supported packet lenght
      if(dataLval & (1<<0)){                      // contained in first read byte
        paclen = 16;                              // 1st bit set ? -> use 16 byte packets
        Serial.println("Using 16 byte packets");  // else keep default 12 bytes
      }
      pktlen = false;                             // and skip the rest of the data.
    }
    readIDE(ComSReg);                             // Read Status Register and check DRQ,
  } while(dataLval & (1<<3));                     // skip rest of data until DRQ=0
  Serial.println("Please insertCD");
     u8g.firstPage();  
  do {
         u8g.drawStr( 0, 10, "ATAPIduino Oled+IR" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
         u8g.drawStr( 0, 40, "Self diagnostic: OK" );
         u8g.drawStr( 0, 50, "Please insert CD ..." );
  } while( u8g.nextPage() );
  DRQ_clear_wait();
  
// Check if unit ready
// ###################
  while (test_unit_ready() == false){}            // No valid CD in tray ?
  Serial.println("Unit Ready!");                         // CD-Master 48E hangs here if
  u8g.firstPage();  
  do {
         u8g.drawStr( 0, 10, "Atapiduino R2.0" );
         u8g.drawStr( 0, 20, "Found ATAPI Device" );
         u8g.drawStr( 0, 30, "Set IDE to Master" );
         u8g.drawStr( 0, 40, "Self diagnostic: OK" );
         u8g.drawStr( 0, 50, "Unit Ready!" );
  } while( u8g.nextPage() );
                                                  // no CD loaded
  get_TOC();                                      // Read TOC and display CD data 
  Disp_CD_data();                                 // (track range and total playing time)
  digitalWrite(LED, HIGH);                        // Arduino LED indicates device ready
  
// ##################################
// IDE CD-ROM initialisation complete
// ##################################

    if(playing == false){                     // If not yet playing -> start play
      play();
      playing = true;
      paused = false;
    }
    play_diff(sw);                            // Depending on play/pause do hold/resume
    delay(350);
    read_subch_cmd();
    disp_track();    
    sw = sw ^ 1;

}

// End of setup ###########################################################################


// Main Loop
void loop(){
  
  BSY_clear_wait();

  

    if (IrReceiver.decode()) {
    keycode = IrReceiver.decodedIRData.command;
    //Serial.println(keycode);
    if ((IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      delay(200);
         IrReceiver.resume();
         return;
    }
    IrReceiver.resume();
  }

  //comando = Serial.read();
 
  //if (comando == 'e') {
  if(digitalRead(EJCT) == LOW | keycode == 4){                   // Scan push buttons
    delay(450);
    switch(chck_disk()) {                         // -> eject
      case 0x00:                                  // If disk in tray case
      eject();
      break;
      case 0xFF:                                  // If tray closed but no disk in case
      eject();
      break;      
      case 0X71:                                  // If tray open -> close it
      load();
      while (test_unit_ready() == false){}
      BSY_clear_wait();
      DRQ_clear_wait();      
      
      get_TOC();                            // try to get TOC data
      Disp_CD_data();                       // and display it
      
    }
    
    a_trck = s_trck;                          // Reset to start track
    sw = 0;
    play();
    playing = true;
    paused = false;
    play_diff(sw);                            // Depending on play/pause do hold/resume
    delay(350);
    read_subch_cmd();
    disp_track();    
    sw = sw ^ 1;
    keycode = 0;
  }

  //if (comando == 's') {
  if(digitalRead(STOP) == LOW | keycode == 25){
    delay(450);
    stop();
    if(chck_disk() == 0x00){                  // If disk in ray
     get_TOC();                               // get TOC data
    }
    playing = false;
    a_trck = s_trck;                          // Reset to start track
    sw = 0;
    keycode = 0;
  }

  //if (comando == 'p') {
  if(digitalRead(PLAY) == LOW | keycode == 12){
    delay(450);   
    if(playing == false){                     // If not yet playing -> start play
      play();
      playing = true;
      paused = false;
    }
    play_diff(sw);                            // Depending on play/pause do hold/resume
    delay(350);
    read_subch_cmd();
    disp_track();    
    sw = sw ^ 1;
    keycode = 0;
  }

  //if (comando == 'n') {
  if(digitalRead(NEXT) == LOW | keycode == 8){
    Serial.print("Next ");
    delay(350);
    a_trck = a_trck + 1;                      // a_track becomes next track
    if(a_trck > e_trck){(a_trck = s_trck);}   // over last track? -> point to start track
    get_TOC();                                // Get MSF for a_trck
    fnc[51] = d_trck_m;                       // Store new play start position 
    fnc[52] = d_trck_s;                       // in play packet and start play
    fnc[53] = d_trck_f;                       // 
    play();                                   // 
    delay(45);                                // device dependent, relev. if paused
    sw = 1;                                   // Mark 'pause' in case PLAY button 
    read_subch_cmd();
    disp_track();                             // is pushed again and display track
    if(paused == true | playing == false){    // Makes NEXT work while paused or stopped
      pause();                                // and remain paused
      sw = 0;                                 // Mark 'resume' in case PLAY pressed again
      playing = false;
    }
    keycode = 0;
  }
  

  //if (comando == 'b') {
  if(digitalRead(PREV) == LOW | keycode == 16){               // Basically like the NEXT function above
    Serial.print("Prev. ");
    delay(350);
    a_trck = a_trck - 1;                      // only backwards
    if(a_trck < s_trck){(a_trck = e_trck);}
    get_TOC();
    fnc[51] = d_trck_m;
    fnc[52] = d_trck_s;
    fnc[53] = d_trck_f;
    play();
    delay(45);
    sw = 1;
    disp_track();    
    if(paused == true | playing == false){    //
      pause();
      sw = 0;
      playing = false;      
    }
    keycode = 0;
  }  
}

// ########################################################################################


// Auxiliary functions
// ###################

// Set for all PCF8475 all pins to high impedance.
void highZ(){

  Wire.beginTransmission(RegSel);       // address IDE Register interface
  Wire.write(255);                       // queue FFh into buffer for setting all pins HIGH
  Wire.endTransmission();               // transmit buffered data to IDE Register interface
  Wire.beginTransmission(DataH);        // address IDE DD8-DD15
  Wire.write(255);                       // as above
  Wire.endTransmission();               //
  Wire.beginTransmission(DataL);        // address IDE DD0-DD7
  Wire.write(255);                       // as above
  Wire.endTransmission();               //
}


// Reset Device
void reset_IDE(){
  Serial.println("Reseting IDE ...");
  Wire.beginTransmission(RegSel);
  Wire.write(B11011111);                // Bit 5 LOW to reset IDE via nRESET
  Wire.endTransmission();
  delay(40);
  Wire.beginTransmission(RegSel);
  Wire.write(B11111111);                // Release reset
  Wire.endTransmission();
  delay(20);
}

// Wait for BSY clear
void BSY_clear_wait(){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<7));
}

// Wait for DRQ clear
void DRQ_clear_wait(){
  do{
    readIDE(ComSReg);
  } while(dataLval & (1<<3));
}

// Wait for DRQ set
void DRQ_set_wait(){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<3)) == true);
}

// Wait for DRY set
void DRY_set_wait(){
     do{
        readIDE(ComSReg);
     }while((dataLval & ~(1<<6)) == true);
}

boolean test_unit_ready(){
        boolean ready = true;
        idx=112;
        SendPac();
        readIDE(ComSReg);
        if(dataLval & (1<<0)){
           ready = false;
        }
        return(ready);
}

// Read one word from IDE port
void readIDE (byte regval){

  reg = regval & B01111111;             // set nDIOR bit LOW preserving register address
  Wire.beginTransmission(RegSel);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(DataH, 1);
  dataHval = Wire.read();

  Wire.requestFrom(DataL, 1);
  dataLval = Wire.read();

  highZ();                              // set all I/O pins to HIGH -> impl. nDIOR release
}

// Write one word to IDE port
void writeIDE (byte regval, byte dataLval, byte dataHval){

  reg = regval | B01000000;             // set nDIOW bit HIGH preserving register address
  Wire.beginTransmission(RegSel);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.beginTransmission(DataH);        // send data for IDE D8-D15
  Wire.write(dataHval);
  Wire.endTransmission();

  Wire.beginTransmission(DataL);        // send data for IDE D0-D7
  Wire.write(dataLval);
  Wire.endTransmission();

  reg = regval & B10111111;             // set nDIOW LOW preserving register address
  Wire.beginTransmission(RegSel);
  Wire.write(reg);
  Wire.endTransmission();

  highZ();                              // All I/O pins to high impedance -> impl. nDIOW release
}

// Send a packet starting at fnc array position idx
void SendPac(){
     writeIDE (AStCReg, B00001010, 0xFF);     // Set nIEN before you send the PACKET command! 
     writeIDE(ComSReg, 0xA0, 0xFF);           // Write Packet Command Opcode indicating
     delay(400);
     for (cnt=0;cnt<paclen;cnt=cnt+2){        // Send packet with length of 'paclen' 
     dataLval = fnc[(idx + cnt)];             // to IDE Data Registeraccording to idx value
     dataHval = fnc[(idx + cnt + 1)];
     writeIDE(DataReg, dataLval, dataHval);
     readIDE(AStCReg);                         // Read alternate stat reg.     
     readIDE(AStCReg);                         // Read alternate stat reg.          
     }
     BSY_clear_wait();
}

void get_TOC(){
//       Serial.print("Reading TOC");
       do{
       read_TOC_cmd();                         // Issue read TOC command
       readIDE(ComSReg);
       } while (dataLval & (1<<0));            // Repeat reading TOC until no error.
       read_TOC();
  }

void read_TOC_cmd(){
        idx =  96;                             // Pointer to Read TOC Packet
        SendPac();                             // Send read TOC command packet
}

void read_TOC(){
        readIDE(DataReg);                      // TOC Data Length not needed, don't care
        readIDE(DataReg);                      // Read first and last session
        s_trck = dataLval;
        e_trck = dataHval;        
        do{
           readIDE(DataReg);                   // Skip Session no. ADR and control fields
           readIDE(DataReg);                   // Read curent track number
           c_trck = dataLval;
           readIDE(DataReg);                   // Read M
           c_trck_m = dataHval;                // Store M of curent track
           readIDE(DataReg);                   // Read S and F
           c_trck_s = dataLval;                // Store S of current track
           c_trck_f = dataHval;                // Store F of current track
           
           if (c_trck == s_trck){              // Store MSF of first track
               fnc[51] = c_trck_m;             // 
               fnc[52] = c_trck_s;
               fnc[53] = c_trck_f;            
           }           
           if (c_trck == a_trck){              // Store MSF of actual track
               d_trck_m = c_trck_m;            // 
               d_trck_s = c_trck_s;
               d_trck_f = c_trck_f;            
           }                      
           if (c_trck == 0xAA){                // Store MSF of end position
               fnc[54] = c_trck_m;
               fnc[55] = c_trck_s;
               fnc[56] = c_trck_f;
           }
           readIDE(ComSReg);
        } while(dataLval & (1<<3));            // Read data from DataRegister until DRQ=0
//        Serial.println(" ready");
}

void read_subch_cmd(){
        idx =  144;                            // Pointer to read Subchannel Packet
        SendPac();                             // Send read Subchannel command packet
          readIDE(DataReg);                    // Get Audio Status
          aud_stat = dataHval;
          readIDE(DataReg);                    // Get (ignore) Subchannel Data Length
          readIDE(DataReg);                    // Get (ignore) Format Code, ADR and Control
          readIDE(DataReg);                    // Get actual track
          a_trck = dataLval;
        do{
          readIDE(DataReg);
          readIDE(ComSReg);
        } while(dataLval & (1<<3));              // Read data from Data Reg. until DRQ=0
}

void play(){
    idx = 48;                                    // pointer to play function and Play MSF
//    Serial.print("PLAY ");
    SendPac();
}

void stop(){
    idx = 32;                                     // pointer to stop function
    Serial.println("STOP");
    SendPac();
    delay(400);    
}

void eject(){
    idx = 0;                                      // pointer to eject function
    Serial.println("EJECT");
    SendPac();
}

void load(){
    idx = 16;
    Serial.println("LOAD");
    SendPac();
}

void pause(){
     idx = 64; // pointer to hold
     Serial.print("PAUSE ");
     SendPac();
     paused = true;
}

void resume(){
     idx = 80; // pointer to resume
     Serial.print("PLAY ");
     SendPac();
}

 void Disp_CD_data(){
     Serial.print("Tracks:");
     Serial.print(s_trck, DEC);
     Serial.print("-");
     Serial.print(e_trck, DEC);
     Serial.print("  Playing time: ");            
     Serial.print(fnc[54], DEC);                           
     Serial.print(":");
     Serial.println(fnc[55], DEC); 
     Serial.println();
     sprintf (text1, "Tracks: %d-%d", s_trck, e_trck);
     sprintf (text2, "Playing time: %d:%d", fnc[54],fnc[55]);
 }

void disp_track(){
     Serial.print("Track:");
     Serial.println(a_trck, DEC);
     sprintf (text, "Track: %d", a_trck);
     u8g.firstPage();  
  do {
    draw(24,44,text);
  } while( u8g.nextPage() );
}

void play_diff(byte sw){
    switch(sw) {
         case 0:
         resume();
         paused = false;
         break;         
         case 1: 
         pause();
         paused = true;         
    }
} 

byte chck_disk(){
     byte disk_ok = 0xFF;                        // assume no valid disk present.
     idx = 128;                                  // Send mode sense packet
     SendPac();                                  // 
     DRQ_set_wait();                             // Wait for data ready to read.
     readIDE(DataReg);                           // Mode Sense data length, dicard
     readIDE(DataReg);                           // Get Medium Type byte
     if (dataLval == 0x02 | dataLval == 06){     // If valid disk present disk_ok = 0x00
        disk_ok = 0x00;
     }    
     if (dataLval == 0x71){                      // Note if door open
        disk_ok = 0x71;
     }
     do{                                         // Skip rest of packet.
       readIDE(DataReg);
       readIDE(ComSReg);
     } while(dataLval & (1<<3));
     return(disk_ok);
}

void draw(int x, int y, char text[]) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_profont12);
  u8g.drawFrame(0,0,128,64);
  u8g.drawStr( 3, 12, text1 );
  u8g.drawStr( 3, 24, text2 );
  u8g.setFont(u8g_font_unifont);
  u8g.drawStr( x, y, text );
}

// END ####################################################################################
