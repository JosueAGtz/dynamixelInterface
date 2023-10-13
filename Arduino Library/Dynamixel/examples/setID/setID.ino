#include <Dynamixel.h>

#define DXL_BAUDRATE    1000000
#define DXL_SERIAL      dxlSerial
#define DXL_PROTOCOL    1
#define DXL_DIR_PIN     37

const uint8_t DXL_ID = 1;
const uint8_t DXL_ID_NEW = 2;

HardwareSerial dxlSerial(0);

void setup() {

  dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200);                               // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&dxlSerial, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  Dynamixel.setID(DXL_ID,DXL_ID_NEW);
  delay(1000);
}

void loop() {

  Dynamixel.setLED(DXL_ID_NEW, ON);
  delay(250);
  Dynamixel.setLED(DXL_ID_NEW, OFF);
  delay(250);
  
}
