#include <Dynamixel.h>

#define DXL_BAUDRATE    1000000
#define DXL_SERIAL      dxlSerial
#define DXL_PROTOCOL    1
#define DXL_DIR_PIN     37

#define DYNARESOLUTION  1023
#define MAXPOSITION     800
#define MINPOSITION     200
#define OFFSET          10

HardwareSerial dxlSerial(0);

const uint8_t DXL_ID = 1;

short Position;
String motorModel;

void setup() {

  dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200);                               // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&dxlSerial, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  delay(1000);

  if (DXL_PROTOCOL == 2) {
    Dynamixel.setTorque( DXL_ID, OFF);
    Dynamixel.setOperationMode(DXL_ID, POSITION_CTRL);
    Dynamixel.setProfileVelocity(DXL_ID, 0);
    Dynamixel.setTorque( DXL_ID, ON);
  }
  
  Dynamixel.move(DXL_ID, MINPOSITION);
}

void loop() {

  Position = Dynamixel.readPosition(DXL_ID);
  motorModel = Dynamixel.mapMotorModel((Dynamixel.readModel(DXL_ID)));

  Serial.print(motorModel);
  Serial.print(" -> Position: "); Serial.print((Position * 360) / DYNARESOLUTION);
  Serial.println("°.");

  if ((Dynamixel.readPosition(DXL_ID) > MAXPOSITION - OFFSET) && (Dynamixel.readPosition(DXL_ID) < MAXPOSITION + OFFSET))
    Dynamixel.move(DXL_ID, MINPOSITION);

  if ((Dynamixel.readPosition(DXL_ID) > MINPOSITION - OFFSET) && (Dynamixel.readPosition(DXL_ID) < MINPOSITION + OFFSET))
    Dynamixel.move(DXL_ID, MAXPOSITION);

  delay(100);

}
