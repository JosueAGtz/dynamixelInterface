#include <Dynamixel.h>

#define DXL_BAUDRATE    1000000
#define DXL_SERIAL      dxlSerial
#define DXL_PROTOCOL    1
#define DXL_DIR_PIN     37

#define AX12RESOLUTION  1023

HardwareSerial dxlSerial(0);

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;

short Position_1, Position_2;
String motorModel_1, motorModel_2;

void setup() {

  dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200);                               // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&dxlSerial, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  delay(1000);
  
  Dynamixel.setTorque(DXL_ID_1, OFF);
}

void loop() {

  Position_1 = Dynamixel.readPosition(DXL_ID_1);
  motorModel_1 = Dynamixel.mapMotorModel((Dynamixel.readModel(DXL_ID_1)));

  Position_2 = Dynamixel.readPosition(DXL_ID_2);
  motorModel_2 = Dynamixel.mapMotorModel((Dynamixel.readModel(DXL_ID_2)));

  Serial.print(motorModel_1);
  Serial.print(" -> Position: "); Serial.print((Position_1 * 360) / AX12RESOLUTION);
  Serial.print("°. - ");

  Serial.print(motorModel_2);
  Serial.print(" -> Position: "); Serial.print((Position_2 * 360) / AX12RESOLUTION);
  Serial.println("°.");

  Dynamixel.move(DXL_ID_2, Dynamixel.readPosition(DXL_ID_1));

  delay(100);
}
