#include <Dynamixel.h>

#define DXL_BAUDRATE    1000000
#define DXL_SERIAL      dxlSerial
#define DXL_PROTOCOL    1
#define DXL_DIR_PIN     37

HardwareSerial dxlSerial(0);    

const uint8_t DXL_ID = 1;

short Position;
short Voltage;
short Temperature;
String motorModel;

void setup() {

  dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200);                               // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&dxlSerial, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  delay(1000);
}

void loop() {

  Position = Dynamixel.readPosition(DXL_ID);
  Voltage = Dynamixel.readVoltage(DXL_ID);
  Temperature = Dynamixel.readTemperature(DXL_ID);
  motorModel = Dynamixel.mapMotorModel((Dynamixel.readModel(DXL_ID)));

  Serial.print(motorModel); 
  Serial.print(" -> Position: ");Serial.print((Position*360)/1023);
  Serial.print("° Temperature: ");Serial.print(Temperature);
  Serial.print("°C Voltage: ");Serial.print(Voltage/10);Serial.println("V.");

  delay(100);
}
