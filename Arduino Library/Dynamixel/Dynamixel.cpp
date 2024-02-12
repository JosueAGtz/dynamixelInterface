/*
 Dynamixel.cpp - Half Duplex USART Comunication
 Copyright (c) 2011 Savage Electronics.
 Created by Josue Alejandro Gutierrez on 27/01/11.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************

 Library Version: 0.4.2 - 10/16/23

 Last Update on 16-OCT-2023
            - Added compatibility with Software Serial
            - Added support for Serial Bus servos (FeeTech/Waveshare y Hiwonder)
            - Added read Instructions for Current, PWM and Velocity
            - Datatype corrections

 SUPPORTED DEVICES:

 PROTOCOL 2.0: XC330,XL330,XC430,XL430,2XC430,2XL430,XW430,XH430,XM430,XD540,XW540,XH540,MX106,MX64,MX28
 PROTOCOL 1.0: AX-12,AX-12+,AX-12A,AX-12W,AX-18A,MX-12W,MX-28T,MX-28AT,MX-28AR,MX-64T,MX-64R,MX-64AT,MX-64AR
 
 *****************************************************************************
 
 Contact: Josue Alejandro Gutierrez - Josue.Gutierrez@savageelectronics.com
 Web:     https://www.savageelectrtonics.com/
 Autor:   Josue Alejandro Gutierrez
 
 */

#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Dynamixel.h"

// Macro for the selection of the softwareSerial Port
/*
#define sendData(args)      (softwareSerialPort->write((uint8_t)args))  // Write Over Serial
#define availableData()     (softwareSerialPort->available())           // Check Serial Data Available
#define availableWrite()    (softwareSerialPort->availableForWrite())   // Bytes left to write
#define readData()          (softwareSerialPort->read())                // Read Serial Data
#define peekData()          (softwareSerialPort->peek())                // Peek Serial Data
#define beginCom(args)      (softwareSerialPort->begin(args))           // Begin Serial Comunication
#define endCom()            (softwareSerialPort->end())                 // End Serial Comunication
*/

#define sendData(args)      (serialPort->write((uint8_t)args))  // Write Over Serial
#define availableData()     (serialPort->available())           // Check Serial Data Available
#define availableWrite()    (serialPort->availableForWrite())   // Bytes left to write
#define readData()          (serialPort->read())                // Read Serial Data
#define peekData()          (serialPort->peek())                // Peek Serial Data
#define beginCom(args)      (serialPort->begin(args))           // Begin Serial Comunication
#define endCom()            (serialPort->end())                 // End Serial Comunication

// Macro for Timing

#define delayus(args) 		(delayMicroseconds(args))  			// Delay Microseconds

// Macro for Comunication Flow Control

#define setDPin(DirPin,Mode)   (pinMode(DirPin,Mode))       // Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) (digitalWrite(DirPin,Mode))  // Switch to TX/RX Mode

unsigned short motorNumber[50] = {10,12,18,24,28,29,30,64,107,113,116,300,310,311,320,321,350,360,1000,
                                1001,1010,1011,1020,1030,1040,1050,1060,1070,1080,1090,1100,1101,1110,
                                1111,1120,1130,1140,1150,1160,1170,1180,1190,1200,1210,1220,1230,1240,1270,1280};

const char motorModel[50][12] = {"RX-10","AX-12A","AX-18A","RX-24","RX-28","MX-28","2MX-28","RX-64","EX-106","DX-113",
                                "DX-116","AX-12W","MX-64","2MX-64","MX-106","2MX-106","XL320","MX-12W","XH430-W350",
                                "XD430-T350","XH430-W210","XD430-T210","XM430-W350","XM430-W210","XH430-V350","XH430-V210",
                                "XL430-W250","XC430-W150","XC430-W240","2XL430-W250","XH540-W270","XD540-T270","XH540-W150",
                                "XD540-T150","XM540-W270","XM540-W150","XH540-V270","XH540-V150","2XC430-W250","XW540-T260",
                                "XW540-T140","XL330-M077","XL330-M288","XC330-T181","XC330-T288","XC330-M181","XC330-M288",
                                "XW430-T333","XW430-T200"};

unsigned short motorP1Errors[15] = {0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x03,0x05,0x06,0x07,0x09,0x0A};
unsigned short motorP2Errors[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};

const char motorP1ErrorList[15][25] = {"OK","VOLTAGE ERROR","ANGLE ERROR","OVERHEAT ERROR","RANGE ERROR","CHECKSUM ERROR","OVERLOAD ERROR","INSTRUCTION ERROR","NO RESPONSE","VLT & ANGL","VLT & OVHT","ANGL & OVHT","VLT, ANGL & OVHT","VLT & RNG","ANGL & RNG"};
const char motorP2ErrorList[8][25] = {"OK","PROCESS ERROR","INSTRUCTION ERROR","CRC ERROR","RANGE ERROR","LENGTH ERROR","LIMIT ERROR","ACCESS ERROR"};


// Private Methods //////////////////////////////////////////////////////////////

unsigned short generateCRC(unsigned char *dataPacket, unsigned short packetSize){
	unsigned short i, j,crc = 0;
	unsigned short crc_table[256] = {
	    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
	    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
	    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
	    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
	    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
	    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
	    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
	    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
	    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
	    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
	    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
	    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
	    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
	    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
	    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
	    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
	    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
	    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
	    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
	    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
	    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
	    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
	    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
	    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
	    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
	    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
	    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
	    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
	    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
	    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
	    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
	    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202 };
	for(j = 0; j < packetSize; j++){
	    i = ((unsigned short)(crc >> 8) ^ dataPacket[j]) & 0xFF;
	    crc = (crc << 8) ^ crc_table[i];}
	return crc;
}

long DynamixelClass::readRegister(unsigned char registerLenght)
{
	long registerValue = -1;
	if (protocolVersion == 2){
		unsigned short timeCounter = 0;
		while( (availableData() < (11 + registerLenght)) && (timeCounter < TIME_OUT) ){
			timeCounter++;									// Wait for Data
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == DNMXL_H1) && (peekData() == DNMXL_H2) ){
				headerStart = readData();                   // Start Bytes   0xFF
				headerStart = readData();                   // Start Bytes   0xFD
				reservedByte = readData();                  // Reserved Byte 0x00
				responseID = readData();                    // Dynamixel ID  
				packetLenghtLow = readData();               // Length Low    0x07
				packetLenghtHigh = readData();              // Length High   0x00
				instructionByte = readData();				// Instruction   0x55 (Status)
				errorByte = readData();        				// Error
				if (registerLenght == 1){
					registerValue = readData();        		// Register
				}else if (registerLenght == 2){
					parameter1 = readData();				// Parameter 1 LSB
					parameter2 = readData();				// Parameter 2 MSB
					registerValue =  parameter2 << 8;
					registerValue += parameter1;
				}else if (registerLenght == 4){
					parameter1 = readData();				// Parameter 1 LSB
					parameter2 = readData();				// Parameter 2 
					parameter3 = readData();				// Parameter 3 
					parameter4 = readData();				// Parameter 4 MSB
					registerValue = parameter4 << 24;
					registerValue += parameter3  << 16;
					registerValue += parameter2  << 8;
					registerValue += parameter1;
				}
				checkSum = readData();						// Checksum Low
				checkSum = readData();						// Checksum High
				return (registerValue);						// Return Register Value
			}
		}
		return (registerValue*NO_RESPONSE_ERROR);			// No Response
	} else if (protocolVersion == 1){
		short registerValue = -1;
		unsigned short timeCounter = 0;
		while( (availableData() <  (6 + registerLenght)) && (timeCounter < TIME_OUT) ){
			timeCounter++;
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == AX_START) && (peekData() == AX_START) ){
				headerStart = readData();                       // Start Bytes
				responseID = readData();                        // Ax-12 ID
				packetLenght = readData();                      // Length
				errorByte = readData();            				// Error
				if (registerLenght == 1){
					registerValue = readData();        			// Register
				}else if (registerLenght == 2){
					registerLowByte = readData();               // Register Bytes
					registerHighByte = readData();
					registerValue =  registerHighByte << 8;
					registerValue += registerLowByte;
				}
				checkSum = readData();
				return (registerValue);							// Return Register Value
			}
		}
	}
	return (registerValue*NO_RESPONSE_ERROR);					// No Response
}



short DynamixelClass::LXreadRegister(unsigned char registerLenght)
{
		long registerValue = -1;
		unsigned short timeCounter = 0;
		while( (availableData() <  (6 + registerLenght)) && (timeCounter < TIME_OUT) ){
			timeCounter++;
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == LX_START_BYTE) && (peekData() == LX_START_BYTE) ){
				headerStart = readData();                       // Start Bytes
				responseID = readData();                        // Ax-12 ID
				packetLenght = readData();                      // Length
				errorByte = readData();            				// Error
				if (registerLenght == 1){
					registerValue = readData();        			// Register
				}else if (registerLenght == 2){
					registerLowByte = readData();               // Register Bytes
					registerHighByte = readData();
					registerValue =  registerHighByte << 8;
					registerValue += registerLowByte;
				}
				checkSum = readData();
				return (registerValue);							// Return Register Value
			}
		}
	
	return (registerValue*NO_RESPONSE_ERROR);					// No Response
}




long DynamixelClass::readRegister(unsigned char registerLenght, unsigned char returnValue)
{
	long registerValue = -1;
	if (protocolVersion == 2){
		unsigned short timeCounter = 0;
		while( (availableData() < (11 + registerLenght)) && (timeCounter < TIME_OUT) ){
			timeCounter++;									// Wait for Data
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == DNMXL_H1) && (peekData() == DNMXL_H2) ){
				headerStart = readData();                   // Start Bytes   0xFF
				headerStart = readData();                   // Start Bytes   0xFD
				reservedByte = readData();                  // Reserved Byte 0x00
				responseID = readData();                    // Dynamixel ID  
				packetLenghtLow = readData();               // Length Low    0x07
				packetLenghtHigh = readData();              // Length High   0x00
				instructionByte = readData();				// Instruction   0x55 (Status)
				errorByte = readData();       				// Error
				if (registerLenght == 1){
					registerValue = readData();        		// Register
				}else if (registerLenght == 2){
					parameter1 = readData();				// Parameter 1 LSB
					parameter2 = readData();				// Parameter 2 MSB
					registerValue =  parameter2 << 8;
					registerValue += parameter1;
				}else if (registerLenght == 4){
					parameter1 = readData();				// Parameter 1 LSB
					parameter2 = readData();				// Parameter 2 
					parameter3 = readData();				// Parameter 3 
					parameter4 = readData();				// Parameter 4 MSB
					registerValue = parameter4 << 24;
					registerValue += parameter3  << 16;
					registerValue += parameter2  << 8;
					registerValue += parameter1;
				}
				checkSum = readData();						// Checksum Low
				checkSum = readData();						// Checksum High
				return (registerValue);						// Return Register Value
			}
		}
		return (registerValue*NO_RESPONSE_ERROR);			// No Response
	} else if (protocolVersion == 1){
		short registerValue = -1;
		unsigned short timeCounter = 0;
		while( (availableData() <  (6 + registerLenght)) && (timeCounter < TIME_OUT) ){
			timeCounter++;
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == AX_START) && (peekData() == AX_START) ){
				headerStart = readData();                       // Start Bytes
				responseID = readData();                        // Ax-12 ID
				packetLenght = readData();                      // Length
				errorByte = readData();							// Error
				if (registerLenght == 1){
					registerValue = readData();        			// Register
				}else if (registerLenght == 2){
					registerLowByte = readData();               // Register Bytes
					registerHighByte = readData();
					registerValue =  registerHighByte << 8;
					registerValue += registerLowByte;
				}
				checkSum = readData();
				return (registerValue);							// Return Register Value
			}
		}
	}
	return (registerValue*NO_RESPONSE_ERROR);					// No Response
}


short DynamixelClass::readError(void)
{
	if (protocolVersion == 2){
		unsigned short timeCounter = 0;
		while((availableData() < (15)) && (timeCounter < TIME_OUT)){  
			timeCounter++;									// Wait for Data
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == DNMXL_H1) && (peekData() == DNMXL_H2) ){
				headerStart = readData();                   // Start Bytes   0xFF
				headerStart = readData();                   // Start Bytes   0xFD
				reservedByte = readData();                  // Reserved Byte 0x00
				responseID = readData();                    // Dynamixel ID  
				packetLenghtLow = readData();               // Length Low    0x07
				packetLenghtHigh = readData();              // Length High   0x00
				instructionByte = readData();				// Instruction   0x55 (Status)
				errorByte = readData();                     // Error  
				parameter1 = readData();					// Parameter 1 Model LSB
				parameter2 = readData();					// Parameter 2 Model MSB
				parameter3 = readData();					// Parameter 3 Firmware version
				checkSum = readData();						// Checksum Low
				checkSum = readData();						// Checksum High
				return (errorByte);							// Return Error
			}
		}
	} else if (protocolVersion == 1){
		unsigned short timeCounter = 0;
		while((availableData() < 6) && (timeCounter < TIME_OUT)){  
			timeCounter++;									// Wait for Data
			delayus(TIME_COUNTER_DELAY);
		}
		delayus(TIME_COUNTER_DELAY*2);

		while (availableData() > 2){
			incomingByte = readData();
			if ( (incomingByte == AX_START) && (peekData() == AX_START) ){
				headerStart = readData();                   // Start Bytes
				responseID = readData();                    // Dynamixel ID
				packetLenght = readData();                  // Length
				errorByte = readData();                     // Error
				checkSum = readData();						// checksum
				return (errorByte);						    // Return Error
			}
		}
	}
	return (NO_RESPONSE_ERROR*(-1));							// No Response
}

// Public Methods //////////////////////////////////////////////////////////////

/*
    void DynamixelClass::begin(SoftwareSerial *sPort, long bRate, unsigned char dPin, unsigned char pVersion)
	{	
		softwareSerialPort = sPort;
		directionPin = dPin;
		protocolVersion = pVersion;
		setDPin(directionPin,OUTPUT);
		beginCom(bRate);

		serialBufferLenght = availableWrite();
	}

*/

void DynamixelClass::begin(HardwareSerial *sPort, long bRate, unsigned char dPin, unsigned char pVersion)
{	
	serialPort = sPort;
	directionPin = dPin;
	protocolVersion = pVersion;
	setDPin(directionPin,OUTPUT);
	beginCom(bRate);

	timeDelay = abs(((9600*DELAY_TX_TIME_MULTP)/bRate)-DELAY_TX_TIME_OFFSET);

	serialBufferLenght = availableWrite();
}


void DynamixelClass::setProtocolVersion(unsigned char pVersion)
{		
	protocolVersion = pVersion;
}	

void DynamixelClass::end()
{
	endCom();
}

short DynamixelClass::reset(unsigned char motorID, unsigned char resetMode)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[9] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE4_LENGTH_LOW,DNMXL_WRITE4_LENGTH_HIGH,DNMXL_RESET,resetMode};
    	unsigned short crcBytes = generateCRC(dataPacket,9);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE4_LENGTH_LOW);
		sendData(DNMXL_WRITE4_LENGTH_HIGH);
		sendData(DNMXL_RESET);
		sendData(resetMode);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_RESET_LENGTH + AX_RESET))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                     
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_RESET_LENGTH);
		sendData(AX_RESET);    
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}

    return (readError());  
}

short DynamixelClass::reset(unsigned char motorID)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[9] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE4_LENGTH_LOW,DNMXL_WRITE4_LENGTH_HIGH,DNMXL_RESET,RESET_ALL};
    	unsigned short crcBytes = generateCRC(dataPacket,9);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE4_LENGTH_LOW);
		sendData(DNMXL_WRITE4_LENGTH_HIGH);
		sendData(DNMXL_RESET);
		sendData(RESET_ALL);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_RESET_LENGTH + AX_RESET))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                     
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_RESET_LENGTH);
		sendData(AX_RESET);    
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}

    return (readError());  
}

short DynamixelClass::ping(unsigned char motorID)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[8] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE3_LENGTH_LOW,DNMXL_WRITE3_LENGTH_HIGH,DNMXL_PING};
    	unsigned short crcBytes = generateCRC(dataPacket,8);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE3_LENGTH_LOW);
		sendData(DNMXL_WRITE3_LENGTH_HIGH);
		sendData(DNMXL_PING);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_READ_DATA + AX_PING))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                     
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_READ_DATA);
		sendData(AX_PING);    
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}

    return (readError());              
}

short DynamixelClass::setID(unsigned char motorID, unsigned char newID)
{    
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_ID_REG_LOW,DNMXL_ID_REG_HIGH,newID};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_ID_REG_LOW);
		sendData(DNMXL_ID_REG_HIGH);
		sendData(newID);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_ID_LENGTH + AX_WRITE_DATA + AX_ID + newID))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_ID_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_ID);
    	sendData(newID);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
  
    return (readError());                
}

short DynamixelClass::setBaudRate(unsigned char motorID, long bRate)
{   
	if (protocolVersion == 2){
		switch(bRate){
		case 9600:    bRate = 0; break;
		case 57600:   bRate = 1; break;
		case 115200:  bRate = 2; break;
		case 1000000: bRate = 3; break;
		case 2000000: bRate = 4; break;
		case 3000000: bRate = 5; break;
		case 4000000: bRate = 6; break;
		default:      bRate = 1; break;}

		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_BR_REG_LOW,DNMXL_BR_REG_HIGH,(uint8_t)bRate};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_BR_REG_LOW);
		sendData(DNMXL_BR_REG_HIGH);
		sendData(bRate);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
		unsigned char baudRate = (2000000/bRate) - 1;
    	unsigned char checksum = (~(motorID + AX_BD_LENGTH + AX_WRITE_DATA + AX_BAUD_RATE + baudRate))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                 
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_BD_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_BAUD_RATE);
    	sendData(baudRate);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }

    return (readError());                
}

short DynamixelClass::setDriveMode(unsigned char motorID, unsigned char  driveMode)
{
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_DRVMODE_REG_LOW,DNMXL_DRVMODE_REG_HIGH,driveMode};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_DRVMODE_REG_LOW);
		sendData(DNMXL_DRVMODE_REG_HIGH);
		sendData(driveMode);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);

    return (readError());                 
}

short DynamixelClass::setOperationMode(unsigned char motorID, unsigned char  operationMode)
{
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_OPMODE_REG_LOW,DNMXL_OPMODE_REG_HIGH,operationMode};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_OPMODE_REG_LOW);
		sendData(DNMXL_OPMODE_REG_HIGH);
		sendData(operationMode);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);

    return (readError());                 
}

short DynamixelClass::setProfileVelocity(unsigned char motorID, long profileVelocity)
{
		unsigned char velocityByte1H = profileVelocity >> 24; 
		unsigned char velocityByte1L = profileVelocity >> 16; 
		unsigned char velocityByteH = profileVelocity >> 8;           // 16 bits - 2 x 8 bits variables
    	unsigned char velocityByteL = profileVelocity;
		unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_PVEL_REG_LOW,DNMXL_PVEL_REG_HIGH,velocityByteL,velocityByteH,velocityByte1L,velocityByte1H};
    	unsigned short crcBytes = generateCRC(dataPacket,14);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE9_LENGTH_LOW);
		sendData(DNMXL_WRITE9_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_PVEL_REG_LOW);
		sendData(DNMXL_PVEL_REG_HIGH);
		sendData(velocityByteL);
		sendData(velocityByteH);
		sendData(velocityByte1L);
		sendData(velocityByte1H);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);

    return (readError());                 
}


short DynamixelClass::setGoalPWM(unsigned char motorID, short motorPWM){
	unsigned char pwmByteH = motorPWM >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char pwmByteL = motorPWM;
	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE7_LENGTH_LOW,DNMXL_WRITE7_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_GOALPWM_REG_LOW,DNMXL_GOALPWM_REG_HIGH,pwmByteL,pwmByteH};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_WRITE7_LENGTH_LOW);
	sendData(DNMXL_WRITE7_LENGTH_HIGH);
	sendData(DNMXL_WRITE);
	sendData(DNMXL_GOALPWM_REG_LOW);
	sendData(DNMXL_GOALPWM_REG_HIGH);
	sendData(pwmByteL);
	sendData(pwmByteH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);

	return (readError()); 

}

short DynamixelClass::setGoalCurrent(unsigned char motorID, short motorCurrent){
	unsigned char currentByteH = motorCurrent >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char currentByteL = motorCurrent;
	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE7_LENGTH_LOW,DNMXL_WRITE7_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_GOALCURRENT_REG_LOW,DNMXL_GOALCURRENT_REG_HIGH,currentByteL,currentByteH};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_WRITE7_LENGTH_LOW);
	sendData(DNMXL_WRITE7_LENGTH_HIGH);
	sendData(DNMXL_WRITE);
	sendData(DNMXL_GOALCURRENT_REG_LOW);
	sendData(DNMXL_GOALCURRENT_REG_HIGH);
	sendData(currentByteL);
	sendData(currentByteH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);

	return (readError()); 
}

short DynamixelClass::setGoalVelocity(unsigned char motorID, long motorVelocity)
{
	unsigned char velocityByte1H = motorVelocity >> 24; 
	unsigned char velocityByte1L = motorVelocity >> 16; 
	unsigned char velocityByteH = motorVelocity >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char velocityByteL = motorVelocity;
	
	unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_GOALVEL_REG_LOW,DNMXL_GOALVEL_REG_HIGH,velocityByteL,velocityByteH,velocityByte1L,velocityByte1H};
    unsigned short crcBytes = generateCRC(dataPacket,14);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_WRITE9_LENGTH_LOW);
	sendData(DNMXL_WRITE9_LENGTH_HIGH);
	sendData(DNMXL_WRITE);
	sendData(DNMXL_GOALVEL_REG_LOW);
	sendData(DNMXL_GOALVEL_REG_HIGH);
	sendData(velocityByteL);
	sendData(velocityByteH);
	sendData(velocityByte1L);
	sendData(velocityByte1H);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);

	return (readError()); 
}

short DynamixelClass::setGoalPosition(unsigned char motorID, long motorPosition){

	unsigned char positionByte1L = motorPosition >> 24;           
    unsigned char positionByte1H = motorPosition >> 16;
	unsigned char positionByteH = motorPosition >> 8;           
    unsigned char positionByteL = motorPosition;
	
	if (protocolVersion == 2){
		unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_GOALPOS_REG_LOW,DNMXL_GOALPOS_REG_HIGH,positionByteL,positionByteH,positionByte1L,positionByte1H};
    	unsigned short crcBytes = generateCRC(dataPacket,14);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
		sendData(DNMXL_H1);
		sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE9_LENGTH_LOW);
		sendData(DNMXL_WRITE9_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_GOALPOS_REG_LOW);
		sendData(DNMXL_GOALPOS_REG_HIGH);
		sendData(positionByteL);
		sendData(positionByteH);
		sendData(positionByte1L);
		sendData(positionByte1H);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + positionByteL + positionByteH))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                 
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_GOAL_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_GOAL_POSITION_L);
    	sendData(positionByteL);
    	sendData(positionByteH);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}

	return (readError()); 
}


short DynamixelClass::move(unsigned char motorID, long motorPosition)
{
		unsigned char positionByte1H = motorPosition >> 24;
		unsigned char positionByte1L = motorPosition >> 16;
		unsigned char positionByteH = motorPosition >> 8;           // 16 bits - 2 x 8 bits variables
    	unsigned char positionByteL = motorPosition;
	if (protocolVersion == 2){
		unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_GOALPOS_REG_LOW,DNMXL_GOALPOS_REG_HIGH,positionByteL,positionByteH,positionByte1L,positionByte1H};
    	unsigned short crcBytes = generateCRC(dataPacket,14);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE9_LENGTH_LOW);
		sendData(DNMXL_WRITE9_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_GOALPOS_REG_LOW);
		sendData(DNMXL_GOALPOS_REG_HIGH);
		sendData(positionByteL);
		sendData(positionByteH);
		sendData(positionByte1L);
		sendData(positionByte1H);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
		unsigned char checksum = (~(motorID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + positionByteL + positionByteH))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                 
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_GOAL_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_GOAL_POSITION_L);
    	sendData(positionByteL);
    	sendData(positionByteH);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}

    return (readError());                 
}

short DynamixelClass::moveSpeed(unsigned char motorID, long motorPosition, long motorSpeed)
{
	short instructionError1 = 0, instructionError2 = 0;
	if (protocolVersion == 2){
		instructionError1 = setProfileVelocity(motorID,motorSpeed);
		instructionError2 = move(motorID,motorPosition);            
	} else if (protocolVersion == 1){
    	unsigned char positionByteH = motorPosition >> 8;
    	unsigned char positionByteL = motorPosition;                // 16 bits - 2 x 8 bits variables
    	unsigned char speedByteH = motorSpeed >> 8;
    	unsigned char speedByteL = motorSpeed;                      // 16 bits - 2 x 8 bits variables
		unsigned char checksum = (~(motorID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L + positionByteL + positionByteH + speedByteL + speedByteH))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_GOAL_SP_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_GOAL_POSITION_L);
    	sendData(positionByteL);
    	sendData(positionByteH);
    	sendData(speedByteL);
    	sendData(speedByteH);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);          
	}
	return (readError() | instructionError1 | instructionError2);  
}

short DynamixelClass::moveRW(unsigned char motorID, long motorPosition)
{
    unsigned char positionByteH = motorPosition >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char positionByteL = motorPosition;
    unsigned char checksum = (~(motorID + AX_GOAL_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + positionByteL + positionByteH))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                 
    sendData(AX_START);
    sendData(motorID);
    sendData(AX_GOAL_LENGTH);
    sendData(AX_REG_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(positionByteL);
    sendData(positionByteH);
    sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError());                 
}

short DynamixelClass::moveSpeedRW(unsigned char motorID, long motorPosition, long motorSpeed)
{
    unsigned char positionByteH = motorPosition >> 8;    
    unsigned char positionByteL = motorPosition;                // 16 bits - 2 x 8 bits variables
    unsigned char speedByteH = motorSpeed >> 8;
    unsigned char speedByteL = motorSpeed;                      // 16 bits - 2 x 8 bits variables
    unsigned char checksum = (~(motorID + AX_GOAL_SP_LENGTH + AX_REG_WRITE + AX_GOAL_POSITION_L + positionByteL + positionByteH + speedByteL + speedByteH))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                
    sendData(AX_START);
    sendData(motorID);
    sendData(AX_GOAL_SP_LENGTH);
    sendData(AX_REG_WRITE);
    sendData(AX_GOAL_POSITION_L);
    sendData(positionByteL);
    sendData(positionByteH);
    sendData(speedByteL);
    sendData(speedByteH);
    sendData(checksum);
    while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE); 
    return (readError());              
}

void DynamixelClass::action()
{	
	unsigned char checksum = (~(BROADCAST_ID + AX_ACTION_LENGTH + AX_ACTION))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                
    sendData(AX_START);
    sendData(BROADCAST_ID);
    sendData(AX_ACTION_LENGTH);
    sendData(AX_ACTION);
    sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
}

short DynamixelClass::setEndless(unsigned char motorID, bool motorMode)
{
 	if ( motorMode ) {	
		unsigned char checksum = (~(motorID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_WHEEL))&0xFF;
	  	switchCom(directionPin,TX_MODE);
      	sendData(AX_START);                
      	sendData(AX_START);
      	sendData(motorID);
      	sendData(AX_GOAL_LENGTH);
      	sendData(AX_WRITE_DATA);
      	sendData(AX_CCW_ANGLE_LIMIT_L );
      	sendData(AX_CCW_AL_WHEEL);
      	sendData(AX_CCW_AL_WHEEL);
      	sendData(checksum);
      	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	  	switchCom(directionPin,RX_MODE);
 	} else {
	 	setRotation(motorID,0,0);
	 	unsigned char checksum = (~(motorID + AX_GOAL_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L + AX_CCW_AL_L + AX_CCW_AL_H))&0xFF;
	 	switchCom(directionPin,TX_MODE);
	 	sendData(AX_START);                 
	 	sendData(AX_START);
	 	sendData(motorID);
	 	sendData(AX_GOAL_LENGTH);
	 	sendData(AX_WRITE_DATA);
	 	sendData(AX_CCW_ANGLE_LIMIT_L);
	 	sendData(AX_CCW_AL_L);
	 	sendData(AX_CCW_AL_H);
	 	sendData(checksum);
	 	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	 	switchCom(directionPin,RX_MODE);
 	}
  	 return (readError());                 
 } 

short DynamixelClass::setRotation(unsigned char motorID, bool turnDirection, short turnSpeed)
{		
	if (turnDirection == CCW_DIRECTION){                  // Move Left///////////////////////////
		unsigned char speedByteH = turnSpeed >> 8;
		unsigned char speedByteL = turnSpeed;                     // 16 bits - 2 x 8 bits variables
		unsigned char checksum = (~(motorID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + speedByteL + speedByteH))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_SPEED_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_GOAL_SPEED_L);
		sendData(speedByteL);
		sendData(speedByteH);
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else {                                            // Move Rigth////////////////////
		unsigned char speedByteH = (turnSpeed >> 8) + 4;
		unsigned char speedByteL = turnSpeed;                     // 16 bits - 2 x 8 bits variables
		unsigned char checksum = (~(motorID + AX_SPEED_LENGTH + AX_WRITE_DATA + AX_GOAL_SPEED_L + speedByteL + speedByteH))&0xFF;		
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_SPEED_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_GOAL_SPEED_L);
		sendData(speedByteL);
		sendData(speedByteH);
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);      
		}
	return(readError());    
}

short DynamixelClass::setTorque( unsigned char motorID, bool torqueStatus)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_TRQ_REG_LOW,DNMXL_TRQ_REG_HIGH,torqueStatus};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_TRQ_REG_LOW);
		sendData(DNMXL_TRQ_REG_HIGH);
		sendData(torqueStatus);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
    	unsigned char checksum = (~(motorID + AX_TORQUE_LENGTH + AX_WRITE_DATA + AX_TORQUE_ENABLE + torqueStatus))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);              
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TORQUE_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_TORQUE_ENABLE);
    	sendData(torqueStatus);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());              
}

short DynamixelClass::setTorqueMode( unsigned char motorID, bool torqueStatus)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_TRQ_REG_LOW,DNMXL_TRQ_REG_HIGH,torqueStatus};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_TRQ_REG_LOW);
		sendData(DNMXL_TRQ_REG_HIGH);
		sendData(torqueStatus);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){
    	unsigned char checksum = (~(motorID + AX_TORQUE_LENGTH + AX_WRITE_DATA + MX_TORQUE_CTRL_MODE_EN + torqueStatus))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);              
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TORQUE_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(MX_TORQUE_CTRL_MODE_EN);
    	sendData(torqueStatus);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());              
}

short DynamixelClass::setLED(unsigned char motorID, bool ledStatus)
{   
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_LED_REG_LOW,DNMXL_LED_REG_HIGH,ledStatus};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_LED_REG_LOW);
		sendData(DNMXL_LED_REG_HIGH);
		sendData(ledStatus);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_LED_LENGTH + AX_WRITE_DATA + AX_LED + ledStatus))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);              
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_LED_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_LED);
    	sendData(ledStatus);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());              
}

short DynamixelClass::setTempLimit(unsigned char motorID, unsigned char motorTemperature)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_TEMPLIMIT_REG_LOW,DNMXL_TEMPLIMIT_REG_HIGH,motorTemperature};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_TEMPLIMIT_REG_LOW);
		sendData(DNMXL_TEMPLIMIT_REG_HIGH);
		sendData(motorTemperature);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
		unsigned char checksum = (~(motorID + AX_TL_LENGTH +AX_WRITE_DATA+ AX_LIMIT_TEMPERATURE + motorTemperature))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                     
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_TL_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_LIMIT_TEMPERATURE);
    	sendData(motorTemperature);
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
    return (readError()); 
}

short DynamixelClass::setVoltageLimit(unsigned char motorID, unsigned char minVoltage, unsigned char maxVoltage)
{
	if (protocolVersion == 2){
		unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_MAXVOLTLIMIT_REG_LOW,DNMXL_MAXVOLTLIMIT_REG_HIGH,maxVoltage,DNMXL_RSVD,minVoltage,DNMXL_RSVD};
    	unsigned short crcBytes = generateCRC(dataPacket,14);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE9_LENGTH_LOW);
		sendData(DNMXL_WRITE9_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_MAXVOLTLIMIT_REG_LOW);
		sendData(DNMXL_MAXVOLTLIMIT_REG_HIGH);
		sendData(maxVoltage);
		sendData(DNMXL_RSVD);
		sendData(minVoltage);
		sendData(DNMXL_RSVD);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
		unsigned char checksum = (~(motorID + AX_VL_LENGTH +AX_WRITE_DATA+ AX_DOWN_LIMIT_VOLTAGE + minVoltage + maxVoltage))&0xFF;
		switchCom(directionPin,TX_MODE);
		sendData(AX_START);                     
		sendData(AX_START);
		sendData(motorID);
		sendData(AX_VL_LENGTH);
		sendData(AX_WRITE_DATA);
		sendData(AX_DOWN_LIMIT_VOLTAGE);
    	sendData(minVoltage);
    	sendData(maxVoltage);
		sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
    return (readError()); 
}

short DynamixelClass::setCWAngleLimit(unsigned char motorID, short motorCWLimit)
{
    unsigned char cwByteH = motorCWLimit >> 8;     // 16 bits - 2 x 8 bits variables
    unsigned char cwByteL = motorCWLimit;               
	unsigned char checksum = (~(motorID + AX_CCW_CW_LENGTH + AX_WRITE_DATA + AX_CW_ANGLE_LIMIT_L + cwByteH + cwByteL))&0xFF;	
	switchCom(directionPin,TX_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(motorID);
	sendData(AX_CCW_CW_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_ANGLE_LIMIT_L);
    sendData(cwByteL);
	sendData(cwByteH);
	sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
	delayus(20000);
    return (readError()); 
}

short DynamixelClass::setCCWAngleLimit(unsigned char motorID, short motorCCWLimit)
{
    unsigned char ccwByteH = motorCCWLimit >> 8; // 16 bits - 2 x 8 bits variables
    unsigned char ccwByteL = motorCCWLimit;  
	unsigned char checksum = (~(motorID + AX_CCW_CW_LENGTH + AX_WRITE_DATA + AX_CCW_ANGLE_LIMIT_L  + ccwByteH + ccwByteL))&0xFF;
	switchCom(directionPin,TX_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(motorID);
	sendData(AX_CCW_CW_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CCW_ANGLE_LIMIT_L);
    sendData(ccwByteL);
	sendData(ccwByteH);
	sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
	delayus(20000);
    return (readError()); 
}

short DynamixelClass::setMaxTorque(unsigned char motorID, short maxTorque)
{
    unsigned char maxTorqueByteH = maxTorque >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char maxTorqueByteL = maxTorque;
	unsigned char checksum = (~(motorID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L + maxTorqueByteL + maxTorqueByteH))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                 
    sendData(AX_START);
    sendData(motorID);
    sendData(AX_MT_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_MAX_TORQUE_L);
    sendData(maxTorqueByteL);
    sendData(maxTorqueByteH);
    sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError());                 
}

short DynamixelClass::setSRL(unsigned char motorID, unsigned char motorSRL)
{    
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_SRL_REG_LOW,DNMXL_SRL_REG_HIGH,motorSRL};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_SRL_REG_LOW);
		sendData(DNMXL_SRL_REG_HIGH);
		sendData(motorSRL);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
		unsigned char checksum = (~(motorID + AX_SRL_LENGTH + AX_WRITE_DATA + AX_RETURN_LEVEL + motorSRL))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_SRL_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_RETURN_LEVEL);
    	sendData(motorSRL);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());                
}

short DynamixelClass::setRDT(unsigned char motorID, unsigned char motorRDT)
{   
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_RDT_REG_LOW,DNMXL_RDT_REG_HIGH,motorRDT};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_RDT_REG_LOW);
		sendData(DNMXL_RDT_REG_HIGH);
		sendData(motorRDT);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
		unsigned char checksum = (~(motorID + AX_RDT_LENGTH + AX_WRITE_DATA + AX_RETURN_DELAY_TIME + (motorRDT/2)))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_RDT_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_RETURN_DELAY_TIME);
    	sendData((motorRDT/2));
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());                
}

short DynamixelClass::setLEDAlarm(unsigned char motorID, unsigned char ledAlarm)
{    
	unsigned char checksum = (~(motorID + AX_LEDALARM_LENGTH + AX_WRITE_DATA + AX_ALARM_LED + ledAlarm))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                
    sendData(AX_START);
    sendData(motorID);
	sendData(AX_LEDALARM_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_ALARM_LED);
    sendData(ledAlarm);
    sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError());                
}

short DynamixelClass::setShutdownAlarm(unsigned char motorID, unsigned char motorAlarm)
{   
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_SHUTDOWN_REG_LOW,DNMXL_SHUTDOWN_REG_HIGH,motorAlarm};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_SHUTDOWN_REG_LOW);
		sendData(DNMXL_SHUTDOWN_REG_HIGH);
		sendData(motorAlarm);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){   
		unsigned char checksum = (~(motorID + AX_SALARM_LENGTH + AX_ALARM_SHUTDOWN + AX_ALARM_LED + motorAlarm))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_SALARM_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_ALARM_SHUTDOWN);
    	sendData(motorAlarm);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());                
}

short DynamixelClass::setCMargin(unsigned char motorID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
	unsigned char checksum = (~(motorID + AX_CM_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_MARGIN + CWCMargin + AX_CCW_COMPLIANCE_MARGIN + CCWCMargin))&0xFF;
	switchCom(directionPin,TX_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(motorID);
	sendData(AX_CM_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_COMPLIANCE_MARGIN);
    sendData(CWCMargin);
	sendData(AX_CCW_COMPLIANCE_MARGIN);
    sendData(CCWCMargin);
	sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError()); 
}

short DynamixelClass::setCSlope(unsigned char motorID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
	unsigned char checksum = (~(motorID + AX_CS_LENGTH +AX_WRITE_DATA+ AX_CW_COMPLIANCE_SLOPE + CWCSlope + AX_CCW_COMPLIANCE_SLOPE + CCWCSlope))&0xFF;
	switchCom(directionPin,TX_MODE);
	sendData(AX_START);                     
	sendData(AX_START);
	sendData(motorID);
	sendData(AX_CS_LENGTH);
	sendData(AX_WRITE_DATA);
	sendData(AX_CW_COMPLIANCE_SLOPE);
    sendData(CWCSlope);
	sendData(AX_CCW_COMPLIANCE_SLOPE);
    sendData(CCWCSlope);
	sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError()); 
}

short DynamixelClass::setPunch(unsigned char motorID, short motorPunch)
{
    unsigned char punchByteH = motorPunch >> 8;           // 16 bits - 2 x 8 bits variables
    unsigned char punchByteL = motorPunch;
	unsigned char checksum = (~(motorID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L + punchByteL + punchByteH))&0xFF;   
	switchCom(directionPin,TX_MODE);
    sendData(AX_START);                 
    sendData(AX_START);
    sendData(motorID);
    sendData(AX_PUNCH_LENGTH);
    sendData(AX_WRITE_DATA);
    sendData(AX_PUNCH_L);
    sendData(punchByteL);
    sendData(punchByteH);
    sendData(checksum);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
    return (readError());                 
}

short DynamixelClass::setLockRegister(unsigned char motorID, bool lockStatus)
{   
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,DNMXL_TRQ_REG_LOW,DNMXL_TRQ_REG_HIGH,lockStatus};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(DNMXL_TRQ_REG_LOW);
		sendData(DNMXL_TRQ_REG_HIGH);
		sendData(lockStatus);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){ 
		unsigned char checksum = (~(motorID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + lockStatus))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);                
    	sendData(AX_START);
    	sendData(motorID);
		sendData(AX_LR_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(AX_LOCK);
    	sendData(lockStatus);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readError());                
}

short DynamixelClass::readMovingStatus(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_MOVING_REG_LOW,DNMXL_MOVING_REG_HIGH,DNMXL_DATA1_LOW,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_MOVING_REG_LOW);
		sendData(DNMXL_MOVING_REG_HIGH);
		sendData(DNMXL_DATA1_LOW);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_MOVING_LENGTH  + AX_READ_DATA + AX_MOVING + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_MOVING_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_MOVING);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
    return (readRegister(1)); 
}

short DynamixelClass::readRWStatus(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_REGINST_REG_LOW,DNMXL_REGINST_REG_HIGH,DNMXL_DATA1_LOW,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_REGINST_REG_LOW);
		sendData(DNMXL_REGINST_REG_HIGH);
		sendData(DNMXL_DATA1_LOW);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_RWS_LENGTH  + AX_READ_DATA + AX_REGISTERED_INSTRUCTION + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_RWS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_REGISTERED_INSTRUCTION);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
    return (readRegister(1)); 
}

short DynamixelClass::readTorque(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_TRQ_REG_LOW,DNMXL_TRQ_REG_HIGH,DNMXL_DATA1_LOW,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_TRQ_REG_LOW);
		sendData(DNMXL_TRQ_REG_HIGH);
		sendData(DNMXL_DATA1_LOW);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_RWS_LENGTH  + AX_READ_DATA + AX_TORQUE_ENABLE + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_RWS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_TORQUE_ENABLE);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
    return (readRegister(1)); 
}

short DynamixelClass::readTemperature(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_TEMPERATURE_REG_LOW,DNMXL_TEMPERATURE_REG_HIGH,DNMXL_DATA1_LOW,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_TEMPERATURE_REG_LOW);
		sendData(DNMXL_TEMPERATURE_REG_HIGH);
		sendData(DNMXL_DATA1_LOW);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_TEM_LENGTH  + AX_READ_DATA + AX_PRESENT_TEMPERATURE + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TEM_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_PRESENT_TEMPERATURE);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(1));      
}

long DynamixelClass::readPosition(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_POSITION_REG_LOW,DNMXL_POSITION_REG_HIGH,DNMXL_DATA4_LOW,DNMXL_DATA4_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_POSITION_REG_LOW);
		sendData(DNMXL_POSITION_REG_HIGH);
		sendData(DNMXL_DATA4_LOW);
		sendData(DNMXL_DATA4_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){   
    	unsigned char checksum = (~(motorID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_POSITION_L + AX_BYTE_READ_POS))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_POS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_PRESENT_POSITION_L);
    	sendData(AX_BYTE_READ_POS);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(2*protocolVersion)); 
}

short DynamixelClass::readVoltage(unsigned char motorID)
{   
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_VOLTAGE_REG_LOW,DNMXL_VOLTAGE_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_VOLTAGE_REG_LOW);
		sendData(DNMXL_VOLTAGE_REG_HIGH);
		sendData(DNMXL_DATA2_LOW);
		sendData(DNMXL_DATA2_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
    	unsigned char checksum = (~(motorID + AX_VOLT_LENGTH  + AX_READ_DATA + AX_PRESENT_VOLTAGE + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_VOLT_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_PRESENT_VOLTAGE);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	}
	return (readRegister(1*protocolVersion));
}

long DynamixelClass::readSpeed(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_VELOCITY_REG_LOW,DNMXL_VELOCITY_REG_HIGH,DNMXL_DATA4_LOW,DNMXL_DATA4_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_VELOCITY_REG_LOW);
		sendData(DNMXL_VELOCITY_REG_HIGH);
		sendData(DNMXL_DATA4_LOW);
		sendData(DNMXL_DATA4_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
    	unsigned char checksum = (~(motorID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_SPEED_L + AX_BYTE_READ_POS))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
   		sendData(AX_POS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_PRESENT_SPEED_L);
    	sendData(AX_BYTE_READ_POS);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    }
    return (readRegister(2*protocolVersion)); 
}

short DynamixelClass::readLoad(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_LOAD_REG_LOW,DNMXL_LOAD_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_LOAD_REG_LOW);
		sendData(DNMXL_LOAD_REG_HIGH);
		sendData(DNMXL_DATA2_LOW);
		sendData(DNMXL_DATA2_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
    	unsigned char checksum = (~(motorID + AX_POS_LENGTH  + AX_READ_DATA + AX_PRESENT_LOAD_L + AX_BYTE_READ_POS))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_POS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_PRESENT_LOAD_L);
    	sendData(AX_BYTE_READ_POS);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    } 
    return (readRegister(2));
}

short DynamixelClass::readCurrent(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_CURRENT_REG_LOW,DNMXL_CURRENT_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
   		unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
		sendData(DNMXL_H1);
		sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_CURRENT_REG_LOW);
		sendData(DNMXL_CURRENT_REG_HIGH);
		sendData(DNMXL_DATA2_LOW);
		sendData(DNMXL_DATA2_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
		unsigned char checksum = (~(motorID + AX_POS_LENGTH  + AX_READ_DATA + MX_CURRENT + AX_BYTE_READ_POS))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_POS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(MX_CURRENT);
    	sendData(AX_BYTE_READ_POS);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(2)); 
}

short DynamixelClass::readPWM(unsigned char motorID)
{	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_PWM_REG_LOW,DNMXL_PWM_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_READ7_LENGTH_LOW);
	sendData(DNMXL_READ7_LENGTH_HIGH);
	sendData(DNMXL_READ);
	sendData(DNMXL_PWM_REG_LOW);
	sendData(DNMXL_PWM_REG_HIGH);
	sendData(DNMXL_DATA2_LOW);
	sendData(DNMXL_DATA2_HIGH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);
	return (readRegister(2)); 
}

short DynamixelClass::readModel(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_MODEL_REG_LOW,DNMXL_MODEL_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_MODEL_REG_LOW);
		sendData(DNMXL_MODEL_REG_HIGH);
		sendData(DNMXL_DATA2_LOW);
		sendData(DNMXL_DATA2_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	} else if (protocolVersion == 1){  
    	unsigned char checksum = (~(motorID + AX_POS_LENGTH  + AX_READ_DATA + AX_MODEL_NUMBER_L + AX_BYTE_READ_POS))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_POS_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_MODEL_NUMBER_L);
    	sendData(AX_BYTE_READ_POS);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
    } 
    return (readRegister(2));
}

short DynamixelClass::readFirmware(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_FVERS_REG_LOW,DNMXL_FVERS_REG_HIGH,DNMXL_DATA1_LOW,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_FVERS_REG_LOW);
		sendData(DNMXL_FVERS_REG_HIGH);
		sendData(DNMXL_DATA1_LOW);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_TEM_LENGTH  + AX_READ_DATA + AX_VERSION + AX_BYTE_READ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TEM_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_VERSION);
    	sendData(AX_BYTE_READ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(1));      
}

short DynamixelClass::readCurrentLimit(unsigned char motorID)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,DNMXL_CURRENTLIMIT_REG_LOW,DNMXL_CURRENTLIMIT_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(DNMXL_CURRENTLIMIT_REG_LOW);
		sendData(DNMXL_CURRENTLIMIT_REG_HIGH);
		sendData(DNMXL_DATA2_LOW);
		sendData(DNMXL_DATA2_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_TORQUE_LENGTH  + AX_READ_DATA + AX_TORQUE_LIMIT_L + AX_BYTE_READ_TRQ))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TORQUE_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(AX_TORQUE_LIMIT_L);
    	sendData(AX_BYTE_READ_TRQ);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(2));      
}

short DynamixelClass::readPWMLimit(unsigned char motorID)
{	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_PWMLIMIT_REG_LOW,DNMXL_PWMLIMIT_REG_HIGH,DNMXL_DATA2_LOW,DNMXL_DATA2_HIGH,};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_READ7_LENGTH_LOW);
	sendData(DNMXL_READ7_LENGTH_HIGH);
	sendData(DNMXL_READ);
	sendData(DNMXL_PWMLIMIT_REG_LOW);
	sendData(DNMXL_PWMLIMIT_REG_HIGH);
	sendData(DNMXL_DATA2_LOW);
	sendData(DNMXL_DATA2_HIGH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE); 
	
	return (readRegister(2));      
}

long DynamixelClass::readVelocityLimit(unsigned char motorID)
{	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_VLCTYLIMIT_REG_LOW,DNMXL_VLCTYLIMIT_REG_HIGH,DNMXL_DATA4_LOW,DNMXL_DATA4_HIGH,};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_READ7_LENGTH_LOW);
	sendData(DNMXL_READ7_LENGTH_HIGH);
	sendData(DNMXL_READ);
	sendData(DNMXL_VLCTYLIMIT_REG_LOW);
	sendData(DNMXL_VLCTYLIMIT_REG_HIGH);
	sendData(DNMXL_DATA4_LOW);
	sendData(DNMXL_DATA4_HIGH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE); 
	
	return (readRegister(4));      
}

long DynamixelClass::readMaxPositionLimit(unsigned char motorID)
{	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_MAXPOSLIMIT_REG_LOW,DNMXL_MAXPOSLIMIT_REG_HIGH,DNMXL_DATA4_LOW,DNMXL_DATA4_HIGH,};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_READ7_LENGTH_LOW);
	sendData(DNMXL_READ7_LENGTH_HIGH);
	sendData(DNMXL_READ);
	sendData(DNMXL_MAXPOSLIMIT_REG_LOW);
	sendData(DNMXL_MAXPOSLIMIT_REG_HIGH);
	sendData(DNMXL_DATA4_LOW);
	sendData(DNMXL_DATA4_HIGH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE); 
	
	return (readRegister(4));      
}

long DynamixelClass::readMinPositionLimit(unsigned char motorID)
{	
	unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
								   DNMXL_READ,DNMXL_MINPOSLIMIT_REG_LOW,DNMXL_MINPOSLIMIT_REG_HIGH,DNMXL_DATA4_LOW,DNMXL_DATA4_HIGH,};
    unsigned short crcBytes = generateCRC(dataPacket,12);
    unsigned char  crcLowByte = (crcBytes & 0x00FF);
    unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
	switchCom(directionPin,TX_MODE);
	sendData(DNMXL_H1);
	sendData(DNMXL_H2);
	sendData(DNMXL_H3);
	sendData(DNMXL_RSVD);
	sendData(motorID);
	sendData(DNMXL_READ7_LENGTH_LOW);
	sendData(DNMXL_READ7_LENGTH_HIGH);
	sendData(DNMXL_READ);
	sendData(DNMXL_MINPOSLIMIT_REG_LOW);
	sendData(DNMXL_MINPOSLIMIT_REG_HIGH);
	sendData(DNMXL_DATA4_LOW);
	sendData(DNMXL_DATA4_HIGH);
	sendData(crcLowByte);
	sendData(crcHighByte);
	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE); 
	
	return (readRegister(4));      
}

short DynamixelClass::readRegisterData(unsigned char motorID, unsigned char registerAddress , unsigned char registerBytes)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_READ7_LENGTH_LOW,DNMXL_READ7_LENGTH_HIGH,
									   DNMXL_READ,registerAddress,DNMXL_FVERS_REG_HIGH,registerBytes,DNMXL_DATA1_HIGH,};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_READ7_LENGTH_LOW);
		sendData(DNMXL_READ7_LENGTH_HIGH);
		sendData(DNMXL_READ);
		sendData(registerAddress);
		sendData(DNMXL_FVERS_REG_HIGH);
		sendData(registerBytes);
		sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_TEM_LENGTH  + AX_READ_DATA + registerAddress + registerBytes))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TEM_LENGTH);
    	sendData(AX_READ_DATA);
    	sendData(registerAddress);
    	sendData(registerBytes);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readRegister(registerBytes));      
}

short DynamixelClass::setRegisterByte(unsigned char motorID, unsigned char registerAddress , unsigned char registerByte)
{	
	if (protocolVersion == 2){
		unsigned char dataPacket[11] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE6_LENGTH_LOW,DNMXL_WRITE6_LENGTH_HIGH,
									   DNMXL_WRITE,registerAddress,DNMXL_FVERS_REG_HIGH,registerByte,};
    	unsigned short crcBytes = generateCRC(dataPacket,11);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE6_LENGTH_LOW);
		sendData(DNMXL_WRITE6_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(registerAddress);
		sendData(DNMXL_FVERS_REG_HIGH);
		sendData(registerByte);
		//sendData(DNMXL_DATA1_HIGH);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_TEM_LENGTH  + AX_WRITE_DATA + registerAddress + registerByte))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_TEM_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(registerAddress);
    	sendData(registerByte);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readError());      
}

short DynamixelClass::setRegisterShort(unsigned char motorID, unsigned char registerAddress , unsigned short registerBytes)
{	
	unsigned char registerByte2 = registerBytes >> 8;           // 16 bits - 2 x 8 bits variables
   	unsigned char registerByte1 = registerBytes;

	if (protocolVersion == 2){
		unsigned char dataPacket[12] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE7_LENGTH_LOW,DNMXL_WRITE7_LENGTH_HIGH,
									   DNMXL_WRITE,registerAddress,DNMXL_FVERS_REG_HIGH,registerByte1,registerByte2};
    	unsigned short crcBytes = generateCRC(dataPacket,12);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE7_LENGTH_LOW);
		sendData(DNMXL_WRITE7_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(registerAddress);
		sendData(DNMXL_FVERS_REG_HIGH);
		sendData(registerByte1);
		sendData(registerByte2);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_GOAL_LENGTH  + AX_WRITE_DATA + registerAddress + registerByte1 + registerByte2))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_GOAL_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(registerAddress);
    	sendData(registerByte1);
    	sendData(registerByte2);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readError());      
}

short DynamixelClass::setRegisterLong(unsigned char motorID, unsigned char registerAddress , unsigned long registerBytes)
{	
	unsigned char registerByte4 = registerBytes >> 24;    
	unsigned char registerByte3 = registerBytes >> 16;    
	unsigned char registerByte2 = registerBytes >> 8;           
   	unsigned char registerByte1 = registerBytes;

	if (protocolVersion == 2){
		unsigned char dataPacket[14] = {DNMXL_H1,DNMXL_H2,DNMXL_H3,DNMXL_RSVD,motorID,DNMXL_WRITE9_LENGTH_LOW,DNMXL_WRITE9_LENGTH_HIGH,
									   DNMXL_WRITE,registerAddress,DNMXL_FVERS_REG_HIGH,registerByte1,registerByte2,registerByte3,registerByte4};
    	unsigned short crcBytes = generateCRC(dataPacket,14);
    	unsigned char  crcLowByte = (crcBytes & 0x00FF);
    	unsigned char  crcHighByte = (crcBytes >> 8) & 0x00FF;
		switchCom(directionPin,TX_MODE);
	    sendData(DNMXL_H1);
	    sendData(DNMXL_H2);
		sendData(DNMXL_H3);
		sendData(DNMXL_RSVD);
		sendData(motorID);
		sendData(DNMXL_WRITE9_LENGTH_LOW);
		sendData(DNMXL_WRITE9_LENGTH_HIGH);
		sendData(DNMXL_WRITE);
		sendData(registerAddress);
		sendData(DNMXL_FVERS_REG_HIGH);
		sendData(registerByte1);
		sendData(registerByte2);
		sendData(registerByte3);
		sendData(registerByte4);
		sendData(crcLowByte);
		sendData(crcHighByte);
		while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE); 
	} else if (protocolVersion == 1){ 
    	unsigned char checksum = (~(motorID + AX_GOAL_SP_LENGTH  + AX_WRITE_DATA + registerAddress + registerByte1 + registerByte2 + registerByte3 + registerByte4))&0xFF;
		switchCom(directionPin,TX_MODE);
    	sendData(AX_START);
    	sendData(AX_START);
    	sendData(motorID);
    	sendData(AX_GOAL_SP_LENGTH);
    	sendData(AX_WRITE_DATA);
    	sendData(registerAddress);
    	sendData(registerByte1);
    	sendData(registerByte2);
    	sendData(registerByte3);
    	sendData(registerByte4);
    	sendData(checksum);
    	while(availableWrite() != serialBufferLenght);delayus(timeDelay);
		switchCom(directionPin,RX_MODE);
	}
	return (readError());      
}

const char* DynamixelClass::mapMotorModel(short motorModelNumber)
{
	for(short i = 0; i < 48; i++){
		if(abs(motorModelNumber) == motorNumber[i])
			return motorModel[i];
	}
	return "UNKNOWN";
}

const char* DynamixelClass::mapMotorError(short motorError)
{
	if(protocolVersion == 1){
		for(short i = 0; i < 15; i++){
	 		if(abs(motorError) == motorP1Errors[i])
	 			return motorP1ErrorList[i];
	 	}
	} else {
		for(short i = 0; i < 8; i++){
	 		if(abs(motorError) == motorP2Errors[i])
	 			return motorP2ErrorList[i];
	 	}
	}
	return "SEVERAL ERRORS";
}


short DynamixelClass::LXmoveTime(unsigned char motorID, unsigned short positionBytes, unsigned short timeBytes)
{
    unsigned char positionByteH = positionBytes >> 8;
    unsigned char positionByteL = positionBytes;                // 16 bits - 2 x 8 bits variables
    unsigned char speedByteH = timeBytes >> 8;
    unsigned char speedByteL = timeBytes;                      // 16 bits - 2 x 8 bits variables
	unsigned char checksum = (~(motorID + LX_MOVETIME_WR_LENGHT + LX_MOVETIME_WR + positionByteL + positionByteH + speedByteL + speedByteH))&0xFF;

	switchCom(directionPin,TX_MODE);
    sendData(LX_START_BYTE);                
    sendData(LX_START_BYTE);
    sendData(motorID);
    sendData(LX_MOVETIME_WR_LENGHT);
    sendData(LX_MOVETIME_WR);
    sendData(positionByteL);
    sendData(positionByteH);
    sendData(speedByteL);
    sendData(speedByteH);
    sendData(checksum);
    while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);          
	
	return (readError());  
}

short DynamixelClass::LXreadPosition(unsigned char motorID)
{
    unsigned char checksum = (~(motorID + LX_POS_RD_LENGHT  + LX_POS_RD))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(LX_START_BYTE);                
    sendData(LX_START_BYTE);
    sendData(motorID);
    sendData(LX_POS_RD_LENGHT);
    sendData(LX_POS_RD);
    sendData(checksum);
    while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);       
	
	return (LXreadRegister(2));   
}

short DynamixelClass::LXreadVoltage(unsigned char motorID)
{
    unsigned char checksum = (~(motorID + LX_VIN_RD_LENGHT  + LX_VIN_RD))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(LX_START_BYTE);                
    sendData(LX_START_BYTE);
    sendData(motorID);
    sendData(LX_VIN_RD_LENGHT);
    sendData(LX_VIN_RD);
    sendData(checksum);
    while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);       
	
	return (LXreadRegister(1));   
}

short DynamixelClass::LXreadTemperature(unsigned char motorID)
{
    unsigned char checksum = (~(motorID + LX_TEMP_RD_LENGHT  + LX_TEMP_RD))&0xFF;
	switchCom(directionPin,TX_MODE);
    sendData(LX_START_BYTE);                
    sendData(LX_START_BYTE);
    sendData(motorID);
    sendData(LX_TEMP_RD_LENGHT);
    sendData(LX_TEMP_RD);
    sendData(checksum);
    while(availableWrite() != serialBufferLenght);delayus(timeDelay);
	switchCom(directionPin,RX_MODE);       
	
	return (LXreadRegister(1));   
}


DynamixelClass Dynamixel;