/*
 Dynamixel.cpp - Ax-12+ Half Duplex USART Comunication
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
 Modifications:
 
 25/07/2011 - Eliminado la modificacion serial para ser modificada dentro del mismo Hardware Serial.
 25/07/2011 - Modificado la funcion setBD() para aceptar todas la velocidades sin PDF.
 25/07/2011 - Agregada la funcion de Rotacion Continua.
 26/07/2011 - Agregada la funcion begin sin seteo de Direction_Pin.
 25/07/2011 - Agregada la funcion Reset.
 26/07/2011 - Agregada la funcion Reg_Write en move y moveSpeed.
 26/07/2011 - Agregada la funcion Action.
 13/12/2011 - Arreglado el manejo y envio de variables.
 22/12/2011 - Compatible con la actualizacion Arduino 1.0.
 10/01/2012 - Utilizacion de Macros y eliminacion codigo no necesario.
 11/01/2012 - Agregadas las funciones:
              int setTempLimit(unsigned char ID, unsigned char Temperature);
              int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
              int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
			  int setMaxTorque(unsigned char ID, int MaxTorque);
              int setSRL(unsigned char ID, unsigned char SRL);
              int setRDT(unsigned char ID, unsigned char RDT);
              int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
              int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
              int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
			  int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
 15/01/2012 - Agregadas las funciones:             
              int setPunch(unsigned char ID, int Punch);
              int moving(unsigned char ID);
              int lockRegister(unsigned char ID);
			  int RWStatus(unsigned char ID);
              int readSpeed(unsigned char ID);
              int readLoad(unsigned char ID);
 08/07/2019
			- Se unifican todas las variantes de la biblioteca en una sola
			- Se agrega funcion de reset general
 
 08/12/2022 - Se actualiza nombre de la biblioteca
 			- Se agrega funcion de lectura de registros
 			- Se corrige funcion CWLimit CCWLimit VoltageLimit
 			- Se agrega definicion de errores del protocolo V1
 			- Se agrega funcion transmision/recepcion dinamica
 			- Se agrega compatibilidad con protocolo 2.0
 			- Se agrega funcion profile Velocity
 			- Se agrega funcion operation Mode
 			- Se actualizan definiciones de registros Dynamixel
            - Se agrega funcion de mapeo de Modelo
            - Se agrega funcuon de mapeo de Hardware Error

 22/06/2023
            - Se agrega compatibilidad con Software Serial

 SUPPORTED DEVICES:

 PROTOCOL 2.0: XC330,XL330,XC430,XL430,2XC430,2XL430,XW430,XH430,XM430,XD540,XW540,XH540,MX106,MX64,MX28
 PROTOCOL 1.0: AX-12,AX-12+,AX-12A,AX-12W,AX-18A,MX-12W,MX-28T,MX-28AT,MX-28AR,MX-64T,MX-64R,MX-64AT,MX-64AR
 
 *****************************************************************************
 
 Contact: Josue Alejandro Gutierrez - Josue.Gutierrez@savageelectronics.com
 Web:     https://www.savageelectrtonics.com/
 Autor:   Josue Alejandro Gutierrez
 
 */

#ifndef Dynamixel_h
#define Dynamixel_h

#define DNMXL_H1                    	0xFF
#define DNMXL_H2                    	0xFF
#define DNMXL_H3                    	0xFD
#define DNMXL_RSVD						0x00

#define DNMXL_READ7_LENGTH_LOW			0x07
#define DNMXL_READ7_LENGTH_HIGH			0x00

#define DNMXL_WRITE3_LENGTH_LOW			0x03
#define DNMXL_WRITE3_LENGTH_HIGH		0x00
#define DNMXL_WRITE4_LENGTH_LOW			0x04
#define DNMXL_WRITE4_LENGTH_HIGH		0x00
#define DNMXL_WRITE6_LENGTH_LOW			0x06
#define DNMXL_WRITE6_LENGTH_HIGH		0x00

#define DNMXL_WRITE7_LENGTH_LOW         0x07
#define DNMXL_WRITE7_LENGTH_HIGH        0x00

#define DNMXL_WRITE9_LENGTH_LOW			0x09
#define DNMXL_WRITE9_LENGTH_HIGH		0x00

#define DNMXL_DATA1_LOW					0x01
#define DNMXL_DATA1_HIGH				0x00
#define DNMXL_DATA2_LOW					0x02
#define DNMXL_DATA2_HIGH				0x00
#define DNMXL_DATA4_LOW					0x04
#define DNMXL_DATA4_HIGH				0x00

// DYNAMIXEL PROTOCOL 2.0 EEPROM AREA  ////////////////////////////////////

#define DNMXL_MODEL_REG_LOW				0x00
#define DNMXL_MODEL_REG_HIGH			0x00
#define DNMXL_MODINFO_REG_LOW			0x02
#define DNMXL_MODINFO_REG_HIGH			0x00
#define DNMXL_FVERS_REG_LOW				0x06
#define DNMXL_FVERS_REG_HIGH			0x00
#define DNMXL_ID_REG_LOW				0x07
#define DNMXL_ID_REG_HIGH				0x00
#define DNMXL_BR_REG_LOW				0x08
#define DNMXL_BR_REG_HIGH				0x00
#define DNMXL_RDT_REG_LOW				0x09
#define DNMXL_RDT_REG_HIGH				0x00
#define DNMXL_DRVMODE_REG_LOW			0x0A
#define DNMXL_DRVMODE_REG_HIGH			0x00
#define DNMXL_OPMODE_REG_LOW			0x0B
#define DNMXL_OPMODE_REG_HIGH			0x00
#define DNMXL_SHDWID_REG_LOW			0x0C
#define DNMXL_SHDWID_REG_HIGH			0x00
#define DNMXL_PROTOCOL_REG_LOW			0x0D
#define DNMXL_PROTOCOL_REG_HIGH			0x00
#define DNMXL_HOMINGOFST_REG_LOW		0x14
#define DNMXL_HOMINGOFST_REG_HIGH		0x00
#define DNMXL_MOVINGTHLD_REG_LOW		0x18
#define DNMXL_MOVINGTHLD_REG_HIGH		0x00
#define DNMXL_TEMPLIMIT_REG_LOW			0x1F
#define DNMXL_TEMPLIMIT_REG_HIGH		0x00
#define DNMXL_MAXVOLTLIMIT_REG_LOW		0x20
#define DNMXL_MAXVOLTLIMIT_REG_HIGH		0x00
#define DNMXL_MINVOLTLIMIT_REG_LOW		0x22
#define DNMXL_MINVOLTLIMIT_REG_HIGH		0x00
#define DNMXL_PWMLIMIT_REG_LOW			0x24
#define DNMXL_PWMLIMIT_REG_HIGH			0x00
#define DNMXL_CURRENTLIMIT_REG_LOW		0x26
#define DNMXL_CURRENTLIMIT_REG_HIGH		0x00
#define DNMXL_VLCTYLIMIT_REG_LOW		0x2C
#define DNMXL_VLCTYLIMIT_REG_HIGH		0x00
#define DNMXL_MAXPOSLIMIT_REG_LOW		0x30
#define DNMXL_MAXPOSLIMIT_REG_HIGH		0x00
#define DNMXL_MINPOSLIMIT_REG_LOW		0x34
#define DNMXL_MINPOSLIMIT_REG_HIGH		0x00
#define DNMXL_EXPORTMODE1_REG_LOW		0x38
#define DNMXL_EXPORTMODE1_REG_HIGH		0x00
#define DNMXL_EXPORTMODE2_REG_LOW		0x39
#define DNMXL_EXPORTMODE2_REG_HIGH		0x00
#define DNMXL_EXPORTMODE3_REG_LOW		0x3A
#define DNMXL_EXPORTMODE3_REG_HIGH		0x00
#define DNMXL_STRPCONF_REG_LOW			0x3C
#define DNMXL_STRPCONF_REG_HIGH			0x00
#define DNMXL_PWMSLOPE_REG_LOW			0x3E
#define DNMXL_PWMSLOPE_REG_HIGH			0x00
#define DNMXL_SHUTDOWN_REG_LOW			0x3F
#define DNMXL_SHUTDOWN_REG_HIGH			0x00

// DYNAMIXEL PROTOCOL 2.0 RAM AREA  ////////////////////////////////////

#define DNMXL_TRQ_REG_LOW				0x40
#define DNMXL_TRQ_REG_HIGH				0x00
#define DNMXL_LED_REG_LOW				0x41
#define DNMXL_LED_REG_HIGH				0x00
#define DNMXL_SRL_REG_LOW				0x44
#define DNMXL_SRL_REG_HIGH				0x00
#define DNMXL_REGINST_REG_LOW			0x45
#define DNMXL_REGINST_REG_HIGH			0x00
#define DNMXL_HWSTATUS_REG_LOW			0x46
#define DNMXL_HWSTATUS_REG_HIGH			0x00
#define DNMXL_VELIG_REG_LOW				0x4C
#define DNMXL_VELIG_REG_HIGH			0x00
#define DNMXL_VELPG_REG_LOW				0x4E
#define DNMXL_VELPG_REG_HIGH			0x00
#define DNMXL_POSDG_REG_LOW				0x50
#define DNMXL_POSDG_REG_HIGH			0x00
#define DNMXL_POSIG_REG_LOW				0x52
#define DNMXL_POSIG_REG_HIGH			0x00
#define DNMXL_POSPG_REG_LOW				0x54
#define DNMXL_POSPG_REG_HIGH			0x00
#define DNMXL_FF2G_REG_LOW				0x58
#define DNMXL_FF2G_REG_HIGH				0x00
#define DNMXL_FF1G_REG_LOW				0x5A
#define DNMXL_FF1G_REG_HIGH				0x00
#define DNMXL_BUSWDG_REG_LOW			0x62
#define DNMXL_BUSWDG_REG_HIGH			0x00
#define DNMXL_GOALPWM_REG_LOW			0x64
#define DNMXL_GOALPWM_REG_HIGH			0x00
#define DNMXL_GOALCURRENT_REG_LOW		0x66
#define DNMXL_GOALCURRENT_REG_HIGH		0x00
#define DNMXL_GOALVEL_REG_LOW			0x68
#define DNMXL_GOALVEL_REG_HIGH			0x00
#define DNMXL_PACCEL_REG_LOW			0x6C
#define DNMXL_PACCEL_REG_HIGH			0x00
#define DNMXL_PVEL_REG_LOW				0x70
#define DNMXL_PVEL_REG_HIGH				0x00
#define DNMXL_GOALPOS_REG_LOW			0x74
#define DNMXL_GOALPOS_REG_HIGH			0x00
#define DNMXL_REALTIMETCK_REG_LOW		0x78
#define DNMXL_REALTIMETCK_REG_HIGH		0x00
#define DNMXL_MOVING_REG_LOW			0x7A
#define DNMXL_MOVING_REG_HIGH			0x00
#define DNMXL_MOVINGSTATUS_LOW			0x7B
#define DNMXL_MOVINGSTATUS_REG_HIGH		0x00
#define DNMXL_PWM_REG_LOW				0x7C
#define DNMXL_PWM_REG_HIGH				0x00
#define DNMXL_LOAD_REG_LOW				0x7E
#define DNMXL_LOAD_REG_HIGH				0x00
#define DNMXL_CURRENT_REG_LOW			0x7E
#define DNMXL_CURRENT_REG_HIGH			0x00
#define DNMXL_VELOCITY_REG_LOW			0x80
#define DNMXL_VELOCITY_REG_HIGH			0x00
#define DNMXL_POSITION_REG_LOW			0x84
#define DNMXL_POSITION_REG_HIGH			0x00
#define DNMXL_VELTRJ_REG_LOW			0x88
#define DNMXL_VELTRJ_REG_HIGH			0x00
#define DNMXL_POSTRJ_REG_LOW			0x8C
#define DNMXL_POSTRJ_REG_HIGH			0x00
#define DNMXL_VOLTAGE_REG_LOW			0x90
#define DNMXL_VOLTAGE_REG_HIGH			0x00
#define DNMXL_TEMPERATURE_REG_LOW		0x92
#define DNMXL_TEMPERATURE_REG_HIGH		0x00
#define DNMXL_BKPRDY_REG_LOW			0x93
#define DNMXL_BKPRDY_REG_HIGH			0x00
#define DNMXL_EXPORTDATA1_REG_LOW		0x98
#define DNMXL_EXPORTDATA1_REG_HIGH		0x00
#define DNMXL_EXPORTDATA2_REG_LOW		0x9A
#define DNMXL_EXPORTDATA2_REG_HIGH		0x00
#define DNMXL_EXPORTDATA3_REG_LOW		0x9C
#define DNMXL_EXPORTDATA3_REG_HIGH		0x00

// DYNAMIXEL PROTOCOL 2.0 INSTRUCTION SET  ////////////////////////////////////

#define DNMXL_PING 						0x01
#define DNMXL_READ 						0x02
#define DNMXL_WRITE 					0x03
#define DNMXL_REGWRITE 					0x04
#define DNMXL_ACTION 					0x05
#define DNMXL_RESET 					0x06
#define DNMXL_REBOOT 					0x08
#define DNMXL_CLEAR 					0x10
#define DNMXL_CTRLTABLE_BCKP 			0x20
#define DNMXL_STATUS 					0x55
#define DNMXL_SYNC_READ 				0x82
#define DNMXL_SYNC_WRITE 				0x83
#define DNMXL_FAST_SYNC_READ 			0x8A
#define DNMXL_BULK_READ 				0x92
#define DNMXL_BULK_WRITE 				0x93
#define DNMXL_FAST_BULK_READ 			0x9A

// DYNAMIXEL PROTOCOL 2.0 INSTRUCTION OPTIONS  ////////////////////////////////////

#define RESET_ALL 				        0xFF
#define RESET_NOID 				        0x01
#define RESET_NOID_NOBR 			    0x02

#define CURRENT_CTRL                    0x00
#define VELOCITY_CTRL                   0x01
#define POSITION_CTRL                   0x03
#define EXTENDED_POSITION_CTRL          0x04
#define CURRENT_BASED_POSITION_CTRL     0x05
#define PWM_CTRL                        0x10


	// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19

#define MX_MULTITURN_OFFSET		    20
#define MX_RESOLUTIONDIVIDER        22

#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

	// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25

#define MX_DGAIN     				26
#define MX_IGAIN    				27
#define MX_PGAIN      				28

#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

#define MX_REALTIME_TICK		    50
#define MX_CURRENT		    		68
#define MX_TORQUE_CTRL_MODE_EN	    70
#define MX_GOAL_TORQUE		  		71
#define MX_GOAL_ACCELERATION	    73

    // Status Return Levels ///////////////////////////////////////////////////////////////
#define RETURN_NONE                 0
#define RETURN_READ                 1
#define RETURN_ALL                  2

    // Instruction Set ///////////////////////////////////////////////////////////////
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

	// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						0
#define RIGTH                       1
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_RESET_LENGTH				2
#define AX_ACTION_LENGTH			2
#define AX_ID_LENGTH                4
#define AX_LR_LENGTH                4
#define AX_SRL_LENGTH               4
#define AX_RDT_LENGTH               4
#define AX_LEDALARM_LENGTH          4
#define AX_SALARM_LENGTH            4
#define AX_TL_LENGTH                4
#define AX_VL_LENGTH                5
#define AX_CM_LENGTH                6
#define AX_CS_LENGTH                6
#define AX_CCW_CW_LENGTH            5
#define AX_BD_LENGTH                4
#define AX_TEM_LENGTH               4
#define AX_MOVING_LENGTH            4
#define AX_RWS_LENGTH               4
#define AX_VOLT_LENGTH              4
#define AX_LED_LENGTH               4
#define AX_TORQUE_LENGTH            4
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5
#define AX_MT_LENGTH                5
#define AX_PUNCH_LENGTH             5
#define AX_SPEED_LENGTH             5
#define AX_GOAL_SP_LENGTH           7
#define AX_ACTION_CHECKSUM			250
#define BROADCAST_ID                254
#define AX_START                    255
#define AX_CCW_AL_L                 255 
#define AX_CCW_AL_H                 3
#define AX_CCW_AL_WHEEL				0
#define CCW_DIRECTION				0
#define CW_DIRECTION				1
#define TX_MODE                     1
#define RX_MODE                     0
#define UNLOCK                      0
#define LOCK                        1

#define DELAY_TX_TIME_MIN           0
#define DELAY_TX_TIME_MAX           320
#define DELAY_TX_TIME_OFFSET        15
#define DELAY_TX_TIME_MULTP         1600
#define TIME_COUNTER_DELAY          600
#define TIME_OUT                    10

#define NO_RESPONSE_ERROR			0x1000
#define INSTRUCTION_ERROR			0x40
#define OVERLOAD_ERROR				0x20
#define CHECKSUM_ERROR				0x10
#define RANGE_ERROR					0x08
#define OVERHEATING_ERROR			0x04
#define ANGLE_LIMIT_ERROR			0x02	
#define INPUT_VOLTAGE_ERROR			0x01

#include <inttypes.h>
#include <HardwareSerial.h>
//#include <SoftwareSerial.h>

class DynamixelClass {
private:

//SoftwareSerial *softwareSerialPort;
HardwareSerial *serialPort;

    unsigned int timeDelay;
    unsigned int  serialBufferLenght;
    unsigned char directionPin;
	unsigned char protocolVersion;
	unsigned char registerLowByte;
	unsigned char registerHighByte;
	unsigned char incomingByte; 
	unsigned char headerStart;
	unsigned char reservedByte;
	unsigned char responseID;
	unsigned char packetLenght;
	unsigned char packetLenghtLow;
	unsigned char packetLenghtHigh;
	unsigned char instructionByte;
	unsigned char errorByte;
	unsigned char parameter1;
	unsigned char parameter2;
	unsigned char parameter3;
	unsigned char parameter4;
	unsigned char checkSum; 

	  
    int readRegister(unsigned char registerLenght, unsigned char returnValue);
	int readRegister(unsigned char registerLenght);  
	int readError(void);
	
public: 
    
    //void begin(SoftwareSerial *sPort, long bRate, unsigned char dPin, unsigned char pVersion);
    void begin(HardwareSerial *sPort, long bRate, unsigned char dPin, unsigned char pVersion);
	void setProtocolVersion(unsigned char pVersion);
	void end(void);
	
	int reset(unsigned char motorID, unsigned char resetMode);
	int reset(unsigned char motorID);
	int ping(unsigned char motorID); 

    void action(void);
	
	int setID(unsigned char motorID, unsigned char newID);
	int setBaudRate(unsigned char motorID, long bRate);
	
	int setDriveMode(unsigned char motorID, unsigned char driveMode);
	int setOperationMode(unsigned char motorID, unsigned char operationMode);
	int setProfileVelocity(unsigned char motorID, int profileVelocity);

    int setGoalPWM(unsigned char motorID, int motorPWM);
    int setGoalCurrent(unsigned char motorID, int motorCurrent);
    int setGoalVelocity(unsigned char motorID, int motorVelocity);
    int setGoalPosition(unsigned char motorID, int motorPosition);

	int move(unsigned char motorID, int motorPosition);
	int moveSpeed(unsigned char motorID, int motorPosition, int motorSpeed);
    int moveRW(unsigned char motorID, int motorPosition);
    int moveSpeedRW(unsigned char motorID, int motorPosition, int motorSpeed);
	int setEndless(unsigned char motorID,bool motorMode);
	int setRotation(unsigned char motorID, bool turnDirection, int motorSpeed);
	
	int setTempLimit(unsigned char motorID, unsigned char motorTemperature);
	int setCWAngleLimit(unsigned char motorID, int motorCWLimit);
	int setCCWAngleLimit(unsigned char motorID, int motorCCWLimit);
	int setVoltageLimit(unsigned char motorID, unsigned char minVoltage, unsigned char maxVoltage);
	int setMaxTorque(unsigned char motorID, int maxTorque);
	int setSRL(unsigned char motorID, unsigned char motorSRL);
	int setRDT(unsigned char motorID, unsigned char motorRDT);
	int setLEDAlarm(unsigned char motorID, unsigned char ledAlarm);
	int setShutdownAlarm(unsigned char motorID, unsigned char motorAlarm);
	int setCMargin(unsigned char motorID, unsigned char CWCMargin, unsigned char CCWCMargin);
	int setCSlope(unsigned char motorID, unsigned char CWCSlope, unsigned char CCWCSlope);
	int setPunch(unsigned char motorID, int motorPunch);
	
	int readTemperature(unsigned char motorID);
	int readVoltage(unsigned char motorID);
	int readPosition(unsigned char motorID);
	int readSpeed(unsigned char motorID);
	int readLoad(unsigned char motorID);
	int readCurrent(unsigned char motorID);
	int readPWM(unsigned char motorID);
	int readModel(unsigned char motorID);
	int readFirmware(unsigned char motorID);
	
	int readMovingStatus(unsigned char motorID);
	int readRWStatus(unsigned char motorID);

    int setLockRegister(unsigned char motorID, bool lockStatus);
	int setTorque(unsigned char motorID, bool torqueStatus);
	int setLED(unsigned char motorID, bool ledStatus);

    int setRegisterByte(unsigned char motorID, unsigned char registerAddress, unsigned char registerByte);
    int setRegisterShort(unsigned char motorID, unsigned char registerAddress, unsigned short registerBytes);
    int setRegisterLong(unsigned char motorID, unsigned char registerAddress, unsigned long registerBytes);

    int readRegisterData(unsigned char motorID, unsigned char registerAddress, unsigned char registerBytes);

    const char* mapMotorModel(int motorModelNumber);
    const char* mapMotorError(int motorError);
};

extern DynamixelClass Dynamixel;

#endif
