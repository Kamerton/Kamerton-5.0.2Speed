/*

 Kamerton5_1_0ArduinoSpeed.ino
 Visual Studio 2010
 
 ��������� ������������ ������ "��������" (������� �������)
 ������:      - 5_1_0
 ����:        - 25.07.2016�.
 �����������: - ��� "������"
 �����:       - �������� �.�.
 ������: ���������� ������ �� 25.07.2016�.

 ��������� �������� ������� "�����1" � "�����2"
 
 �����������:
 -
 - ���������� 30��,
 - ��������/����� �� ��� �����,
 - ������� ���������� ����, ����� � ����������, 
 - ���������� MCP23017
 - ������ ����, 
 - ������ ���� ������, 
 - ��������� �������� ���������
 - ���������� SD ������
 - ���������� ����, ������, 
 */


#include <SPI.h>
#include <SdFat.h>
#include <SdFatUtil.h>
#include <Wire.h> 
#include <RTClib.h>
#include <MsTimer2.h> 
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include "MCP23017.h"
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdlib.h> // div, div_t

#define  ledPin13  13                               // ���������� ����������� �� �����
#define  ledPin12  12                               // ���������� ����������� �� �����
#define  ledPin11  11                               // ���������� ����������� �� �����
#define  ledPin10  10                               // ���������� ����������� �� �����
#define  Front_led_Blue 14                          // ���������� ����������� �� �������� ������
#define  Front_led_Red  15                          // ���������� ����������� �� �������� ������

//+++++++++++++++++++++++++++++ ������� ������ +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // ����� ���������� ������
unsigned int eeaddress   =  0;                      // ����� ������ ������
byte hi;                                            // ������� ���� ��� �������������� �����
byte low;                                           // ������� ���� ��� �������������� �����


//  ����� ���������� ������ ��������
#define DTR  8                                      // DTR out �������� ������  ������������ 0 ��� ������
#define RTS  9                                      // RTS out �������� ������   
#define CTS  5                                      // CTS in  ������� ������  ���� ������� �������� ��������������!!!!
#define DSR  6                                      // DSR in  ������� ������  ���� ������� "����� - ��������"
#define DCD  7                                      // DCD in  ������� ������  ���� ������ ������ � ��������� 


//  ����� ���������� ������ Arduino Nano
#define  kn1Nano   34                               // ���������� ������ ���������� Nano ��������� �������
#define  kn2Nano   36                               // ���������� ������ ���������� Nano ��������� 1000 ��
#define  kn3Nano   38                               // ���������� ������ ���������� Nano ��������� 2000 ��
#define  InNano12  40                               // ���������� ������ - ��������� ��������� 1000 ��� 2000 ��
#define  InNano13  39                               // ���������� ������ - ��������� ��������� ������� 
bool var_sound = false;                             // ������� ������� ������� �����

// 0 - 500-1000��
// 1 - 500-3500��
// 2 - 100-5500��
// 3 - 100-1000��
// 4 - 500��
// 5 - 1000��
// 6 - 2000��
// 7 - 3500��

//+++++++++++++++++++++++ ��������� ������������ ��������� +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // ����� ���������� AD5252  
#define control_word1    0x07                       // ���� ���������� �������� �1
#define control_word2    0x87                       // ���� ���������� �������� �2
byte resistance        = 0x00;                      // ������������� 0x00..0xFF - 0��..100���
//byte level_resist      = 0;                       // ���� ��������� ������ �������� ���������
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
volatile unsigned int Array_volume[600];                     //
unsigned int Array_min[40];                         //
unsigned int Array_max[40];                         //
unsigned int volume_porog_D = 40;                   // ������������ �������� ������ ��� �������� ����������� FrontL,FrontR
unsigned int volume_porog_L = 200;                  // ����������� �������� ������ ��� �������� ����������� FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // ����� �������� ��������� �����
unsigned int  voltage10 ;
unsigned long number_audio ;   

//==========================================================================================================================

// ������ �� ���������� � �������� (1 sbi) ��� (0 cbi)
// ��������� ��� ������ � ��� (��������� ������� ������������)
// ���������� ��� ��������� � ������ ���� ��������
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//---------------------------------------------------------------------------------------------------------

//***************************** ���������� ���������� ������   ****************************************
int analog_tok            = 0;       // ��������� ���� ������� ����� �����-1 (�� �����������)
int analog_12V            = 1;       // ��������� ���������� ������� 12�. ����� �����-1
int analog_tok_x10        = 2;       // ��������� ���� ������� ����� �����-1 � 10  (�� �����������)
int analog_mag_radio      = 3;       //
int analog_mag_phone      = 4;       //
int analog_gg_radio1      = 5;       //
int analog_gg_radio2      = 6;       //
int analog_ggs            = 7;       //
int analog_LineL          = 8;       //
int analog_LineR          = 9;       //
int analog_FrontL         = 10;      //
int analog_FrontR         = 11;      //
int analog_W              = 12;      // 
int analog_13             = 13;      // ��������� ���������� �������  12�.�� ��������  ����� ��������
int analog_14             = 14;      // ��������� ���������� �������  12�.�� ��������  ����� ��������
int analog_3_6            = 15;      // ��������� ���������� ������� 3,6�. �� �������� ����� ��������
int analogIn_oscill       = A0;      // ���������� ���� ������������  

//===========================================================================================

bool blink_red   = false;
bool portFound   = false;
bool portFound2  = false;
uint32_t logTime = 0;
int32_t diff     = 0;
byte inputByte_0;
byte inputByte_1;
byte inputByte_2;
byte inputByte_3;
byte inputByte_4;

//************************************************************************************************

RTC_DS1307 RTC;                                     // define the Real Time Clock object

//-----------------------------------------------------------------------------------------------

uint8_t second = 0;       //Initialization time
uint8_t minute = 0;
uint8_t hour   = 0;
uint8_t dow    = 1;
uint8_t day    = 1;
uint8_t month  = 1;
uint16_t year  = 15 ;

//------------------------------------------------------------------------------------------------------------
MCP23017 mcp_Out1;                                                 // ���������� ������ ���������� MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                                 // ���������� ������ ���������� MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                               // ���������� ������ ���������� MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------
const int adr_reg_ind_CTS      PROGMEM           = 10081;          // ����� ����a ��������� ��������� ������� CTS
const int adr_reg_ind_DSR      PROGMEM           = 10082;          // ����� ����a ��������� ��������� ������� DSR
const int adr_reg_ind_DCD      PROGMEM           = 10083;          // ����� ����a ��������� ��������� ������� DCD

// **************** ������ ������� ������ ��� �������� ����. ����������� ��� ������������ ����� ����� *************
//const int adr_temp_day         PROGMEM           = 240;          // ����� �������� ���������� ����
//const int adr_temp_mon         PROGMEM           = 241;          // ����� �������� ���������� �����
//const int adr_temp_year        PROGMEM           = 242;          // ����� �������� ���������� ���  
//const int adr_file_name_count  PROGMEM           = 243;          // ����� �������� ���������� �������� ������ �����
//------------------------------------------------------------------------------------------------------------------
int regcount_err        = 0;                                       // ���������� ��� �������� ���� ������

//++++++++++++++++++++++ ������ � ������� +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49                                              // ��������� ������ SD
SdFat sd;
File myFile;
SdFile file;
Sd2Card card;
uint32_t cardSizeBlocks;
uint16_t cardCapacityMB;

// cache for SD block
cache_t cache;

//------------------------------------------------------------------------------
// ������� ����������, ������������ ������� ���������� SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ ���������� ����� ����� ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "160101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.KAM";
char fileName_p[13];
char fileName_F[13];
//------------------------------------------------------------------------------

char c;  // ��� ����� ������� � ��� �����

// Serial output stream
//ArduinoOutStream cout(Serial);
//char bufferSerial2[128];  

//*********************������ � ������ ����� ******************************

//byte file_name_count = 0;
char str_day_file[3];
char str_day_file0[3];
char str_day_file10[3];
char str_mon_file[3];
char str_mon_file0[3];
char str_mon_file10[3];
char str_year_file[3];

char str0[10];
char str1[10];
char str2[10];

//+++++++++++++++++++++ ��������� ���������� +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

//byte regs_in[5];                                  // �������� ������ � ������ �����-1 CPLL
byte regs_out[4];                                   // �������� ������ � ������ �����-1
byte regs_crc[1];                                   // �������� ������ � ������ �����-1 ����������� �����
byte regs_temp = 0;
byte regs_temp1 = 0;
byte Stop_Kam = 0;                                  // ���� ��������� ������ ���. �� ����� �����-1
volatile bool prer_Kmerton_On = true;               // ���� ���������� ���������� �����-1
bool test_repeat     = true;                        // ���� ���������� �����
volatile bool prer_Kmerton_Run = false;             // ���� ���������� ���������� �����-1
#define BUFFER_SIZEK 64                             // ������ ������ �������� �� ����� 128 ����
#define BUFFER_SIZEKF 128                           // ������ ������ Serial2 �� ����� 128 ����
unsigned char bufferK;                              // ������� ���������� ����������� ����

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ������� ����� 
const unsigned int adr_kontrol_day        PROGMEM      = 40046;  // ����� ����
const unsigned int adr_kontrol_month      PROGMEM      = 40047;  // ����� �����
const unsigned int adr_kontrol_year       PROGMEM      = 40048;  // ����� ���
const unsigned int adr_kontrol_hour       PROGMEM      = 40049;  // ����� ���
const unsigned int adr_kontrol_minute     PROGMEM      = 40050;  // ����� ������
const unsigned int adr_kontrol_second     PROGMEM      = 40051;  // ����� �������

// ��������� ������� � �����������
const unsigned int adr_set_kontrol_day    PROGMEM      = 40052;  // ����� ����
const unsigned int adr_set_kontrol_month  PROGMEM      = 40053;  // ����� �����
const unsigned int adr_set_kontrol_year   PROGMEM      = 40054;  // ����� ���
const unsigned int adr_set_kontrol_hour   PROGMEM      = 40055;  // ����� ���
const unsigned int adr_set_kontrol_minute PROGMEM      = 40056;  // ����� ������

// ����� ������ �����
const unsigned int adr_Mic_Start_day      PROGMEM      = 40096;  // ����� ����
const unsigned int adr_Mic_Start_month    PROGMEM      = 40097;  // ����� �����
const unsigned int adr_Mic_Start_year     PROGMEM      = 40098;  // ����� ���
const unsigned int adr_Mic_Start_hour     PROGMEM      = 40099;  // ����� ���
const unsigned int adr_Mic_Start_minute   PROGMEM      = 40100;  // ����� ������
const unsigned int adr_Mic_Start_second   PROGMEM      = 40101;  // ����� �������
// ����� ��������� �����
const unsigned int adr_Mic_Stop_day       PROGMEM       = 40102; // ����� ����
const unsigned int adr_Mic_Stop_month     PROGMEM       = 40103; // ����� �����
const unsigned int adr_Mic_Stop_year      PROGMEM       = 40104; // ����� ���
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 40105; // ����� ���
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 40106; // ����� ������
const unsigned int adr_Mic_Stop_second    PROGMEM       = 40107; // ����� �������

// ����������������� ���������� �����
const unsigned int adr_Time_Test_day      PROGMEM       = 40108; // ����� ����
const unsigned int adr_Time_Test_hour     PROGMEM       = 40109; // ����� ���
const unsigned int adr_Time_Test_minute   PROGMEM       = 40110; // ����� ������
const unsigned int adr_Time_Test_second   PROGMEM       = 40111; // ����� �������
// ����� �������� �����
const unsigned int adr_reg_temp_year      PROGMEM       = 40112; // ������� �������� ���������� ���  
const unsigned int adr_reg_temp_mon       PROGMEM       = 40113; // ������� �������� ���������� �����
const unsigned int adr_reg_temp_day       PROGMEM       = 40114; // ������� �������� ���������� ���� 
const unsigned int adr_reg_file_name      PROGMEM       = 40115; // ������� �������� ������� ������  
const unsigned int adr_reg_file_tek       PROGMEM       = 40116; // ������� �������� ������� ������  

const unsigned int adr_control_command    PROGMEM       = 40120; // ����� �������� ������� �� ���������� 
const unsigned int adr_reg_count_err      PROGMEM       = 40121; // ����� �������� ���� ������

const unsigned int adr_set_time           PROGMEM       = 36;    // ����� ���� ���������



//+++++++++ ������ ������� ������ ��� �������� ��������� ������ � ������� �������� +++++


const unsigned int adr_reg40000      PROGMEM       =    1024;  // 
const unsigned int adr_reg40001      PROGMEM       =    1026;  // �������� ������ � ����� 1
const unsigned int adr_reg40002      PROGMEM       =    1028;  // �������� ������ � ����� 1
const unsigned int adr_reg40003      PROGMEM       =    1030;  // �������� ������ � ����� 1
const unsigned int adr_reg40004      PROGMEM       =    1032;  // �������� ������ � ����� 1
const unsigned int adr_reg40005      PROGMEM       =    1034;  // �������� ������ � ����� 1
const unsigned int adr_reg40006      PROGMEM       =    1036;  // �������� ������ � ����� 1
const unsigned int adr_reg40007      PROGMEM       =    1038;  // �������� ������ � ����� 1
const unsigned int adr_reg40008      PROGMEM       =    1040;  // 
const unsigned int adr_reg40009      PROGMEM       =    1042;  // 

const unsigned int adr_reg40010      PROGMEM       =    1044;  // �  ����� 1
const unsigned int adr_reg40011      PROGMEM       =    1046;  // �  ����� 1
const unsigned int adr_reg40012      PROGMEM       =    1048;  // �  ����� 1
const unsigned int adr_reg40013      PROGMEM       =    1050;  // �  ����� 1
const unsigned int adr_reg40014      PROGMEM       =    1052;  // 
const unsigned int adr_reg40015      PROGMEM       =    1054;  // 
const unsigned int adr_reg40016      PROGMEM       =    1056;  // 
const unsigned int adr_reg40017      PROGMEM       =    1058;  // 
const unsigned int adr_reg40018      PROGMEM       =    1060;  // 
const unsigned int adr_reg40019      PROGMEM       =    1062;  // 
const unsigned int adr_reg40040      PROGMEM       =    1064;  // analogIn_oscill   �������� ����� ����������� ������ �������� 5.0
const unsigned int adr_reg40041      PROGMEM       =    1066;  //  ���������� ������� ��� ��������� ������������� (0-7)

						// ������� ����� 
const unsigned int adr_reg40046      PROGMEM       =    1068;  // ����� ���� ������ ����� �����������
const unsigned int adr_reg40047      PROGMEM       =    1070;  // ����� ����� ������ ����� �����������
const unsigned int adr_reg40048      PROGMEM       =    1072;  // ����� ��� ������ ����� �����������
const unsigned int adr_reg40049      PROGMEM       =    1074;  // ����� ��� ������ ����� �����������
const unsigned int adr_reg40050      PROGMEM       =    1076;  // ����� ������ ������ ����� �����������
const unsigned int adr_reg40051      PROGMEM       =    1078;  // ����� ������� ������ ����� �����������
 
						// ��������� ������� � �����������
const unsigned int adr_reg40052      PROGMEM       =    1080;  // ����� ����
const unsigned int adr_reg40053      PROGMEM       =    1082;  // ����� �����
const unsigned int adr_reg40054      PROGMEM       =    1084;  // ����� ���
const unsigned int adr_reg40055      PROGMEM       =    1086;  // ����� ���
const unsigned int adr_reg40056      PROGMEM       =    1088;  // ����� ������
const unsigned int adr_reg40057      PROGMEM       =    1090;  // 
const unsigned int adr_reg40058      PROGMEM       =    1092;  // 
const unsigned int adr_reg40059      PROGMEM       =    1094;  // 
	
const unsigned int adr_reg40060      PROGMEM       =    1096;  // ����� �������� �������� ������� ���������� � 1
const unsigned int adr_reg40061      PROGMEM       =    1098;  // ����� �������� �������� ������� ��� ����������
const unsigned int adr_reg40062      PROGMEM       =    1100;  // ����� �������� �������� ������� ��� �������� � ���������
const unsigned int adr_reg40063      PROGMEM       =    1102;  // ����� �������� ������������ �������� ������� ��� �������� � ��������� ��
const unsigned int adr_reg40064      PROGMEM       =    1104;  // ����� �������� �������� ������� ���������� � 2

const unsigned int adr_reg40065      PROGMEM       =    1106;  // ����� ������
const unsigned int adr_reg40066      PROGMEM       =    1108;  // ����� ������
const unsigned int adr_reg40067      PROGMEM       =    1110;  // ����� ������
const unsigned int adr_reg40068      PROGMEM       =    1112;  // ����� ������
const unsigned int adr_reg40069      PROGMEM       =    1114;  // ����� ������
const unsigned int adr_reg40070      PROGMEM       =    1116;  // ����� ������
const unsigned int adr_reg40071      PROGMEM       =    1118;  // ����� ������

const unsigned int adr_reg40072      PROGMEM       =    1120;  // ����� ������ � %
const unsigned int adr_reg40073      PROGMEM       =    1122;  // ����� ������ � %
const unsigned int adr_reg40074      PROGMEM       =    1124;  // ����� ������ � %
const unsigned int adr_reg40075      PROGMEM       =    1126;  // ����� ������ %
const unsigned int adr_reg40076      PROGMEM       =    1128;  // ����� ������ %
const unsigned int adr_reg40077      PROGMEM       =    1130;  // ����� ������ %
const unsigned int adr_reg40078      PROGMEM       =    1132;  // ����� ������ %
const unsigned int adr_reg40079      PROGMEM       =    1134;  // ����� ������ %
const unsigned int adr_reg40080      PROGMEM       =    1136;  // ����� ������ %
const unsigned int adr_reg40081      PROGMEM       =    1138;  // ����� ������ %
const unsigned int adr_reg40082      PROGMEM       =    1140;  // ����� ������ %
const unsigned int adr_reg40083      PROGMEM       =    1142;  // ����� ������ %

// ����� ������ �� ���������
const unsigned int adr_reg40084      PROGMEM       =    1144;  // ����� ���� adr_Mic_On_day 
const unsigned int adr_reg40085      PROGMEM       =    1146;  // ����� ����� adr_Mic_On_month  
const unsigned int adr_reg40086      PROGMEM       =    1148;  // ����� ��� adr_Mic_On_year  
const unsigned int adr_reg40087      PROGMEM       =    1150;  // ����� ��� adr_Mic_On_hour 
const unsigned int adr_reg40088      PROGMEM       =    1152;  // ����� ������ adr_Mic_On_minute 
const unsigned int adr_reg40089      PROGMEM       =    1154;  // ����� �������  adr_Mic_On_second    

// ����� ������ �� ����������
const unsigned int adr_reg40090      PROGMEM       =    1156;  // ����� ���� adr_Mic_Off_day    
const unsigned int adr_reg40091      PROGMEM       =    1158;  // ����� �����  adr_Mic_Off_month 
const unsigned int adr_reg40092      PROGMEM       =    1160;  // ����� ��� adr_Mic_Off_year  
const unsigned int adr_reg40093      PROGMEM       =    1162;  // ����� ��� adr_Mic_Off_hour   
const unsigned int adr_reg40094      PROGMEM       =    1164;  // ����� ������ adr_Mic_Off_minute   
const unsigned int adr_reg40095      PROGMEM       =    1166;  // ����� ������� adr_Mic_Off_second    
	
// ����� ������ �����
const unsigned int adr_reg40096      PROGMEM       =    1168;  // ����� ����  adr_Mic_Start_day    
const unsigned int adr_reg40097      PROGMEM       =    1170;  // ����� ����� adr_Mic_Start_month  
const unsigned int adr_reg40098      PROGMEM       =    1172;  // ����� ��� adr_Mic_Start_year  
const unsigned int adr_reg40099      PROGMEM       =    1174;  // ����� ��� adr_Mic_Start_hour 
const unsigned int adr_reg40100      PROGMEM       =    1176;  // ����� ������ adr_Mic_Start_minute 
const unsigned int adr_reg40101      PROGMEM       =    1178;  // ����� ������� adr_Mic_Start_second  

// ����� ��������� �����
const unsigned int adr_reg40102      PROGMEM       =    1180;  // ����� ���� adr_Mic_Stop_day 
const unsigned int adr_reg40103      PROGMEM       =    1182;  // ����� ����� adr_Mic_Stop_month 
const unsigned int adr_reg40104      PROGMEM       =    1184;  // ����� ��� adr_Mic_Stop_year
const unsigned int adr_reg40105      PROGMEM       =    1186;  // ����� ��� adr_Mic_Stop_hour 
const unsigned int adr_reg40106      PROGMEM       =    1188;  // ����� ������ adr_Mic_Stop_minute  
const unsigned int adr_reg40107      PROGMEM       =    1190;  // ����� ������� adr_Mic_Stop_second 

// ����������������� ���������� �����
const unsigned int adr_reg40108      PROGMEM       =    1192;  // ����� ���� adr_Time_Test_day 
const unsigned int adr_reg40109      PROGMEM       =    1194;  // ����� ��� adr_Time_Test_hour 
const unsigned int adr_reg40110      PROGMEM       =    1196;  // ����� ������ adr_Time_Test_minute
const unsigned int adr_reg40111      PROGMEM       =    1198;  // ����� ������� adr_Time_Test_second

// ��� �����
const unsigned int adr_reg40112      PROGMEM       =    1200;  // ����� �������� ���������� ��� 
const unsigned int adr_reg40113      PROGMEM       =    1202;  // ����� �������� ���������� ����� 
const unsigned int adr_reg40114      PROGMEM       =    1204;  // ����� �������� ���������� ����
const unsigned int adr_reg40115      PROGMEM       =    1206;  // ����� �������� ���������� �������� ���������� ������ �����
const unsigned int adr_reg40116      PROGMEM       =    1208;  // ����� �������� ���������� �������� �������� ������ �����

const unsigned int adr_reg40120      PROGMEM       =    1210;  // adr_control_command ����� �������� ������� �� ����������
const unsigned int adr_reg40121      PROGMEM       =    1212;  // ����� �������� ���� ������
const unsigned int adr_reg40122      PROGMEM       =    1214;  //
const unsigned int adr_reg40123      PROGMEM       =    1216;  //
const unsigned int adr_reg40124      PROGMEM       =    1218;  //
const unsigned int adr_reg40125      PROGMEM       =    1220;  //  
const unsigned int adr_reg40126      PROGMEM       =    1222;  //  
const unsigned int adr_reg40127      PROGMEM       =    1224;  //  ����� ����� ��������� ��� �������� � �� ������� �������.
const unsigned int adr_reg40128      PROGMEM       =    1226;  //  ����� ����� ������ ��� �������� � �� ������� �������.
const unsigned int adr_reg40129      PROGMEM       =    1228;  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.
//--------------------------------------------------------------------------------------
const unsigned int adr_reg40130      PROGMEM       =    1230;  //  �������� ���������� �������� ��� �������� ������� ������� 
const unsigned int adr_reg40131      PROGMEM       =    1232;  //
const unsigned int adr_reg40132      PROGMEM       =    1234;  //  
const unsigned int adr_reg40133      PROGMEM       =    1236;  //
const unsigned int adr_reg40134      PROGMEM       =    1238;  //  
const unsigned int adr_reg40135      PROGMEM       =    1240;  //
const unsigned int adr_reg40136      PROGMEM       =    1242;  //  
const unsigned int adr_reg40137      PROGMEM       =    1244;  //
const unsigned int adr_reg40138      PROGMEM       =    1246;  //  
const unsigned int adr_reg40139      PROGMEM       =    1248;  //

const unsigned int adr_reg40140      PROGMEM       =    1250;  //  
const unsigned int adr_reg40141      PROGMEM       =    1252;  //
const unsigned int adr_reg40142      PROGMEM       =    1254;  //  
const unsigned int adr_reg40143      PROGMEM       =    1256;  //
const unsigned int adr_reg40144      PROGMEM       =    1258;  //  
const unsigned int adr_reg40145      PROGMEM       =    1260;  //
const unsigned int adr_reg40146      PROGMEM       =    1262;  //  
const unsigned int adr_reg40147      PROGMEM       =    1264;  //
const unsigned int adr_reg40148      PROGMEM       =    1266;  //  
const unsigned int adr_reg40149      PROGMEM       =    1268;  //
//---------------------------------------------------------------------------------------

const unsigned int adr_reg40150      PROGMEM       =    1270;  //  
const unsigned int adr_reg40151      PROGMEM       =    1272;  //
const unsigned int adr_reg40152      PROGMEM       =    1274;  //  
const unsigned int adr_reg40153      PROGMEM       =    1276;  //
const unsigned int adr_reg40154      PROGMEM       =    1278;  //  
const unsigned int adr_reg40155      PROGMEM       =    1280;  //
const unsigned int adr_reg40156      PROGMEM       =    1282;  //  
const unsigned int adr_reg40157      PROGMEM       =    1284;  //
const unsigned int adr_reg40158      PROGMEM       =    1286;  //  
const unsigned int adr_reg40159      PROGMEM       =    1288;  //

const unsigned int adr_reg40160      PROGMEM       =    1290;  //  
const unsigned int adr_reg40161      PROGMEM       =    1292;  //
const unsigned int adr_reg40162      PROGMEM       =    1294;  //  
const unsigned int adr_reg40163      PROGMEM       =    1296;  //
const unsigned int adr_reg40164      PROGMEM       =    1298;  //  
const unsigned int adr_reg40165      PROGMEM       =    1300;  //
const unsigned int adr_reg40166      PROGMEM       =    1302;  //  
const unsigned int adr_reg40167      PROGMEM       =    1304;  //
const unsigned int adr_reg40168      PROGMEM       =    1306;  //  
const unsigned int adr_reg40169      PROGMEM       =    1308;  // 
	
const unsigned int adr_reg40200      PROGMEM       =    1310;  // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
const unsigned int adr_reg40400      PROGMEM       =    1312;  // A���� ���������� ADC0  ��� x1 
const unsigned int adr_reg40201      PROGMEM       =    1314;  // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
const unsigned int adr_reg40401      PROGMEM       =    1316;  // A���� ���������� ADC1 ���������� 12/3 �����
const unsigned int adr_reg40202      PROGMEM       =    1318;  // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
const unsigned int adr_reg40402      PROGMEM       =    1320;  // A���� ���������� ADC2 ��� x10
const unsigned int adr_reg40203      PROGMEM       =    1322;  // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
const unsigned int adr_reg40403      PROGMEM       =    1324;  // A���� ���������� ADC14 ���������� 12/3 ����� Radio1
const unsigned int adr_reg40204      PROGMEM       =    1326;  // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
const unsigned int adr_reg40404      PROGMEM       =    1328;  // A���� ���������� ADC14 ���������� 12/3 ����� Radio2
const unsigned int adr_reg40205      PROGMEM       =    1330;  // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
const unsigned int adr_reg40405      PROGMEM       =    1332;  // A���� ���������� ADC14 ���������� 12/3 ����� ���
const unsigned int adr_reg40206      PROGMEM       =    1334;  // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
const unsigned int adr_reg40406      PROGMEM       =    1336;  // A���� ���������� ADC15 ���������� ���������� 3,6 ������
const unsigned int adr_reg40207      PROGMEM       =    1338;  // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
const unsigned int adr_reg40407      PROGMEM       =    1340;  // A���� 
const unsigned int adr_reg40208      PROGMEM       =    1342;  // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
const unsigned int adr_reg40408      PROGMEM       =    1344;  // A���� 
const unsigned int adr_reg40209      PROGMEM       =    1346;  // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
const unsigned int adr_reg40409      PROGMEM       =    1348;  // A����  

const unsigned int adr_reg40210      PROGMEM       =    1350;  // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
const unsigned int adr_reg40410      PROGMEM       =    1352;  // A���� �������� 
const unsigned int adr_reg40211      PROGMEM       =    1354;  // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
const unsigned int adr_reg40411      PROGMEM       =    1356;  // A���� ��������  
const unsigned int adr_reg40212      PROGMEM       =    1358;  // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
const unsigned int adr_reg40412      PROGMEM       =    1360;  // A���� ��������  
const unsigned int adr_reg40213      PROGMEM       =    1362;  // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
const unsigned int adr_reg40413      PROGMEM       =    1364;  // A���� ��������  
const unsigned int adr_reg40214      PROGMEM       =    1366;  // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
const unsigned int adr_reg40414      PROGMEM       =    1368;  // A���� ��������  
const unsigned int adr_reg40215      PROGMEM       =    1370;  // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
const unsigned int adr_reg40415      PROGMEM       =    1372;  // A���� ��������  
const unsigned int adr_reg40216      PROGMEM       =    1374;  // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
const unsigned int adr_reg40416      PROGMEM       =    1376;  // A���� ��������  
const unsigned int adr_reg40217      PROGMEM       =    1378;  // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
const unsigned int adr_reg40417      PROGMEM       =    1380;  // A���� ��������  
const unsigned int adr_reg40218      PROGMEM       =    1382;  // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
const unsigned int adr_reg40418      PROGMEM       =    1384;  // A���� ��������  
const unsigned int adr_reg40219      PROGMEM       =    1386;  // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
const unsigned int adr_reg40419      PROGMEM       =    1388;  // A���� ��������  

const unsigned int adr_reg40220      PROGMEM       =    1390;  // A���� �������� ������ "Command PTT headset instructor (CTS)                        OFF - ";
const unsigned int adr_reg40420      PROGMEM       =    1392;  // A����  ;
const unsigned int adr_reg40221      PROGMEM       =    1394;  // A���� �������� ������ "Command PTT headset instructor (CTS)                        ON  - ";
const unsigned int adr_reg40421      PROGMEM       =    1396;  // A����  ;
const unsigned int adr_reg40222      PROGMEM       =    1398;  // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
const unsigned int adr_reg40422      PROGMEM       =    1400;  // A����  ;
const unsigned int adr_reg40223      PROGMEM       =    1402;  // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
const unsigned int adr_reg40423      PROGMEM       =    1404;  // A����  ;
const unsigned int adr_reg40224      PROGMEM       =    1406;  // A���� �������� ������  "Test headset instructor ** Signal LineL                    ON  - ";
const unsigned int adr_reg40424      PROGMEM       =    1408;  // A���� ������ ��������� "Test headset instructor ** Signal LineL                    ON  - ";
const unsigned int adr_reg40225      PROGMEM       =    1410;  // A���� �������� ������  "Test headset instructor ** Signal LineR                    ON  - ";   
const unsigned int adr_reg40425      PROGMEM       =    1412;  // A���� ������ ��������� "Test headset instructor ** Signal LineR                    ON  - ";   
const unsigned int adr_reg40226      PROGMEM       =    1414;  // A���� �������� ������  "Test headset instructor ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40426      PROGMEM       =    1416;  // A���� ������ ��������� "Test headset instructor ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40227      PROGMEM       =    1418;  // A���� �������� ������  "Test headset dispatcher ** Signal LineL                    ON  - ";
const unsigned int adr_reg40427      PROGMEM       =    1420;  // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                    ON  - ";
const unsigned int adr_reg40228      PROGMEM       =    1422;  // A���� �������� ������  "Test headset dispatcher ** Signal LineR                    ON  - ";  
const unsigned int adr_reg40428      PROGMEM       =    1424;  // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                    ON  - ";  
const unsigned int adr_reg40229      PROGMEM       =    1426;  // A���� �������� ������  "Test headset dispatcher ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40429      PROGMEM       =    1428;  // A���� ������ ��������� "Test headset dispatcher ** Signal Mag phone                ON  - ";

const unsigned int adr_reg40230      PROGMEM       =    1430;  // A���� �������� ������  "Test headset instructor ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40430      PROGMEM       =    1432;  // A���� ������ ��������� "Test headset instructor ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40231      PROGMEM       =    1434;  // A���� �������� ������  "Test headset instructor ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40431      PROGMEM       =    1436;  // A���� ������ ��������� "Test headset instructor ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40232      PROGMEM       =    1438;  // A���� �������� ������  "Test headset instructor ** Signal LineL                    OFF - ";
const unsigned int adr_reg40432      PROGMEM       =    1440;  // A���� ������ ��������� "Test headset instructor ** Signal LineL                    OFF - ";
const unsigned int adr_reg40233      PROGMEM       =    1442;  // A���� �������� ������  "Test headset instructor ** Signal LineR                    OFF - ";
const unsigned int adr_reg40433      PROGMEM       =    1444;  // A���� ������ ��������� "Test headset instructor ** Signal LineR                    OFF - ";
const unsigned int adr_reg40234      PROGMEM       =    1446;  // A���� �������� ������  "Test headset instructor ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40434      PROGMEM       =    1448;  // A���� ������ ��������� "Test headset instructor ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40235      PROGMEM       =    1450;  // A���� �������� ������  "Test headset instructor ** Signal mag phone                OFF - ";
const unsigned int adr_reg40435      PROGMEM       =    1452;  // A���� ������ ��������� "Test headset instructor ** Signal mag phone                OFF - ";
const unsigned int adr_reg40236      PROGMEM       =    1454;  // A���� �������� ������  "Test headset instructor ** Signal GGS                      OFF - ";
const unsigned int adr_reg40436      PROGMEM       =    1456;  // A���� ������ ��������� "Test headset instructor ** Signal GGS                      OFF - ";
const unsigned int adr_reg40237      PROGMEM       =    1458;  // A���� �������� ������  "Test headset instructor ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40437      PROGMEM       =    1460;  // A���� ������ ��������� "Test headset instructor ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40238      PROGMEM       =    1462;  // A���� �������� ������  "Test headset instructor ** Signal GG Radio2                OFF - ";
const unsigned int adr_reg40438      PROGMEM       =    1464;  // A���� ������ ��������� "Test headset instructor ** Signal GG Radio2                OFF - ";
const unsigned int adr_reg40239      PROGMEM       =    1466;  // A���� �������� ������ ADC0  ��� x1 
const unsigned int adr_reg40439      PROGMEM       =    1468;  // A���� ������ ��������� ADC0  ��� x1 

const unsigned int adr_reg40240      PROGMEM       =    1470;  // A���� �������� ������  "Test headset dispatcher ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40440      PROGMEM       =    1472;  // A���� ������ ��������� "Test headset dispatcher ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40241      PROGMEM       =    1474;  // A���� �������� ������  "Test headset dispatcher ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40441      PROGMEM       =    1476;  // A���� ������ ��������� "Test headset dispatcher ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40242      PROGMEM       =    1478;  // A���� �������� ������  "Test headset dispatcher ** Signal LineL                    OFF - "; 
const unsigned int adr_reg40442      PROGMEM       =    1480;  // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                    OFF - "; 
const unsigned int adr_reg40243      PROGMEM       =    1482;  // A���� �������� ������  "Test headset dispatcher ** Signal LineR                    OFF - ";
const unsigned int adr_reg40443      PROGMEM       =    1484;  // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                    OFF - ";
const unsigned int adr_reg40244      PROGMEM       =    1486;  // A���� �������� ������  "Test headset dispatcher ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40444      PROGMEM       =    1488;  // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40245      PROGMEM       =    1490;  // A���� �������� ������  "Test headset dispatcher ** Signal mag phone                OFF - ";
const unsigned int adr_reg40445      PROGMEM       =    1492;  // A���� ������ ��������� "Test headset dispatcher ** Signal mag phone                OFF - ";
const unsigned int adr_reg40246      PROGMEM       =    1494;  // A���� �������� ������  "Test headset dispatcher ** Signal GGS                      OFF - "; 
const unsigned int adr_reg40446      PROGMEM       =    1496;  // A���� ������ ��������� "Test headset dispatcher ** Signal GGS                      OFF - "; 
const unsigned int adr_reg40247      PROGMEM       =    1498;  // A���� �������� ������  "Test headset dispatcher ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40447      PROGMEM       =    1500;  // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40248      PROGMEM       =    1502;  // A���� �������� ������  "Test headset dispatcher ** Signal GG Radio2                OFF - "; 
const unsigned int adr_reg40448      PROGMEM       =    1504;  // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio2                OFF - "; 
const unsigned int adr_reg40249      PROGMEM       =    1506;  // A���� �������� ������ ADC2 ��� x10
const unsigned int adr_reg40449      PROGMEM       =    1508;  // A���� ������ ��������� ADC2 ��� x10

const unsigned int adr_reg40250      PROGMEM       =    1510;  // A���� �������� ������  "Test MTT ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40450      PROGMEM       =    1512;  // A���� ������ ��������� "Test MTT ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40251      PROGMEM       =    1514;  // A���� �������� ������  "Test MTT ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40451      PROGMEM       =    1516;  // A���� ������ ��������� "Test MTT ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40252      PROGMEM       =    1518;  // A���� �������� ������  "Test MTT ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40452      PROGMEM       =    1520;  // A���� ������ ��������� "Test MTT ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40253      PROGMEM       =    1522;  // A���� �������� ������  "Test MTT ** Signal LineR                                   OFF - "; 
const unsigned int adr_reg40453      PROGMEM       =    1524;  // A���� ������ ��������� "Test MTT ** Signal LineR                                   OFF - "; 
const unsigned int adr_reg40254      PROGMEM       =    1526;  // A���� �������� ������  "Test MTT ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40454      PROGMEM       =    1528;  // A���� ������ ��������� "Test MTT ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40255      PROGMEM       =    1530;  // A���� �������� ������  "Test MTT ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40455      PROGMEM       =    1532;  // A���� ������ ��������� "Test MTT ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40256      PROGMEM       =    1534;  // A���� �������� ������  "Test MTT ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40456      PROGMEM       =    1536;  // A���� ������ ��������� "Test MTT ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40257      PROGMEM       =    1538;  // A���� �������� ������  "Test MTT ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40457      PROGMEM       =    1540;  // A���� ������ ��������� "Test MTT ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40258      PROGMEM       =    1542;  // A���� �������� ������  "Test MTT ** Signal GG Radio2                               OFF - "; 
const unsigned int adr_reg40458      PROGMEM       =    1544;  // A���� ������ ��������� "Test MTT ** Signal GG Radio2                               OFF - "; 
const unsigned int adr_reg40259      PROGMEM       =    1546;  // A���� �������� ������  "Test MTT ** Signal GGS                                     ON  - ";
const unsigned int adr_reg40459      PROGMEM       =    1548;  // A���� ������ ��������� "Test MTT ** Signal GGS                                     ON  - ";

const unsigned int adr_reg40260      PROGMEM       =    1550;  // A���� �������� ������  "Test MTT ** Signal LineL                                   ON  - ";
const unsigned int adr_reg40460      PROGMEM       =    1552;  // A���� ������ ��������� "Test MTT ** Signal LineL                                   ON  - ";
const unsigned int adr_reg40261      PROGMEM       =    1554;  // A���� �������� ������  "Test MTT ** Signal LineR                                   ON  - ";  
const unsigned int adr_reg40461      PROGMEM       =    1556;  // A���� ������ ��������� "Test MTT ** Signal LineR                                   ON  - ";  
const unsigned int adr_reg40262      PROGMEM       =    1558;  // A���� �������� ������  "Test MTT ** Signal Mag phone                               ON  - ";
const unsigned int adr_reg40462      PROGMEM       =    1560;  // A���� ������ ��������� "Test MTT ** Signal Mag phone                               ON  - ";
const unsigned int adr_reg40263      PROGMEM       =    1562;  // A���� �������� ������  "Test MTT PTT    (CTS)                                      OFF - ";
const unsigned int adr_reg40463      PROGMEM       =    1564;  // A���� ������ ��������� "Test MTT PTT    (CTS)                                      OFF - ";
const unsigned int adr_reg40264      PROGMEM       =    1566;  // A���� �������� ������  "Test microphone PTT  (CTS)                                 OFF - ";
const unsigned int adr_reg40464      PROGMEM       =    1568;  // 
const unsigned int adr_reg40265      PROGMEM       =    1570;  // A���� �������� ������  "Test MTT PTT    (CTS)                                      ON  - ";
const unsigned int adr_reg40465      PROGMEM       =    1572;  // A���� ������ ��������� "Test MTT PTT    (CTS)                                      ON  - ";
const unsigned int adr_reg40266      PROGMEM       =    1574;  // A���� �������� ������  "Test microphone PTT  (CTS)                                 ON  - ";
const unsigned int adr_reg40466      PROGMEM       =    1576;  // 
const unsigned int adr_reg40267      PROGMEM       =    1578;  // A���� �������� ������  "Test MTT HangUp (DCD)                                      OFF - ";
const unsigned int adr_reg40467      PROGMEM       =    1580;  // A���� ������ ��������� "Test MTT HangUp (DCD)                                      OFF - ";
const unsigned int adr_reg40268      PROGMEM       =    1582;  // A���� �������� ������  "Test MTT HangUp (DCD)                                      ON  - ";
const unsigned int adr_reg40468      PROGMEM       =    1584;  // A���� ������ ��������� "Test MTT HangUp (DCD)                                      ON  - ";
const unsigned int adr_reg40269      PROGMEM       =    1586;  // A���� �������� ������ ������������ ����������� �������
const unsigned int adr_reg40469      PROGMEM       =    1588;  // ������������ �������� ����������� ������� �������

const unsigned int adr_reg40270      PROGMEM       =    1590;  // A���� �������� ������  "Command PTT1 tangenta ruchnaja (CTS)                       OFF - ";
const unsigned int adr_reg40470      PROGMEM       =    1592;  // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                       OFF - ";
const unsigned int adr_reg40271      PROGMEM       =    1594;  // A���� �������� ������  "Command PTT2 tangenta ruchnaja (DCR)                       OFF - ";
const unsigned int adr_reg40471      PROGMEM       =    1596;  // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                       OFF - ";
const unsigned int adr_reg40272      PROGMEM       =    1598;  // A���� �������� ������  "Command PTT1 tangenta ruchnaja (CTS)                       ON  - ";
const unsigned int adr_reg40472      PROGMEM       =    1600;  // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                       ON  - ";
const unsigned int adr_reg40273      PROGMEM       =    1602;  // A���� �������� ������  "Command PTT2 tangenta ruchnaja (DCR)                       ON  - ";
const unsigned int adr_reg40473      PROGMEM       =    1604;  // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                       ON  - ";
const unsigned int adr_reg40274      PROGMEM       =    1606;  // A���� �������� ������  "Command sensor tangenta ruchnaja                           OFF - ";
const unsigned int adr_reg40474      PROGMEM       =    1608;  // A���� ������ ��������� "Command sensor tangenta ruchnaja                           OFF - ";
const unsigned int adr_reg40275      PROGMEM       =    1610;  // A���� �������� ������  "Command sensor tangenta ruchnaja                           ON  - ";
const unsigned int adr_reg40475      PROGMEM       =    1612;  // A���� ������ ��������� "Command sensor tangenta ruchnaja                           ON  - ";
const unsigned int adr_reg40276      PROGMEM       =    1614;  // A���� �������� ������  "Command sensor tangenta nognaja                            OFF - ";
const unsigned int adr_reg40476      PROGMEM       =    1616;  // A���� ������ ��������� "Command sensor tangenta nognaja                            OFF - ";
const unsigned int adr_reg40277      PROGMEM       =    1618;  // A���� �������� ������  "Command sensor tangenta nognaja                            ON  - ";
const unsigned int adr_reg40477      PROGMEM       =    1620;  // A���� ������ ��������� "Command sensor tangenta nognaja                            ON  - ";
const unsigned int adr_reg40278      PROGMEM       =    1622;  // A���� �������� ������  "Command PTT tangenta nognaja (CTS)                         OFF - ";
const unsigned int adr_reg40478      PROGMEM       =    1624;  // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                         OFF - ";
const unsigned int adr_reg40279      PROGMEM       =    1626;  // A���� �������� ������  "Command PTT tangenta nognaja (CTS)                         ON  - ";
const unsigned int adr_reg40479      PROGMEM       =    1628;  // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                         ON  - ";

const unsigned int adr_reg40280      PROGMEM       =    1630;  // A���� �������� ������  "Test GGS ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40480      PROGMEM       =    1632;  // A���� ������ ��������� "Test GGS ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40281      PROGMEM       =    1634;  // A���� �������� ������  "Test GGS ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40481      PROGMEM       =    1636;  // A���� ������ ��������� "Test GGS ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40282      PROGMEM       =    1638;  // A���� �������� ������  "Test GGS ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40482      PROGMEM       =    1640;  // A���� ������ ��������� "Test GGS ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40283      PROGMEM       =    1642;  // A���� �������� ������  "Test GGS ** Signal LineR                                   OFF - ";
const unsigned int adr_reg40483      PROGMEM       =    1644;  // A���� ������ ��������� "Test GGS ** Signal LineR                                   OFF - ";
const unsigned int adr_reg40284      PROGMEM       =    1646;  // A���� �������� ������  "Test GGS ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40484      PROGMEM       =    1648;  // A���� ������ ��������� "Test GGS ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40285      PROGMEM       =    1650;  // A���� �������� ������  "Test GGS ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40485      PROGMEM       =    1652;  // A���� ������ ��������� "Test GGS ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40286      PROGMEM       =    1654;  // A���� �������� ������  "Test GGS ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40486      PROGMEM       =    1656;  // A���� ������ ��������� "Test GGS ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40287      PROGMEM       =    1658;  // A���� �������� ������  "Test GGS ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40487      PROGMEM       =    1660;  // A���� ������ ��������� "Test GGS ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40288      PROGMEM       =    1662;  // A���� �������� ������  "Test GGS ** Signal GG Radio2                               OFF - ";
const unsigned int adr_reg40488      PROGMEM       =    1664;  // A���� ������ ��������� "Test GGS ** Signal GG Radio2                               OFF - ";
const unsigned int adr_reg40289      PROGMEM       =    1666;  // A���� �������� ������  "Test GGS ** Signal GGS                                     ON  - ";
const unsigned int adr_reg40489      PROGMEM       =    1668;  // A���� ������ ��������� "Test GGS ** Signal GGS                                     ON  - ";

const unsigned int adr_reg40290      PROGMEM       =    1670;  // A���� �������� ������  "Test GGS ** Signal FrontL                                  ON  - ";
const unsigned int adr_reg40490      PROGMEM       =    1672;  // A���� ������ ��������� "Test GGS ** Signal FrontL                                  ON  - ";
const unsigned int adr_reg40291      PROGMEM       =    1674;  // A���� �������� ������  "Test GGS ** Signal FrontR                                  ON  - ";
const unsigned int adr_reg40491      PROGMEM       =    1676;  // A���� ������ ��������� "Test GGS ** Signal FrontR                                  ON  - ";
const unsigned int adr_reg40292      PROGMEM       =    1678;  // A���� �������� ������  "Test GGS ** Signal mag phone                               ON  - ";
const unsigned int adr_reg40492      PROGMEM       =    1680;  // A���� ������ ��������� "Test GGS ** Signal mag phone                               ON  - ";
const unsigned int adr_reg40293      PROGMEM       =    1682;  // A���� ��������  ������ ADC1 ���������� 12/3 �����
const unsigned int adr_reg40493      PROGMEM       =    1684;  // A���� ������ ��������� ADC1 ���������� 12/3 �����
const unsigned int adr_reg40294      PROGMEM       =    1686;  // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
const unsigned int adr_reg40494      PROGMEM       =    1688;  // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
const unsigned int adr_reg40295      PROGMEM       =    1690;  // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
const unsigned int adr_reg40495      PROGMEM       =    1692;  // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
const unsigned int adr_reg40296      PROGMEM       =    1694;  // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
const unsigned int adr_reg40496      PROGMEM       =    1696;  // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
const unsigned int adr_reg40297      PROGMEM       =    1698;  // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������
const unsigned int adr_reg40497      PROGMEM       =    1700;  // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������
const unsigned int adr_reg40298      PROGMEM       =    1702;  // A���� �������� ������  "Test Microphone ** Signal mag phone                        ON  - ";    
const unsigned int adr_reg40498      PROGMEM       =    1704;  // A���� ������ ��������� "Test Microphone ** Signal mag phone                        ON  - "; 
const unsigned int adr_reg40299      PROGMEM       =    1706;  // A���� �������� ������  "Test Microphone ** Signal LineL                            ON  - ";   
const unsigned int adr_reg40499      PROGMEM       =    1708;  // A���� ������ ��������� "Test Microphone ** Signal LineL                            ON  - ";   

const unsigned int adr_reg40300      PROGMEM       =    1710;  // A���� �������� ������  "Test Radio1 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40500      PROGMEM       =    1712;  // A���� ������ ��������� "Test Radio1 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40301      PROGMEM       =    1714;  // A���� �������� ������  "Test Radio1 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40501      PROGMEM       =    1716;  // A���� ������ ��������� "Test Radio1 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40302      PROGMEM       =    1718;  // A���� �������� ������  "Test Radio1 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40502      PROGMEM       =    1720;  // A���� ������ ��������� "Test Radio1 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40303      PROGMEM       =    1722;  // A���� �������� ������  "Test Radio1 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40503      PROGMEM       =    1724;  // A���� ������ ��������� "Test Radio1 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40304      PROGMEM       =    1726;  // A���� �������� ������  "Test Radio1 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40504      PROGMEM       =    1728;  // A���� ������ ��������� "Test Radio1 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40305      PROGMEM       =    1730;  // A���� �������� ������  "Test Radio1 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40505      PROGMEM       =    1732;  // A���� ������ ��������� "Test Radio1 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40306      PROGMEM       =    1734;  // A���� �������� ������  "Test Radio1 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40506      PROGMEM       =    1736;  // A���� ������ ��������� "Test Radio1 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40307      PROGMEM       =    1738;  // A���� �������� ������  "Test Radio1 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40507      PROGMEM       =    1740;  // A���� ������ ��������� "Test Radio1 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40308      PROGMEM       =    1742;  // A���� �������� ������  "Test Radio1 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40508      PROGMEM       =    1744;  // A���� ������ ��������� "Test Radio1 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40309      PROGMEM       =    1746;  // A���� �������� ������  "Test Radio1 ** Signal Radio1                               ON  - ";
const unsigned int adr_reg40509      PROGMEM       =    1748;  // A���� ������ ��������� "Test Radio1 ** Signal Radio1                               ON  - ";

const unsigned int adr_reg40310      PROGMEM       =    1750;  // A���� �������� ������  "Test Radio2 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40510      PROGMEM       =    1752;  // A���� ������ ��������� "Test Radio2 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40311      PROGMEM       =    1754;  // A���� �������� ������  "Test Radio2 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40511      PROGMEM       =    1756;  // A���� ������ ��������� "Test Radio2 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40312      PROGMEM       =    1758;  // A���� �������� ������  "Test Radio2 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40512      PROGMEM       =    1760;  // A���� ������ ��������� "Test Radio2 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40313      PROGMEM       =    1762;  // A���� �������� ������  "Test Radio2 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40513      PROGMEM       =    1764;  // A���� ������ ��������� "Test Radio2 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40314      PROGMEM       =    1766;  // A���� �������� ������  "Test Radio2 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40514      PROGMEM       =    1768;  // A���� ������ ��������� "Test Radio2 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40315      PROGMEM       =    1770;  // A���� �������� ������  "Test Radio2 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40515      PROGMEM       =    1772;  // A���� ������ ��������� "Test Radio2 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40316      PROGMEM       =    1774;  // A���� �������� ������  "Test Radio2 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40516      PROGMEM       =    1776;  // A���� ������ ��������� "Test Radio2 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40317      PROGMEM       =    1778;  // A���� �������� ������  "Test Radio2 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40517      PROGMEM       =    1780;  // A���� ������ ��������� "Test Radio2 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40318      PROGMEM       =    1782;  // A���� �������� ������  "Test Radio2 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40518      PROGMEM       =    1784;  // A���� ������ ��������� "Test Radio2 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40319      PROGMEM       =    1786;  // A���� �������� ������  "Test Radio2 ** Signal Radio2                               ON  - ";
const unsigned int adr_reg40519      PROGMEM       =    1788;  // A���� ������ ��������� "Test Radio2 ** Signal Radio2                               ON  - ";

const unsigned int adr_reg40320      PROGMEM       =    1790;  // A���� �������� ������  "Test Microphone ** Signal FrontL                           OFF - ";
const unsigned int adr_reg40520      PROGMEM       =    1792;  // A���� ������ ��������� "Test Microphone ** Signal FrontL                           OFF - ";
const unsigned int adr_reg40321      PROGMEM       =    1794;  // A���� �������� ������  "Test Microphone ** Signal FrontR                           OFF - ";
const unsigned int adr_reg40521      PROGMEM       =    1796;  // A���� ������ ��������� "Test Microphone ** Signal FrontR                           OFF - ";
const unsigned int adr_reg40322      PROGMEM       =    1798;  // A���� �������� ������  "Test Microphone ** Signal LineL                            OFF - ";
const unsigned int adr_reg40522      PROGMEM       =    1800;  // A���� ������ ��������� "Test Microphone ** Signal LineL                            OFF - ";
const unsigned int adr_reg40323      PROGMEM       =    1802;  // A���� �������� ������  "Test Microphone ** Signal LineR                            OFF - ";
const unsigned int adr_reg40523      PROGMEM       =    1804;  // A���� ������ ��������� "Test Microphone ** Signal LineR                            OFF - ";
const unsigned int adr_reg40324      PROGMEM       =    1806;  // A���� �������� ������  "Test Microphone ** Signal mag radio                        OFF - ";
const unsigned int adr_reg40524      PROGMEM       =    1808;  // A���� ������ ��������� "Test Microphone ** Signal mag radio                        OFF - ";
const unsigned int adr_reg40325      PROGMEM       =    1810;  // A���� �������� ������  "Test Microphone ** Signal mag phone                        OFF - ";
const unsigned int adr_reg40525      PROGMEM       =    1812;  // A���� ������ ��������� "Test Microphone ** Signal mag phone                        OFF - ";
const unsigned int adr_reg40326      PROGMEM       =    1814;  // A���� �������� ������  "Test Microphone ** Signal GGS                              OFF - ";
const unsigned int adr_reg40526      PROGMEM       =    1816;  // A���� ������ ��������� "Test Microphone ** Signal GGS                              OFF - ";
const unsigned int adr_reg40327      PROGMEM       =    1818;  // A���� �������� ������  "Test Microphone ** Signal GG Radio1                        OFF - ";
const unsigned int adr_reg40527      PROGMEM       =    1820;  // A���� ������ ��������� "Test Microphone ** Signal GG Radio1                        OFF - ";
const unsigned int adr_reg40328      PROGMEM       =    1822;  // A���� �������� ������  "Test Microphone ** Signal GG Radio2                        OFF - ";
const unsigned int adr_reg40528      PROGMEM       =    1824;  // A���� ������ ��������� "Test Microphone ** Signal GG Radio2                        OFF - ";
const unsigned int adr_reg40329      PROGMEM       =    1826;  // A���� �������� ������ ��� ����������� �������                             // 
const unsigned int adr_reg40529      PROGMEM       =    1828;  // ��� ����������� ������� �������

const unsigned int adr_reg40330      PROGMEM       =    1830;  // A���� �������� ������  "Test Radio1 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40530      PROGMEM       =    1832;  // A���� ������ ��������� "Test Radio1 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40331      PROGMEM       =    1834;  // A���� �������� ������  "Test Radio2 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40531      PROGMEM       =    1836;  // A���� ������ ��������� "Test Radio2 ** Signal mag radio                            ON  - ";                    // 
const unsigned int adr_reg40332      PROGMEM       =    1838;  // A���� �������� ������  "Test GGS    ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40532      PROGMEM       =    1840;  // A���� ������ ��������� "Test GGS    ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40333      PROGMEM       =    1842;  // A���� �������� ������  "Test headset instructor ** Signal mag radio                ON  - ";
const unsigned int adr_reg40533      PROGMEM       =    1844;  // A���� ������ ��������� "Test headset instructor ** Signal mag radio                ON  - ";
const unsigned int adr_reg40334      PROGMEM       =    1846;  // A���� �������� ������  "Test headset dispatcher ** Signal mag radio                ON  - ";
const unsigned int adr_reg40534      PROGMEM       =    1848;  // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                ON  - ";
const unsigned int adr_reg40335      PROGMEM       =    1850;  // A���� �������� ������  "Test MTT ** Signal mag radio                               ON  - ";
const unsigned int adr_reg40535      PROGMEM       =    1852;  // A���� ������ ��������� "Test MTT ** Signal mag radio                               ON  - ";
const unsigned int adr_reg40336      PROGMEM       =    1854;  // A���� �������� ������  "Test Microphone ** Signal mag radio                        ON  - "; 
const unsigned int adr_reg40536      PROGMEM       =    1856;  // A���� ������ ��������� "Test Microphone ** Signal mag radio                        ON  - "; 
const unsigned int adr_reg40337      PROGMEM       =    1858;  // A���� �������� ������   
const unsigned int adr_reg40537      PROGMEM       =    1860;  // A���� ������ ���������   
const unsigned int adr_reg40338      PROGMEM       =    1862;  // A���� �������� ������   
const unsigned int adr_reg40538      PROGMEM       =    1864;  // A���� ������ ���������   
const unsigned int adr_reg40339      PROGMEM       =    1866;  // A���� �������� ������        
const unsigned int adr_reg40539      PROGMEM       =    1868;  // 

//------------------------- ������ ��������� �������� �������� ��� ������������ ���������--------------------------------------
//++++++++++++++++++++++++++++ ��������� ��������� ������� ������� +++++++++++++++++++++++++++++++++++++

// ������ ������� ������ ��� �������� ������� ������� ��������� ��������

const  int adr_set_USB                     = 180;  

//����� ������ ��� ������, 2 �����
const  int adr_int_porog_instruktor            = 201;          // 201,203 ������� �����, 206 - 276   ������
const  int adr_int_porog_dispatcher            = 277;          // 277,279 ������� �����, 282 - 352   ������
const  int adr_int_porog_MTT                   = 353;          // 353,355 ������� �����, 358 - 428   ������
const  int adr_int_porog_Microphone            = 429;          // 429,431 ������� �����, 434 - 504   ������
const  int adr_int_porog_GGS                   = 505;          // 505,507 ������� �����, 510 - 580   ������
const  int adr_int_porog_Radio1                = 581;          // 581,584 ������� �����, 586 - 656   ������
const  int adr_int_porog_Radio2                = 657;          // 657,659 ������� �����, 662 - 734   ������

int por_int_buffer[40] ;                                       // ����� �������� ��������� ���������� ������� �������                      
const unsigned int porog_default[]    PROGMEM  = {             // 270 �����
//++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
	44,  // [1] resistor(1, 25);     ���������� ������� ������� 30 ��
	44,  // [2] resistor(2, 25);     ���������� ������� ������� 30 ��
	8,   // [3] measure_vol_min(analog_FrontL,   40230,230,18);  ������� ������� �� ������ FrontL  
	8,   // [4] measure_vol_min(analog_FrontL,   40230,230,18);  ������� ������� �� ������ FrontL  
	8,   // [5] measure_vol_min(analog_FrontR,   40231,231,18);  ������� ������� �� ������ FrontR
	8,   // [6] measure_vol_min(analog_FrontR,   40231,231,18);  ������� ������� �� ������ FrontR
	8,   // [7] measure_vol_min(analog_LineL,    40232,232,18);  ������� ������� �� ������ LineL 
	8,   // [8] measure_vol_min(analog_LineL,    40232,232,18);  ������� ������� �� ������ LineL 
	8,   // [9] measure_vol_min(analog_LineR,    40233,233,18);  ������� ������� �� ������ LineR
	8,   //[10] measure_vol_min(analog_LineR,    40233,233,18);  ������� ������� �� ������ LineR
	8,   //[11] measure_vol_min(analog_mag_radio,40234,234,18);  ������� ������� �� ������ mag radio 
	8,   //[12] measure_vol_min(analog_mag_radio,40234,234,18);  ������� ������� �� ������ mag radio 
	8,   //[13] measure_vol_min(analog_mag_phone,40238,238,18);  ������� ������� �� ������ mag phone
	8,   //[14] measure_vol_min(analog_mag_phone,40235,235,15);  ������� ������� �� ������ mag phone
	20,  //[15] measure_vol_min(analog_ggs,      40236,236,15);  ������� ������� �� ������ GGS 
	20,  //[16] measure_vol_min(analog_ggs,      40236,236,15);  ������� ������� �� ������ GGS 
	20,  //[17] measure_vol_min(analog_gg_radio1,40237,237,15);  ������� ������� �� ������ GG Radio1
	20,  //[18] measure_vol_min(analog_gg_radio1,40237,237,15);  ������� ������� �� ������ GG Radio1
	20,  //[19] measure_vol_min(analog_gg_radio2,40238,238,15);  ������� ������� �� ������ GG Radio2 
	20,  //[20] measure_vol_min(analog_gg_radio2,40238,238,15);  ������� ������� �� ������ GG Radio2 
	// ------------- MAX ---------------------------------------------
	0,   //[21]
	0,   //[22]
	0,   //[23]
	0,   //[24]
	75,  //[25] min measure_vol_max(analog_LineL,    40224,224,100); ������� ������� �� ������ LineL
	95,  //[26] max measure_vol_max(analog_LineL,    40224,224,150); ������� ������� �� ������ LineL
	0,   //[27]
	0,   //[28]
	45,  //[29] min measure_vol_max(analog_mag_radio,40333,333,40) ; ������� ������� �� ������ mag phone 
	65,  //[30] max measure_vol_max(analog_mag_radio,40333,333,100); ������� ������� �� ������ mag phone 
	45,  //[31] min measure_vol_max(analog_mag_phone,40226,226,80) ; ������� ������� �� ������ mag phone 
	65,  //[32] max measure_vol_max(analog_mag_phone,40226,226,120); ������� ������� �� ������ mag phone 
	0,   //[33]
	0,   //[34]
	0,   //[35]
	0,   //[36]
	0,   //[37]
	0,   //[38]
//++++++++++++++++  Test headset dispatcher ++++++++++++++++++++++++++++
	44,  //[39] resistor(1, 30);   ���������� ������� ������� 30 ��
	44,  //[40] resistor(2, 30);   ���������� ������� ������� 30 ��

	8,   //[41] measure_vol_min(analog_FrontL,   40240,240,18);  ������� ������� �� ������ FrontL 
	8,   //[42] measure_vol_min(analog_FrontL,   40240,240,18);  ������� ������� �� ������ FrontL 
	8,   //[43] measure_vol_min(analog_FrontR,   40241,241,18);  ������� ������� �� ������ FrontR
	8,   //[44] measure_vol_min(analog_FrontR,   40241,241,18);  ������� ������� �� ������ FrontR
	8,   //[45] measure_vol_min(analog_LineL,    40242,242,38);  ������� ������� �� ������ LineL
	8,   //[46] measure_vol_min(analog_LineL,    40242,242,38);  ������� ������� �� ������ LineL
	8,   //[47] measure_vol_min(analog_LineR,    40243,243,38);  ������� ������� �� ������ LineR
	8,   //[48] measure_vol_min(analog_LineR,    40243,243,38);  ������� ������� �� ������ LineR
	8,   //[49] measure_vol_min(analog_mag_radio,40244,244,35);  ������� ������� �� ������ mag radio
	8,   //[50] measure_vol_min(analog_mag_radio,40244,244,35);  ������� ������� �� ������ mag radio
	8,   //[51] measure_vol_min(analog_mag_phone,40245,245,35);  ������� ������� �� ������ mag phone
	8,   //[52] measure_vol_min(analog_mag_phone,40245,245,35);  ������� ������� �� ������ mag phone
	20,  //[53] measure_vol_min(analog_ggs,      40246,246,35);  ������� ������� �� ������ GGS 
	20,  //[54] measure_vol_min(analog_ggs,      40246,246,35);  ������� ������� �� ������ GGS 
	20,  //[55] measure_vol_min(analog_gg_radio1,40247,247,35);  ������� ������� �� ������ GG Radio1
	20,  //[56] measure_vol_min(analog_gg_radio1,40247,247,35);  ������� ������� �� ������ GG Radio1
	20,  //[57] measure_vol_min(analog_gg_radio2,40248,248,35);  ������� ������� �� ������ GG Radio2 
	20,  //[58] measure_vol_min(analog_gg_radio2,40248,248,35);  ������� ������� �� ������ GG Radio2 
  // ------------- MAX ---------------------------------------------
	0,   //[59]
	0,   //[60]
	0,   //[61]
	0,   //[62]
	75,  //[63] min measure_vol_max(analog_LineL,    40227,227,200); ������� ������� �� ������ LineL
	95,  //[64] max measure_vol_max(analog_LineL,    40227,227,200); ������� ������� �� ������ LineL
	0,   //[65]
	0,   //[66]
	45,  //[67] min  measure_vol_max(analog_mag_radio
	65,  //[68] max  measure_vol_max(analog_mag_radio
	45,  //[69] min  measure_vol_max(analog_mag_phone,40229,229,200); ������� ������� �� ������ mag phone
	65,  //[70] max  measure_vol_max(analog_mag_phone,40229,229,200); ������� ������� �� ������ mag phone
	0,   //[71]
	0,   //[72]
	0,   //[73]
	0,   //[74]
	0,   //[75]
	0,   //[76]

//++++++++++++++++  Test MTT ++++++++++++++++++++++++++++
	173, //[77] resistor(1, 200);  ���������� ������� ������� 60 ��
	173, //[78] resistor(2, 200);  ���������� ������� ������� 60 ��

	8,   //[79]
	8,   //[80]
	8,   //[81]
	8,   //[82]
	8,   //[83]
	8,   //[84]
	8,   //[85]
	8,   //[86]
	8,   //[87]
	8,   //[88]
	8,   //[89]
	8,   //[90]
	20,  //[91]
	20,  //[92]
	20,  //[93]
	20,  //[94]
	20,  //[95]
	20,  //[96]

	0,   //[97] min FrontL 
	0,   //[98] max FrontL 
	0,   //[99] min FrontR 
	0,   //[100] max FrontR 
	0,   //[110] min measure_vol_max(analog_LineL,     40260,260,70);  "Test MTT ** Signal LineL 
	0,   //[102] max measure_vol_max(analog_LineL,     40260,260,70);  "Test MTT ** Signal LineL 
	75,  //[103] min measure_vol_max(analog_LineR,     40261,261,70);  "Test MTT ** Signal LineR 
	95,  //[104] max measure_vol_max(analog_LineR,     40261,261,70);  "Test MTT ** Signal LineR 
	0,   //[105]
	0,   //[106]
	45,  //[107]
	75,  //[108]
	500, //[109] 
	630, //[110]
	0,   //[111]
	0,   //[112]
	0,   //[113]
	0,   //[114]
//++++++++++++++++  Test Microphone ++++++++++++++++++++++++++++
	16, //[115] 
	16, //[116]
	 
	8,   //[117]
	8,   //[118]
	8,   //[119]
	8,   //[120]
	8,   //[121]
	8,   //[122]
	8,   //[123]
	8,   //[124]
	8,   //[125]
	8,   //[126]
	8,   //[127]
	8,   //[128]
	20,  //[129]
	20,  //[130]
	20,  //[131]
	20,  //[132]
	20,  //[133]
	20,  //[134]
	 
	0,   //[135]
	0,   //[136]
	0,   //[137]
	0,   //[138]
	75, //[139] 
	95, //[140]
	0,   //[141]
	0,   //[142]
	45,  //[143] min  measure_vol_max(analog_mag_radio
	75,  //[144] max  measure_vol_max(analog_mag_radio
	45,  //[145]
	75,  //[146]
	0,   //[147]
	0,   //[148]
	0,   //[149]
	0,   //[150]
	0,   //[151]
	0,   //[152]
//++++++++++++++++  Test GGS  ++++++++++++++++++++++++++++
	250,
	250,

	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	20,
	20,
	20,
	20,
	20,
	20,

	65,
	100,
	65,
	100,
	0,
	0,
	0,
	0,
	110,
	135,
	110,
	135,
	500,
	650,
	0,
	0,
	0,
	0,
//++++++++++++++++  Test Radio1  ++++++++++++++++++++++++++++
	160,
	160,

	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	20,
	20,
	20,
	20,
	20,
	20,

	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	55,
	75,
	0,
	0,
	0,
	0,
	480,
	550,
	0,
	0,
//++++++++++++++++  Test Radio2  ++++++++++++++++++++++++++++
	160,
	160,

	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	8,
	20,
	20,
	20,
	20,
	20,
	20,

	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	55,
	75,
	0,
	0,
	0,
	0,
	0,
	0,
	480,
	550
};

/*

const byte  porog_MTT[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++++ Test MTT ++++++++++++++++++++++++++++++++++++++++++++++++++++++++

15,			// 2                          // measure_vol_min(analog_FrontL,    40250,250,35); ������� ������� �� ������ FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,    40251,251,35); ������� ������� �� ������ FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40252,252,35); ������� ������� �� ������ LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40253,253,35); ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40254,254,35); ������� ������� �� ������ mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40255,255,35); ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40256,256,35); ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40257,257,35); ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40258,258,35); ������� ������� �� ������ GG Radio2 
			//++++++++++++++++++++++++++++++++++ ������ ����� �� ���� ��� ++++++++++++++++++++++++++++++++++++++++++++
15,			// 11                         // measure_vol_min(analog_FrontL,    40250,250,35); ������� ������� �� ������ FrontL
15,			// 12                         // measure_vol_min(analog_FrontR,    40251,251,35); ������� ������� �� ������ FrontR
15,			// 13                         // measure_vol_min(analog_mag_radio, 40254,254,35); ������� ������� �� ������ mag radio
15,			// 14                         // measure_vol_min(analog_ggs,       40256,256,35); ������� ������� �� ������ GGS
15,			// 15                         // measure_vol_min(analog_gg_radio1, 40257,257,35); ������� ������� �� ������ GG Radio1
15,			// 16                         // measure_vol_min(analog_gg_radio2, 40258,258,35); ������� ������� �� ������ GG Radio2 
70,	    	// 17                         // measure_vol_max(analog_LineL,     40260,260,35);  "Test MTT ** Signal LineL 
55,	        // 18                         // measure_vol_max(analog_mag_phone, 40262,262, + 18)); 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);  // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_ggs,       40259,259,  + 20));         //  �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS             On 
};




const byte  porog_Microphone[]    PROGMEM = {
		//+++++++++++++++++++++++++++++++++ Test Microphone +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
150,		// 0                          // resistor(1, 200);  ���������� ������� ������� 60 ��
150,		// 1                          // resistor(2, 200);  ���������� ������� ������� 60 ��
15,			// 2                          // measure_vol_min(analog_FrontL,    40320,320,35); ������� ������� �� ������ FrontL
15,			// 3                          // measure_vol_min(analog_FrontR,    40321,321,35); ������� ������� �� ������ FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40322,322,35); ������� ������� �� ������ LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40323,323,35); ������� ������� �� ������ LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40324,324,35); ������� ������� �� ������ mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40325,325,35); ������� ������� �� ������ mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40326,326,35); ������� ������� �� ������ GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40327,327,35); ������� ������� �� ������ GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40328,328,35); ������� ������� �� ������ GG Radio2
			//++++++++++++++++++++++++++++++++++ ������ ����� �� ���� ��������� ++++++++++++++++++++++++
60,	    	// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);������� ������� �� ������ mag phone
75,		    // 12                         // measure_vol_max(analog_LineL,     40299,299,180);������� ������� �� ������ "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); ������� ������� �� ������ FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); ������� ������� �� ������ FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); ������� ������� �� ������ LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); ������� ������� �� ������ mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); ������� ������� �� ������ GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); ������� ������� �� ������ GG Radio1
15          // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); ������� ������� �� ������ GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
230,		//  0                         // resistor(1, 200); ���������� ������� ������� xx ��
230,		//  1                         // resistor(2, 200); ���������� ������� ������� xx ��
				//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); ������� ������� �� ������ "Test GGS ** Signal FrontL 
15, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); ������� ������� �� ������ "Test GGS ** Signal FrontR        
15, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL 
15, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR 
15, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); ������� ������� �� ������ "Test GGS ** Signal mag radio
15, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); ������� ������� �� ������ "Test GGS ** Signal mag phone 
15, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); ������� ������� �� ������ "Test GGS ** Signal GGS    
15, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1   
15, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ ������ ����� �� ���� GGS ++++++++++++++++++++++++
110,    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
130,	   	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";
15,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
15,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
16,			// 15	                 	  //measure_vol_max(analog_mag_radio, 40332,332,35); ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
16,	    	// 16						  //measure_vol_max(analog_mag_phone, 40292,292,50); ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
130,	    // 17			     		  //measure_vol_max(analog_ggs,       40286,289,160); ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
15,			// 18						  //measure_vol_min(analog_gg_radio1, 40287,287,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
15			// 19						  //measure_vol_min(analog_gg_radio2, 40288,288,35); ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
150,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
15,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio1 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40330,330,35); ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
220,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
15			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  ���������� ������� ������� xx ��
150,		// 1							//resistor(2, 250);  ���������� ������� ������� xx ��
				 //+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
15,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ ������ ����� �� ����  Radio2 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40331,331,35); ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
200			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);������� ������� �� ������ "Test Radio2 ** Signal GG Radio2        
};


*/


//---------------------------������ ���������   ---------------------------------------------------
const char  txt_message0[]    PROGMEM            = "Error! - ";                           
const char  txt_message1[]    PROGMEM            = "Pass";      
const char  txt_message2[]    PROGMEM            = " ****** Test sensor OFF start! ******";    
const char  txt_message3[]    PROGMEM            = " ****** Test sensor ON  start! ******";      
const char  txt_message4[]    PROGMEM            = "Signal headset instructor microphone 30mv   ON"            ;   
const char  txt_message5[]    PROGMEM            = "Microphone headset instructor signal        ON"            ;  
const char  txt_message6[]    PROGMEM            = "Command sensor OFF headset instructor 2     send!"   ;    
const char  txt_message7[]    PROGMEM            = "Command sensor OFF headset instructor       send!"   ;    
const char  txt_message8[]    PROGMEM            = "Command PTT    OFF headset instructor       send!"   ;    
const char  txt_message9[]    PROGMEM            = "Command sensor OFF microphone               send!"   ;    
const char  txt_message10[]   PROGMEM            = "Command sensor ON  headset instructor 2     send!"   ;     
const char  txt_message11[]   PROGMEM            = "Command sensor ON  headset instructor       send!"   ;     
const char  txt_message12[]   PROGMEM            = "Command  ON  PTT headset instructor (CTS)   send!"   ;  
const char  txt_message13[]   PROGMEM            = "Signal headset dispatcher microphone 30 mV  ON"            ;   
const char  txt_message14[]   PROGMEM            = "Microphone headset dispatcher signal        ON"            ;     
const char  txt_message15[]   PROGMEM            = "Command sensor OFF headset dispatcher 2     send!"   ;    
const char  txt_message16[]   PROGMEM            = "Command sensor OFF headset dispatcher       send!"   ;   
const char  txt_message17[]   PROGMEM            = "Command PTT    OFF headset dispatcher       send!"   ;    
const char  txt_message18[]   PROGMEM            = "Command sensor OFF microphone               send!"   ;   
const char  txt_message19[]   PROGMEM            = "Command sensor ON  headset dispatcher 2     send!"   ;   
const char  txt_message20[]   PROGMEM            = "Command sensor ON  headset dispatcher       send!"   ;    
const char  txt_message21[]   PROGMEM            = "Command  ON PTT headset dispatcher (CTS)    send!"   ;  
const char  txt_message22[]   PROGMEM            = " ****** Test headset instructor start! ******"               ; 
const char  txt_message23[]   PROGMEM            = " ****** Test headset dispatcher start! ******"               ;
const char  txt_message24[]   PROGMEM            = " ****** Test MTT start! ******"                              ;  
const char  txt_message25[]   PROGMEM            = " ****** Test tangenta nognaja start!  ********"               ;  
const char  txt_message26[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ********"              ; 
const char  txt_message27[]   PROGMEM            = "Command sensor OFF MTT                      send!"  ;
const char  txt_message28[]   PROGMEM            = "Command PTT    OFF MTT                      send!"  ;
const char  txt_message29[]   PROGMEM            = "Command HangUp OFF MTT -> DOWN              send!"  ;
const char  txt_message30[]   PROGMEM            = "Command sensor ON  MTT                      send!"   ;
const char  txt_message31[]   PROGMEM            = "Command PTT    ON  MTT                      send!"   ;
const char  txt_message32[]   PROGMEM            = "Command HangUp ON  MTT -> UP                send!"   ;
const char  txt_message33[]   PROGMEM            = "Signal MTT microphone 30 mV                 ON"            ;
const char  txt_message34[]   PROGMEM            = "Microphone MTT signal                       ON"            ;  
const char  txt_message35[]   PROGMEM            = "Signal FrontL, FrontR                       ON "           ;
const char  txt_message36[]   PROGMEM            = " ****** Test tangenta ruchnaja start! ******"                ;
const char  txt_message37[]   PROGMEM            = "Command sensor OFF tangenta ruchnaja        send!"   ;
const char  txt_message38[]   PROGMEM            = "Command PTT1   OFF tangenta ruchnaja        send!"   ;
const char  txt_message39[]   PROGMEM            = "Command PTT2   OFF tangenta ruchnaja        send!"   ; 
const char  txt_message40[]   PROGMEM            = "Command sensor ON  tangenta ruchnaja        send!"   ;
const char  txt_message41[]   PROGMEM            = "Command PTT1   ON  tangenta ruchnaja        send!"   ;
const char  txt_message42[]   PROGMEM            = "Command PTT2   ON  tangenta ruchnaja        send!"   ;
const char  txt_message43[]   PROGMEM            = " ****** Test tangenta nognaja start! ******"                 ;
const char  txt_message44[]   PROGMEM            = "Command sensor OFF tangenta nognaja         send!"   ;
const char  txt_message45[]   PROGMEM            = "Command PTT    OFF tangenta nognaja         send!"   ;
const char  txt_message46[]   PROGMEM            = "Command sensor ON  tangenta nognaja         send!"   ;
const char  txt_message47[]   PROGMEM            = "Command PTT    ON  tangenta nognaja         send!"   ;
const char  txt_message48[]   PROGMEM            = " ****** Test GGS start! ******"      ;
const char  txt_message49[]   PROGMEM            = "Signal GGS  FrontL, FrontR   0,7V           ON"            ;
const char  txt_message50[]   PROGMEM            = " ****** Test Radio1 start! ******"                           ;
const char  txt_message51[]   PROGMEM            = "Signal Radio1 300 mV    LFE                 ON"            ;
const char  txt_message52[]   PROGMEM            = " ****** Test Radio2 start! ******"      ;
const char  txt_message53[]   PROGMEM            = "Signal Radio1 300 mV    Center              ON"            ;
const char  txt_message54[]   PROGMEM            = " ****** Test microphone start! ******"                       ;
const char  txt_message55[]   PROGMEM            = "Signal microphone 30  mV                    ON"            ;
const char  txt_message56[]   PROGMEM            = "Command PTT    OFF microphone               send!"   ;
const char  txt_message57[]   PROGMEM            = "Command PTT    ON  microphone               send!"   ;
const char  txt_message58[]   PROGMEM            = "Command sensor OFF microphone               send!"   ;  
const char  txt_message59[]   PROGMEM            = "Command sensor ON  microphone               send!"   ;
const char  txt_message60[]   PROGMEM            = "Power amperage                               mA - "   ;
const char  txt_message61[]   PROGMEM            = "Power module Audio 2                            - "   ;
const char  txt_message62[]   PROGMEM            = "Power amperage                               mA - "   ;
const char  txt_message63[]   PROGMEM            = "Power Radio1                                    - "   ;
const char  txt_message64[]   PROGMEM            = "Power Radio2                                    - "   ;
const char  txt_message65[]   PROGMEM            = "Power GGS                                       - "   ;
const char  txt_message66[]   PROGMEM            = "Power Led microphone                            - "   ;
const char  txt_message67[]   PROGMEM            = " ****** Test power start! ******"                            ;
const char  txt_message68[]   PROGMEM            = " ****** Test Adjusting the brightness of the display! ******"; 
const char  txt_message69[]   PROGMEM            = "Adjusting the brightness code              - "   ;
const char  txt_message70[]   PROGMEM            = "Adjusting the brightness mks               - "   ;
const char  txt_message71[]   PROGMEM            = "Signal GGS  FrontL, FrontR   0,7V           OFF"            ;
const char  txt_message72[]   PROGMEM            = "Command radioperedacha  ON "    ;
const char  txt_message73[]   PROGMEM            = "Command GGS mute  ON "  ;

// ������ ������
const char  txt_error0[]   PROGMEM               = "Sensor MTT                  XP1- 19 HaSs    OFF - ";
const char  txt_error1[]   PROGMEM               = "Sensor tangenta ruchnaja    XP7 - 2         OFF - ";
const char  txt_error2[]   PROGMEM               = "Sensor tangenta nognaja     XP8 - 2         OFF - "; 
const char  txt_error3[]   PROGMEM               = "Sensor headset instructor 2 XP1 - 16 HeS2Rs OFF - ";
const char  txt_error4[]   PROGMEM               = "Sensor headset instructor   XP1 - 13 HeS2Ls OFF - "; 
const char  txt_error5[]   PROGMEM               = "Sensor headset dispatcher 2 XP1 - 5  HeS1Rs OFF - "; 
const char  txt_error6[]   PROGMEM               = "Sensor headset dispatcher   XP1 - 1  HeS1Ls OFF - ";
const char  txt_error7[]   PROGMEM               = "Sensor microphone           XS1 - 6         OFF - "; 
const char  txt_error8[]   PROGMEM               = "Microphone headset instr.   XP1 - 12 HeS2e  OFF - "; 
const char  txt_error9[]   PROGMEM               = "Microphone headset dispat.  XP1 - 12 HeS2e  OFF - ";  
const char  txt_error10[]  PROGMEM               = "Sensor MTT                  XP1 - 19 HaSs   ON~ - "; 
const char  txt_error11[]  PROGMEM               = "Sensor tangenta ruchnaja    XP7 - 2         ON~ - ";
const char  txt_error12[]  PROGMEM               = "Sensor tangenta nognaja     XP8 - 2         ON~ - "; 
const char  txt_error13[]  PROGMEM               = "Sensor headset instructor 2 XP1 - 16 HeS2Rs ON~ - ";
const char  txt_error14[]  PROGMEM               = "Sensor headset instructor   XP1 - 13 HeS2Ls ON~ - "; 
const char  txt_error15[]  PROGMEM               = "Sensor headset dispatcher 2 XP1 - 5  HeS1Rs ON~ - "; 
const char  txt_error16[]  PROGMEM               = "Sensor headset dispatcher   XP1 - 1  HeS1Ls ON~ - ";
const char  txt_error17[]  PROGMEM               = "Sensor microphone           XS1 - 6         ON~ - "; 
const char  txt_error18[]  PROGMEM               = "Microphone headset instr.   XP1 - 12 HeS2e  ON~ - "; 
const char  txt_error19[]  PROGMEM               = "Microphone headset dispat.  XP1 - 10 HeS1e  ON~ - "; 
const char  txt_error20[]  PROGMEM               = "Command PTT headset instructor (CTS)        OFF - ";
const char  txt_error21[]  PROGMEM               = "Command PTT headset instructor (CTS)        ON~ - ";
const char  txt_error22[]  PROGMEM               = "Command PTT headset dispatcher (CTS)        OFF - ";
const char  txt_error23[]  PROGMEM               = "Command PTT headset dispatcher (CTS)        ON~ - ";
const char  txt_error24[]  PROGMEM               = "Test headset instructor ** Signal LineL     ON~ - ";
const char  txt_error25[]  PROGMEM               = "Test headset instructor ** Signal LineR     ON~ - ";   
const char  txt_error26[]  PROGMEM               = "Test headset instructor ** Signal Mag phone ON~ - ";
const char  txt_error27[]  PROGMEM               = "Test headset dispatcher ** Signal LineL     ON~ - ";
const char  txt_error28[]  PROGMEM               = "Test headset dispatcher ** Signal LineR     ON~ - ";  
const char  txt_error29[]  PROGMEM               = "Test headset dispatcher ** Signal Mag phone ON~ - ";
const char  txt_error30[]  PROGMEM               = "Test headset instructor ** Signal FrontL    OFF - ";
const char  txt_error31[]  PROGMEM               = "Test headset instructor ** Signal FrontR    OFF - ";
const char  txt_error32[]  PROGMEM               = "Test headset instructor ** Signal LineL     OFF - ";
const char  txt_error33[]  PROGMEM               = "Test headset instructor ** Signal LineR     OFF - ";
const char  txt_error34[]  PROGMEM               = "Test headset instructor ** Signal mag radio OFF - ";
const char  txt_error35[]  PROGMEM               = "Test headset instructor ** Signal mag phone OFF - ";
const char  txt_error36[]  PROGMEM               = "Test headset instructor ** Signal GGS       OFF - ";
const char  txt_error37[]  PROGMEM               = "Test headset instructor ** Signal GG Radio1 OFF - ";
const char  txt_error38[]  PROGMEM               = "Test headset instructor ** Signal GG Radio2 OFF - ";
const char  txt_error39[]  PROGMEM               = "";
const char  txt_error40[]  PROGMEM               = "Test headset dispatcher ** Signal FrontL    OFF - ";
const char  txt_error41[]  PROGMEM               = "Test headset dispatcher ** Signal FrontR    OFF - ";
const char  txt_error42[]  PROGMEM               = "Test headset dispatcher ** Signal LineL     OFF - ";
const char  txt_error43[]  PROGMEM               = "Test headset dispatcher ** Signal LineR     OFF - ";
const char  txt_error44[]  PROGMEM               = "Test headset dispatcher ** Signal mag radio OFF - ";
const char  txt_error45[]  PROGMEM               = "Test headset dispatcher ** Signal mag phone OFF - ";
const char  txt_error46[]  PROGMEM               = "Test headset dispatcher ** Signal GGS       OFF - ";
const char  txt_error47[]  PROGMEM               = "Test headset dispatcher ** Signal GG Radio1 OFF - ";
const char  txt_error48[]  PROGMEM               = "Test headset dispatcher ** Signal GG Radio2 OFF - ";
const char  txt_error49[]  PROGMEM               = "";
const char  txt_error50[]  PROGMEM               = "Test MTT ** Signal FrontL                   OFF - ";
const char  txt_error51[]  PROGMEM               = "Test MTT ** Signal FrontR                   OFF - ";
const char  txt_error52[]  PROGMEM               = "Test MTT ** Signal LineL                    OFF - ";
const char  txt_error53[]  PROGMEM               = "Test MTT ** Signal LineR                    OFF - ";
const char  txt_error54[]  PROGMEM               = "Test MTT ** Signal mag radio                OFF - ";
const char  txt_error55[]  PROGMEM               = "Test MTT ** Signal mag phone                OFF - ";
const char  txt_error56[]  PROGMEM               = "Test MTT ** Signal GGS                      OFF - ";
const char  txt_error57[]  PROGMEM               = "Test MTT ** Signal GG Radio1                OFF - ";
const char  txt_error58[]  PROGMEM               = "Test MTT ** Signal GG Radio2                OFF - ";
const char  txt_error59[]  PROGMEM               = "Test MTT ** Signal GGS                      ON~ - ";
const char  txt_error60[]  PROGMEM               = "Test MTT ** Signal LineL                    ON~ - ";
const char  txt_error61[]  PROGMEM               = "Test MTT ** Signal LineR                    ON~ - ";  
const char  txt_error62[]  PROGMEM               = "Test MTT ** Signal Mag phone                ON~ - ";
const char  txt_error63[]  PROGMEM               = "Test MTT PTT    (CTS)                       OFF - ";
const char  txt_error64[]  PROGMEM               = "Test microphone PTT  (CTS)                  OFF - ";
const char  txt_error65[]  PROGMEM               = "Test MTT PTT    (CTS)                       ON~ - ";
const char  txt_error66[]  PROGMEM               = "Test microphone PTT  (CTS)                  ON~ - ";
const char  txt_error67[]  PROGMEM               = "Test MTT HangUp (DCD)                       OFF - ";
const char  txt_error68[]  PROGMEM               = "Test MTT HangUp (DCD)                       ON~ - ";
const char  txt_error69[]  PROGMEM               = "";
const char  txt_error70[]  PROGMEM               = "Command PTT1 tangenta ruchnaja (CTS)        OFF - ";
const char  txt_error71[]  PROGMEM               = "Command PTT2 tangenta ruchnaja (DCR)        OFF - ";
const char  txt_error72[]  PROGMEM               = "Command PTT1 tangenta ruchnaja (CTS)        ON~ - ";
const char  txt_error73[]  PROGMEM               = "Command PTT2 tangenta ruchnaja (DCR)        ON~ - ";
const char  txt_error74[]  PROGMEM               = "Command sensor tangenta ruchnaja    XP7 - 2 OFF - ";
const char  txt_error75[]  PROGMEM               = "Command sensor tangenta ruchnaja    XP7 - 2 ON~ - ";
const char  txt_error76[]  PROGMEM               = "Command sensor tangenta nognaja     XP8 - 2 OFF - ";
const char  txt_error77[]  PROGMEM               = "Command sensor tangenta nognaja     XP8 - 2 ON~ - ";
const char  txt_error78[]  PROGMEM               = "Command PTT tangenta nognaja (CTS)  XP8 - 1 OFF - ";
const char  txt_error79[]  PROGMEM               = "Command PTT tangenta nognaja (CTS)  XP8 - 1 ON~ - ";
const char  txt_error80[]  PROGMEM               = "Test GGS ** Signal FrontL                   OFF - ";
const char  txt_error81[]  PROGMEM               = "Test GGS ** Signal FrontR                   OFF - ";
const char  txt_error82[]  PROGMEM               = "Test GGS ** Signal LineL                    OFF - ";
const char  txt_error83[]  PROGMEM               = "Test GGS ** Signal LineR                    OFF - ";
const char  txt_error84[]  PROGMEM               = "Test GGS ** Signal mag radio                OFF - ";
const char  txt_error85[]  PROGMEM               = "Test GGS ** Signal mag phone                OFF - ";
const char  txt_error86[]  PROGMEM               = "Test GGS ** Signal GGS                      OFF - ";
const char  txt_error87[]  PROGMEM               = "Test GGS ** Signal GG Radio1                OFF - ";
const char  txt_error88[]  PROGMEM               = "Test GGS ** Signal GG Radio2                OFF - ";
const char  txt_error89[]  PROGMEM               = "Test GGS ** Signal GGS                      ON~ - ";
const char  txt_error90[]  PROGMEM               = "Test GGS ** Signal FrontL                   ON~ - ";
const char  txt_error91[]  PROGMEM               = "Test GGS ** Signal FrontR                   ON~ - ";
const char  txt_error92[]  PROGMEM               = "Test GGS ** Signal mag phone                ON~ - ";
const char  txt_error93[]  PROGMEM               = "Error power 12 Volt"                               ; // ������ ADC1 ���������� 12/3 �����"
const char  txt_error94[]  PROGMEM               = "Error power 12 Volt Radio1"                        ; // ������ ADC14 ���������� 12/3 ����� Radio1
const char  txt_error95[]  PROGMEM               = "Error power 12 Volt Radio2"                        ; // O����� ADC14 ���������� 12/3 ����� Radio2
const char  txt_error96[]  PROGMEM               = "Error power 12 Volt GGS"                           ; // O����� ADC14 ���������� 12/3 ����� ���
const char  txt_error97[]  PROGMEM               = "Error power 3,6 Volt Led Mic"                      ; // O����� ADC15 ���������� ���������� 3,6 ������
const char  txt_error98[]  PROGMEM               = "Test Microphone ** Signal mag phone         ON~ - ";  
const char  txt_error99[]  PROGMEM               = "Test Microphone ** Signal LineL             ON~ - "; 
const char  txt_error100[]  PROGMEM              = "Test Radio1 ** Signal FrontL                OFF - ";
const char  txt_error101[]  PROGMEM              = "Test Radio1 ** Signal FrontR                OFF - ";
const char  txt_error102[]  PROGMEM              = "Test Radio1 ** Signal LineL                 OFF - ";
const char  txt_error103[]  PROGMEM              = "Test Radio1 ** Signal LineR                 OFF - ";
const char  txt_error104[]  PROGMEM              = "Test Radio1 ** Signal mag radio             OFF - ";
const char  txt_error105[]  PROGMEM              = "Test Radio1 ** Signal mag phone             OFF - ";
const char  txt_error106[]  PROGMEM              = "Test Radio1 ** Signal GGS                   OFF - ";
const char  txt_error107[]  PROGMEM              = "Test Radio1 ** Signal GG Radio1             OFF - ";
const char  txt_error108[]  PROGMEM              = "Test Radio1 ** Signal GG Radio2             OFF - ";
const char  txt_error109[]  PROGMEM              = "Test Radio1 ** Signal Radio1                ON~ - ";
const char  txt_error110[]  PROGMEM              = "Test Radio2 ** Signal FrontL                OFF - ";
const char  txt_error111[]  PROGMEM              = "Test Radio2 ** Signal FrontR                OFF - ";
const char  txt_error112[]  PROGMEM              = "Test Radio2 ** Signal LineL                 OFF - ";
const char  txt_error113[]  PROGMEM              = "Test Radio2 ** Signal LineR                 OFF - ";
const char  txt_error114[]  PROGMEM              = "Test Radio2 ** Signal mag radio             OFF - ";
const char  txt_error115[]  PROGMEM              = "Test Radio2 ** Signal mag phone             OFF - ";
const char  txt_error116[]  PROGMEM              = "Test Radio2 ** Signal GGS                   OFF - ";
const char  txt_error117[]  PROGMEM              = "Test Radio2 ** Signal GG Radio1             OFF - ";
const char  txt_error118[]  PROGMEM              = "Test Radio2 ** Signal GG Radio2             OFF - ";
const char  txt_error119[]  PROGMEM              = "Test Radio2 ** Signal Radio2                ON~ - ";
const char  txt_error120[]  PROGMEM              = "Test Microphone ** Signal FrontL            OFF - ";
const char  txt_error121[]  PROGMEM              = "Test Microphone ** Signal FrontR            OFF - ";
const char  txt_error122[]  PROGMEM              = "Test Microphone ** Signal LineL             OFF - ";
const char  txt_error123[]  PROGMEM              = "Test Microphone ** Signal LineR             OFF - ";
const char  txt_error124[]  PROGMEM              = "Test Microphone ** Signal mag radio         OFF - ";
const char  txt_error125[]  PROGMEM              = "Test Microphone ** Signal mag phone         OFF - ";
const char  txt_error126[]  PROGMEM              = "Test Microphone ** Signal GGS               OFF - ";
const char  txt_error127[]  PROGMEM              = "Test Microphone ** Signal GG Radio1         OFF - ";
const char  txt_error128[]  PROGMEM              = "Test Microphone ** Signal GG Radio2         OFF - ";
const char  txt_error129[]  PROGMEM              = "";
const char  txt_error130[]  PROGMEM              = "Test Radio1 ** Signal mag radio             ON~ - ";
const char  txt_error131[]  PROGMEM              = "Test Radio2 ** Signal mag radio             ON~ - ";
const char  txt_error132[]  PROGMEM              = "Test GGS ** Signal mag radio                ON~ - ";
const char  txt_error133[]  PROGMEM              = "Test headset instructor ** Signal mag radio ON~ - ";
const char  txt_error134[]  PROGMEM              = "Test headset dispatcher ** Signal mag radio ON~ - ";
const char  txt_error135[]  PROGMEM              = "Test MTT ** Signal mag radio                ON~ - ";
const char  txt_error136[]  PROGMEM              = "Test Microphone ** Signal mag radio         ON~ - ";



char buffer[140];  

const char* const table_message[] PROGMEM = 
{
txt_message0,                                 // "    Error! - ";                           
txt_message1,                                 // "Pass";      
txt_message2,                                 // " ****** Test sensor OFF start! ******";    
txt_message3,                                 // " ****** Test sensor ON  start! ******";      
txt_message4,                                 // "Signal headset instructor microphone 30mv     ON"            ;   
txt_message5,                                 // "Microphone headset instructor signal          ON"            ;  
txt_message6,                                 // "Command sensor OFF headset instructor 2          send!"      ; 
txt_message7,                                 // "Command sensor OFF headset instructor            send!"      ;  
txt_message8,                                 // "Command PTT    OFF headset instructor            send!"      ;    
txt_message9,                                 // "Command sensor OFF microphone                    send!"      ;  
txt_message10,                                // "Command sensor ON  headset instructor 2          send!"      ;    
txt_message11,                                // "Command sensor ON  headset instructor            send!"      ;   
txt_message12,                                // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
txt_message13,                                // "Signal headset dispatcher microphone 30mv     ON"            ;   
txt_message14,                                // "Microphone headset dispatcher signal          ON"            ;     
txt_message15,                                // "Command sensor OFF headset dispatcher 2          send!"      ;  
txt_message16,                                // "Command sensor OFF headset dispatcher            send!"      ;  
txt_message17,                                // "Command PTT    OFF headset dispatcher            send!"      ;    
txt_message18,                                // "Command sensor OFF microphone                    send!"      ;   
txt_message19,                                // "Command sensor ON  headset dispatcher 2          send!"      ;  
txt_message20,                                // "Command sensor ON  headset dispatcher            send!"      ;   
txt_message21,                                // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
txt_message22,                                // " ****** Test headset instructor start! ******"               ; 
txt_message23,                                // " ****** Test headset dispatcher start! ******"               ;
txt_message24,                                // " ****** Test MTT start! ******"                              ;  
txt_message25,                                // " ****** Test tangenta nognaja start! ********"               ;  
txt_message26,                                // " ****** Test tangenta ruchnaja start! ********"              ; 
txt_message27,                                // "Command sensor OFF MTT                           send! "     ;
txt_message28,                                // "Command PTT    OFF MTT                           send! "     ;
txt_message29,                                // "Command HangUp OFF MTT                           send! "     ;
txt_message30,                                // "Command sensor ON  MTT                           send!"      ;
txt_message31,                                // "Command PTT    ON  MTT                           send!"      ;
txt_message32,                                // "Command HangUp ON  MTT                           send!"      ;
txt_message33,                                // "Signal MTT microphone 30mv                    ON"            ;
txt_message34,                                // "Microphone MTT signal                         ON"            ;  
txt_message35,                                // "Signal FrontL, FrontR                         ON "           ;
txt_message36,                                // " ****** Test tangenta ruchnaja start! ******"                ;
txt_message37,                                // "Command sensor OFF tangenta ruchnaja             send!"      ;
txt_message38,                                // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
txt_message39,                                // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
txt_message40,                                // "Command sensor ON  tangenta ruchnaja             send!"      ;
txt_message41,                                // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
txt_message42,                                // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
txt_message43,                                // " ****** Test tangenta nognaja start! ******"                 ;
txt_message44,                                // "Command sensor OFF tangenta nognaja              send!"      ;
txt_message45,                                // "Command PTT    OFF tangenta nognaja              send!"      ;
txt_message46,                                // "Command sensor ON  tangenta nognaja              send!"      ;
txt_message47,                                // "Command PTT    ON  tangenta nognaja              send!"      ;
txt_message48,                                // " ****** Test GGS start! ******"      ;
txt_message49,                                // "Signal GGS  FrontL, FrontR   0,7v             ON"            ;

txt_message50,                                // " ****** Test Radio1 start! ******"      ;
txt_message51,                                // "Signal Radio1 200 mV    LFE                   ON"            ;
txt_message52,                                //" ****** Test Radio2 start! ******"      ;
txt_message53,                                // "Signal Radio1 300 mV    Center                ON"            ;
txt_message54,                                // " ****** Test mi�rophone start! ******"                       ;
txt_message55,                                // "Signal mi�rophone 30  mV                      ON"            ;
txt_message56,                                // "Command PTT    OFF microphone                    send!"      ;
txt_message57,                                // "Command PTT    ON  microphone                    send!"      ;
txt_message58,                                // "Command sensor OFF microphone                    send!"      ;  
txt_message59,                                // "Command sensor ON  microphone                    send!"      ;

txt_message60,                                // "Power amperage mA - "                                        ;
txt_message61,                                // "Power Kamerton V  - "                                        ;
txt_message62,                                // "Power amperage mA - "                                        ;
txt_message63,                                // "Power Radio1 V    - "                                        ;
txt_message64,                                // "Power Radio2 V    - "                                        ;
txt_message65,                                // "Power GGS    V    - "                                        ;
txt_message66,                                // "Power Led mic.V   - "                                        ;
txt_message67,                                // " ****** Test power start! ******"                            ;
txt_message68,                                // " ****** Test Adjusting the brightness of the display! ******"; 
txt_message69,                                // "Adjusting the brightness code                              - "   ;

txt_message70,                                // "Adjusting the brightness mks                               - "   ;
txt_message71,                                // "Signal GGS  FrontL, FrontR   0,7V             OFF"            ;
txt_message72,                                // "Command radioperedacha  ON "    ;
txt_message73                                 // "Command GGS mute  ON "  ;
};


const char* const string_table_err[] PROGMEM = 
{
txt_error0,                                   // "Sensor MTT                          XP1- 19 HaSs            OFF - ";
txt_error1,                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
txt_error2,                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
txt_error3,                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
txt_error4,                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
txt_error5,                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
txt_error6,                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
txt_error7,                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
txt_error8,                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
txt_error9,                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - "; 

txt_error10,                                  // "Sensor MTT                          XP1- 19 HaSs            ON  - ";
txt_error11,                                  // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
txt_error12,                                  // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
txt_error13,                                  // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
txt_error14,                                  // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
txt_error15,                                  // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
txt_error16,                                  // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
txt_error17,                                  // "Sensor microphone                   XS1 - 6                 ON  - "; 
txt_error18,                                  // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
txt_error19,                                  // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 

txt_error20,                                  // "Command PTT headset instructor (CTS)                        OFF - ";
txt_error21,                                  // "Command PTT headset instructor (CTS)                        ON  - ";
txt_error22,                                  // "Command PTT headset dispatcher (CTS)                        OFF - ";
txt_error23,                                  // "Command PTT headset dispatcher (CTS)                        ON  - ";
txt_error24,                                  // "Test headset instructor ** Signal LineL                     ON  - ";
txt_error25,                                  // "Test headset instructor ** Signal LineR                     ON  - ";   
txt_error26,                                  // "Test headset instructor ** Signal Mag phone                 ON  - ";
txt_error27,                                  // "Test headset dispatcher ** Signal LineL                     ON  - ";
txt_error28,                                  // "Test headset dispatcher ** Signal LineR                     ON  - ";  
txt_error29,                                  // "Test headset dispatcher ** Signal Mag phone                 ON  - ";

txt_error30,                                  // "Test headset instructor ** Signal FrontL                    OFF - ";
txt_error31,                                  // "Test headset instructor ** Signal FrontR                    OFF - ";
txt_error32,                                  // "Test headset instructor ** Signal LineL                     OFF - ";
txt_error33,                                  // "Test headset instructor ** Signal LineR                     OFF - ";
txt_error34,                                  // "Test headset instructor ** Signal mag radio                 OFF - "; 
txt_error35,                                  // "Test headset instructor ** Signal mag phone                 OFF - ";
txt_error36,                                  // "Test headset instructor ** Signal GGS                       OFF - ";
txt_error37,                                  // "Test headset instructor ** Signal GG Radio1                 OFF - ";
txt_error38,                                  // "Test headset instructor ** Signal GG Radio2                 OFF - ";
txt_error39,                                  //

txt_error40,                                  // "Test headset dispatcher ** Signal FrontL                    OFF - ";
txt_error41,                                  // "Test headset dispatcher ** Signal FrontR                    OFF - ";
txt_error42,                                  // "Test headset dispatcher ** Signal LineL                     OFF - "; 
txt_error43,                                  // "Test headset dispatcher ** Signal LineR                     OFF - ";
txt_error44,                                  // "Test headset dispatcher ** Signal mag radio                 OFF - "; 
txt_error45,                                  // "Test headset dispatcher ** Signal mag phone                 OFF - ";
txt_error46,                                  // "Test headset dispatcher ** Signal GGS                       OFF - "; 
txt_error47,                                  // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
txt_error48,                                  // "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
txt_error49,                                  //  

txt_error50,                                  // "Test MTT ** Signal FrontL                                   OFF - ";
txt_error51,                                  // "Test MTT ** Signal FrontR                                   OFF - ";
txt_error52,                                  // "Test MTT ** Signal LineL                                    OFF - ";
txt_error53,                                  // "Test MTT ** Signal LineR                                    OFF - "; 
txt_error54,                                  // "Test MTT ** Signal mag radio                                OFF - ";
txt_error55,                                  // "Test MTT ** Signal mag phone                                OFF - ";
txt_error56,                                  // "Test MTT ** Signal GGS                                      OFF - ";
txt_error57,                                  // "Test MTT ** Signal GG Radio1                                OFF - ";
txt_error58,                                  // "Test MTT ** Signal GG Radio2                                OFF - "; 
txt_error59,                                  // "Test MTT ** Signal GGS                                      ON  - ";

txt_error60,                                  // "Test MTT ** Signal LineL                                    ON  - ";
txt_error61,                                  // "Test MTT ** Signal LineR                                    ON  - ";  
txt_error62,                                  // "Test MTT ** Signal Mag phone                                ON  - ";
txt_error63,                                  // "Test MTT PTT    (CTS)                                       OFF - ";
txt_error64,                                  // "Test microphone PTT  (CTS)                                  OFF - ";
txt_error65,                                  // "Test MTT PTT    (CTS)                                       ON  - ";
txt_error66,                                  // "Test microphone PTT  (CTS)                                  ON  - ";
txt_error67,                                  // "Test MTT HangUp (DCD)                                       OFF - ";
txt_error68,                                  // "Test MTT HangUp (DCD)                                       ON  - ";
txt_error69,                                  //

txt_error70,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
txt_error71,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
txt_error72,                                  // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
txt_error73,                                  // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
txt_error74,                                  // "Command sensor tangenta ruchnaja                            OFF - ";
txt_error75,                                  // "Command sensor tangenta ruchnaja                            ON  - ";
txt_error76,                                  // "Command sensor tangenta nognaja     XP8 - 2                 OFF - ";
txt_error77,                                  // "Command sensor tangenta nognaja     XP8 - 2                 ON  - ";
txt_error78,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 OFF - ";  
txt_error79,                                  // "Command PTT tangenta nognaja (CTS)  XP8 - 1                 ON  - "; 

txt_error80,                                  // "Test GGS ** Signal FrontL                                   OFF - ";
txt_error81,                                  // "Test GGS ** Signal FrontR                                   OFF - ";
txt_error82,                                  // "Test GGS ** Signal LineL                                    OFF - ";
txt_error83,                                  // "Test GGS ** Signal LineR                                    OFF - ";
txt_error84,                                  // "Test GGS ** Signal mag radio                                OFF - ";
txt_error85,                                  // "Test GGS ** Signal mag phone                                OFF - ";
txt_error86,                                  // "Test GGS ** Signal GGS                                      OFF - ";
txt_error87,                                  // "Test GGS ** Signal GG Radio1                                OFF - ";
txt_error88,                                  // "Test GGS ** Signal GG Radio2                                OFF - ";
txt_error89,                                  // "Test GGS ** Signal GGS                                      ON  - ";

txt_error90,                                  // "Test GGS ** Signal FrontL                                   ON  - ";
txt_error91,                                  // "Test GGS ** Signal FrontR                                   ON  - ";
txt_error92,                                  // "Test GGS ** Signal mag phone                                ON  - ";
txt_error93,                                  // 
txt_error94,                                  // 
txt_error95,                                  // 
txt_error96,                                  // 
txt_error97,                                  // 
txt_error98,                                  // "Test Microphone ** Signal mag phone                         ON  - ";   
txt_error99,                                  // "Test Microphone ** Signal LineL                             ON  - "; 

txt_error100,                                 // "Test Radio1 ** Signal FrontL                                OFF - ";
txt_error101,                                 // "Test Radio1 ** Signal FrontR                                OFF - ";
txt_error102,                                 // "Test Radio1 ** Signal LineL                                 OFF - ";
txt_error103,                                 // "Test Radio1 ** Signal LineR                                 OFF - ";
txt_error104,                                 // "Test Radio1 ** Signal mag radio                             OFF - ";
txt_error105,                                 // "Test Radio1 ** Signal mag phone                             OFF - ";
txt_error106,                                 // "Test Radio1 ** Signal GGS                                   OFF - ";
txt_error107,                                 // "Test Radio1 ** Signal GG Radio1                             OFF - ";
txt_error108,                                 // "Test Radio1 ** Signal GG Radio2                             OFF - ";
txt_error109,                                 // "Test Radio1 ** Signal Radio1                                ON  - ";

txt_error110,                                 // "Test Radio2 ** Signal FrontL                                OFF - ";
txt_error111,                                 // "Test Radio2 ** Signal FrontR                                OFF - ";
txt_error112,                                 // "Test Radio2 ** Signal LineL                                 OFF - ";
txt_error113,                                 // "Test Radio2 ** Signal LineR                                 OFF - ";
txt_error114,                                 // "Test Radio2 ** Signal mag radio                             OFF - ";
txt_error115,                                 // "Test Radio2 ** Signal mag phone                             OFF - ";
txt_error116,                                 // "Test Radio2 ** Signal GGS                                   OFF - ";
txt_error117,                                 // "Test Radio2 ** Signal GG Radio1                             OFF - ";
txt_error118,                                 // "Test Radio2 ** Signal GG Radio2                             OFF - ";
txt_error119,                                 // "Test Radio2 ** Signal Radio2                                ON  - ";

txt_error120,                                 // "Test Microphone ** Signal FrontL                            OFF - ";
txt_error121,                                 // "Test Microphone ** Signal FrontR                            OFF - ";
txt_error122,                                 // "Test Microphone ** Signal LineL                             OFF - ";
txt_error123,                                 // "Test Microphone ** Signal LineR                             OFF - ";
txt_error124,                                 // "Test Microphone ** Signal mag radio                         OFF - ";
txt_error125,                                 // "Test Microphone ** Signal mag phone                         OFF - ";
txt_error126,                                 // "Test Microphone ** Signal GGS                               OFF - ";
txt_error127,                                 // "Test Microphone ** Signal GG Radio1                         OFF - ";
txt_error128,                                 // "Test Microphone ** Signal GG Radio2                         OFF - ";
txt_error129,                                 // ";
txt_error130,                                 // "Test Radio1 ** Signal mag radio                             ON  - ";
txt_error131,                                 // "Test Radio2 ** Signal mag radio                             ON  - ";
txt_error132,                                 // "Test GGS    ** Signal mag radio                             ON  - ";
txt_error133,                                 // "Test headset instructor ** Signal mag radio                 ON  - ";
txt_error134,                                 // "Test headset dispatcher ** Signal mag radio                 ON  - ";
txt_error135,                                 // "Test MTT ** Signal mag radio                                ON  - ";
txt_error136                                  // "Test Microphone ** Signal mag radio                         ON  - ";
};

// ========================= ���� �������� ============================================

void dateTime(uint16_t* date, uint16_t* time)                  // ��������� ������ ������� � ���� �����
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void serial_print_date()                           // ������ ���� � �������    
{
	  DateTime now = RTC.now();
	  Serial.print(now.day(), DEC);
	  Serial.print('/');
	  Serial.print(now.month(), DEC);
	  Serial.print('/');
	  Serial.print(now.year(), DEC);//Serial display time
	  Serial.print(' ');
	  Serial.print(now.hour(), DEC);
	  Serial.print(':');
	  Serial.print(now.minute(), DEC);
	  Serial.print(':');
	  Serial.print(now.second(), DEC);
}

const int8_t ERROR_LED_PIN = 13;



void set_time()
{
	RTC.adjust(DateTime(__DATE__, __TIME__));
//	DateTime now = RTC.now();
//	second = now.second();       //Initialization time
//	minute = now.minute();
//	hour   = now.hour();
//	day    =  now.day();
//	day++;
//	if(day > 31)day = 1;
//	month  = now.month();
//	year   = now.year();
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
//	RTC.adjust(set_time);             
}

void flash_time()                                              // ��������� ���������� ���������� 
{ 
	   // PORTB = B00000000; // ��� 12 ��������� � ��������� LOW
		prer_Kmerton_Run = true;
		prer_Kamerton();
		//sendPacketK ();  
		slave.run();
		prer_Kmerton_Run = false;
	   // PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH
}

void serialEvent3()
{

	////wdt_reset();  // ����� ����������� ������� ��� ������� ����� � ��
	while (prer_Kmerton_Run){}
	//digitalWrite(ledPin13,HIGH);
	control_command();
	////if (portFound == true)
	////{
	//slave.run(); 
	//digitalWrite(ledPin13,LOW);
	//}
}
//fileName_F
//void serialEvent2()
//{
//	//if (portFound2 == false) set_serial2();
//
//
//
//
//	/*
//	if (Serial2.available())                             // ���� ���-�� ���������? ���� ������ � ������?
//		  {
//			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
//			unsigned char buffer_count = 0;                      // ���������� � ������ ������ ������
//
//			while (Serial2.available())
//				{
//				  if (overflowFlag)                        // ���� ����� ���������� - ��������
//					 Serial2.read();
//				  else                                     // ������ ������ � �����, ������� ����������
//					{
//					if (buffer_count == BUFFER_SIZEKF)           // ��������� ������ ������
//						{
//							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
//						}
//							fileName_F[buffer_count] = Serial2.read(); 
//						buffer_count++;
//					}
//				}
//		   }
//	 else 
//		{
//	
//		}
//
////	int len = readString(buffer, sizeof(buffer), '\r');
//
//	Serial.println(fileName_F);
//	*/
////	wdt_reset();  // ����� ����������� ������� ��� ������� ����� � ��
//}
//void serialEvent1()
//{
//	/*
//	if (Serial1.available())                               // ���� ���-�� ���������? ���� ������ � ������?
//		  {
//			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
//			unsigned char buffer = 0;                      // ���������� � ������ ������ ������
//
//			while (Serial1.available())
//				{
//				  if (overflowFlag)                        // ���� ����� ���������� - ��������
//					 Serial1.read();
//				  else                                     // ������ ������ � �����, ������� ����������
//					{
//					if (bufferK == BUFFER_SIZEK)           // ��������� ������ ������
//						{
//							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
//						}
//						 regBank.set(40004+buffer,Serial1.read());
//						//regs_in[buffer] = Serial1.read(); 
//						buffer++;
//					}
//				}
////			calculateCRC_In();
//			regBank.set(124,1);                              // ����� � "��������" �����������
//		   }
//	 else 
//		{
//			Stop_Kam = 0;                                    // ���� ��������. ���. �� ���������
//			regBank.set(124,0);                              // ���� ������  ����� � "��������"
//		}
//	*/
//	
//	//digitalWrite(ledPin13,LOW);
//	//prer_Kmerton_Run = false;
//}

int readString(char *buffer, int max_len, int terminator)
{
  int pos = 0;

  while (true) {
	if (Serial2.available() > 0) 
	{
	  int readch = Serial2.read();
	  if (readch == terminator) 
	  {
		buffer[pos] = '\0';
		break;
	  }
	  else if (readch != '\n' && pos < max_len-1) 
	  {  // Ignore new-lines
		buffer[pos++] = readch;
		fileName_F[pos++] = readch;
	  }
	}
  }
  return pos;
}
/*
void read_Serial2()
{
	if (Serial2.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer_count = 0;                      // ���������� � ������ ������ ������

			while (Serial2.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial2.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (buffer_count == BUFFER_SIZEKF)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
					bufferSerial2[buffer_count] = Serial2.read(); 
					buffer_count++;
					}
				}
		   }
	 else 
		{
	
		}

}
*/
void prer_Kamerton()                                          // ���������� ����� ���������� � ������� ��������
{
//	clear_serial1();
	sendPacketK ();  
	// ��������� ���������� � ������ ��������
	waiting_for_replyK();                                  // �������� �������������
}
void sendPacketK () 
{              // ��������� �������� ������ � ��������
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
			regBank.set(40001+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // ������ ������ �� ���������
{
	delayMicroseconds(5);
	   //blink_red = !blink_red;
	   //digitalWrite(ledPin13,!digitalRead(ledPin13));
	//  �������� �������� � ���������� Stop_Kam = 0; 
	if (Serial1.available())                               // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;               // ���� ���������� ������� ������
			unsigned char buffer = 0;                      // ���������� � ������ ������ ������

			while (Serial1.available())
				{
				  if (overflowFlag)                        // ���� ����� ���������� - ��������
					 Serial1.read();
				  else                                     // ������ ������ � �����, ������� ����������
					{
					if (bufferK == BUFFER_SIZEK)           // ��������� ������ ������
						{
							overflowFlag = 1;              // ���������� ���� ���������� ������� ������
						}
						 regBank.set(40004+buffer,Serial1.read());
						//regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
//			calculateCRC_In();
			regBank.set(124,1);                              // ����� � "��������" �����������
		   }
	 else 
		{
			Stop_Kam = 0;                                    // ���� ��������. ���. �� ���������
			regBank.set(124,0);                              // ���� ������  ����� � "��������"
		}

	  //if( regBank.get(40007) != regs_temp)
	  //{
		 //Serial.println(regBank.get(40004),BIN);
		 //Serial.println(regBank.get(40005),BIN);
		 //Serial.println(regBank.get(40006),BIN);
		 //Serial.println(regBank.get(40007),BIN);
	  //}
   //   regs_temp = regBank.get(40007);

}
void Stop_Kamerton ()                  //���� �� �������� ���������� � ��������� - �������� ��������
  {
	 for (unsigned char i = 0; i <4; i++)
	 {
		 regBank.set(40004+i,0);
		// regs_in[i]=0;
	 }
  }

void calculateCRC_Out()                // ���������� ����������� ����� ������ �����
{ 
// ���� �� regs_out[0] ���������� ��� ���������,� ���������� ����������� ����� �� �����������; 

  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // ��������  1 ���� � temp1
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 ����� XXXX0000
  temp2 = temp1>>4;                    // ����������� ������� ���� � ������� 0000XXXX(������ ���� ����� � temp2)
 
  temp3 = regs_out[2];                 // �������� 2 ���� � temp3
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 2 ����� XXXX0000
  temp3 = temp3>>4;                    // ����������� ������� ���� � ������� 0000XXXX(������ ���� ����� � temp3)
  temp4 = regs_out[2];                 // �������� 2 ���� � temp4
  temp4 = temp4&0x0F;                  // �������� ����� 0F �� ������� ���� 2 ����� 0000XXXX(������ ���� ����� � temp4)
  crc =  temp2 ^  temp3 ^  temp4  ;    // ������� ����� ���� ������
  crc = crc&0x0F;                      // �������� ����� 0F �� ������� ���� crc
  regs_out[1]= temp1 | crc;            // �������� 2 ���� � ������ ����������� �����
}
void calculateCRC_In()                 // ���������� ����������� ����� ������ �����
{ 
	/*
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // ��������  
  temp1 = temp1&0xF0;                  // �������� ����� F0 �� ������� ���� 1 �����
  temp1H = temp1>>4;                   // ����������� ������� ���� � �������
  temp1 = regs_in[0];                  // �������� 
  temp1L = temp1&0x0F;                 // �������� ����� 0F �� ������� ���� 1 �����

  temp2 = regs_in[1];                  // ��������  
  temp2 = temp2&0xF0;                  // �������� ����� F0 �� ������� ���� 2 �����
  temp2H = temp2>>4;                   // ����������� ������� ���� � �������
  temp2 = regs_in[1];                  // �������� 
  temp2L = temp2&0x0F;                 // �������� ����� 0F �� ������� ���� 2 �����

  temp3 = regs_in[2];                  // ��������  
  temp3 = temp3&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp3H = temp3>>4;                   // ����������� ������� ���� � �������
  temp3 = regs_in[2];                  // �������� 
  temp3L = temp3&0x0F;                 // �������� ����� 0F �� ������� ���� 3 �����

  temp4 = regs_in[3];                  // ��������  
  temp4 = temp4&0xF0;                  // �������� ����� F0 �� ������� ���� 3 �����
  temp4H = temp4>>4;                   // ����������� ������� ���� � �������
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // �������� ����� F0 �� ������� ���� 4 �����
  regs_crc[0]= crc_in;
  */
}
void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data )
{
	int rdata = data;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.write(rdata);
	Wire.endTransmission();
	delay(10);
}
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress ) {
	byte rdata = 0xFF;
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,1);
	if (Wire.available()) rdata = Wire.read();
	return rdata;
}
/*
void i2c_eeprom_read_buffer( int deviceaddress, unsigned int eeaddress, byte *buffer, int length )
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8)); // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();
	Wire.requestFrom(deviceaddress,length);
	int c = 0;
	for ( c = 0; c < length; c++ )
	if (Wire.available()) buffer[c] = Wire.read();
	
}
void i2c_eeprom_write_page( int deviceaddress, unsigned int eeaddresspage, byte* data, byte length ) 
{
	
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddresspage >> 8)); // MSB
	Wire.write((int)(eeaddresspage & 0xFF)); // LSB
	byte c;
	for ( c = 0; c < length; c++)
	Wire.write(data[c]);
	Wire.endTransmission();
	
}
*/
void UpdateRegs()                                        // �������� ��������
{
	//-----������ ���� ------------
	//-----���������� ��� 0
	 while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // ��������� ���������� �������� ??
	// reg_Kamerton();                                     // �������� ������ �� �������� �    �������� 
		// ������������ �������� ������ �� ��������� �� ����� ������
	  //-----���������� ��� 0
	 set_rele = regBank.get(1);
	 mcp_Out1.digitalWrite(0, set_rele);                 // ���� RL0 ����  ���� Mic1p ���������

	 //-----���������� ��� 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // ���� RL1 ���� Mic2p  ����������

	 //-----���������� ��� 2
	  set_rele = regBank.get(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // ���� RL2 ���� Mic3p MTT
  
	 //-----���������� ��� 3
	  set_rele = regBank.get(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // ���� RL3 ����

	 //-----���������� ��� 4                            // ���� RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----���������� ��� 5
	  set_rele = regBank.get(6);                        // ���� RL5 ����
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----���������� ��� 6	 
	  set_rele = regBank.get(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // ���� RL6 ����

	 //-----���������� ��� 7
	  set_rele = regBank.get(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // ���� RL7 ������� �����

	 //---- ������ ����----------
	 //-----���������� ��� 8
	  set_rele = regBank.get(9);                        // ���� RL8 ���� �� ��������
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----���������� ��� 9
	  set_rele = regBank.get(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // ���� RL9 XP1 10

	 //-----���������� ��� 10                           // ���� RL10 ��������� ������� �� �������������� ������ 
	  set_rele = regBank.get(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----���������� ��� 11                            // adr_set_USB
	





	 //-----���������� ��� 12
	  set_rele = regBank.get(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor �������� ������

	 //-----���������� ��� 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT �������� ������

	 //-----���������� ��� 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT ���

	  //-----���������� ��� 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor ���

	  //  Test 3
	 //-----������ ���� ------------
	 //-----���������� ��� 0

	  set_rele = regBank.get(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   ����. �.

	 //-----���������� ��� 1
	  set_rele = regBank.get(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----���������� ��� 2
	  set_rele = regBank.get(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  ����. �.
  
	//-----���������� ��� 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 ����. �.

	 //-----���������� ��� 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "���." 

	 //-----���������� ��� 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "��C."

	 //-----���������� ��� 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "��-�����1."

	 //-----���������� ��� 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "��-�����2."

	  // Test 4
	//-----������ ���� ------------
	 //-----���������� ��� 8
	  set_rele = regBank.get(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      ���� ����������� ������  

	  //-----���������� ��� 9
	  set_rele = regBank.get(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR ���.

	  //-----���������� ��� 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    ���� ����������� ��������� ����������� � 2 ����������

	  //-----���������� ��� 11
	  set_rele = regBank.get(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS ���

	  //-----���������� ��� 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    ���� ����������� ��������� ����������� 

	  //-----���������� ��� 13
	  set_rele = regBank.get(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS ���

	  //-----���������� ��� 14
	  set_rele = regBank.get(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    ���� ���������� ��������� ���������� � 2 ����������

	  //-----���������� ��� 15
	  set_rele = regBank.get(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    ���� ���������� ��������� ����������

		 if (regBank.get(118)== 0)
		 {
			 test_repeat = false;
		 }
	else
		 {
			test_repeat = true;
		 }

	regBank.set(adr_reg_ind_CTS, !mcp_Analog.digitalRead(CTS));
	regBank.set(adr_reg_ind_DSR, !mcp_Analog.digitalRead(DSR));
	regBank.set(adr_reg_ind_DCD, !mcp_Analog.digitalRead(DCD));

	  time_control();
	  prer_Kmerton_On = true;
}

void UpdateRegs_istr()
{ 
	 boolean set_rele ;
		//-----���������� ��� 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);                  // ���� RL1 ���� Mic2p  ����������
		 //-----���������� ��� 4                           // ���� RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    
		//-----���������� ��� 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    ���� ����������� ��������� ����������� � 2 ����������
		  //-----���������� ��� 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    ���� ����������� ��������� ����������� 
	
	  regBank.set(adr_control_command,0);

}

void Reg_count_clear()
{
	for(unsigned int i = 1309; i<=1870;i++)
		{
			i2c_eeprom_write_byte(deviceaddress,i,0);
		}

	for(int k = 200; k<=337;k++)
		{
			regBank.set(k,false);
		}
	regBank.set(40121,0);
	regBank.set(adr_control_command,0);
}
void set_clock()
{    
		int day    = regBank.get(adr_set_kontrol_day);  
		int month  = regBank.get(adr_set_kontrol_month);          
		int year   = regBank.get(adr_set_kontrol_year);  
		int hour   = regBank.get(adr_set_kontrol_hour);  
		int minute = regBank.get(adr_set_kontrol_minute);  
		int second = 0;
		DateTime set_time = DateTime(year, month, day, hour, minute, second); // ������� ������ � ������� � ������ "set_time"
		RTC.adjust(set_time);                                                 // �������� ����� � ���������� �����  
		regBank.set(adr_set_time, 0);                                         // �������� � ������� ������� ��������� ���������� �������
		regBank.set(adr_control_command,0);
}
void data_clock_exchange()
{
	/*
	DateTime now = RTC.now();
	uint16_t year_temp = now.year()-2000;
	uint8_t day_temp = now.day();
	uint8_t mon_temp = now.month();

	byte  b = i2c_eeprom_read_byte(0x50, adr_temp_day);                             //access an address from the memory
		delay(10);

		if (b != day_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_day, day_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // ��� ����� ���� ������� ������ ����� �������� � "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // �������  �������� ���������� ����� �����
		  */
}
void time_control() // ��������� ������ �������� ������� � �������� ��� �������� � ��
{
	DateTime now = RTC.now();
	regBank.set(adr_kontrol_day  , now.day());
	regBank.set(adr_kontrol_month, now.month());
	regBank.set(adr_kontrol_year, now.year());
	regBank.set(adr_kontrol_hour, now.hour());
	regBank.set(adr_kontrol_minute, now.minute());
	regBank.set(adr_kontrol_second, now.second());
}

void list_file()
{
 while (file.openNext(sd.vwd(), O_READ))
  {
	file.printName(&Serial);
	Serial.write(' ');
	file.printModifyDateTime(&Serial);
	Serial.write(' ');
	file.printFileSize(&Serial);
	if (file.isDir()) {
	  // Indicate a directory.
	  Serial.write('/');
	}
	Serial.println();
	file.close();
  }
}
void load_list_files()
{

	if (!sd.begin(chipSelect)) 
		{
			Serial.println("initialization SD failed!");
		}
	else
		{
	
		while (file.openNext(sd.vwd(), O_READ))
		  {
			file.printName(&Serial2);
			Serial2.println();
			file.printName(&Serial);
			Serial.println();

			file.close();
		  } 
		   Serial2.flush();
		 }
		delay(100);
	//	Serial2.println("Files end");
	//	Serial.println("Files end");
  regBank.set(adr_control_command,0);
}

void file_print_date()  //���������  ������ ���� � ����
	{
	  DateTime now = RTC.now();
	  myFile.print(now.day(), DEC);
	  myFile.print('/');
	  myFile.print(now.month(), DEC);
	  myFile.print('/');
	  myFile.print(now.year(), DEC);//Serial display time
	  myFile.print(' ');
	  myFile.print(now.hour(), DEC);
	  myFile.print(':');
	  myFile.print(now.minute(), DEC);
	  myFile.print(':');
	  myFile.print(now.second(), DEC);
  }
//void serial_print_date()                           // ������ ���� � �������    
//{
//	  DateTime now = RTC.now();
//	  Serial.print(now.day(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.month(), DEC);
//	  Serial.print('/');
//	  Serial.print(now.year(), DEC);//Serial display time
//	  Serial.print(' ');
//	  Serial.print(now.hour(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.minute(), DEC);
//	  Serial.print(':');
//	  Serial.print(now.second(), DEC);
//}
void resistor(int resist, int valresist)
{
	resistance = valresist;
	switch (resist)
	{
	case 1:
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word1));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	case 2:				
			Wire.beginTransmission(address_AD5252);     // transmit to device
			Wire.write(byte(control_word2));            // sends instruction byte  
			Wire.write(resistance);                     // sends potentiometer value byte  
			Wire.endTransmission();                     // stop transmitting
			break;
	}
			//Wire.requestFrom(address_AD5252, 1, true);  // ������� ��������� ������ ��������� 
			//level_resist = Wire.read();                 // sends potentiometer value byte  
	// regBank.set(adr_control_command,0);
}

void controlFileName()
{
  int temp_file_name = 0;
  char _fileName[13];
  for(int i=0;i<13;i++)
  {
	 _fileName[i] = fileName[i];
  }

	while (sd.exists(_fileName)) 
  {
	if (_fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  _fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (_fileName[BASE_NAME_SIZE] != '9') 
	{
	  _fileName[BASE_NAME_SIZE + 1] = '0';
	  _fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		regBank.set(122,1);                              // ���� ������  �������� �����
	}
  }

  temp_file_name = ((_fileName[BASE_NAME_SIZE]-48)*10) + (_fileName[BASE_NAME_SIZE + 1]-48); // �������������� ����������� ������ ����� � �����
   
	if (temp_file_name == 0)
	{
		regBank.set(adr_reg_file_name,temp_file_name);   
	}
	else
	{
		regBank.set(adr_reg_file_name,temp_file_name-1); 
	}
  regBank.set(adr_control_command,0);  
}
void FileOpen()
{
  Serial.println("FileOpen");
  int temp_file_name = 0;
  preob_num_str();
  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		regBank.set(122,1);                              // ���� ������  �������� �����
	}
  }

 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // �������������� ����������� ������ ����� � �����
  regBank.set(adr_reg_file_name,temp_file_name);      
//  i2c_eeprom_write_byte(0x50, adr_file_name_count,temp_file_name);                 // ��� ����� ���� ������� ������ ����� �������� � "0"

  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
	regBank.set(122,1);                              // ���� ������  �������� �����
  }
  else
  {
	Serial.print(fileName);
	Serial.println(F("  Open Ok!"));

	DateTime now = RTC.now();

	regBank.set(adr_Mic_Start_day , now.day());           // ����� ������ �����
	regBank.set(adr_Mic_Start_month, now.month());
	regBank.set(adr_Mic_Start_year, now.year());
	regBank.set(adr_Mic_Start_hour, now.hour());
	regBank.set(adr_Mic_Start_minute, now.minute());
	regBank.set(adr_Mic_Start_second, now.second());
	// �������� 			
	regBank.set(adr_Time_Test_day, 0); 
	regBank.set(adr_Time_Test_hour, 0); 
	regBank.set(adr_Time_Test_minute, 0); 
	regBank.set(adr_Time_Test_second, 0); 
	myFile.println ("");
	myFile.print ("Report of test module Audio-1 N ");
	// �������� ������ int �� ���������
	int val_3 = regBank.get(40010);
	int val_2 = regBank.get(40011);
	int val_1 = regBank.get(40012);
	int val_0 = regBank.get(40013);
	 
	// ����������� ��� �������������� ��������� ������
	 byte *x3 = (byte *)&val_3;  
	 byte *x2 = (byte *)&val_2;
	 byte *x1 = (byte *)&val_1;
	 byte *x0 = (byte *)&val_0;

	 // �������� ����� � ���� �������� byte  � ������
	byte y[4];  
		y[3]= x3[0];
		y[2]= x2[0];
		y[1]= x1[0];
		y[0]= x0[0];

  unsigned long *yx = (unsigned long *)&y;

  number_audio = *yx;

 // Serial.println(number_audio);

	myFile.print (number_audio);
	myFile.println ("");
	myFile.println ("");
	myFile.println ("");
	myFile.print ("Start test   ");
	file_print_date();
	myFile.println ("");
	myFile.println ("");
	regBank.set(122,0);                              // ���� ��������� �������� �����                                   
//	Serial.println(fileName);
   }
  regBank.set(adr_control_command,0);  
}
void FileClose()
{
	//Serial.println(fileName);
	myFile.println ("");
	myFile.print ("Stop test   ");
	file_print_date();
	myFile.println ("");
	myFile.close();

	if (sd.exists(fileName))
		{ 
			Serial.println();
			Serial.print(fileName);
			Serial.println("  Close  OK!.");
			regBank.set(123,0);                                  // ���� �������� �����
		}
	else 
		{
			Serial.println();
			Serial.print(fileName);
			Serial.println(" doesn't exist.");  
			regBank.set(123,1);                              // ���� ������  �������� �����
		}
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}

void file_name()
{

   preob_num_str();

  while (sd.exists(fileName)) 
  {
	if (fileName[BASE_NAME_SIZE + 1] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1]++;
	}
	else if (fileName[BASE_NAME_SIZE] != '9') 
	{
	  fileName[BASE_NAME_SIZE + 1] = '0';
	  fileName[BASE_NAME_SIZE]++;
	}
	else 
	{
		Serial.println("Can't create file name");
//	  sdError("Can't create file name");
	}
  }
  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
  }
  Serial.print(F("Logging to: "));
  Serial.println(fileName);
  myFile.close();
  Serial.println("done.");
 } 
void preob_num_str() // ��������� ������������ ����� �����, ���������� �� ������� ���� � �������� ������
{
	DateTime now = RTC.now();
	day   = now.day();
	month = now.month();
	year  = now.year();
	int year_temp = year-2000;
	itoa (year_temp,str_year_file, 10);                                        // �������������� ���� ��� � ������ ( 10 - ���������� ������) 

	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  �������������� ���� �����  � ������ ( 10 - ���������� ������) 
		   itoa (month,str_mon_file10, 10);                                    //  �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // �������� 2 �����
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // �������������� ����� � ������ ( 10 - ���������� ������) 
		   itoa (day,str_day_file10, 10);                                      // �������������� ����� � ������ ( 10 - ���������� ������) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // �������� 2 �����
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // �������������� ����� � ������ ( 10 - ���������� ������) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // �������� 2 �����
	sprintf(str2, "%s%s",str1, str_day_file);                                  // �������� 2 �����
	sprintf(fileName, "%s%s", str2, "00.KAM");                                 // ��������� ����� ����� � file_name
	//Serial.println(fileName);
	regBank.set(adr_reg_temp_day, day);  
	regBank.set(adr_reg_temp_mon, month); 
	regBank.set(adr_reg_temp_year, year-2000); 
	//char* strcpy(char* fileName_p, const char* fileName);
	//Serial.println(fileName_p);
}

void control_command()
{
	/*
	��� ������ ������������ �������� ���������� �������� ����� �������� �� ������ adr_control_command (40120) 
	��� ��������
	0 -  ���������� ������� ��������
	1 -  ��������� ��� �������
	2 -  �������� ��� �������
	3 -  ���� �����������
	4 -  ���� ����������
	5 -  ���� ���
	6 -  ���� ����.�
	7 -  ���� ��������
	8 -  ���� ���
	9 -  ���� ����� 1
	10 - ���� ����� 2
	11 - ���� ����������
	12 - ������� ����
	13 - ������� ����
	14 - �������� �����
	15 - ���������� ������� �������
	16 - Reg_count_clear();			                                        // ����� ��������� ������                    
	17 - test_power();                                                    	// ��������� ����������  �������
	18 - set_video();				 //
	19 - test_video();				 //
	20 - �������� ������ ������� ���������
	21 - �������� ������ ������� ����������������
	22 - �������� ������ ������� ����������������
	23 - �������� ����� �����
	24 - 
	25 - 
	26 - 
	27 - 
	28 - 
	29 - 
	30 - 
	31 - 
	32 - 
	33 - 
	34 - 
	35 - 
	36 - 




	*/
	UpdateRegs() ;

	int test_n = regBank.get(adr_control_command);                                  //�����  40120
	if (test_n != 0)
	{
		if(test_n != 0) Serial.println(test_n);	
		switch (test_n)
		{
			case 1:
				 sensor_all_off();                           // ��������� ��� �������
				 break;
			case 2:	
				 sensor_all_on();                            // �������� ��� �������
				 break;
			case 3:
				 test_headset_instructor();
				 break;
			case 4:	
				 test_headset_dispatcher();                  //
				 break;
			case 5:
				 test_MTT();                                 //
				 break;
			case 6:	
				 test_tangR();                               //
				 break;
			case 7:
				 test_tangN();
				 break;
			case 8:	
				 testGGS();
				 break;
			case 9:
				 test_GG_Radio1();
				 break;
			case 10:
				 test_GG_Radio2();
				 break;
			case 11:
				 test_mikrophon();                           // ������������ ���������
				 break;
			case 12:
				 FileOpen();
				 break;
			case 13:
				 FileClose();
				 break;
			case 14:
				 set_clock();
				 break;
			case 15:
				 read_reg_error();                           // �������� ������
				 break;
			case 16:
				 Reg_count_clear();			                 // ����� ��������� ������                    
				 break;
			case 17:
				 test_power();                               // ��������� ����������  �������
				 break;
			case 18:
				 set_video();				                 //
				 break;
			case 19:
				 test_video();				                 //
				 break;
			case 20:                                         // �������� ������ ������� ���������
				 default_mem_porog();
				 break;
			case 21:                      		 		     // �������� ������ ������� ����������������
				 set_mem_porog();
				 break;
			case 22:                                         // �������� ������ ������� ����������������
				 read_mem_porog();
				 break;
			case 23: 
				 controlFileName();                          // �������� ����� �����
				 break;
			case 24: 
				 set_SD();                                   // �������� SD ������
				 break;
			case 25: 
				 send_file_PC();                             // 
				 break;
			case 26: 
				 load_list_files();                          // ��������� ������ ������ � ��������   
				 break;
			case 27: 
				 file_del_SD();
				 break;
			case 28:
				 clear_serial2();                            // �������� ����� serial2
				 break;
			case 29:
				 set_serial2();                              // ����� serial 2
				 break;
			case 30:  
				 mem_byte_trans_read();
				 break;
			case 31:
				 mem_byte_trans_save();
				 break;
			case 32: 
				 i2c_test1();
				 break;
			case 33: 
				 i2c_test_int();
				 break;
			case 34:
				 set_analog_pin();                           // ���������� ����� ��������� ����� ������������
				 break;
			case 35:
				 oscilloscope();                             // ���������� ��������� �������������
				 break;
			case 36: 
															 // Serial.println("  Reset");	
				 wdt_disable();                              // ���������, ��� ��� �� ����� ���� ��� �������
				 wdt_enable(WDTO_15MS);                      // ����������� �����. ��������� ���������� ��������� ����� 15��
				 while (1) {}	                             // ��������� 1 ���
				 break;
			case 37: 
				 set_sound_oscill();                         // ���������� ������� ��� ��������� �������������
				 break;
			case 38: 
				 set_rezistor1();                            // ���������� ���������� �1 ������� �������
				 break;
			case 39:
				 set_rezistor2();                            // ���������� ���������� �2 ������� �������
				 break;
			case 40:
				 set_rezistor_All();                           
				 break;
			case 41:
				 test_serial2();
				 break;
			case 42:
				 i2c_eeprom_write_byte(deviceaddress,adr_set_USB,0);    // ���� ������������ USB - RS232 
				 set_USB0();
				 break;
			case 43:
				 i2c_eeprom_write_byte(deviceaddress,adr_set_USB,255);  // ���� ������������ USB - RS232 
				 set_USB0();
				 break;
		   case 44:
				 UpdateRegs();
				 break;
		   case 45:
				 UpdateRegs_istr();
				 break;
		   case 46:
				// UpdateRegs_disp();
				 break;
		   case 47:
				// UpdateRegs_mtt();
				 break;
		   case 48:
				// UpdateRegs_mic();
				 break;
		   case 49:
				// UpdateRegs_ggs();
				 break;
		   case 50:
				// UpdateRegs_radio1();
				 break;
		   case 51:
				 //UpdateRegs_radio2();
				 break;
		   case 52:
				 set_radio_send();
				 break;
		   case 53:
				 set_ggs_mute();
				 break;
		   case 54:
				// 
				 break;

			default:
				 regBank.set(adr_control_command,0);        // ���������� ���������� �1,�2  ������� �������
				 break;
		 }

	}
	else
	{
	   regBank.set(adr_control_command,0);
	}
}

void sensor_all_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{
		myFile.println("");
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[2])));                    //  " ****** Test sensor OFF start! ******" ;      
		myFile.println(buffer);                                                         //  " ****** Test sensor OFF start! ******" ;      
		file_print_date();
		myFile.println();
	}
	regBank.set(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(100);
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT     �������� ������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	byte i50 = regBank.get(40004);    
	byte i52 = regBank.get(40006);     
	byte i53 = regBank.get(40007);     

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40201);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			save_reg_eeprom(adr_reg40201,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(201,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja ��������  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = read_reg_eeprom(adr_reg40202);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			save_reg_eeprom(adr_reg40202,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(202,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  ��������  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			save_reg_eeprom(adr_reg40203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = read_reg_eeprom(adr_reg40204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			save_reg_eeprom(adr_reg40204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor ��������  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40205);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			save_reg_eeprom(adr_reg40205,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(205,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 ��������  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = read_reg_eeprom(adr_reg40206);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			save_reg_eeprom(adr_reg40206,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(206,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			save_reg_eeprom(adr_reg40207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // ���� RL4 XP1 12  HeS2e   ���������� ��������� �����������
		  {
			regcount = read_reg_eeprom(adr_reg40208);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			save_reg_eeprom(adr_reg40208,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(208,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� ��������  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = read_reg_eeprom(adr_reg40209);                                          // ����� �������� ������ ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ���������� ��������� ����������
			save_reg_eeprom(adr_reg40209,regcount);                                            // ����� �������� ������ ���������� ��������� ����������
			regBank.set(209,1);                                                     // ���������� ���� ������ ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. ��������  - Pass
			   }
		  }
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void sensor_all_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{
		myFile.println("");
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
		myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
		file_print_date();
		myFile.println();
	}
	regBank.set(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(100);
	regBank.set(5,1);                                                               // �������� ����������� ��������
	regBank.set(10,1);                                                              // �������� ���������� ��������
	regBank.set(13,1);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	//regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	//regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	//regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
//	delay(500);
	UpdateRegs(); 
	delay(100);

	byte i50 = regBank.get(40004);    
	byte i52 = regBank.get(40006);     
	byte i53 = regBank.get(40007);    

	 //Serial.print(regs_in[0],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[1],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[2],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_in[3],HEX);
	 //Serial.println("    ");

	 //Serial.println(i50,BIN);
	 //Serial.println(i52,BIN);
	 //Serial.println(i53,BIN);

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  �������� ������                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			save_reg_eeprom(adr_reg40211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor �������� ������                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = read_reg_eeprom(adr_reg40212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			save_reg_eeprom(adr_reg40212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  - Pass
			  }
		  }
//UpdateRegs(); 
//delay(500);
// test_instr_on();
// UpdateRegs(); 
// delay(500);
// test_disp_on();

/*
		//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	Serial.println("Instr2");
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);
	int kl=0;

	do{
		kl++;

	  }while (regBank.get(40007) == regs_temp1);
	  //if( regBank.get(40007) != regs_temp)
	  //{
		 //Serial.println(regBank.get(40004),BIN);
		 //Serial.println(regBank.get(40006),BIN);
		 //Serial.println(regBank.get(40007),BIN);
	  //}
	  regs_temp1 = regBank.get(40007);
	  kl=0;


	i52 = regBank.get(40006);     

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = regBank.get(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

//--------------------------------------------------------------------------------
		Serial.println("Instr");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);
	i52 = regBank.get(40006);     


		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = regBank.get(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
			  }
		  }
//-------------------------------------------------------------------------------
		Serial.println("Disp2");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = regBank.get(40006);    

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
		  {
			regcount = regBank.get(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
			  }
		  }
		//--------------------------------------------------------------------------------------------------
		Serial.println("Disp");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = regBank.get(40006);    
		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor ���������� ��������� ���������� 
		  {
			regcount = regBank.get(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			regBank.set(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
			  }
		  }
//------------------------------------------------------------------------------------------------------------------------
*/
		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor ��������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			save_reg_eeprom(adr_reg40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = read_reg_eeprom(adr_reg40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			save_reg_eeprom(adr_reg40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // ���� RL9 XP1 10 ���������� ��������� ����������
		  {
			regcount = read_reg_eeprom(adr_reg40219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			save_reg_eeprom(adr_reg40219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			regBank.set(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
			   }
		  }



	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������




	/*
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	regBank.set(8,1);                                                               // �������� ������� ��������
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ ������ �������� ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	UpdateRegs(); 
	delay(500);
	UpdateRegs(); 
	delay(500);
	byte i50 = regs_in[0];    
	byte i52 = regs_in[2];    
	byte i53 = regs_in[3];  

	 Serial.print(regs_in[0],HEX);
	 Serial.print("--");
	 Serial.print(regs_in[2],HEX);
	 Serial.print("--");
	 Serial.print(regs_in[3],HEX);
	 Serial.println("    ");

   if (i50 == 0xA3)                                                       
	   {
		if(i52 == 0x43 )
			{
			if(i53 == 0x02 )
				{
				if (test_repeat == false)
					{
						strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
						myFile.print(buffer);                                               // 
						strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 �������  - Pass
					}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)

	   {
			regcount = regBank.get(40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x45 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			regBank.set(40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x49 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40215);                                          // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(40215,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ���������� � 2 ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������                                            // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  

	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x51 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher ��������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40216);                                          // ����� �������� ������ sensor ���������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ��������� ���������� 
			regBank.set(40216,regcount);                                            // ����� �������� ������ sensor ���������� ��������� ����������
			regBank.set(216,1);                                                     // ���������� ���� ������ sensor ���������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������        
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA7)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x04 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������        
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xAB)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x08 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40211);                                          // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // ����� �������� ������ sensor �������� ������     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // ���������� ���� ������ sensor �������� ������    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(13,1);                                                              // XP8 - 2   sensor �������� ������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xB3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x01 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  �������  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40212);                                          // ����� �������� ������ sensor �������� ������      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // ��������� ������� ������  sensor �������� ������  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // ����� �������� ������  sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // ���������� ���� ������ sensor �������� ������     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x61 )
			   {
				if(i53 == 0x02 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			regBank.set(40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }



	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ��������� ���������
	regBank.set(5,1);                                                               // �������� ����������� �������� "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x11 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // �������� ����������� �������  - Pass
						}

				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40218);                                          // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			regBank.set(40218,regcount);                                            // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,true);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	test_sens = true;                                                               // ���� ���������� �����
	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,1);                                                              // �������� ���������� ��������
	UpdateRegs(); 
	delay(500);
	i50 = regs_in[0];    
	i52 = regs_in[2];    
	i53 = regs_in[3];  


	   if (i50 == 0xA3)                                                       
	   {
		   if(i52 == 0x41 )
			   {
				if(i53 == 0x44 )
					{
					if (test_repeat == false)
						{
							strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
							myFile.print(buffer);                                               // 
							strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. �������  - Pass
						}
				}
				else
					{
					   test_sens = false;
					}
			}

		else
		   {
			 test_sens = false;
		   }

	   }

   else 
   {
	 test_sens = false;
   }

if(test_sens == false)
	   {
			regcount = regBank.get(40219);                                          // ����� �������� ������ ��������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������
			regBank.set(40219,regcount);                                            // ����� �������� ������ ��������� ��������� ����������
			regBank.set(219,1);                                                     // ���������� ���� ������ ��������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
	   }

	regBank.set(5,0);                                                               // �������� ����������� ���������
	regBank.set(10,0);                                                              // �������� ���������� ���������
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT     �������� ������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 �������� ������ DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR ���.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ���
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS ���
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	*/
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    

}

void set_rezistor1()
{
	int mwt1 = regBank.get(40060);             // ����� �������� �������� ������� ���������� � 1
	resistor(1, mwt1);
	//Serial.println(mwt1);
	regBank.set(adr_control_command,0);
}
void set_rezistor2()
{
	int mwt2 = regBank.get(40064);             // ����� �������� �������� ������� ���������� � 2
	resistor(2, mwt2);
	regBank.set(adr_control_command,0);
}
void set_rezistor_All()
{
	int mwt1 = regBank.get(40060);             // ����� �������� �������� ������� ���������� � 1
	resistor(1, mwt1);
	resistor(2, mwt1);
	regBank.set(adr_control_command,0);
}

void test_headset_instructor()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(201, 39);                                       // ������� �� EEPROM ������ ������� �����������
	set_sound(1);
	if (test_repeat == false)
	{
		myFile.println("");
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[22])));
		myFile.println(buffer);                                                         // " ****** Test headset instructor start! ******"               ; 
		file_print_date();
		myFile.println("");
	}
	unsigned int regcount = 0;
	test_instr_off();                                                               // ��������� ���� � �������, �������� ����������
	test_instr_on();                                                                // �������� ����������� �������, ��������� ���������
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, por_int_buffer[1]);                                                                // ���������� ������� ������� 30 ��
	resistor(2, por_int_buffer[2]);                                                                // ���������� ������� ������� 30 ��
	regBank.set(2,1);                                                               // ������ ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();                                                                   // ��������� �������
	delay(300);

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                              // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                                        // "Signal headset instructor microphone 30mv     ON"            ;   
	
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40230, adr_reg40430, 230,por_int_buffer[3]);      // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40231, adr_reg40431, 231,por_int_buffer[5]);      // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineL,    adr_reg40232, adr_reg40432, 232,por_int_buffer[7]);      // �������� ������� ������� �� ������ LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40233, adr_reg40433, 233,por_int_buffer[9]);      // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,adr_reg40234, adr_reg40434, 234,por_int_buffer[11]);     // �������� ������� ������� �� ������ mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,adr_reg40235, adr_reg40435, 235,por_int_buffer[13]);     // �������� ������� ������� �� ������ mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	measure_vol_min(analog_ggs,      adr_reg40236, adr_reg40436, 236,por_int_buffer[15]);     // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40237, adr_reg40437, 237,por_int_buffer[17]);     // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40238, adr_reg40438, 238,por_int_buffer[19]);     // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
												   //
	regBank.set(5,1);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (�������� ��������)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT �������� PTT �����������
	regBank.set(16,0);                                                              // ������ ��������� ���������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(29,1);                                                              // ��� XP1- 13 HeS2Ls ������  ��� ���� ����������� ��������� ����������� 
	UpdateRegs();                                                                   // 
	delay(300); 

	byte i53 = regBank.get(40007);                                                  // �������� ������� ��������� ���������
		if(bitRead(i53,4) == 0)                                                     // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
		  {
			regcount = read_reg_eeprom(adr_reg40218);                               // ����� �������� ������ ��������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� �����������
			save_reg_eeprom(adr_reg40218,regcount);                                 // ����� �������� ������ ��������� ��������� �����������
			regBank.set(218,1);                                                     // ���������� ���� ������ ��������� ��������� �����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // �������� ����������� �������  - Pass
			  }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    �������� ������ ����� �� ���� ��������� �����������
	delay(100);

	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    adr_reg40224, adr_reg40424, 224,por_int_buffer[25],por_int_buffer[26]);                                // �������� ������� ������� �� ������ LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,adr_reg40226, adr_reg40426, 226,por_int_buffer[31],por_int_buffer[32]);                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40230, adr_reg40430, 230,por_int_buffer[4]);                                 // �������� ������� ������� �� ������ FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40231, adr_reg40431, 231,por_int_buffer[6]);                                 // �������� ������� ������� �� ������ FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40233, adr_reg40433, 233,por_int_buffer[10]);                                 // �������� ������� ������� �� ������ LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      adr_reg40236, adr_reg40436, 236,por_int_buffer[16]);                                 // �������� ������� ������� �� ������ GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40237, adr_reg40437, 237,por_int_buffer[18]);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40238, adr_reg40438, 238,por_int_buffer[20]);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";
	
// �������� analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));  // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // ���� ���������� ��������������
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40333, adr_reg40533, 333,por_int_buffer[29],por_int_buffer[30]);                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));  // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio,adr_reg40234, adr_reg40434, 234,por_int_buffer[11]);     // �������� ������� ������� �� ������ mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.set(42,0);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // ���� ���������� ��������������
	delay(50);
	set_radio_send1();
	delay(100);
	
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(5,0);                                                               // ������ ����������� ������� �� ����� 12 ��1 HeS2e (��������� �������� �����������)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	regBank.set(2,0);                                                               // ��������� ������ �� ���� ��������� �����������  Mic2p
	UpdateRegs();     
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_headset_dispatcher()
 {
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(277, 39);                                                    // ������� �� EEPROM ������ ������� �����������
	set_sound(1);
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[23])));              // " ****** Test headset dispatcher start! ******"               ;
		myFile.println(buffer);                                                    // " ****** Test headset dispatcher start! ******"               ;
		file_print_date();
		myFile.println("");
	}
	unsigned int regcount = 0;
	test_disp_off();                                                                // ��������� ���� � �������, �������� ����������
	test_disp_on();                                                                 // �������� ����������� �������, ��������� ���������
	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, por_int_buffer[1]);                                                                // ���������� ������� ������� 30 ��
	resistor(2, por_int_buffer[2]);                                                                // ���������� ������� ������� 30 ��
	regBank.set(1,1);                                                               // ������ ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs();                                                                   // ��������� �������
	delay(100);

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40240, adr_reg40440, 240,por_int_buffer[3]);                  // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40241, adr_reg40441, 241,por_int_buffer[5]);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� "���"  ������ Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    adr_reg40242, adr_reg40442, 242,por_int_buffer[7]);                  // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40243, adr_reg40443, 243,por_int_buffer[9]);                  // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,adr_reg40244, adr_reg40444, 244,por_int_buffer[11]);                 // �������� ������� ������� �� ������ mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,adr_reg40245, adr_reg40445, 245,por_int_buffer[13]);                                 // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ ��� +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      adr_reg40246, adr_reg40446, 246,por_int_buffer[15]);                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40247, adr_reg40447, 247,por_int_buffer[17]);                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40248, adr_reg40448, 248,por_int_buffer[19]);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ �������� �������� ����������� ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
	regBank.set(10,1);                                                              // ������ ����������� ������� �� ����� XP1 10 ��������� ��������� ����������
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	regBank.set(16,0);                                                              // ������ ��������� ���������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������

	UpdateRegs();                                                                   // 
	delay(100);                                                                     //

	byte i53 = regBank.get(40007);     
		if(bitRead(i53,6) == 0)                                                      // ��������  ��������� ��������� ����������
		  {
			regcount = read_reg_eeprom(adr_reg40219);                               // ����� �������� ������ ��������� ��������� ����������          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // ��������� ������� ������ ��������� ��������� ����������       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			save_reg_eeprom(adr_reg40219,regcount);                                 // ����� �������� ������ ��������� ��������� ����������
			regBank.set(219,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			resistor(1, 255);                                                       // ���������� ������� ������� � �������� ��������e
			resistor(2, 255);                                                       // ���������� ������� ������� � �������� ��������e
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - ";  - Pass
			   }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[14])));                   // "Microphone headset dispatcher signal          ON" 
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  �������� ������ ����� �� ���� ��������� ����������
	delay(100);

	//+++++++++++++++++++++++++++ ��������� ������� ������� �� ������ LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    adr_reg40227, adr_reg40427, 227,por_int_buffer[25],por_int_buffer[26]);                                // �������� ������� ������� �� ������ LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,adr_reg40229, adr_reg40429, 229,por_int_buffer[31],por_int_buffer[32]);                                // �������� ������� ������� �� ������ mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������ +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40240, adr_reg40440, 240,por_int_buffer[4]);                                 // �������� ������� ������� �� ������ FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40241, adr_reg40441, 241,por_int_buffer[6]);                                 // �������� ������� ������� �� ������ FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40243, adr_reg40443, 243,por_int_buffer[10]);                                 // �������� ������� ������� �� ������ LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      adr_reg40246, adr_reg40446, 246,por_int_buffer[16]);                                 // �������� ������� ������� �� ������ GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40247, adr_reg40447, 247,por_int_buffer[18]);                                 // �������� ������� ������� �� ������ GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40248, adr_reg40448, 248,por_int_buffer[20]);                                 // �������� ������� ������� �� ������ GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	// �������� analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));     // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // ���� ���������� ��������������
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40334, adr_reg40534, 334,por_int_buffer[29],por_int_buffer[30]);              // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));     // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio,adr_reg40244, adr_reg40444, 244,por_int_buffer[11]);                                 // �������� ������� ������� �� ������ mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	regBank.set(42,0);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // ���� ���������� ��������������
	delay(50);
	set_radio_send1();
	delay(100);
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   ��������� sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   ���������  sensor ���������� ��������� ����������
	regBank.set(15,0);                                                              // ��� ��������� ���������
	regBank.set(10,0);                                                              // ������ ����������� ������� �� ����� XP1 10  (��������� �������� ����������)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.set(1,0);                                                               // ��������� ������ �� ���� ��������� ���������� Mic1p
	UpdateRegs(); 
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
 } 
void test_MTT()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(353, 39);
	set_sound(1);
	if (test_repeat == false)
	{	
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[24])));                   // " ****** Test MTT start! ******"                              ; 
		myFile.println(buffer);                                                         // " ****** Test MTT start! ******"                              ; 
		file_print_date();
		myFile.println("");
	}
	test_MTT_off();                                                                 // ��������� ���� � �������, �������� ����������
	test_MTT_on();                                                                  // �������� ����������� �������, ��������� ���������
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT �������� ������ ���� � "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD ������ ������� DCD ������ ���� � "0"
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"
	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������ ��������� �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40250, adr_reg40450, 250,por_int_buffer[3]);                // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40251, adr_reg40451, 251,por_int_buffer[5]);                // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40252, adr_reg40452, 252,por_int_buffer[7]);                // �������� ������� ������� �� ������ LineL     "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40253, adr_reg40453, 253,por_int_buffer[9]);                // �������� ������� ������� �� ������ LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40254, adr_reg40454, 254,por_int_buffer[11]);               // �������� ������� ������� �� ������ mag radio "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40255, adr_reg40455, 255,por_int_buffer[13]);               // �������� ������� ������� �� ������ mag phone "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[15]);               // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40257, adr_reg40457, 257,por_int_buffer[17]);               // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40258, adr_reg40458, 258,por_int_buffer[19]);               // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,por_int_buffer[1]);                                                                       // ���������� ������� ������� 60 ��
	resistor(2,por_int_buffer[2]);                                                                       // ���������� ������� ������� 60 ��
	regBank.set(3,1);                                                                                    // �������� ������ �� ���� ��������� ������ Mic3p
	UpdateRegs();                                                                                        // ��������� �������
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                                        // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;
	//++++++++++++++++++++++++++++++++++ ��������� ���������� ������� �� ������  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40250, adr_reg40450, 250,por_int_buffer[4]);                 // �������� ������� ������� �� ������ FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40251, adr_reg40451, 251,por_int_buffer[6]);                 // �������� ������� ������� �� ������ FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40254, adr_reg40454, 254,por_int_buffer[12]);                // �������� ������� ������� �� ������ mag radio   "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[16]);                // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40257, adr_reg40457, 257,por_int_buffer[18]);                // �������� ������� ������� �� ������ GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40258, adr_reg40458, 258,por_int_buffer[20]);                // �������� ������� ������� �� ������ GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ ��������� ������� �������  ++++++++++++++++++++++++++++++++++++
	//measure_vol_max(analog_LineL,     40260,260,por_int_buffer[25],por_int_buffer[26]);                               // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,     adr_reg40261, adr_reg40461,  261,por_int_buffer[27],por_int_buffer[28]);                              // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone, adr_reg40262, adr_reg40462,  262,por_int_buffer[31],por_int_buffer[32]);                               // �������� ������� ������� �� ������ mag phone  "Test MTT ** Signal Mag phone                                ON  - ";
	// +++++++++++++++++++++ �������� ������������ ������ ��� �� ������ HangUp  DCD ON +++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                                                      // ��������� ������ �� ���� ��������� ������ Mic3p
	regBank.set(6,1);                                                                                      // ���� RL5. ������ ���� Front L, Front R
	UpdateRegs();                                                                                          // ��������� �������
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                                          //   
	if (test_repeat == false) myFile.println(buffer);                                                      // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[16]);                 // �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS                                      OFF - ";
	regBank.set(18,1);                                                                                     // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                                          // ��������� �������
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                                          // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                      // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,  adr_reg40289, adr_reg40489, 289,  por_int_buffer[33],por_int_buffer[34]);                       //  �������� ������� ������� �� ������ GGS       "Test MTT ** Signal GGS             On      
	regBank.set(18,0);                                                                                     // XP1 - 20  HangUp  DCD ON  �������� ������
	regBank.set(26,0);                                                                                     // XP1- 17 HaSPTT    CTS DSR ���. ��������� PTT MTT
	regBank.set(25,1);                                                                                     //  XP1- 19 HaSs  sensor ����������� ������    MTT ��������� ������ ���� � "1"
	regBank.set(6,0);                                                                                      // ���� RL5. ��������� ���� Front L, Front R
	UpdateRegs();                                                                                          // ��������� �������
	set_sound(0);
	regBank.set(adr_control_command,0);                                                                    // ��������� ���������    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
}
void test_tangR()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{	
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[36])));                   // " ****** Test tangenta ruchnaja start! ******"                ;
		myFile.println(buffer);                              //
		file_print_date();
		myFile.println("");
	}
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = regBank.get(40004);    

	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor �������� ������               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = read_reg_eeprom(adr_reg40274);                               // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(adr_reg40274,regcount);                                     // ����� �������� ������ sensor �������� ������     "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(274,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            OFF - ";  - Pass
		  }
		}

	 UpdateRegs(); 
	 delay(400);
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40270);                               // ����� �������� ������                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(adr_reg40270,regcount);                                     // ����� �������� ������ 
			regBank.set(270,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";  - Pass
		  }
		 }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40271);                                          // ����� �������� ������  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			regBank.set(adr_reg40271,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regBank.set(271,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";  - Pass
		   }
		  }

	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(17,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	regBank.set(20,1);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // ��������� �������
	delay(400);
	i50 = regBank.get(40004);    

		if(bitRead(i50,3) == 0)                                          // J8-11     XP7 2 sensor �������� ������             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40275);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta ruchnaja                            ON  - ";
			regBank.set(adr_reg40275,regcount);                                            // ����� �������� ������ sensor �������� ������
			regBank.set(275,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta ruchnaja                            ON  - "; - Pass
		   }
		  }
	 UpdateRegs(); 
	 delay(400);
	  // 2)  ��������  �� ���������� J8-23     XP7 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP7 1 PTT1 �������� ������ CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40272);                                          // ����� �������� ������ PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40272,regcount);                                            // ����� �������� ������ PTT  MTT (CTS)
			regBank.set(272,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - "; - Pass
		   }
		  }

	 // 3)  ��������  �� ���������� PTT2 �������� ������ (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                       // ��������  �� ����������  PTT2 �������� ������ (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40273);                                          // ����� �������� ������  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(40273,regcount);                                            // ����� �������� ������  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			regBank.set(273,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";  - Pass
		   }
		  }
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor �������� ������
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 �������� ������ CTS
	UpdateRegs();                                                                   // ��������� �������
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
}
void test_tangN()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[43])));                   // " ****** Test tangenta nognaja start! ******"                 ;
		myFile.println(buffer);                                                         // "Command sensor OFF tangenta nognaja              send!"      ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(13,0);                                                              // XP8 - 2   sensor �������� ������
	regBank.set(14,0);                                                              // XP8 - 1   PTT �������� ������
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
  
	byte i50 = regBank.get(40004);    

	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor ��������                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = read_reg_eeprom(adr_reg40276);                                          // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������  "Command sensor tangenta nognaja                             OFF - ";
			save_reg_eeprom(adr_reg40276,regcount);                                            // ����� �������� ������ sensor �������� ������     "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(276,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		}
	else
		{
		  if (test_repeat == false)
		  {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             OFF - "; - Pass
		  }
		}

	 UpdateRegs(); 
	  // 2)  ��������  �� ����������  XP8 1 PTT1 �������� ������ CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� XP8 1 PTT1 ��������   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40278);                                          // ����� �������� ������ 
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40278,regcount);                                            // ����� �������� ������                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			regBank.set(278,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  }
		 }
	regBank.set(13,1);                                                              // XP8 2 sensor �������� ������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(14,1);                                                              // J8-12     XP7 4 PTT2 �������� ������ DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;

	UpdateRegs();                                                                   // ��������� �������
	delay(100);

	i50 = regBank.get(40004);    

			if(bitRead(i50,4) == 0)                                                 // J8-11     XP7 2 sensor ��������                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40277);                                          // ����� �������� ������ sensor �������� ������       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor �������� ������    "Command sensor tangenta nognaja                             ON  - ";
			save_reg_eeprom(adr_reg40277,regcount);                                            // ����� �������� ������ sensor �������� ������
			regBank.set(277,1);                                                     // ���������� ���� ������ sensor �������� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                         // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command sensor tangenta nognaja                             ON  - ";
		   }
		  }
	 UpdateRegs(); 
	  // 2)  ��������  �� ����������  XP8 1 PTT1 ��������  CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������� XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40279);                                          // ����� �������� ������                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40279,regcount);                                            // ����� �������� ������                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			regBank.set(279,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT tangenta nognaja (CTS)                          ON  - ";
		   }
		  }

	regBank.set(14,0);                                                              //   XP8 1 PTT ��������  
	regBank.set(13,0);                                                              //   XP8 2 sensor ��������  
	UpdateRegs();                                                                   // ��������� �������
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_mikrophon()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	read_porog_eeprom(429, 39);
	set_sound(0);
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test mi�rophone start! ******"                       ;
		myFile.println(buffer);                                                         // " ****** Test mi�rophone start! ******"                       ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������

	regBank.set(14,0);    // XP8 - 1   PTT �������� ������
	regBank.set(17,0);    // J8-12     XP7 4 PTT2   ����. �.
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 ����. �.
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR ���.  
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS ���   ��� ����������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   // "Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(100);

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������� ������� �  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++

	byte i52 = regBank.get(40006);     

			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			save_reg_eeprom(adr_reg40207,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone ��������  - Pass
			  }
		  }

	 UpdateRegs(); 
	 delay(100);

	  // 2)  ��������  �� ���������� PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40264);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40264,regcount);                                            // ����� �������� ������ 
			regBank.set(264,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  OFF - ";  - Pass
		  }
		 }

	 // +++++++++++++++++++++++++++++++++++++++ ��������  �� ��������� �������  microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(100);

	i52 = regBank.get(40006);     

	  if(bitRead(i52,5) == 0)                                                       // XS1 - 6   sensor ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			save_reg_eeprom(adr_reg40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ ��������  �� ���������  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(15,1);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������

	i52 = regBank.get(40006);     
 

	if(bitRead(i52,5) == 0)                                                         // XS1 - 6   sensor ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // ����� �������� ������ sensor ����������� ���������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ���������
			save_reg_eeprom(adr_reg40217,regcount);                                            // ����� �������� ������ sensor ����������� ���������
			regBank.set(217,1);                                                     // ���������� ���� ������ sensor ����������� ���������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone �������  - Pass;  �������  -
			}
		  }

	 UpdateRegs(); 

	  // 2)  ��������  �� ���������  PTT microphone
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ���������      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = read_reg_eeprom(adr_reg40266);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40266,regcount);                                            // ����� �������� ������ 
			regBank.set(266,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  ON  - ";
		  }
		 }

	// ++++++++++++++++++++++++++++++++++ ��������� ����������� ������  �� ���������� ������� ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320, por_int_buffer[3]);                 // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[5]);                 // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40322, adr_reg40522, 322,por_int_buffer[7]);                 // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[9]);                 // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[11]);                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40325, adr_reg40525, 325,por_int_buffer[13]);                // �������� ������� ������� �� ������ mag phone "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[15]);                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[17]);                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[19]);                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ ������ ������ �� ���� ��������� +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,por_int_buffer[1]);                                                  // ���������� ������� ������� 60 ��
	resistor(2,por_int_buffer[2]);                                                  // ���������� ������� ������� 60 ��
	regBank.set(9,1);                                                               // �������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs();                                                                   // ��������� �������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[55])));                   // "Signal mi�rophone 30  mV                      ON"     
	if (test_repeat == false) myFile.println(buffer);                               //

	measure_vol_max(analog_mag_phone, adr_reg40298, adr_reg40498, 298,por_int_buffer[31],por_int_buffer[32]);                // �������� ������� ������� �� ������ mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,     adr_reg40299, adr_reg40499, 299,por_int_buffer[25],por_int_buffer[26]);                // �������� ������� ������� �� ������ "Test Microphone ** Signal LineL                      ON  - ";  

	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320,por_int_buffer[4]);                 // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[6]);                 // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[10]);                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[16]);                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[18]);                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[20]);                // �������� ������� ������� �� ������ GG Radio2 "Test Microphone ** Signal GG Radio2     

	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	regBank.set(16,1);                                                              // XS1 - 6   sensor ����������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   // "Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs();                                                                   // ��������� �������
	delay(100);

	  // 2)  ��������  �� ���������� PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40264);                                          // ����� �������� ������       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40264,regcount);                                            // ����� �������� ������ 
			regBank.set(264,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Test microphone PTT  (CTS)                                  OFF - ";  - Pass
		  }
		 }

	measure_vol_max(analog_mag_phone, adr_reg40298, adr_reg40498, 298,por_int_buffer[31],por_int_buffer[32]);                // �������� ������� ������� �� ������ mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,     adr_reg40299, adr_reg40499, 299,por_int_buffer[25],por_int_buffer[26]);                // �������� ������� ������� �� ������ "Test Microphone ** Signal LineL                      ON  - ";  

	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320,por_int_buffer[4]);                 // �������� ������� ������� �� ������ FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[6]);                 // �������� ������� ������� �� ������ FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[10]);                // �������� ������� ������� �� ������ LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[16]);                // �������� ������� ������� �� ������ GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[18]);                // �������� ������� ������� �� ������ GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[20]);    
	// �������� analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));  // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // ���� ���������� ��������������
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40336, adr_reg40536, 336,por_int_buffer[29],por_int_buffer[30]);                                // �������� ������� ������� �� ������ mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));  // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // �������� ������� ������� �� ������ mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	regBank.set(42,0);                                                // ���� ���������� ��� (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // ���� ���������� ��������������
	delay(50);
	set_radio_send1();
	delay(100);
	regBank.set(9,0);                                                               // ��������� ������ �� ���� ��������� ���� RL8 ���� �� ��������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ����������� ���������
	regBank.set(15,0);                                                              // XS1 - 5   PTT ��� CTS
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	set_sound(0);
	delay(200);
	regBank.set(adr_control_command,0);                                             // ��������� ���������    

}
void testGGS()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(505, 39);
	set_sound(5);
	delay(1000);
	unsigned int regcount = 0;
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[48])));                   // " ****** Test GGS start! ******"      ;
		myFile.println(buffer);                                                         // " ****** Test GGS start! ******"      ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������  
	regBank.set(18,0);     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                  // "Command sensor OFF  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                  // "Command HangUp OFF MTT                              send! "      ;
	if (test_repeat == false) myFile.println(buffer);              
	resistor(1,por_int_buffer[1]);                                                            // ���������� ������� ������� 60 ��
	resistor(2,por_int_buffer[2]);                                                              // ���������� ������� ������� 60 ��
	regBank.set(6,0);                                                               // ���� RL5 ���� Front L, Front R
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[71])));                   // "Signal FrontL, FrontR                         OFF "      ;
	if (test_repeat == false) myFile.println(buffer);              
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	byte i50 = regBank.get(40004);                                                 


		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ ��������  - Pass
			   }
		  }
		if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                          // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40267,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));    // "Test MTT HangUp (DCD)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT HangUp (DCD)                                       OFF - ";
			   }             
		 }

	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40280, adr_reg40480, 280,por_int_buffer[3]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40281, adr_reg40481, 281,por_int_buffer[5]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[7]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[9]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40284, adr_reg40484, 284,por_int_buffer[11]);                // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40285, adr_reg40485, 285,por_int_buffer[13]);                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40286, adr_reg40486, 286,por_int_buffer[15]);                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[17]);                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[19]);                                // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	regBank.set(6,1);                                                               // ���� RL5 ���� Front L, Front R
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7V             ON"   
	if (test_repeat == false) myFile.println(buffer);              
	delay(100);
	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);                // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";

	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);                                    // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);                                   // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";

	measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);      
	measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);                // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       adr_reg40289, adr_reg40489, 289,por_int_buffer[33],por_int_buffer[34]);                // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      ON  - ";

	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);                                   // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                                   // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor ����������� ������          
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs(); 
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;

	i50 = regBank.get(40004);    

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  ������ �������  - Pass
			   }
		  }

		if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                           // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                              // ��������� ������� ������
			save_reg_eeprom(adr_reg40267,regcount);                                             // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                      // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                      // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                           // �������� ������ �������� ���� ������
			regcount_err++;                                                          // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                             // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));         // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                    // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));             // "    Error! - "; 
			myFile.print(buffer);                                                    // "    Error! - "; 
			myFile.println(regcount);                                                // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));     // "Test MTT HangUp (DCD)                                       OFF - ";
				myFile.print(buffer);                                                // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));         // "Pass";
				myFile.println(buffer);                                              // "Test MTT HangUp (DCD)                                       OFF - ";
			   }             
		 }
		//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
		measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);             // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
		measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);             // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	
		measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);              // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
		measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);             // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";
	
		measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);             // �������� ������� ������� �� ������ "Test GGS ** Signal mag radio                                OFF - ";
		measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);             // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                OFF - ";
	
		measure_vol_min(analog_ggs,       adr_reg40286, adr_reg40486, 286,por_int_buffer[16]);             // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      OFF - ";
		measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);             // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
		measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                                 // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
		//----------------------------------------------------------------------------------------------------------------------------------------

	regBank.set(18,1);                                                               // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                    // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(100);
	UpdateRegs();

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                          // ��������� ��������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40268);                                           // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                              // ��������� ������� ������
			save_reg_eeprom(adr_reg40268,regcount);                                             // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                      // ���������� ���� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                      // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                           // �������� ������ �������� ���� ������
			regcount_err++;                                                          // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                             // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));         // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                    // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));             // "    Error! - "; 
			myFile.print(buffer);                                                    // "    Error! - "; 
			myFile.println(regcount);                                                // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));     // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                                // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));         // "Pass";
				myFile.println(buffer);                                              //  "Test MTT HangUp (DCD)                                       ON  - ";������ �������  - Pass
			   }
		 }

	measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal FrontR                                   ON  - ";

	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);                  // �������� ������� ������� �� ������ "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal LineR                                    OFF - ";

	measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);         
	measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       adr_reg40289, adr_reg40489, 289,por_int_buffer[33],por_int_buffer[34]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal GGS                                      ON  - ";

	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                 // �������� ������� ������� �� ������ "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(6,0);                                                                // ���� RL5 ���� Front L, Front R
	UpdateRegs();    
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                              // ��������� ���������    
}
void test_GG_Radio1()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(581, 39);
	set_sound(5);
	delay(1000);
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[50])));                   // " ****** Test Radio1 start! ******"                           ;
		myFile.println(buffer);                                                         // " ****** Test Radio1 start! ******"                           ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1,por_int_buffer[1]);                                                  // ���������� ������� ������� 300 ��
	resistor(2,por_int_buffer[2]);                                                  // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs(); 
	
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40300, adr_reg40500, 300,por_int_buffer[3]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40301, adr_reg40501, 301,por_int_buffer[5]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40302, adr_reg40502, 302,por_int_buffer[7]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40303, adr_reg40503, 303,por_int_buffer[9]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40304, adr_reg40504, 304,por_int_buffer[11]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40305, adr_reg40505, 305,por_int_buffer[13]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40306, adr_reg40506, 306,por_int_buffer[15]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40307, adr_reg40507, 307,por_int_buffer[17]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40308, adr_reg40508, 308,por_int_buffer[19]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(4,1);                                                               // ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs();  

//	Serial.println("test_GG_Radio1 - on ");
	measure_vol_min(analog_FrontL,    adr_reg40300, adr_reg40500, 300,por_int_buffer[4]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40301, adr_reg40501, 301,por_int_buffer[6]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40302, adr_reg40502, 302,por_int_buffer[8]);                 // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40303, adr_reg40503, 303,por_int_buffer[10]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_max(analog_mag_radio, adr_reg40330, adr_reg40530, 330,por_int_buffer[29],por_int_buffer[30]);  // !!         // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40305, adr_reg40505, 305,por_int_buffer[14]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40306, adr_reg40506, 306,por_int_buffer[16]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, adr_reg40309, adr_reg40509, 309,por_int_buffer[35],por_int_buffer[36]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, adr_reg40308, adr_reg40508, 308,por_int_buffer[20]);                // �������� ������� ������� �� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	set_sound(0);
	delay(100);
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void test_GG_Radio2()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(657, 39);
	set_sound(5);
	delay(1000);
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[52])));                   // " ****** Test Radio2 start! ******"                           ;
		myFile.println(buffer);                                                         // " ****** Test Radio2 start! ******"                           ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(7,0);                                                               // ���� RL3 ����  LFE  "���."
	resistor(1,por_int_buffer[1]);                                                              // ���������� ������� ������� 300 ��
	resistor(2,por_int_buffer[2]);                                                               // ���������� ������� ������� 300 ��
	UpdateRegs();                                                                   // ��������� �������
	delay(500);
	UpdateRegs(); 
	delay(100);
	//+++++++++++++++++++++++++++++++++++   �������� ���������� ������� �� ������� +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40310, adr_reg40510, 310,por_int_buffer[3]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40311, adr_reg40511, 311,por_int_buffer[5]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40312, adr_reg40512, 312,por_int_buffer[7]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40313, adr_reg40513, 313,por_int_buffer[9]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40314, adr_reg40514, 314,por_int_buffer[11]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40315, adr_reg40515, 315,por_int_buffer[13]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40316, adr_reg40516, 316,por_int_buffer[15]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40317, adr_reg40517, 317,por_int_buffer[17]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40318, adr_reg40518, 318,por_int_buffer[19]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(7,1);                                                               //  ���� RL3 ����  LFE  "���."
	UpdateRegs();                                                                   // ��������� �������
	delay(100);
	UpdateRegs();  

	//Serial.println("test_GG_Radio2 - on ");

	measure_vol_min(analog_FrontL,    adr_reg40310, adr_reg40510, 310,por_int_buffer[4]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40311, adr_reg40511, 311,por_int_buffer[6]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40312, adr_reg40512, 312,por_int_buffer[8]);                 // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40313, adr_reg40513, 313,por_int_buffer[10]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_max(analog_mag_radio, adr_reg40331, adr_reg40531, 331,por_int_buffer[29],por_int_buffer[30]);   // !!        // �������� ������� ������� �� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40315, adr_reg40515, 315,por_int_buffer[14]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40316, adr_reg40516, 316,por_int_buffer[16]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40317, adr_reg40517, 317,por_int_buffer[18]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, adr_reg40319, adr_reg40519, 319,por_int_buffer[37],por_int_buffer[38]);                // �������� ������� ������� �� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.set(7,0);                                                               // ���� RL6 ���� Center
	UpdateRegs();     
	set_sound(0);
	delay(500);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);    
}
void test_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{	
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[67])));                   // " ****** Test power start! ******"                           ;
		myFile.println(buffer);                                                         // " ****** Test power start! ******"                           ;
		file_print_date();
		myFile.println("");
	}
	//const unsigned int adr_reg40000      PROGMEM       = 40293);                         // A���� ��������  ������ ADC1 ���������� 12/3 �����
	//regBank.add(40294);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
	//regBank.add(40295);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
	//regBank.add(40296);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
	//regBank.add(40297);                         // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������

	//regBank.add(40493);                         // A���� ������ ��������� ADC1 ���������� 12/3 �����
	//regBank.add(40494);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
	//regBank.add(40495);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
	//regBank.add(40496);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
	//regBank.add(40497);                         // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������

	measure_power();

	// �������� ���������� 12 ����� ����� ��������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[61])));                   // "Power Kamerton V  - "                                        ;
	if(read_reg_eeprom(adr_reg40493)*2.61/100 < 11 || read_reg_eeprom(adr_reg40493)*2.61/100 >13)
	{
		if (test_repeat)                                          // ��������� � �����. ���� ���������
			{
				file_print_date();
				myFile.print ("  ");  
			}
		myFile.print(buffer);   
		myFile.print("Error! - "); 
		regBank.set(293,1); 
		regcount = read_reg_eeprom(adr_reg40293);
		regcount++;
		save_reg_eeprom(adr_reg40293,regcount); 
		myFile.print(regcount);
		myFile.print(" | power ");
		myFile.print(read_reg_eeprom(adr_reg40493)*2.61/100);
		myFile.print(regcount);
		regBank.set(120,1);  
		myFile.println("v / porog min 11v - max 13v");
	}

	else
	{
		if (test_repeat == false)
			{
				myFile.print(buffer);                               // 
				myFile.print("Pass | power ");
				myFile.print(read_reg_eeprom(adr_reg40493)*2.61/100);
				myFile.println("v / porog min 11v - max 13v");
			}
	}

	// �������� ���������� 12 ����� ����� 1
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[63])));                   // "Power Radio1 V    - "                                        ;
	if(read_reg_eeprom(adr_reg40494)*2.61/100 < 11 || read_reg_eeprom(adr_reg40494)*2.61/100 >13)
	{
		if (test_repeat)                                          // ��������� � �����. ���� ���������
			{
				file_print_date();
				myFile.print ("  ");  
			}
		myFile.print(buffer);   
		myFile.print("Error! - "); 
		regBank.set(294,1); 
		regcount = read_reg_eeprom(adr_reg40294);
		regcount++;
		save_reg_eeprom(adr_reg40294,regcount); 
		myFile.print(regcount);
		myFile.print(" | power ");
		myFile.print(read_reg_eeprom(adr_reg40494)*2.61/100);
		myFile.print(regcount);
		regBank.set(120,1);  
		myFile.println("v / porog min 11v - max 13v");
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                               // "Power Radio1 V    - "                                        ;
				myFile.print("Pass | power ");
				myFile.print(read_reg_eeprom(adr_reg40494)*2.61/100);
				myFile.println("v / porog min 11v - max 13v");
			}
	}

	// �������� ���������� 12 ����� ����� 2
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[64])));                   // "Power Radio2 V    - "                                        ;
	if(read_reg_eeprom(adr_reg40495)*2.61/100 < 11 || read_reg_eeprom(adr_reg40495)*2.61/100 >13)
	{
		if (test_repeat)                                          // ��������� � �����. ���� ���������
			{
				file_print_date();
				myFile.print ("  ");  
			}
		myFile.print(buffer);   
		myFile.print("Error! - "); 
		regBank.set(295,1); 
		regcount = read_reg_eeprom(adr_reg40295);
		regcount++;
		save_reg_eeprom(adr_reg40295,regcount); 
		myFile.print(regcount);
		myFile.print(" | power ");
		myFile.print(read_reg_eeprom(adr_reg40495)*2.61/100);
		myFile.print(regcount);
		regBank.set(120,1);  
		myFile.println("v / porog min 11v - max 13v");
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                               // "Power Radio2 V    - "                                        ;
				myFile.print("Pass | power ");
				myFile.print(read_reg_eeprom(adr_reg40495)*2.61/100);
				myFile.println("v / porog min 11v - max 13v");
			}
	}

	// �������� ���������� 12 �����  ���
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[65])));                   // "Power GGS    V    - "                                        ;
	if(read_reg_eeprom(adr_reg40496)*2.61/100 < 11 || read_reg_eeprom(adr_reg40496)*2.61/100 >13)
	{
		if (test_repeat)                                          // ��������� � �����. ���� ���������
			{
				file_print_date();
				myFile.print ("  ");  
			}
		myFile.print(buffer);   
		myFile.print("Error! - "); 
		regBank.set(296,1); 
		regcount = read_reg_eeprom(adr_reg40296);
		regcount++;
		save_reg_eeprom(adr_reg40296,regcount); 
		myFile.print(regcount);
		myFile.print(" | power ");
		myFile.print(read_reg_eeprom(adr_reg40496)*2.61/100);
		myFile.print(regcount);
		regBank.set(120,1);  
		myFile.println("v / porog min 11v - max 13v");
	}

	else
	{	
		if (test_repeat == false) 
		{
			myFile.print(buffer);                               // "Power GGS    V    - "                                        ;
			myFile.print("Pass | power ");
			myFile.print(read_reg_eeprom(adr_reg40496)*2.61/100);
			myFile.println("v / porog min 11v - max 13v");
		}
	}

	// �������� ���������� 3,6 ����� �� ���������� ���������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[66])));                   // "Power Led mic.V   - "  
	if(read_reg_eeprom(adr_reg40497)/100 < 2 || read_reg_eeprom(adr_reg40497)/100 >4)
	{
		if (test_repeat)                                          // ��������� � �����. ���� ���������
			{
				file_print_date();
				myFile.print ("  ");  
			}
		myFile.print(buffer);   
		myFile.print("Error! - "); 
		regBank.set(297,1); 
		regcount = read_reg_eeprom(adr_reg40297);
		regcount++;
		save_reg_eeprom(adr_reg40297,regcount); 
		myFile.print(regcount);
		myFile.print(" | power  ");
		myFile.print(read_reg_eeprom(adr_reg40497)/100);
		myFile.print(regcount);
		regBank.set(120,1);  
		myFile.println("v / porog min 2.0v - max 4.0v");
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                                 // "Power Led mic.V   - " 
				myFile.print("Pass | power  ");
				myFile.print(read_reg_eeprom(adr_reg40497)/100.0);
				myFile.println("v / porog min 2.0v - max 4.0v");
			}
	}
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);    
}
void test_video()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	if (test_repeat == false)
	{
		myFile.println(""); 
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[68])));                   // " ****** Test Adjusting the brightness of the display! ******"; 
		myFile.println(buffer);                                                         // " ****** Test Adjusting the brightness of the display! ******"; 
		file_print_date();
		myFile.println("");
	}
	regBank.set(40061,100);                                                         // ������� ������� 100
	while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	//delay(300);
	regs_out[0]= 0x2B;                                                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0x84;                                                              // 
	regs_out[2]= regBank.get(40061);                                                // ������� �������
	delay(100);
	while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	regs_out[0]= 0x2B;                                                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                                                              // 
	regs_out[2]= 0x7F;                                                              // ������� �������
	measure_mks();                                                                  // �������� ������������ ���������
	
	regBank.set(40062,regBank.get(40005));                                          // �������� ������� ������� � ���������

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[69])));                   // !!!!! 
	if (regBank.get(40062) != 50)                                                   //   
	{
		file_print_date();
		myFile.println("  ");  
		myFile.print(buffer);                                                       // 
		myFile.print(regBank.get(40062));
		myFile.println(" - Error!");
		regBank.set(329,1); 
		regcount = read_reg_eeprom(adr_reg40329);
		regcount++;
		save_reg_eeprom(adr_reg40329,regcount); 
		regBank.set(120,1);  
		save_reg_eeprom(adr_reg40529,regcount); 
	}

	else
	{
	if (test_repeat == false) 
		{
			myFile.print(buffer);                                                   // 
			myFile.print(regBank.get(40062));
			myFile.println(" - pass");
		}
	}

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[70])));                   // 
	if (regBank.get(40063) < 18 || regBank.get(40063) > 26)                         // �������� ��������� ������������ �������� �������
	{
		file_print_date();
		myFile.println("  ");  
		myFile.print(buffer);                                                       // 
		myFile.print(regBank.get(40063));
		myFile.println(" - Error!");
		regBank.set(269,1); 
		regcount = read_reg_eeprom(adr_reg40269);
		regcount++;
		save_reg_eeprom(adr_reg40269,regcount); 
		regBank.set(120,1);  
		save_reg_eeprom(adr_reg40469,regBank.get(40063));                   // �������� ������������ �������� � ��
	}

	else
	{
		if (test_repeat == false) 
			{
				myFile.print(buffer);                                               // 
				myFile.print(regBank.get(40063));
				myFile.println(" - pass");
			}
	}
	while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);    
}
void set_video()
{
	while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	//delay(300);
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0x84;                              // 
	regs_out[2]= regBank.get(40061);                // ������� �������
	delay(300);
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 
	regs_out[2]= 0x7F;                              // ������� �������
	measure_mks();                                  // �������� ������������ ���������
	regBank.set(40062,regBank.get(40005));          // �������� ������� ������� � ���������
	regs_out[0]= 0x2B;                              // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                              // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                              // 127 �������� � �������� �����
	regBank.set(adr_control_command,0);  
}
void measure_mks()
{
  unsigned long duration = 0;
  unsigned long duration1 = 0;
  duration = pulseIn(A12, HIGH);
	  for (int imp = 0;imp < 10; imp++)
	  {
	   duration = pulseIn(A12, HIGH);
	   duration1 += duration;
	  }
	  duration = duration1/10;
  //Serial.println(duration);
 save_reg_eeprom(adr_reg40469,duration); 	 
 regBank.set(40063,duration);                          // �������� ������������ �������� ������� � ���������
}
void set_radio_send()
{
	bool radio_on_off =	regBank.get(41);            // ���� ���������� ��������������
	while(prer_Kmerton_Run == true){}               // ���� ��������� ��������� ������ �� ��������

	if (radio_on_off)
	{
		bitSet(regs_out[1], 5);                     //
	}
	else
	{
		bitClear(regs_out[1], 5);
	}
	regBank.set(adr_control_command,0);  
}
void set_ggs_mute()
{
	bool ggs_mute = regBank.get(42);                // ���� ��� (mute)
	while(prer_Kmerton_Run == true){}               // ���� ��������� ��������� ������ �� ��������
	if (ggs_mute)
	{
		bitSet(regs_out[1], 4);                     //
	}
	else
	{
		bitClear(regs_out[1], 4);
	}
	regBank.set(adr_control_command,0);  
}
void set_radio_send1()
{
	bool radio_on_off =	regBank.get(41);            // ���� ���������� ��������������
	while(prer_Kmerton_Run == true){}               // ���� ��������� ��������� ������ �� ��������

	if (radio_on_off)
	{
		bitSet(regs_out[1], 5);                     //
	}
	else
	{
		bitClear(regs_out[1], 5);
	}
}
void set_ggs_mute1()
{
	bool ggs_mute = regBank.get(42);                // ���� ��� (mute)
	while(prer_Kmerton_Run == true){}                  // ���� ��������� ��������� ������ �� ��������
	if (ggs_mute)
	{
		bitSet(regs_out[1], 4);                     //
	}
	else
	{
		bitClear(regs_out[1], 4);
	}
}

void test_instr_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[7])));                    // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"                   ;  
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  ��������� ������ �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  ��������� ������ ����������� c 2  ����������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls ��������� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(100);
	UpdateRegs(); 
 
	byte i52 = regBank.get(40006);     
	 
	  // 1)  �������� ������� �� ���������� ��������� ����������� 2 ����������
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40203);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			save_reg_eeprom(adr_reg40203,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(203,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 ��������  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = read_reg_eeprom(adr_reg40204);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			save_reg_eeprom(adr_reg40204,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(204,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = read_reg_eeprom(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40207,regcount);                                            // ����� �������� ������ ������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40220);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40220,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(220,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	//   delay(100);
}
void test_instr_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[10])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor    send!"
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	regBank.set(16,1);                                                              // XS1 - 6   sensor ���
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	regBank.set(13,1);                                                              // XP8 - 2           sensor �������� ������
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(100);
	UpdateRegs(); 
 
	byte i52 = regBank.get(40006);     

	  // 3)  �������� ������� �� ����������� ��������� ����������� 2 ����������
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40213);                                          // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������� � 2 ����������
			save_reg_eeprom(adr_reg40213,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� � 2 ����������
			regBank.set(213,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 �������  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
		  {
			regcount = read_reg_eeprom(adr_reg40214);                                          // ����� �������� ������ sensor ����������� ��������� �����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� �����������
			save_reg_eeprom(adr_reg40214,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������� 
			regBank.set(214,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor �������  - Pass
			  }
		  }

		UpdateRegs(); 
	
	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT �����������   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40221);                                          // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40221,regcount);                                            // ����� �������� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(221,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ����������� "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  �������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	//   delay(100);
}

void test_disp_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[16])));                   // "Command sensor OFF headset instructor            send!"                   ; // OK   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor            send!"     
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    ��������� ������ ��������� ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   ��������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(100);
	UpdateRegs(); 
  
	byte i52 = regBank.get(40006);     
	
	  // 1)  �������� ������� �� ���������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40205);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			save_reg_eeprom(adr_reg40205,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(205,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " ��������  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  ����������� ��������� ����������
		  {
			regcount = read_reg_eeprom(adr_reg40206);                                          // ����� �������� ������ sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			save_reg_eeprom(adr_reg40206,regcount);                                            // ����� �������� ������ sensor ����������� ��������� ����������
			regBank.set(206,1);                                                     // ���������� ���� ������ sensor ����������� ��������� ����������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " ��������  - Pass
			  }
		  }

	 // 3)  �������� ������� �� ���������� ���������

		if(bitRead(i52,5) != 0)                                                     // ��������  ����� �� ���������� ���������
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // ����� �������� ������ ������� ��������� 
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40207,regcount);                                            // ����� �������� ������ ������� ���������
			regBank.set(207,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   ��������  - Pass
			   }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // ��������� ���������� PTT ����������   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40222);                                          // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40222,regcount);                                            // ����� ��������   ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(222,1);                                                     // ���������� ����  ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  ��������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
}
void test_disp_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[19])));                   // "Command sensor ON  headset dispatcher 2          send!"                   ;   
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher 2          send!"                   ;    
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor ����������� ��������� ���������� 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  ����. �.
	regBank.set(16,1);                                                              // XS1 - 6   sensor ���
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor ����������� ������      
	regBank.set(13,1);                                                              // XP8 - 2           sensor �������� ������
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   �������� PTT ����������
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // ��������� ������� ��������� ��������
	delay(100);
	UpdateRegs(); 

	byte i52 = regBank.get(40006);     

	  // 3)  �������� ������� �� ����������� ��������� ���������� 2 ����������
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor ����������� ��������� ���������� � 2 ����������
		  {
			regcount = read_reg_eeprom(adr_reg40215);                                          // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ���������� � 2 ����������
			save_reg_eeprom(adr_reg40215,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� � 2 ����������
			regBank.set(215,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� � 2 ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  �������  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor ����������� ��������� ���������� 
		  {
			regcount = read_reg_eeprom(adr_reg40216);                                          // ����� �������� ������    sensor ����������� ��������� ����������
			regcount++;                                                             // ��������� ������� ������ sensor ����������� ��������� ����������
			save_reg_eeprom(adr_reg40216,regcount);                                            // ����� �������� ������    sensor ����������� ��������� ���������� 
			regBank.set(216,1);                                                     // ���������� ���� ������   sensor ����������� ��������� ���������� 
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " �������  - Pass
			  }
		  }

		UpdateRegs(); 
		//wdt_reset();
	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // ��������� ��������� PTT ����������   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40223);                                          // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40223,regcount);                                            // ����� �������� ������  ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(223,1);                                                     // ���������� ���� ������ ���������� PTT ��������� ���������� "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  �������  - Pass
		   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	 //  delay(100);
}

void test_MTT_off()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	unsigned int regcount = 0;
	regBank.set(25,1);                                                              // "Command sensor OFF MTT                           send! "     ;     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                   // "Command sensor OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF MTT                           send! "     ;      
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS  ���. ��������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
	regBank.set(16,0);                                                              // XS1 - 6   sensor ���
	regBank.set(1,0);                                                               // ���� RL0 ����
	regBank.set(2,0);                                                               // ���� RL1 ����
	regBank.set(3,0);                                                               // ���� RL2 ����
	regBank.set(4,0);                                                               // ���� RL3 ����  LFE  "���."
	regBank.set(5,0);                                                               // ���� RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // ���� RL5 ����
	regBank.set(7,0);                                                               // ���� RL6 ����
	regBank.set(9,0);                                                               // ���� RL8 ���� �� ��������
	regBank.set(10,0);                                                              // ���� RL9 XP1 10
	UpdateRegs();                                                                   // ��������� ������� ���������� ��������
	delay(100);
	UpdateRegs(); 
	delay(100);
	byte i50 = regBank.get(40004);    

	//wdt_reset();
		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ ��������  - Pass
			   }
		  }
		   UpdateRegs(); 
		   delay(100);

	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // ��������  �� ���������� CTS MTT
		  {
			regcount = read_reg_eeprom(adr_reg40263);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40263,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regBank.set(263,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));    // "Test MTT PTT    (CTS)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT PTT    (CTS)                                       OFF - ";
			   }                   
		  }
		//wdt_reset();
	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // ��������� ��������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                          // ����� �������� ������ ���������� HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40267,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));    // "Test MTT HangUp (DCD)                                       OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Test MTT HangUp (DCD)                                       OFF - ";
			   }             
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	//   delay(100);
}
void test_MTT_on()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	delay(600);
	unsigned int regcount = 0;
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor ����������� ������    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	regBank.set(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR ���. �������� PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(100);
	UpdateRegs();

	  // 1)  �������� ������� MTT �� ��������� 
	byte i50 = regBank.get(40004);    

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor �������� ����������� ������    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // ��������� ������� ������ sensor ���������� ������  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // ����� �������� ������                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // ���������� ���� ������                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  ������ �������  - Pass
			   }
		  }

		delay(100);
		UpdateRegs(); 

	  // 2)  ��������  �� ���������� PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // ��������  �� ��������� CTS MTT
		  {
			regcount = read_reg_eeprom(adr_reg40265);                                          // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40265,regcount);                                            // ����� �������� ������ PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regBank.set(265,1);                                                     // ���������� ���� ������
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));    // "Test MTT PTT    (CTS)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " ������ �������  - Pass
			   }
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                         // ��������� ��������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40268);                                          // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // ��������� ������� ������
			save_reg_eeprom(adr_reg40268,regcount);                                            // ����� �������� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                     // ���������� ���� ������ ���������� HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                     // ���������� ����� ���� ������
			regcount_err = regBank.get(adr_reg_count_err);                          // �������� ������ �������� ���� ������
			regcount_err++;                                                         // ��������� ������� ���� ������ 
			regBank.set(adr_reg_count_err,regcount_err);                            // ��������� ������ �������� ���� ������
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // ��������� �������� ������
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";������ �������  - Pass
			   }
		 }
	   mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	//   delay(100);
}

void measure_vol_min(int istochnik, unsigned int adr_count, unsigned int adr_count200 , int adr_flagErr, unsigned int porogV)
{
		mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
		int _istochnik             = istochnik;
		unsigned int _adr_count    = adr_count;
		unsigned int _adr_count200 = adr_count200;
		int _adr_flagErr           = adr_flagErr;
		unsigned int _porogV       = porogV;
		float _porogVF             = porogV;  
		unsigned int regcount      = 0;
		measure_volume(_istochnik);                                                 // �������� ������� ������� �� ������
		switch (_adr_flagErr) 
		{
			case 230:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[30])));    // "Test headset instructor ** Signal FrontL                    OFF - ";
				break;
			case 231:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[31])));    // "Test headset instructor ** Signal FrontR                    OFF - ";
				break;
			case 232:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[32])));    // "Test headset instructor ** Signal LineL                     OFF - ";
				break;
			case 233:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[33])));    // "Test headset instructor ** Signal LineR                     OFF - ";
				break;
			case 234:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[34])));    // "Test headset instructor ** Signal mag radio                 OFF - ";
				break;
			case 235:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[35])));    // "Test headset instructor ** Signal mag phone                 OFF - ";
				break;
			case 236:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[36])));    // "Test headset instructor ** Signal GGS                       OFF - ";
				break;
			case 237:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[37])));    // "Test headset instructor ** Signal GG Radio1                 OFF - ";
				break;
			case 238:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[38])));    // "Test headset instructor ** Signal GG Radio2                 OFF - ";
				break;
			case 240:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[40])));    // "Test headset dispatcher ** Signal FrontL                    OFF - ";
				break;
			case 241:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[41])));    // "Test headset dispatcher ** Signal FrontR                    OFF - ";
				break;
			case 242:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[42])));    // "Test headset dispatcher ** Signal LineL                     OFF - ";
				break;
			case 243:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[43])));    // "Test headset dispatcher ** Signal LineR                     OFF - ";
				break;
			case 244:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[44])));    // "Test headset dispatcher ** Signal mag radio                 OFF - ";
				break;
			case 245:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[45])));    // "Test headset dispatcher ** Signal mag phone                 OFF - ";
				break;
			case 246:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[46])));    // "Test headset dispatcher ** Signal GGS                       OFF - ";
				break;
			case 247:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[47])));    // "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
				break;
			case 248:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[48])));    // "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
				break;
			case 250:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[50])));    //  "Test MTT ** Signal FrontL                                   OFF - ";
				break;
			case 251:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[51])));    // "Test MTT ** Signal FrontR                                   OFF - ";
				break;
			case 252:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[52])));    // "Test MTT ** Signal LineL                                    OFF - ";
				break;
			case 253:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[53])));    // "Test MTT ** Signal LineR                                    OFF - ";
				break;
			case 254:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[54])));    // "Test MTT ** Signal mag radio                                OFF - ";
				break;
			case 255:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[55])));    // "Test MTT ** Signal mag phone                                OFF - ";
				break;
			case 256:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[56])));    // "Test MTT ** Signal GGS                                      OFF - ";
				break;
			case 257:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[57])));    // "Test MTT ** Signal GG Radio1                                OFF - ";
				break;
			case 258:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[58])));    // "Test MTT ** Signal GG Radio2                                OFF - ";
				break;
			case 280:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ������ "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ������ "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ������ "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ������ "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ������ "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ������ "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ������ "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ������ "Test GGS ** Signal GG Radio1                                OFF - ";
				break;
			case 288:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[88])));    // "Test GGS ** Signal GG Radio2                                OFF - ";
				break;
			case 300:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[100])));    //  "Test Radio1 ** Signal FrontL                                OFF - ";
				break;
			case 301:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[101])));    // "Test Radio1 ** Signal FrontR                                OFF - ";
				break;
			case 302:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[102])));    // "Test Radio1 ** Signal LineL                                 OFF - ";
				break;
			case 303:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[103])));    // "Test Radio1 ** Signal LineR                                 OFF - ";
				break;
			case 304:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[104])));    // "Test Radio1 ** Signal mag radio                             OFF - ";
				break;
			case 305:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[105])));    // "Test Radio1 ** Signal mag phone                             OFF - ";
				break;
			case 306:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[106])));    // "Test Radio1 ** Signal GGS                                   OFF - ";
				break;
			case 307:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[107])));    // "Test Radio1 ** Signal GG Radio1                             OFF - ";
				break;
			case 308:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[108])));    // "Test Radio1 ** Signal GG Radio2                             OFF - ";
				break;
			case 310:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[110])));    //  "Test Radio2 ** Signal FrontL                                OFF - ";
				break;
			case 311:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[111])));    // "Test Radio2 ** Signal FrontR                                OFF - ";
				break;
			case 312:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[112])));    // "Test Radio2 ** Signal LineL                                 OFF - ";
				break;
			case 313:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[113])));    // "Test Radio2 ** Signal LineR                                 OFF - ";
				break;
			case 314:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[114])));    // "Test Radio2 ** Signal mag radio                             OFF - ";
				break;
			case 315:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[115])));    // "Test Radio2 ** Signal mag phone                             OFF - ";
				break;
			case 316:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[116])));    // "Test Radio2 ** Signal GGS                                   OFF - ";
				break;
			case 317:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[117])));    // "Test Radio2 ** Signal GG Radio1                             OFF - ";
				break;
			case 318:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[118])));    // "Test Radio2 ** Signal GG Radio2                             OFF - ";
				break;
			case 320:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[120])));    //  "Test Microphone ** Signal FrontL                                   OFF - ";
				break;
			case 321:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[121])));    // "Test Microphone ** Signal FrontR                                   OFF - ";
				break;
			case 322:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[122])));    // "Test Microphone ** Signal LineL                                    OFF - ";
				break;
			case 323:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[123])));    // "Test Microphone ** Signal LineR                                    OFF - ";
				break;
			case 324:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[124])));    // "Test Microphone ** Signal mag radio                                OFF - ";
				break;
			case 325:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[125])));    // "Test Microphone ** Signal mag phone                                OFF - ";
				break;
			case 326:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[126])));    // "Test Microphone ** Signal GGS                                      OFF - ";
				break;
			case 327:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[127])));    // "Test Microphone ** Signal GG Radio1                                OFF - ";
				break;
			case 328:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[128])));    // "Test Microphone ** Signal GG Radio2                                OFF - ";
				break;

		}
		//Serial.println(">");
		save_reg_eeprom(_adr_count200,voltage10);                                     // ��������� ��������� ��������� V
		if(voltage10 > _porogV)                                                      // ��������� ����������� ������
			{
				if (test_repeat)                                          // ��������� � �����. ���� ���������
				{
					file_print_date();
					myFile.print ("  ");  
				}
				myFile.print(buffer); 
				regcount = read_reg_eeprom(_adr_count);                              // ����� �������� ������ 
				regcount++;                                                          // ��������� ������� ������ ������ 
				save_reg_eeprom(_adr_count,regcount);                                // ����� �������� ������ ������ 
				regBank.set(_adr_flagErr,1);                                         // ���������� ���� ������  ������ 
				regcount_err = regBank.get(adr_reg_count_err);                       // �������� ������ �������� ���� ������
				regcount_err++;                                                      // ��������� ������� ���� ������ 
				regBank.set(adr_reg_count_err,regcount_err);                         // ��������� ������ �������� ���� ������
				regBank.set(120,1);                                                  // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // ��������� �������� ������
				myFile.print(" | signal ");  
				myFile.print(voltage); 
				myFile.print("v");
				myFile.print(" / porog min ");  
				myFile.print(_porogVF/100);
				myFile.println("v");
			}
		else
			{
				if (test_repeat == false)
				{
					myFile.print(buffer);                                           // ������������ ��������
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print(" | signal ");  
					myFile.print(voltage); 
					myFile.print("v");
					myFile.print(" / porog min ");  
					myFile.print(_porogVF/100);
					myFile.println("v");
				}
			}  
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	//delay(100);
}
void measure_vol_max(int istochnik, unsigned int adr_count, unsigned int adr_count200 , int adr_flagErr, unsigned int porogV,unsigned int porogVmax )
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	int _istochnik                   = istochnik;
	unsigned int _adr_count          = adr_count;
	unsigned int _adr_count200       = adr_count200;
	int _adr_flagErr                 = adr_flagErr;
	unsigned int _porogV             = porogV;
	unsigned int _porogVmax          = porogVmax;
	float _porogVF                   = porogV;  
	float _porogVFmax                = porogVmax; 
	unsigned int regcount            = 0;

	measure_volume(_istochnik);                                                     // �������� ������� ������� �� ������
	switch (_adr_flagErr) 
		{
			case 224:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[24])));    // "Test headset instructor ** Signal LineL                     ON  - ";
				break;
			case 225:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[25])));    // "Test headset instructor ** Signal LineR                     ON  - "; 
				break;
			case 226:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[26])));    // "Test headset instructor ** Signal Mag phone                 ON  - ";
				break;
			case 227:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[27])));    // "Test headset dispatcher ** Signal LineL                     ON  - ";
				break;
			case 228:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[28])));    // "Test headset dispatcher ** Signal LineR                     ON  - "; 
				break;
			case 229:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[29])));    // "Test headset dispatcher ** Signal Mag phone                 ON  - ";
				break;
			case 259:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[59])));    // "Test MTT ** Signal GGS                                      ON  - ";
				break;
			case 260:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[60])));    // "Test MTT ** Signal LineL                                    ON  - ";
				break;
			case 261:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[61])));    // "Test MTT ** Signal LineR                                    ON  - ";  
				break;
			case 262:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[62])));    // "Test MTT ** Signal Mag phone                                ON  - ";
				break;
			case 289:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ������ "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ������ "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ������ "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ������ "Test GGS ** Signal mag phone                                ON  - ";
				break;
			case 298:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[98])));    // "Test Microphone ** Signal mag phone                         ON  - ";      
				break;
			case 299:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[99])));    // "Test Microphone ** Signal LineL                             ON  - ";  
				break;
			case 309:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[109])));   // "Test Radio1 ** Signal Radio1                                ON  - ";
				break;
			case 319:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[119])));   // "Test Radio1 ** Signal Radio2                                ON  - ";
				break;
			case 330:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[130])));   // "Test Radio1 ** Signal mag radio                              ON  - ";
				break;
			case 331:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[131])));   // "Test Radio2 ** Signal mag radio                              ON  - ";
				break;
			case 332:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[132])));   // "Test GGS ** Signal mag radio                                 ON  - ";
				break;
			case 333:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[133])));   // "Test headset instructor ** Signal mag radio                  ON  - ";
				break;
			case 334:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[134])));   // "Test headset dispatcher ** Signal mag radio                  ON  - ";
				break;
			case 335:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[135])));   // "Test MTT ** Signal mag radio                                 ON  - ";
				break;
			case 336:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[136])));   // "Test Microphone ** Signal mag radio                          ON  - ";
				break;
		}

		save_reg_eeprom(_adr_count200,voltage10);                                   // ����� ������ ������ ������ 
		if(voltage10 < _porogV || voltage10 > _porogVmax )                          // ��������� ����������� ������
			   {
				if (test_repeat)                                          // ��������� � �����. ���� ���������
				{
				  file_print_date();
				  myFile.print ("  ");  
				}
				myFile.print(buffer); 
				regcount = read_reg_eeprom(_adr_count);                             // ����� �������� ������ 
				regcount++;                                                         // ��������� ������� ������ ������ 
				save_reg_eeprom(_adr_count,regcount);                               // ����� �������� ������ ������ 
				regBank.set(_adr_flagErr,1);                                        // ���������� ���� ������  ������ 
				regcount_err = regBank.get(adr_reg_count_err);                      // �������� ������ �������� ���� ������
				regcount_err++;                                                     // ��������� ������� ���� ������ 
				regBank.set(adr_reg_count_err,regcount_err);                        // ��������� ������ �������� ���� ������
				regBank.set(120,1);                                                 // ���������� ����� ���� ������ 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // ��������� �������� ������
				myFile.print(" | signal ");  
				myFile.print(voltage); 
				myFile.print("v");
				myFile.print(" / porog min ");  
				myFile.print(_porogVF/100);
				myFile.print("v - max ");  
				myFile.print(_porogVFmax/100);
				myFile.println("v");
			}
		else
			{
			if (test_repeat == false)                                          // ��������� � �����. ���� ���������
				{
					myFile.print(buffer);                                           // ������������ ��������
					strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));    // "Pass";
					myFile.print(buffer);                                           // "Pass";
					myFile.print(" | signal ");  
					myFile.print(voltage); 
					myFile.print("v");
					myFile.print(" / porog min ");  
					myFile.print(_porogVF/100);
					myFile.print("v - max ");  
					myFile.print(_porogVFmax/100);
					myFile.println("v");
				}
			} 
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
}
void measure_volume(int analog)
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	volume1     = 0;
	unsigned long	volume_maxx = 0;
	unsigned long   volume_minx = 0;
	int stix=100;
	bool strobe_sound = 0;
	int i_stop = 254;


	if (var_sound)// ������� ������� ������� �����
	{
		do                           //�������� ������ ������
			{
				strobe_sound = digitalRead(InNano12);
			} while(strobe_sound == LOW);    
		do                          //�������� ����� ������
			{
				strobe_sound = digitalRead(InNano12);
			}  while(strobe_sound == HIGH); 


		 for (int sti = 0;sti<= stix; sti++)
		  {
			volume_max  = 0;
			volume_min  = 1023;
			volume_fact = 0;
			int i;
	
			for (i = 0;i<= i_stop; i++)  //i_stop
				{
					Array_volume[i] = analogRead(analog);               // ��������� ��������
				}

			for (i = 0; i<= i_stop; i++)
				{
					volume_max = max(volume_max, Array_volume[i]);
					volume_min = min(volume_min, Array_volume[i]);
				}
				volume_maxx += volume_max;
				volume_minx += volume_min;
		   }


	}
	else
	{
	
	 for (int sti = 0;sti<= stix; sti++)
	  {
		volume_max  = 0;
		volume_min  = 1023;
		volume_fact = 0;
		int i;

		for (i = 0;i<= i_stop; i++)  //i_stop
			{
				Array_volume[i] = analogRead(analog);               // ��������� ��������
			}

		for (i = 0; i<= i_stop; i++)
			{
				volume_max = max(volume_max, Array_volume[i]);
				volume_min = min(volume_min, Array_volume[i]);
			}
			volume_maxx += volume_max;
			volume_minx += volume_min;
	   }


	}

			//PORTB = B01000000; // ��� 12 ��������� � ��������� HIGH
			//delay(5);
			//PORTB = B00000000; // ��� 12 ��������� � ��������� LOW
		volume_fact = (volume_maxx/stix) - (volume_minx/stix);
		voltage = volume_fact * (5.0 / 1023.0);
		if(analog == analog_FrontL )    voltage = volume_fact * (5.0 / 1023.0 / 5);
		if(analog == analog_FrontR )    voltage = volume_fact * (5.0 / 1023.0 / 5);
		if(analog == analog_LineL )     voltage = volume_fact * (5.0 / 1023.0 / 2);
		if(analog == analog_LineR )     voltage = volume_fact * (5.0 / 1023.0 / 2);
		if(analog == analog_mag_radio ) voltage = volume_fact * (5.0 / 1023.0 / 2);
		if(analog == analog_mag_phone ) voltage = volume_fact * (5.0 / 1023.0 / 2);
		if(analog == analog_ggs )     voltage = volume_fact * (5.0 / 1023.0 * 2);
		if(analog == analog_gg_radio1 ) voltage = volume_fact * (5.0 / 1023.0 * 2);
		if(analog == analog_gg_radio2 ) voltage = volume_fact * (5.0 / 1023.0 * 2);

		voltage10 = voltage * 100;
		//Serial.print("volume_max - ");
		//Serial.print((volume_maxx/stix) * (5.0 / 1023.0));
		//Serial.print("- volume_min - ");
		//Serial.print((volume_minx /stix) * (5.0 / 1023.0));
		//Serial.print(" = ");
		//Serial.println(((volume_maxx/stix) -(volume_minx /stix)) * (5.0 / 1023.0));
		//Serial.print("voltage - ");
		//Serial.println(voltage10);
		mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
		delay(100);
}
void measure_volume_P(int analog)
{
		volume_fact = 0;
		volume_fact = analogRead(analog);               // ��������� ��������
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;
}
void measure_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	regBank.set(21,0);                           // XP2-2     sensor "���."  
	regBank.set(22,0);                           // XP5-3     sensor "��C."
	regBank.set(23,0);                           // XP3-3     sensor "��-�����1."
	regBank.set(24,0);                           // XP4-3     sensor "��-�����2."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_tok);     
	save_reg_eeprom(adr_reg40400,voltage10);                     
	measure_volume_P(analog_12V);   
	save_reg_eeprom(adr_reg40493,voltage10);   
	measure_volume_P(analog_tok_x10);   
	save_reg_eeprom(adr_reg40402,voltage10);   

	regBank.set(23,1);                           // XP3-3     sensor "��-�����1."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40494,voltage10);   

	regBank.set(23,0);                           // XP3-3     sensor "��-�����1."
	regBank.set(24,1);                           // XP4-3     sensor "��-�����2."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40495,voltage10);   

	regBank.set(24,0);                           // XP4-3     sensor "��-�����2."
	regBank.set(22,1);                           // XP5-3     sensor "��C."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40496,voltage10);   
	regBank.set(22,0);                           // XP5-3     sensor "��C."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_3_6);     
	save_reg_eeprom(adr_reg40497,voltage10);   
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	regBank.set(adr_control_command,0);
	//Serial.println(regBank.get(adr_control_command));
}

void set_sound_oscill()
{
	set_sound(regBank.get(40041));
	regBank.set(adr_control_command,0);
}
void set_sound(byte cannel_sound)
{
	 if (cannel_sound<4) var_sound = true;
	 else var_sound = false;   // ������� ������� ������� �����
	digitalWrite(kn1Nano, !bitRead(cannel_sound, 0));
	digitalWrite(kn2Nano, !bitRead(cannel_sound, 1));
	digitalWrite(kn3Nano, !bitRead(cannel_sound, 2));
}

void read_reg_error()
{
	 //regBank.add(40150);  // ���� ��� �������� ����������
	unsigned int res_eeprom;
	unsigned int _adr_reg  = regBank.get(40124);                         // ����� ����� ��������� ��� �������� � �� ����������� ������. 
	unsigned int _adr_mem  = regBank.get(40125);                         // ����� ����� ������ ��� �������� � �� ����������� ������.
	int _step_mem          = regBank.get(40126);                         // ����� ����� ����� ������ ��� �������� � �� ����������� ������.
	int i_k                = 0;                                                   // �������� ������ ����� ������
	//Serial.println("");
	//Serial.println(_adr_reg);
	//Serial.println(_adr_mem);
	//Serial.println(_step_mem);


	for (int i = 0; i < _step_mem;i++)                                   // ����������� ����� ������ � ��������.        
		{
			hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem + i_k);    // 
			i_k++;
			low = i2c_eeprom_read_byte(deviceaddress,_adr_mem + i_k);
			i_k++;
			res_eeprom = (hi<<8) | low; 
			regBank.set(_adr_reg+40000+i,res_eeprom);                    //regBank.add(40150);  //  �������� ��������� ������
		}

	//for (int i = 0; i < _step_mem;i++)
	//{
	//		Serial.print(_adr_reg+40000+i);
	//		Serial.print(" - ");
	//		Serial.println(regBank.get(_adr_reg+40000+i));
	//}

	regBank.set(adr_control_command,0);                                  // ��������� ���������    
}
void save_reg_error()
{
	//regBank.add(40150);                                     // ���� ��� �������� ����������
	unsigned int _adr_reg  = regBank.get(40124);              // ��������� ����� ����� ���������, 
	unsigned int _adr_mem  = regBank.get(40125);              // ��������� ����� ����� ������
	unsigned int _step_mem = regBank.get(40126);              // ����� ����� � ������ �������� ����������� �����
	unsigned int _u_count  = 0;                               // ��������� �������� ����������� ��������.
	unsigned int i_k       = 0;                                        // �������� ������ ����� ������

	for (unsigned int i = 0; i < _step_mem;i++)                        // ����������� ����� ������ � ��������.        
		{
			_u_count = regBank.get(_adr_reg+40000+i);
			// ��������� _u_count �� byte
			hi=highByte(_u_count);
			low=lowByte(_u_count);
			// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, low); 
			i_k++;
		}

	regBank.set(adr_control_command,0);                        // ��������� ���������    
	delay(200);

}

int read_reg_eeprom(unsigned int adr)
{
	unsigned int res_eeprom;
	hi  = i2c_eeprom_read_byte(deviceaddress,adr);                // 
	low = i2c_eeprom_read_byte(deviceaddress,adr+1);
	res_eeprom = (hi<<8) | low; 
	return res_eeprom;
}
void save_reg_eeprom(unsigned int adr,unsigned int res)
{
			hi=highByte(res);
			low=lowByte(res);
			// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,adr, hi); 
			i2c_eeprom_write_byte(deviceaddress,adr+1, low); 
}

void clear_reg_eeprom()
{
	for (int i = 1023; i < 1870;i++)                            // �������� ����� ��������� � ������.        
		{
			i2c_eeprom_write_byte(deviceaddress,i, 0); 
		}

}

void i2c_test()
{ 
	/*
	
	Serial.println("--------  EEPROM Test  ---------");
	char somedata[] = "this data from the eeprom i2c"; // data to write
	i2c_eeprom_write_page(0x50, 0, (byte *)somedata, sizeof(somedata)); // write to EEPROM 
	delay(100); //add a small delay
	Serial.println("Written Done");    
	delay(10);
	Serial.print("Read EERPOM:");
	byte b = i2c_eeprom_read_byte(0x50, 0); // access the first address from the memory
	char addr=0; //first address
	
	while (b!=0) 
	{
	  Serial.print((char)b); //print content to serial port
	  if (b!=somedata[addr]){
	   e1=0;
	   break;
	   }      
	  addr++; //increase address
	  b = i2c_eeprom_read_byte(0x50, addr); //access an address from the memory
	}
	 Serial.println();
	 */
}

void i2c_test1()
{
		int _start1     = regBank.get(40124);  //  ����� ������ ����� EEPROM ��� �������� � �� �����������.
		int _end1       = regBank.get(40125);  //  ����� ����� ����� ������ ��� �������� � �� �����������.
		int _start2     = regBank.get(40124);  //  ����� ������ ����� EEPROM ��� ������ � �� �����������.
		int _end2       = regBank.get(40125);  //  ����� ����� ����� ������ ��� ������ � �� �����������.

		int _u_porog  = 0;  
		for (int i    = _start1; i < _end1;i++)                            // ����������� ����� ������ � ��������.        
			{
				_u_porog = i;
				// ��������� _u_porog �� byte
				hi=highByte(_u_porog);
				low=lowByte(_u_porog);
				// ��� �� ��� hi,low ����� ��������� EEPROM
				i2c_eeprom_write_byte(deviceaddress,i, hi); 
				i++;
				i2c_eeprom_write_byte(deviceaddress,i, low); 
			}

		for (int i = _start2; i < _end2;i++)                            // ����������� ����� ������ � ��������.        
			{

			  hi  = i2c_eeprom_read_byte(deviceaddress,i);   // 
			  i++;
			  low = i2c_eeprom_read_byte(deviceaddress,i);
			   _u_porog = (hi<<8) | low;                              // �������� ��� "��������� �����������"
 
			   Serial.print(i-1);
			   Serial.print(" - ");   
			   Serial.println(_u_porog);
			 }
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
}
void i2c_test_int()
{
		int i_k       = 0;      
		int _u_porog  = 0;  
		for (int i = 0; i < 267;i++)                            // ����������� ����� ������ � ��������.        
		{
			_u_porog = i+1;

		//	Serial.println(_u_porog);
		   // ��������� _u_porog �� byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,i_k+201, low); 
			i_k++;
		}

		i_k       = 0;  
		for (int i = 0; i < 267;i++)                            // ����������� ����� ������ � ��������.        
		{

		  hi  = i2c_eeprom_read_byte(deviceaddress,i_k+201);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,i_k+201);
		  i_k++;
		   _u_porog = (hi<<8) | low;                              // �������� ��� "��������� �����������"
 /*
		   Serial.print(i+201);
		   Serial.print(" - ");   
		   Serial.println(_u_porog);*/
		 }

}

void test_RS232()
{
	regBank.set(adr_control_command,0);                                         // ��������� ���������    
	delay(100);
}
void set_USB0()
{
	 // ��������!
	 // true  - �������� ���� � ��������� RS232 
	 // false - �������� ���� � ��������� USB 
	
	  byte usb_rs232 = i2c_eeprom_read_byte(deviceaddress, adr_set_USB);    // ���� ������������ USB - RS232 

	  if (usb_rs232 == 0)
		  {
			  regBank.set(12,false);
			  mcp_Out1.digitalWrite(11, false);                             //  ���� ������������ USB - RS232 
			  Serial.println("Serial 2 - USB");
		  }
	  else
		  {
			 regBank.set(12,true);
			 mcp_Out1.digitalWrite(11, true);                               //  ���� ������������ USB - RS232 
			 Serial.println("Serial 2 - RS232");
		  }
	regBank.set(adr_control_command,0);                                     // ��������� ���������    
	delay(100);
}

void read_porog_eeprom(int adr_eeprom, int step_mem )                       // 
{
	int _adr_eeprom = adr_eeprom;                                           //
	int _step_mem = step_mem;                                               // 
	int _u_porog = 0;                                                       // ��������� �������� ����������� ��������.
	int i_eeprom = 0;                                                       // �������� ������ ����� ������

	for (int i = 1; i < _step_mem;i++)                                      // ����������� ����� ������ � ��������.        
		{
		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_eeprom+i_eeprom);   // 
		  i_eeprom++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_eeprom+i_eeprom);
		  i_eeprom++;
		   _u_porog = (hi<<8) | low;                                        // �������� ��� "��������� �����������"
		  por_int_buffer[i] = _u_porog;
		  //Serial.print(i);
		  //Serial.print(" - ");
		  //Serial.println(por_int_buffer[i]);
		 }
		 
}

void default_mem_porog()                                                    // ������ ��������� ��������� ������� ������
{
	int _step_mem = 266;                                                    // ����� ����� � ������ �������� ����������� �����
	int _u_porog  = 0;                                                      // ��������� �������� ����������� ��������.
	int i_k       = 0;                                                      // �������� ������ ����� ������
	for (int i = 0; i < _step_mem;i++)                    
		{
			_u_porog = pgm_read_word_near(porog_default+i);
		   // ��������� _u_porog �� byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,i_k+201, low); 
			i_k++;
			//Serial.print(i);
			//Serial.print(" - ");
			//Serial.println(_u_porog);
		}
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}
// ������ � ������ ���������� ���� ������� ����
void set_mem_porog()
{
	/*
		��������� ������ ������� � EEPROM
		��������� ����� ������ 200
		��������� ����� ��������� 40130 
		����� ����� �� ����� ??? ����
		regBank.get(40127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
		regBank.get(40128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
		regBank.get(40129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.
	*/
	int _adr_reg  = regBank.get(40127);              // ��������� ����� ����� ���������, 
	int _adr_mem  = regBank.get(40128);              // ��������� ����� ����� ������
	int _step_mem = regBank.get(40129);              // ����� ����� � ������ �������� ����������� �����
	int _u_porog  = 0;                               // ��������� �������� ����������� ��������.
	int i_k       = 0;                               // �������� ������ ����� ������

	for (int i = 0; i < _step_mem;i++)                            // ����������� ����� ������ � ��������.        
		{
			_u_porog = regBank.get(_adr_reg+40000+i);

		//	Serial.println(_u_porog);
		   // ��������� _u_porog �� byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// ��� �� ��� hi,low ����� ��������� EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k+201, low); 
			i_k++;
		}

	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
}
void read_mem_porog() 
{
	/*
		��������� ������ ������� ������ �� ������ � ��������
		regBank.get(40127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
		regBank.get(40128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
		regBank.get(40129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.
		��������� ����� ������ 200
		��������� ����� ��������� 40130 
		����� ����� �� ����� ??? ����
	*/
	unsigned int _adr_reg  = regBank.get(40127)+40000;              // ��������� ����� ����� ���������, 
	int _adr_mem  = regBank.get(40128)+200;                         // ��������� ����� ����� ������
	int _step_mem = regBank.get(40129);                             // ����� ����� � ������ �������� ����������� �����
	int _u_porog = 0;                                               // ��������� �������� ����������� ��������.
	int i_k = 0;                                                    // �������� ������ ����� ������

	for (int i = 0; i < _step_mem;i++)                              // ����������� ����� ������ � ��������.        
		{
		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);
		  i_k++;
		   _u_porog = (hi<<8) | low;                                // �������� ��� "��������� �����������"
		  regBank.set(_adr_reg+i,_u_porog);
		  //Serial.print(_adr_reg+i);
		  //Serial.print(" - ");
		  //Serial.print(_adr_mem+i_k-1);
		  //Serial.print(" - ");
		  //Serial.println(_u_porog);
		 }

	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
}
// ������ � ������ ���������� ���� ������� ����
void mem_byte_trans_read()
{
	unsigned int _adr_reg = regBank.get(40127)+40000;           //  ����� ����� ��������� ��� �������� � �� ������� �������.
	unsigned int _adr_mem = regBank.get(40128)+200;             //  ����� ����� ������ ��� �������� � �� ������� �������.
	unsigned int _size_block = regBank.get(40129);                       //  ����� ����� �����

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		regBank.set(_adr_reg+x_mem,i2c_eeprom_read_byte(deviceaddress,_adr_mem + x_mem));
	}

	regBank.set(adr_control_command,0);                         // ��������� ���������    
	delay(200);
}
void mem_byte_trans_save()
{
	unsigned int _adr_reg    = regBank.get(40127);                     //  ����� ����� ��������� ��� �������� � �� ������� �������.
	unsigned int _adr_mem    = regBank.get(40128);                     //  ����� ����� ������ ��� �������� � �� ������� �������.
	unsigned int _size_block = regBank.get(40129);                     //  ����� ����� �����

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		i2c_eeprom_write_byte(deviceaddress, _adr_mem + x_mem, regBank.get(_adr_reg+x_mem));
	}
	regBank.set(adr_control_command,0);                                // ��������� ���������    
	delay(200);
}
void set_mem_regBank(int adr_mem , int step_mem)
{
	int _adr_mem = adr_mem;
	int _step_mem = step_mem;
	for (int i = 0; i < _step_mem;i++)
		{
			i2c_eeprom_write_byte(deviceaddress, _adr_mem + i, regBank.get(40130)+i);
		}
}
void read_mem_regBank(int adr_mem , int step_mem)
{
	int _adr_mem = adr_mem;
	int _step_mem = step_mem;
	for (int i = 0; i < _step_mem;i++)
	{
	  regBank.set(40130+i,i2c_eeprom_read_byte(deviceaddress,_adr_mem +i));   
	}
}
void send_file_PC()
{
	//delay(1000);
	if (Serial2.available())                                     // ���� ���-�� ���������? ���� ������ � ������?
		  {
			unsigned char overflowFlag = 0 ;                     // ���� ���������� ������� ������
			unsigned char buffer_count = 0;                      // ���������� � ������ ������ ������

			while (Serial2.available())
				{
				  if (overflowFlag)                              // ���� ����� ���������� - ��������
					 Serial2.read();
				  else                                           // ������ ������ � �����, ������� ����������
					{
					if (buffer_count == BUFFER_SIZEKF)           // ��������� ������ ������
						{
							overflowFlag = 1;                    // ���������� ���� ���������� ������� ������
						}
						 fileName_F[buffer_count] = Serial2.read(); 
						 buffer_count++;
					}
				}
		   }
	 else 
		{
	
		}

  Serial.println(fileName_F);
  myFile = sd.open(fileName_F);
 // myFile = sd.open(fileName);
 // if (myFile) 
 // {
 //   Serial.println(fileName_p);

 //   // read from the file until there's nothing else in it:
 //   while (myFile.available()) 
	//{
 //     Serial.print(myFile.read());
 //   }
 //   // close the file:
 //    myFile.close();
 // }
 // else 
 // {
 //   // if the file didn't open, print an error:
 //   Serial.println("error opening file");
 // }

  // if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  //{
	 //Serial.println("error opening file");                            // ���� ������  �������� �����
  //}
  //else
  //{
	   // read from the file until there's nothing else in it:
	while (myFile.available()) 
	{
	  Serial2.write(myFile.read());
	} 
	delay(100);

	// close the file:
	 Serial2.flush();
	 myFile.close();

   //}
	 delay(1000);

	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}

void set_analog_pin()
{
	int _analogInPin = regBank.get(40040);  // analogIn_oscill

	switch (_analogInPin)
		{
			case 0:
				 analogIn_oscill = A0;                                                  // 
				 break;
			case 1:
				 analogIn_oscill = A1;                                //
				 break;
			case 2:		
				 analogIn_oscill = A2;                                                       // 
				 break;
			case 3:
				 analogIn_oscill = A3; 
				 break;
			case 4:	
				 analogIn_oscill = A4;                                              //
				 break;
			case 5:
				 analogIn_oscill = A5;                                                            //
				 break;
			case 6:	
				 analogIn_oscill = A6;                                                           //
				 break;
			case 7:
				 analogIn_oscill = A7;
				 break;
			case 8:				
				 analogIn_oscill = A8; 
				 break;
			case 9:
				 analogIn_oscill = A9; 
				 break;
			case 10:	
				 analogIn_oscill = A10; 
				 break;
			case 11:				
				 analogIn_oscill = A11;                                // 
				 break;
			case 12:
				 analogIn_oscill = A12; 
				 break;
			case 13:
				 analogIn_oscill = A13; 
				 break;
			case 14:
				 analogIn_oscill = A14; 
				 break;
			case 15:
				 analogIn_oscill = A15; 
				 break;
			default:
				 analogIn_oscill = A0; 
				 break;
		 }
	//Serial.println(analogIn_oscill);
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(100);
}
void oscilloscope()
{

		do{

			set_sound_oscill();                      // ���������� ������� ��� ��������� �������������
			set_rezistor1();                         // ���������� ������� ������� �1 
			set_rezistor2();                         // ���������� ������� ������� �2
			set_analog_pin();                        // ���������� ���� ������������



		 for (int i = 0;i < 600;i++)
				{
					 Array_volume[i] = analogRead(analogIn_oscill);               // ��������� ��������
				}

			for (int i = 0;i < 600;i++)
				{
					PrintInt(Array_volume[i]);
					Serial2.write('\r'); // print new line
				}
	
		 }while(regBank.get(40));    
						   
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
}

// Fast int to ASCII conversion
void PrintInt(int i) {
	static const char asciiDigits[10] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	div_t n;
	int print = 0;
	if(i < 0) {
		Serial2.write('-');
		i = -i;
	}
	if(i >= 10000) {
		n = div(i, 10000);
		Serial2.write(asciiDigits[n.quot]);
		i = n.rem;
		print = 1;
	}
	if(i >= 1000 || print) {
		n = div(i, 1000);
		Serial2.write(asciiDigits[n.quot]);
		i = n.rem;
		print = 1;
	}
	if(i >= 100 || print) {
		n = div(i, 100);
		Serial2.write(asciiDigits[n.quot]);
		i = n.rem;
		print = 1;
	}
	if(i >= 10 || print) {
		n = div(i, 10);
		Serial2.write(asciiDigits[n.quot]);
		i = n.rem;
	}
	Serial2.write(asciiDigits[i]);
} 

//====================== ����������� ����������� ������ =========================================


void setup_mcp()
{
	// ��������� ����������� ������
 
  mcp_Out1.begin(4);              //  ����� (4) �������  ����������� ������
  mcp_Out1.pinMode(0, OUTPUT);    // ���� �0  ����    
  mcp_Out1.pinMode(1, OUTPUT);    // ���� �1  ����    
  mcp_Out1.pinMode(2, OUTPUT);    // ���� �2  ����    
  mcp_Out1.pinMode(3, OUTPUT);    // ���� �3  ����    
  mcp_Out1.pinMode(4, OUTPUT);    // ���� �4  ����   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // ���� �5  ����    
  mcp_Out1.pinMode(6, OUTPUT);    // ���� �6  ����   
  mcp_Out1.pinMode(7, OUTPUT);    // ���� �7  �������� +12�����  ������� ����� ��������
  
  mcp_Out1.pinMode(8, OUTPUT);    //  ���� �8 ���� �� �������� ����.  
  mcp_Out1.pinMode(9, OUTPUT);    // �������� J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // �������� J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // �������� J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  ����� (6) �������  ����������� ������
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "���."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "��C." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "��-�����1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "��-�����2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  ����� (5)  ����������� ������ 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 ��������
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 ��������
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 ��������
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 ��������
  mcp_Analog.pullUp(0, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(1, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 �������� �������� ����������� ������ �� ���� 
  mcp_Analog.pullUp(2, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 �������� ����������� ������ �� ����
  mcp_Analog.pullUp(3, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 ��������  ����������� ������ �� ����
  mcp_Analog.pullUp(4, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(5, INPUT);   //CTS ����������� ������ �� ����
  mcp_Analog.pullUp(5, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(6, INPUT);   // DSR ����������� ������ �� ����
  mcp_Analog.pullUp(6, HIGH);     // ���������� ���������� �������� 100K � 5�.
  mcp_Analog.pinMode(7, INPUT);   //  DCD ����������� ������ �� ����
  mcp_Analog.pullUp(7, HIGH);     // ���������� ���������� �������� 100K � 5�.

}
void setup_resistor()
{ 
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word1));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
	Wire.beginTransmission(address_AD5252);      // transmit to device
	Wire.write(byte(control_word2));             // sends instruction byte  
	Wire.write(0);                               // sends potentiometer value byte  
	Wire.endTransmission();                      // stop transmitting
}

void setup_regModbus()
{

/*
��������� ������ Modbus ���������� ���������� ���������
��� ��, ��� ���������� ��������� ����� ��������, ����� ������ � ������
������������������ ������. � ��������� ����� �������� Modbus Slave ��������� �����
������� ������ ���� ���������� ����������� �� ����.
*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Assign the modbus device ID.  
  regBank.setId(1);               // Slave ID 1

/*
modbus registers follow the following format
00001-09999  Digital Outputs, A master device can read and write to these registers
10001-19999  Digital Inputs,  A master device can only read the values from these registers
30001-39999  Analog Inputs,   A master device can only read the values from these registers
40001-49999  Analog Outputs,  A master device can read and write to these registers 
����� �����, ����� ��������� �������� ��� ���� � ������� ������. ���
������������ ����� ����������� ����� � �������� � ��������� ���������� ���������
��������� ������� ��� ���������� ������.
*/
	regBank.add(1);                           // ���� RL0 ����  MIC1P
	regBank.add(2);                           // ���� RL1 ����  MIC2P
	regBank.add(3);                           // ���� RL2 ����  MIC3P
	regBank.add(4);                           // ���� RL3 ����  LFE  "���."
	regBank.add(5);                           // ���� RL4 XP1 12  HeS2e   ��������� ��������� �����������
	regBank.add(6);                           // ���� RL5 ���� Front L, Front R
	regBank.add(7);                           // ���� RL6 ���� Center
	regBank.add(8);                           // ���� RL7 ������� �����
  
	regBank.add(9);                           // ���� RL8 ���� �� ��������
	regBank.add(10);                          // ���� RL9 XP1 10 ��������� ��������� ����������
	regBank.add(11);                          // ���� RL10 ��������� ������� �� �������������� ������ 
	regBank.add(12);                          // �������� J24 - 1 
	regBank.add(13);                          // XP8 - 2   sensor �������� ������
	regBank.add(14);                          // XP8 - 1   PTT �������� ������
	regBank.add(15);                          // XS1 - 5   PTT ���
	regBank.add(16);                          // XS1 - 6   sensor ���
 
	regBank.add(17);                          // J8-12     XP7 4 PTT2   ����. �.
	regBank.add(18);                          // XP1 - 20  HangUp  DCD
	regBank.add(19);                          // J8-11     XP7 2 sensor  ����. �.
	regBank.add(20);                          // J8-23     XP7 1 PTT1 ����. �.
	regBank.add(21);                          // XP2-2     sensor "���."  
	regBank.add(22);                          // XP5-3     sensor "��C."
	regBank.add(23);                          // XP3-3     sensor "��-�����1."
	regBank.add(24);                          // XP4-3     sensor "��-�����2."
 
	regBank.add(25);                          // XP1- 19 HaSs      sensor ����������� ������    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);                          // XP1- 17 HaSPTT    CTS DSR ���.  
	regBank.add(27);                          // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	regBank.add(28);                          // XP1- 15 HeS2PTT   CTS ��� PTT �����������
	regBank.add(29);                          // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 
	regBank.add(30);                          // XP1- 6  HeS1PTT   CTS ���   ��� ����������
	regBank.add(31);                          // XP1- 5  HeS1Rs    sensor ���������� ��������� ���������� � 2 ����������
	regBank.add(32);                          // XP1- 1  HeS1Ls    sensor ���������� ��������� ����������
	regBank.add(39);                          // ���� ��������� �������� �������
	regBank.add(40);                          // ���� ���������� ��������� �������������
	regBank.add(41);                          // ���� ���������� ��������������
	regBank.add(42);                          // ���� ��� (mute)

	regBank.add(118);                         // ���� ��������� ������������ ��������
	regBank.add(119);                         // 

	regBank.add(120);                         // ���� ��������� ������������� ����� ������
	regBank.add(122);                         // ���� ��������� �������� �����
	regBank.add(123);                         // ���� ��������� �������� �����
	regBank.add(124);                         // ���� ��������� ����� � ������� "��������"
	regBank.add(125);                         // ���� ��������� ������������� SD ������
	regBank.add(126);                         //  
	regBank.add(127);                         //  
	regBank.add(128);                         //  
	regBank.add(129);                         //  

	regBank.add(130);                         //  ���� ��������� ����� 0 - RS232, 1 - USB0
	regBank.add(131);                         //  


	regBank.add(200);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(201);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(202);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(203);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(204);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(205);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(206);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(207);                         // ���� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(208);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(209);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(210);                         // ���� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(211);                         // ���� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(212);                         // ���� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(213);                         // ���� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(214);                         // ���� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(215);                         // ���� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(216);                         // ���� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(217);                         // ���� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(218);                         // ���� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(219);                         // ���� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	regBank.add(220);                         // ���� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(221);                         // ���� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(222);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(223);                         // ���� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(224);                         // ���� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(225);                         // ���� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(226);                         // ���� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(227);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(228);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(229);                         // ���� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(230);                         // ���� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(231);                         // ���� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(232);                         // ���� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(233);                         // ���� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(234);                         // ���� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(235);                         // ���� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(236);                         // ���� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(237);                         // ���� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(238);                         // ���� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(239);                         // ���� ������  ADC0  ��� x1 

	regBank.add(240);                         // ���� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(241);                         // ���� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(242);                         // ���� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(243);                         // ���� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(244);                         // ���� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(245);                         // ���� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(246);                         // ���� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(247);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(248);                         // ���� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(249);                         // ���� ������ ADC2 ��� x10  

	regBank.add(250);                         // ���� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(251);                         // ���� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(252);                         // ���� ������ "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(253);                         // ���� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(254);                         // ���� ������ "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(255);                         // ���� ������ "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(256);                         // ���� ������ "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(257);                         // ���� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(258);                         // ���� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(259);                         // ���� ������ "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(260);                         // ���� ������ "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(261);                         // ���� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(262);                         // ���� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(263);                         // ���� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(264);                         // ���� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(265);                         // ���� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(266);                         // ���� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(267);                         // ���� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(268);                         // ���� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(269);                         // ���� ������ ������������ ����������� ������� 

	regBank.add(270);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(271);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(272);                         // ���� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(273);                         // ���� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(274);                         // ���� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(275);                         // ���� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(276);                         // ���� ������ "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(277);                         // ���� ������ "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(278);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(279);                         // ���� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(280);                         // ���� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(281);                         // ���� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(282);                         // ���� ������ "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(283);                         // ���� ������ "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(284);                         // ���� ������ "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(285);                         // ���� ������ "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(286);                         // ���� ������ "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(287);                         // ���� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(288);                         // ���� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(289);                         // ���� ������ "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(290);                         // ���� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(291);                         // ���� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(292);                         // ���� ������ "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(293);                         // ���� ������ ADC1 ���������� 12/3 �����
	regBank.add(294);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio1
	regBank.add(295);                         // ���� ������ ADC14 ���������� 12/3 ����� Radio2
	regBank.add(296);                         // ���� ������ ADC14 ���������� 12/3 ����� ���
	regBank.add(297);                         // ���� ������ ADC15 ���������� ���������� 3,6 ������
	regBank.add(298);                         // ���� ������ "Test Microphone ** Signal mag phone                         ON  - ";      
	regBank.add(299);                         // ���� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(300);                         // ���� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(301);                         // ���� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(302);                         // ���� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(303);                         // ���� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(304);                         // ���� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(305);                         // ���� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(306);                         // ���� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(307);                         // ���� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(308);                         // ���� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(309);                         // ���� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(310);                         // ���� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(311);                         // ���� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(312);                         // ���� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(313);                         // ���� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(314);                         // ���� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(315);                         // ���� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(316);                         // ���� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(317);                         // ���� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(318);                         // ���� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(319);                         // ���� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(320);                         // ���� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(321);                         // ���� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(322);                         // ���� ������ "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(323);                         // ���� ������ "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(324);                         // ���� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(325);                         // ���� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(326);                         // ���� ������ "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(327);                         // ���� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(328);                         // ���� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(329);                         // ���� ������ ��� ������� �������

	regBank.add(330);                         // ���� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(331);                         // ���� ������ "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	regBank.add(332);                         // ���� ������ "Test GGS ** Signal mag radio                                ON  - ";
	regBank.add(333);                         // "Test headset instructor ** Signal mag radio                             ON  - ";
	regBank.add(334);                         // "Test headset dispatcher ** Signal mag radio                             ON  - ";
	regBank.add(335);                         // "Test MTT ** Signal mag radio                                            ON  - ";
	regBank.add(336);                         // "Test Microphone ** Signal mag radio                                     ON  - ";
	regBank.add(337);                         // �������� 

	regBank.add(10081);                       // ����� ����a ��������� ��������� ������� CTS
	regBank.add(10082);                       // ����� ����a ��������� ��������� ������� DSR
	regBank.add(10083);                       // ����� ����a ��������� ��������� ������� DCD

						                      //Add Input registers 30001-30040 to the register bank

	//regBank.add(30000);  // ���� 0 ���� ��� 0 - ��������   ��� "D"
	//regBank.add(30001);  // ���� 0 ���� ��� 1 - ��������   "1"
	//regBank.add(30002);  // ���� 0 ���� ��� 2 - ��������   "0"  
	//regBank.add(30003);  // ���� 0 ���� ��� 3 - ��������   "1"
	//regBank.add(30004);  // ���� 0 ���� ��� 4 - ��������   "0" ����� ���� ��������� (2)
	//regBank.add(30005);  // ���� 0 ���� ��� 5 - ��������   "1" ����� ���� ��������� (2)
	//regBank.add(30006);  // ���� 0 ���� ��� 6 - ��������   "0" ����� ���� ��������� (2)
	//regBank.add(30007);  // ���� 0 ���� ��� 7 - ��������   "0"
 // 
	//regBank.add(30008);  // ���� 1 ���� ��� 0 - ��������   CRC0
	//regBank.add(30009);  // ���� 1 ���� ��� 1 - ��������   CRC1
	//regBank.add(30010);  // ���� 1 ���� ��� 2 - ��������   CRC2
	//regBank.add(30011);  // ���� 1 ���� ��� 3 - ��������   CRC3
	//regBank.add(30012);  // ���� 1 ���� ��� 4 - ��������   ���������� ��� (Mute)
	//regBank.add(30013);  // ���� 1 ���� ��� 5 - ��������   �������������
	//regBank.add(30014);  // ���� 1 ���� ��� 6 - ��������   ������/ ������ ���� �������
	//regBank.add(30015);  // ���� 1 ���� ��� 7 - ��������   "1"
 // 
	//regBank.add(30016);  // ���� 2 ���� ��� 0 - ��������   ��� ������� ������
	//regBank.add(30017);  // ���� 2 ���� ��� 1 - ��������   ��� ������� ������
	//regBank.add(30018);  // ���� 2 ���� ��� 2 - ��������   ��� ������� ������
	//regBank.add(30019);  // ���� 2 ���� ��� 3 - ��������   ��� ������� ������
	//regBank.add(30020);  // ���� 2 ���� ��� 4 - ��������   ��� ������� ������
	//regBank.add(30021);  // ���� 2 ���� ��� 5 - ��������   ��� ������� ������
	//regBank.add(30022);  // ���� 2 ���� ��� 6 - ��������   ��� ������� ������
	//regBank.add(30023);  // ���� 2 ���� ��� 7 - ��������   "0" 

	//					 // ���� ������  ���������� ��  ����� ��������. �������� 4 �����
 // 
	//regBank.add(30024);  // ���� 1 ����� ��� 0 - ��������  ���� ����������� �� �����2
	//regBank.add(30025);  // ���� 1 ����� ��� 1 - ��������  ���� ����������� �� �����1
	//regBank.add(30026);  // ���� 1 ����� ��� 2 - ��������  ���� ����������� ������
	//regBank.add(30027);  // ���� 1 ����� ��� 3 - ��������  ���� ����������� ������ ��������
	//regBank.add(30028);  // ���� 1 ����� ��� 4 - ��������  ���� ����������� ������
	//regBank.add(30029);  // ���� 1 ����� ��� 5 - ��������   "1"
	//regBank.add(30030);  // ���� 1 ����� ��� 6 - ��������   "0" 
	//regBank.add(30031);  // ���� 1 ����� ��� 7 - ��������   "1"
 // 
	//regBank.add(30032);  // ���� 2 ����� ��� 0 - ��������   ��� ������� ������
	//regBank.add(30033);  // ���� 2 ����� ��� 1 - ��������   ��� ������� ������
	//regBank.add(30034);  // ���� 2 ����� ��� 2 - ��������   ��� ������� ������
	//regBank.add(30035);  // ���� 2 ����� ��� 3 - ��������   ��� ������� ������
	//regBank.add(30036);  // ���� 2 ����� ��� 4 - ��������   ��� ������� ������
	//regBank.add(30037);  // ���� 2 ����� ��� 5 - ��������   ��� ������� ������
	//regBank.add(30038);  // ���� 2 ����� ��� 6 - ��������   ��� ������� ������
	//regBank.add(30039);  // ���� 2 ����� ��� 7 - ��������   "0" 
 // 
	//regBank.add(30040);  // ���� 3 ����� ��� 0 - ��������   ���� ����������� �����������
	//regBank.add(30041);  // ���� 3 ����� ��� 1 - ��������   ���� ����������� ��������� ����������� 2 ����������
	//regBank.add(30042);  // ���� 3 ����� ��� 2 - ��������   ���� ����������� ��������� �����������
	//regBank.add(30043);  // ���� 3 ����� ��� 3 - ��������   ���� ����������� ��������� ���������� � 2 ����������
	//regBank.add(30044);  // ���� 3 ����� ��� 4 - ��������   ���� ����������� ��������� ����������
	//regBank.add(30045);  // ���� 3 ����� ��� 5 - ��������   ���� ����������� ��������� XS1 - 6 sensor
	//regBank.add(30046);  // ���� 3 ����� ��� 6 - ��������   ���� ����������� ���
	//regBank.add(30047);  // ���� 3 ����� ��� 7 - ��������   "0" 
 // 
	//regBank.add(30048);  // ���� 4 ����� ��� 0 - ��������   CRC0
	//regBank.add(30049);  // ���� 4 ����� ��� 1 - ��������   CRC1
	//regBank.add(30050);  // ���� 4 ����� ��� 2 - ��������   CRC2   
	//regBank.add(30051);  // ���� 4 ����� ��� 3 - ��������   CRC3   
	//regBank.add(30052);  // ���� 4 ����� ��� 4 - ��������   ���� ���������� ��������� �����������
	//regBank.add(30053);  // ���� 4 ����� ��� 5 - ��������    ���� �������������
	//regBank.add(30054);  // ���� 4 ����� ��� 6 - ��������   ���� ���������� ��������� ����������
	//regBank.add(30055);  // ���� 4 ����� ��� 7 - ��������   "0" 


	//regBank.set(40004+buffer,Serial1.read());
	regBank.add(40000);  // 
	regBank.add(40001);  // �������� ������ � ����� 1
	regBank.add(40002);  // �������� ������ � ����� 1
	regBank.add(40003);  // �������� ������ � ����� 1
	regBank.add(40004);  // �������� ������ � ����� 1
	regBank.add(40005);  // �������� ������ � ����� 1
	regBank.add(40006);  // �������� ������ � ����� 1
	regBank.add(40007);  // �������� ������ � ����� 1
	regBank.add(40008);  // 
	regBank.add(40009);  // 

	regBank.add(40010);  // �  ����� 1
	regBank.add(40011);  // �  ����� 1
	regBank.add(40012);  // �  ����� 1
	regBank.add(40013);  // �  ����� 1
	regBank.add(40014);  // 
	regBank.add(40015);  // 
	regBank.add(40016);  // 
	regBank.add(40017);  // 
	regBank.add(40018);  // 
	regBank.add(40019);  // 
	regBank.add(40040);  // analogIn_oscill   �������� ����� ����������� ������ �������� 5.0
	regBank.add(40041);  //  ���������� ������� ��� ��������� ������������� (0-7)

						 // ������� ����� 
	regBank.add(40046);  // ����� ���� ������ ����� �����������
	regBank.add(40047);  // ����� ����� ������ ����� �����������
	regBank.add(40048);  // ����� ��� ������ ����� �����������
	regBank.add(40049);  // ����� ��� ������ ����� �����������
	regBank.add(40050);  // ����� ������ ������ ����� �����������
	regBank.add(40051);  // ����� ������� ������ ����� �����������
 
						 // ��������� ������� � �����������
	regBank.add(40052);  // ����� ����
	regBank.add(40053);  // ����� �����
	regBank.add(40054);  // ����� ���
	regBank.add(40055);  // ����� ���
	regBank.add(40056);  // ����� ������
	regBank.add(40057);  // 
	regBank.add(40058);  // 
	regBank.add(40059);  // 
	
	regBank.add(40060);  // ����� �������� �������� ������� ���������� � 1
	regBank.add(40061);  // ����� �������� �������� ������� ��� ����������
	regBank.add(40062);  // ����� �������� �������� ������� ��� �������� � ���������
	regBank.add(40063);  // ����� �������� ������������ �������� ������� ��� �������� � ��������� ��
	regBank.add(40064);  // ����� �������� �������� ������� ���������� � 2

	/*

	regBank.add(40064); // ����� ������
	regBank.add(40065); // ����� ������
	regBank.add(40066); // ����� ������
	regBank.add(40067); // ����� ������
	regBank.add(40068); // ����� ������
	regBank.add(40069); // ����� ������
	regBank.add(40070); // ����� ������
	regBank.add(40071); // ����� ������

	regBank.add(40072); // ����� ������ � %
	regBank.add(40073); // ����� ������ � %
	regBank.add(40074); // ����� ������ � %
	regBank.add(40075); // ����� ������ %
	regBank.add(40076); // ����� ������ %
	regBank.add(40077); // ����� ������ %
	regBank.add(40078); // ����� ������ %
	regBank.add(40079); // ����� ������ %
	regBank.add(40080); // ����� ������ %
	regBank.add(40081); // ����� ������ %
	regBank.add(40082); // ����� ������ %
	regBank.add(40083); // ����� ������ %

	// ����� ������ �� ���������
	regBank.add(40084); // ����� ���� adr_Mic_On_day 
	regBank.add(40085); // ����� ����� adr_Mic_On_month  
	regBank.add(40086); // ����� ��� adr_Mic_On_year  
	regBank.add(40087); // ����� ��� adr_Mic_On_hour 
	regBank.add(40088); // ����� ������ adr_Mic_On_minute 
	regBank.add(40089); // ����� �������  adr_Mic_On_second    

	// ����� ������ �� ����������
	regBank.add(40090); // ����� ���� adr_Mic_Off_day    
	regBank.add(40091); // ����� �����  adr_Mic_Off_month 
	regBank.add(40092); // ����� ��� adr_Mic_Off_year  
	regBank.add(40093); // ����� ��� adr_Mic_Off_hour   
	regBank.add(40094); // ����� ������ adr_Mic_Off_minute   
	regBank.add(40095); // ����� ������� adr_Mic_Off_second    
	*/
	// ����� ������ �����
	regBank.add(40096);  // ����� ����  adr_Mic_Start_day    
	regBank.add(40097);  // ����� ����� adr_Mic_Start_month  
	regBank.add(40098);  // ����� ��� adr_Mic_Start_year  
	regBank.add(40099);  // ����� ��� adr_Mic_Start_hour 
	regBank.add(40100);  // ����� ������ adr_Mic_Start_minute 
	regBank.add(40101);  // ����� ������� adr_Mic_Start_second  

	// ����� ��������� �����
	regBank.add(40102);  // ����� ���� adr_Mic_Stop_day 
	regBank.add(40103);  // ����� ����� adr_Mic_Stop_month 
	regBank.add(40104);  // ����� ��� adr_Mic_Stop_year
	regBank.add(40105);  // ����� ��� adr_Mic_Stop_hour 
	regBank.add(40106);  // ����� ������ adr_Mic_Stop_minute  
	regBank.add(40107);  // ����� ������� adr_Mic_Stop_second 

	// ����������������� ���������� �����
	regBank.add(40108);  // ����� ���� adr_Time_Test_day 
	regBank.add(40109);  // ����� ��� adr_Time_Test_hour 
	regBank.add(40110);  // ����� ������ adr_Time_Test_minute
	regBank.add(40111);  // ����� ������� adr_Time_Test_second

	// ��� �����
	regBank.add(40112);  // ����� �������� ���������� ��� 
	regBank.add(40113);  // ����� �������� ���������� ����� 
	regBank.add(40114);  // ����� �������� ���������� ����
	regBank.add(40115);  // ����� �������� ���������� �������� ���������� ������ �����
	regBank.add(40116);  // ����� �������� ���������� �������� �������� ������ �����

	regBank.add(40120);  // adr_control_command ����� �������� ������� �� ����������
	regBank.add(40121);  // ����� �������� ���� ������
	regBank.add(40122);  //
	regBank.add(40123);  //
	regBank.add(40124);  //  ����� ����� ��������� ��� �������� � �� ����������� ������.
	regBank.add(40125);  //  ����� ����� ������ ��� �������� � �� ����������� ������.
	regBank.add(40126);  //  ����� ����� ����� ������ ��� �������� � �� ����������� ������.
	regBank.add(40127);  //  ����� ����� ��������� ��� �������� � �� ������� �������.
	regBank.add(40128);  //  ����� ����� ������ ��� �������� � �� ������� �������.
	regBank.add(40129);  //  ����� ����� ����� ������ ��� �������� � �� ������� �������.

	regBank.add(40130);  //  �������� ���������� �������� ��� �������� ������� ������� 
	regBank.add(40131);  //
	regBank.add(40132);  //  
	regBank.add(40133);  //
	regBank.add(40134);  //  
	regBank.add(40135);  //
	regBank.add(40136);  //  
	regBank.add(40137);  //
	regBank.add(40138);  //  
	regBank.add(40139);  //

	regBank.add(40140);  //  
	regBank.add(40141);  //
	regBank.add(40142);  //  
	regBank.add(40143);  //
	regBank.add(40144);  //  
	regBank.add(40145);  //
	regBank.add(40146);  //  
	regBank.add(40147);  //
	regBank.add(40148);  //  
	regBank.add(40149);  //
	//---------------------------------------------------------------------------------------

	regBank.add(40150);  //  �������� ��������� ������
	regBank.add(40151);  //
	regBank.add(40152);  //  
	regBank.add(40153);  //
	regBank.add(40154);  //  
	regBank.add(40155);  //
	regBank.add(40156);  //  
	regBank.add(40157);  //
	regBank.add(40158);  //  
	regBank.add(40159);  //

	regBank.add(40160);  //  
	regBank.add(40161);  //
	regBank.add(40162);  //  
	regBank.add(40163);  //
	regBank.add(40164);  //  
	regBank.add(40165);  //
	regBank.add(40166);  //  
	regBank.add(40167);  //
	regBank.add(40168);  //  
	regBank.add(40169);  //
	
	/*
	regBank.add(40200);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(40201);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(40202);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(40203);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(40204);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(40205);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(40206);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(40207);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(40208);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(40209);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(40210);                         // A���� �������� ������ "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(40211);                         // A���� �������� ������ "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(40212);                         // A���� �������� ������ "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(40213);                         // A���� �������� ������ "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(40214);                         // A���� �������� ������ "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(40215);                         // A���� �������� ������ "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(40216);                         // A���� �������� ������ "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(40217);                         // A���� �������� ������ "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(40218);                         // A���� �������� ������ "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(40219);                         // A���� �������� ������ "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	regBank.add(40220);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(40221);                         // A���� �������� ������ "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(40222);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(40223);                         // A���� �������� ������ "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(40224);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40225);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40226);                         // A���� �������� ������ "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40227);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40228);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40229);                         // A���� �������� ������ "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40230);                         // A���� �������� ������ "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40231);                         // A���� �������� ������ "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40232);                         // A���� �������� ������ "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40233);                         // A���� �������� ������ "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40234);                         // A���� �������� ������ "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40235);                         // A���� �������� ������ "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40236);                         // A���� �������� ������ "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40237);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40238);                         // A���� �������� ������ "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40239);                         // A���� �������� ������ ADC0  ��� x1 

	regBank.add(40240);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40241);                         // A���� �������� ������ "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40242);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40243);                         // A���� �������� ������ "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40244);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40245);                         // A���� �������� ������ "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40246);                         // A���� �������� ������ "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40247);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40248);                         // A���� �������� ������ "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40249);                         // A���� �������� ������ ADC2 ��� x10

	regBank.add(40250);                         // A���� �������� ������ "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40251);                         // A���� �������� ������ "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40252);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40253);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40254);                         // A���� �������� ������ "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40255);                         // A���� �������� ������ "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40256);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40257);                         // A���� �������� ������ "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40258);                         // A���� �������� ������ "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40259);                         // A���� �������� ������ "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40260);                         // A���� �������� ������ "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40261);                         // A���� �������� ������ "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40262);                         // A���� �������� ������ "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40263);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40264);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(40265);                         // A���� �������� ������ "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40266);                         // A���� �������� ������ "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(40267);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40268);                         // A���� �������� ������ "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40269);                         // A���� �������� ������ ������������ ����������� �������

	regBank.add(40270);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40271);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40272);                         // A���� �������� ������ "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40273);                         // A���� �������� ������ "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40274);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40275);                         // A���� �������� ������ "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40276);                         // A���� �������� ������ "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40277);                         // A���� �������� ������ "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40278);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40279);                         // A���� �������� ������ "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40280);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40281);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40282);                         // A���� �������� ������ "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40283);                         // A���� �������� ������ "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40284);                         // A���� �������� ������ "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40285);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40286);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40287);                         // A���� �������� ������ "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40288);                         // A���� �������� ������ "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40289);                         // A���� �������� ������ "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40290);                         // A���� �������� ������ "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40291);                         // A���� �������� ������ "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40292);                         // A���� �������� ������ "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40293);                         // A���� ��������  ������ ADC1 ���������� 12/3 �����
	regBank.add(40294);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio1
	regBank.add(40295);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� Radio2
	regBank.add(40296);                         // A���� ��������  ������ ADC14 ���������� 12/3 ����� ���
	regBank.add(40297);                         // A���� ��������  ������ ADC15 ���������� ���������� 3,6 ������
	regBank.add(40298);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         ON  - ";    
	regBank.add(40299);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40300);                         // A���� �������� ������ "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40301);                         // A���� �������� ������ "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40302);                         // A���� �������� ������ "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40303);                         // A���� �������� ������ "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40304);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40305);                         // A���� �������� ������ "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40306);                         // A���� �������� ������ "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40307);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40308);                         // A���� �������� ������ "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40309);                         // A���� �������� ������ "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40310);                         // A���� �������� ������ "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40311);                         // A���� �������� ������ "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40312);                         // A���� �������� ������ "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40313);                         // A���� �������� ������ "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40314);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40315);                         // A���� �������� ������ "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40316);                         // A���� �������� ������ "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40317);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40318);                         // A���� �������� ������ "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40319);                         // A���� �������� ������ "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40320);                         // A���� �������� ������ "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40321);                         // A���� �������� ������ "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40322);                         // A���� �������� ������ "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40323);                         // A���� �������� ������ "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40324);                         // A���� �������� ������ "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40325);                         // A���� �������� ������ "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40326);                         // A���� �������� ������ "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40327);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40328);                         // A���� �������� ������ "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40329);                         // A���� �������� ������ ��� ����������� �������                             // 

	regBank.add(40330);                         // A���� �������� ������ "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(40331);                         // A���� �������� ������ "Test Radio2 ** Signal mag radio                             ON  - ";
	regBank.add(40332);                         // A���� �������� ������ "Test GGS    ** Signal mag radio                             ON  - ";

	*/
	// ++++++++++++++++++++++ �������� �������� ������ ��� �������� ������� ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
	regBank.add(40400);                         // A���� ���������� ADC0  ��� x1 
	regBank.add(40401);                         // A���� ���������� ADC1 ���������� 12/3 �����
	regBank.add(40402);                         // A���� ���������� ADC2 ��� x10
	regBank.add(40403);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio1
	regBank.add(40404);                         // A���� ���������� ADC14 ���������� 12/3 ����� Radio2
	regBank.add(40405);                         // A���� ���������� ADC14 ���������� 12/3 ����� ���
	regBank.add(40406);                         // A���� ���������� ADC15 ���������� ���������� 3,6 ������
	regBank.add(40407);                         // A���� 
	regBank.add(40408);                         // A���� 
	regBank.add(40409);                         // A����  

	regBank.add(40410);                         // A���� �������� 
	regBank.add(40411);                         // A���� ��������  
	regBank.add(40412);                         // A���� ��������  
	regBank.add(40413);                         // A���� ��������  
	regBank.add(40414);                         // A���� ��������  
	regBank.add(40415);                         // A���� ��������  
	regBank.add(40416);                         // A���� ��������  
	regBank.add(40417);                         // A���� ��������  
	regBank.add(40418);                         // A���� ��������  
	regBank.add(40419);                         // A���� ��������  

	regBank.add(40420);                         // A����  ;
	regBank.add(40421);                         // A����  ;
	regBank.add(40422);                         // A����  ;
	regBank.add(40423);                         // A����  ;
	regBank.add(40424);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40425);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40426);                         // A���� ������ ��������� "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40427);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40428);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40429);                         // A���� ������ ��������� "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40430);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40431);                         // A���� ������ ��������� "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40432);                         // A���� ������ ��������� "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40433);                         // A���� ������ ��������� "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40434);                         // A���� ������ ��������� "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40435);                         // A���� ������ ��������� "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40436);                         // A���� ������ ��������� "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40437);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40438);                         // A���� ������ ��������� "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40439);                         // A���� ������ ��������� ADC0  ��� x1 

	regBank.add(40440);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40441);                         // A���� ������ ��������� "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40442);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40443);                         // A���� ������ ��������� "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40444);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40445);                         // A���� ������ ��������� "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40446);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40447);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40448);                         // A���� ������ ��������� "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40449);                         // A���� ������ ��������� ADC2 ��� x10

	regBank.add(40450);                         // A���� ������ ��������� "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40451);                         // A���� ������ ��������� "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40452);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40453);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40454);                         // A���� ������ ��������� "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40455);                         // A���� ������ ��������� "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40456);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40457);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40458);                         // A���� ������ ��������� "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40459);                         // A���� ������ ��������� "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40460);                         // A���� ������ ��������� "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40461);                         // A���� ������ ��������� "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40462);                         // A���� ������ ��������� "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40463);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40464);                         // 
	regBank.add(40465);                         // A���� ������ ��������� "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40466);                         // 
	regBank.add(40467);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40468);                         // A���� ������ ��������� "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40469);                         // ������������ �������� ����������� ������� �������

	regBank.add(40470);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40471);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40472);                         // A���� ������ ��������� "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40473);                         // A���� ������ ��������� "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40474);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40475);                         // A���� ������ ��������� "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40476);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40477);                         // A���� ������ ��������� "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40478);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40479);                         // A���� ������ ��������� "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40480);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40481);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40482);                         // A���� ������ ��������� "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40483);                         // A���� ������ ��������� "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40484);                         // A���� ������ ��������� "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40485);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40486);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40487);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40488);                         // A���� ������ ��������� "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40489);                         // A���� ������ ��������� "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40490);                         // A���� ������ ��������� "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40491);                         // A���� ������ ��������� "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40492);                         // A���� ������ ��������� "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40493);                         // A���� ������ ��������� ADC1 ���������� 12/3 �����
	regBank.add(40494);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio1
	regBank.add(40495);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� Radio2
	regBank.add(40496);                         // A���� ������ ��������� ADC14 ���������� 12/3 ����� ���
	regBank.add(40497);                         // A���� ������ ��������� ADC15 ���������� ���������� 3,6 ������
	regBank.add(40498);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         ON  - "; 
	regBank.add(40499);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40500);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40501);                         // A���� ������ ��������� "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40502);                         // A���� ������ ��������� "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40503);                         // A���� ������ ��������� "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40504);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40505);                         // A���� ������ ��������� "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40506);                         // A���� ������ ��������� "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40507);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40508);                         // A���� ������ ��������� "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40509);                         // A���� ������ ��������� "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40510);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40511);                         // A���� ������ ��������� "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40512);                         // A���� ������ ��������� "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40513);                         // A���� ������ ��������� "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40514);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40515);                         // A���� ������ ��������� "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40516);                         // A���� ������ ��������� "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40517);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40518);                         // A���� ������ ��������� "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40519);                         // A���� ������ ��������� "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40520);                         // A���� ������ ��������� "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40521);                         // A���� ������ ��������� "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40522);                         // A���� ������ ��������� "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40523);                         // A���� ������ ��������� "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40524);                         // A���� ������ ��������� "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40525);                         // A���� ������ ��������� "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40526);                         // A���� ������ ��������� "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40527);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40528);                         // A���� ������ ��������� "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40529);                         // ��� ����������� ������� �������
	regBank.add(40530);                         // A���� ������ ��������� "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(40531);                         // A���� ������ ��������� "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	regBank.add(40532);                         // A���� ������ ��������� "Test GGS ** Signal mag radio   
	*/
	slave._device = &regBank;  
}

void test_serial2()
{
  // ����� ��� �����
	Serial.println("COM port find...");
	wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	do
	{

	  if (Serial2.available() == 5) 
	  {
		//Read buffer
		inputByte_0 = Serial2.read();
		delay(100);    
		inputByte_1 = Serial2.read();
		delay(100);      
		inputByte_2 = Serial2.read();
		delay(100);      
		inputByte_3 = Serial2.read();
		delay(100);
		inputByte_4 = Serial2.read();   
	  }
	  //Check for start of Message
	  if(inputByte_0 == 16)
	  {       
		   //Detect Command type
		   switch (inputByte_1) 
		   {
			  case 127:
				 //Set PIN and value
				 switch (inputByte_2)
				{
				  case 4:
					if(inputByte_3 == 255)
					{
					//  digitalWrite(ledPin_13, HIGH); 
					  break;
					}
					else
					{
					 // digitalWrite(ledPin_13, LOW); 
					  break;
					}
				  break;
				} 
				break;
			  case 128:
				wdt_disable();                                     // 
				//Say hello
				Serial2.println("HELLO FROM SERIAL2");
				portFound2 = true;
				break;
			} 
			//Clear Message bytes
			inputByte_0 = 0;
			inputByte_1 = 0;
			inputByte_2 = 0;
			inputByte_3 = 0;
			inputByte_4 = 0;
	   }
	   Serial.print(".");
	   delay(300);
	
	} while(portFound2 == false);
	//} while(portFound2 == false && COM_POrt_Seek < 30);

	if (portFound2)
		{

			Serial.println();
			Serial.println("COM port find OK!.");
		}
	else
		{
			Serial.println();
			Serial.println("COM port find false!.");
			wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
		}
	clear_serial2();
	
//	wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
	Serial.println("COM port find End");
}
void set_serial2()
{
  // ����� ��� �����
	Serial.println("COM port find...");
	wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	do
	{

	  if (Serial2.available() == 5) 
	  {
		//Read buffer
		inputByte_0 = Serial2.read();
		delay(100);    
		inputByte_1 = Serial2.read();
		delay(100);      
		inputByte_2 = Serial2.read();
		delay(100);      
		inputByte_3 = Serial2.read();
		delay(100);
		inputByte_4 = Serial2.read();   
	  }
	  //Check for start of Message
	  if(inputByte_0 == 16)
	  {       
		   //Detect Command type
		   switch (inputByte_1) 
		   {
			  case 127:
				 //Set PIN and value
				 switch (inputByte_2)
				{
				  case 4:
					if(inputByte_3 == 255)
					{
					//  digitalWrite(ledPin_13, HIGH); 
					  break;
					}
					else
					{
					 // digitalWrite(ledPin_13, LOW); 
					  break;
					}
				  break;
				} 
				break;
			  case 128:
				wdt_disable();                                     // 
				//Say hello
				Serial2.println("HELLO FROM SERIAL2");
				portFound2 = true;
				break;
			} 
			//Clear Message bytes
			inputByte_0 = 0;
			inputByte_1 = 0;
			inputByte_2 = 0;
			inputByte_3 = 0;
			inputByte_4 = 0;
	   }
	   Serial.print(".");
	   delay(300);
	
	} while(portFound2 == false);
	//} while(portFound2 == false && COM_POrt_Seek < 30);

	if (portFound2)
		{

			Serial.println();
			Serial.println("COM port find OK!.");
		}
	else
		{
			Serial.println();
			Serial.println("COM port find false!.");
			wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
		}
	clear_serial2();
	
//	wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	regBank.set(adr_control_command,0);                                             // ��������� ���������    
	delay(200);
	Serial.println("COM port find End");
}
void set_serial3()
{
   clear_serial3();
   delay(200);
// ����� ��� �����
	Serial.println("COM port find...");
	do
	{
	  if (Serial3.available() == 5) 
	  {
		//Read buffer
		inputByte_0 = Serial3.read();
		delay(100);    
		inputByte_1 = Serial3.read();
		delay(100);      
		inputByte_2 = Serial3.read();
		delay(100);      
		inputByte_3 = Serial3.read();
		delay(100);
		inputByte_4 = Serial3.read();   
	  }
	  //Check for start of Message
	  if(inputByte_0 == 16)
	  {       
		   //Detect Command type
		   switch (inputByte_1) 
		   {
			  case 127:
				 //Set PIN and value
				 switch (inputByte_2)
				{
				  case 4:
					if(inputByte_3 == 255)
					{
					//  digitalWrite(ledPin_13, HIGH); 
					  break;
					}
					else
					{
					 // digitalWrite(ledPin_13, LOW); 
					  break;
					}
				  break;
				} 
				break;
			  case 128:
				//Say hello
				Serial3.println("HELLO FROM KAMERTON");
				portFound = true;
				Serial.println("COM port find OK!.");
				break;
			} 
			//Clear Message bytes
			inputByte_0 = 0;
			inputByte_1 = 0;
			inputByte_2 = 0;
			inputByte_3 = 0;
			inputByte_4 = 0;
	   }
	   Serial.print(".");
	   //clear_serial3();
	   delay(500);
	   //mcp_Analog.digitalWrite(Front_led_Red, blink_red); 
	   //mcp_Analog.digitalWrite(Front_led_Blue, !blink_red); 
	   //blink_red = !blink_red;
	   //digitalWrite(ledPin13,!digitalRead(ledPin13));
	} while(portFound == false);
//	wdt_enable (WDTO_8S); // ��� ������ �� ������������� ������������� �������� ����� 8 ���.
	//digitalWrite(ledPin13,LOW);
	//mcp_Analog.digitalWrite(Front_led_Red, LOW); 
}
void clear_serial()
{
  if (Serial.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial.available())
				{
					 Serial.read();
				}
		   }
}
void clear_serial2()
{
  if (Serial2.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial2.available())
				{
					 Serial2.read();
				}
		   }
   regBank.set(adr_control_command,0);
}
void clear_serial1()
{
  if (Serial1.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial1.available())
				{
					 Serial1.read();
				}
		   }
}
void clear_serial3()
{
  if (Serial3.available())                             // ���� ���-�� ���������? ���� ������ � ������?
		  {

			while (Serial3.available())
				{
					 Serial3.read();
				}
		   }
}

void set_SD()
{
		if (!sd.begin(chipSelect)) 
		{
			//Serial.println("initialization SD failed!");
			regBank.set(125,false); 
		}
	else
		{
			  myFile = sd.open("example.txt", FILE_WRITE);
			  myFile.close();

			  // Check to see if the file exists:
			  if (sd.exists("example.txt")) 
			  {
				  regBank.set(125,true); 
				  sd.remove("example.txt");
			   // Serial.println("example.txt exists.");
			  }
			  else 
			  {
			   // Serial.println("example.txt doesn't exist.");
				regBank.set(125,false); 
			  }
			}

	UpdateRegs(); 
	delay(100);
	regBank.set(adr_control_command,0);  
}
void file_del_SD()
{
	if (!sd.begin(chipSelect)) 
		{
			//Serial.println("initialization SD failed!");
			//regBank.set(125,false); 
		}
	else
		{
			  myFile = sd.open(fileName_F);
				//myFile = sd.open("example.txt", FILE_WRITE);
				//myFile.close();

			  // Check to see if the file exists:
			  if (sd.exists(fileName_F)) 
			  {
				 // regBank.set(125,true); 
				  sd.remove(fileName_F);
			   // Serial.println("example.txt exists.");
			  }
			  else 
			  {
			   // Serial.println("example.txt doesn't exist.");
				//regBank.set(125,false); 
			  }
			}

	UpdateRegs(); 
	delay(100);
	regBank.set(adr_control_command,0);  

}
//------------------------------------------------------------------------------

void setup()
{
	wdt_disable();                                                 // ����������� ������ �� ������� �� ������� ���������� ��� bootloop
	Wire.begin();
	if (!RTC.begin())                                              // ��������� ����� 
		{
			Serial.println("RTC failed");
			while(1);
		};
	setup_mcp();                                                   // ��������� ����� ����������  MCP23017
	mcp_Analog.digitalWrite(DTR, HIGH);                            // ���������� ������ (������)���������� � ������� �����-1
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	Serial.begin(9600);                                            // ����������� � USB ��
	Serial1.begin(115200);                                         // ����������� � ��������� ������ ��������
	slave.setSerial(3,57600);                                      // ����������� � ��������� MODBUS ���������� Serial3 
	Serial2.begin(115200);                                         // 
	Serial.println(" ");
	Serial.println(" ***** Start system  *****");
	Serial.println("Kamerton5_0_1ArduinoSpeed2 ");
	Serial.println(" ");
	portFound = false;

	pinMode(ledPin13, OUTPUT);   
	pinMode(ledPin12, OUTPUT);  
	pinMode(ledPin11, OUTPUT);  
	pinMode(ledPin10, OUTPUT);  
	pinMode(kn1Nano, OUTPUT);                                     // ���������� ������ ���������� Nano ��������� �������
	pinMode(kn2Nano, OUTPUT);                                     // ���������� ������ ���������� Nano ��������� 1000 ��
	pinMode(kn3Nano, OUTPUT);                                     // ���������� ������ ���������� Nano ��������� 2000 ��
	pinMode(InNano12, INPUT);                                     // ���������� ������ - ��������� ��������� 1000 ��� 2000 ��
	pinMode(InNano13, INPUT);                                     // ���������� ������ - ��������� ��������� ������� 
	digitalWrite(InNano12, HIGH);                                 // ��������� �������������� ���������

	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0);        // ������� ������ � ������� � ������ "set_time"
	// RTC.adjust(set_time);                                      // �������� ���� � �����
	serial_print_date();
	Serial.println(" ");

	setup_resistor();                                             // ��������� ��������� ���������

	setup_regModbus();                                            // ��������� ��������� MODBUS

	regs_out[0]= 0x2B;                                            // ��� ������� ����� ����������� � ��������� 43
	regs_out[1]= 0xC4;                                            // 196 �������� � �������� �����
	regs_out[2]= 0x7F;                                            // 127 �������� � �������� �����

	regBank.set(21,0);                                            // XP2-2     sensor "���."  
	regBank.set(22,0);                                            // XP5-3     sensor "��C."
	regBank.set(23,0);                                            // XP3-3     sensor "��-�����1."
	regBank.set(24,0);                                            // XP4-3     sensor "��-�����2."
	//regBank.set(8,1);                                           // �������� ������� ��������
	regBank.set(40,0);                                            // ����������� ���������
	UpdateRegs();                                                 // �������� ���������� � ���������

	Serial.println("Initializing SD card...");
	pinMode(49, OUTPUT);                                          // ��������� ������ SD
	if (!sd.begin(chipSelect)) 
		{
			Serial.println("initialization SD failed!");
			regBank.set(125,false); 
		}
	else
		{
			Serial.println("initialization SD successfully.");
			regBank.set(125,true); 
		}

	SdFile::dateTimeCallback(dateTime);                          // ��������� ������� ������ �����
	regBank.set(40120,0);                                        // �������� ��� ������� �� ����������
	regBank.set(adr_reg_count_err,0);                            // �������� ������ �������� ���� ������
	MsTimer2::set(30, flash_time);                               // 30ms ������ ������� ���������
	resistor(1, 200);                                            // ���������� ������� �������
	resistor(2, 200);                                            // ���������� ������� �������
	preob_num_str();                                             // �������� ��������� ��� ����� 
	//list_file();                                               // ����� ������ ������ � ��� ����  
	//controlFileName();
	//default_mem_porog();
	prer_Kmerton_On = true;                                      // ��������� ���������� �� ��������
	clear_serial2();                                             // 

	Serial.println("Clear memory and registry");                 //
	for (int i = 120; i <= 131; i++)                             // �������� ����� ������
		{
		   regBank.set(i,0);   
		}
	Reg_count_clear();                                           // �������� �������� ������
	Serial.println("Clear Ok!");                                 //
	// ------------  ��������� �������������� ��� --------------------------
	sbi(ADCSRA,ADPS2) ;                                          // ������� ADCSRA ��� 2 = 1
	cbi(ADCSRA,ADPS1) ;                                          // ������� ADCSRA ��� 1 = 0
	cbi(ADCSRA,ADPS0) ;                                          // ������� ADCSRA ��� 0 = 0

	delay(50);
	set_USB0();
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	MsTimer2::start();                                           // �������� ������ ����������
	regBank.set(adr_control_command,0);  
	Serial.println(" ");                                         //
	Serial.println("System initialization OK!.");                // ���������� � ���������� ���������
	regBank.set(39,1);                                           // �������� ������� ���������
}

void loop()
{
	//control_command();

//	delay(100);
	
	 //Serial.print(regs_out[0],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_out[1],HEX);
	 //Serial.print("--");
	 //Serial.print(regs_out[2],HEX);
	 //Serial.print("    ");
	 //
	 //Serial.print(regBank.get(40004),HEX); 
	 //Serial.print("--");
	 //Serial.print(regBank.get(40005),HEX); 
	 //Serial.print("--");
	 //Serial.print(regBank.get(40006),HEX); 
	 //Serial.print("--");
	 //Serial.println(regBank.get(40007),HEX); 

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    sensor ����������� ��������� ����������� � 2 ����������
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    sensor ����������� ��������� ����������� 



}
