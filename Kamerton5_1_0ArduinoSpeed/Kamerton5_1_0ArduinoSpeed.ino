/*

 Kamerton5_1_0ArduinoSpeed.ino
 Visual Studio 2010
 
 Программа тестирования модуля "Камертон" (Базовый вариант)
 Версия:      - 5_1_0
 Дата:        - 25.07.2016г.
 Организация: - ООО "Децима"
 Автор:       - Мосейчук А.В.
 Версия: Измененная версия от 25.07.2016г.

 Добавлена проверка модулей "Аудио1" и "Аудио2"
 
 Реализовано:
 -
 - прерывание 30мс,
 - передача/прием по СОМ порту,
 - подсчет контролных сумм, связь с Камертоном, 
 - Расширение MCP23017
 - модуль реле, 
 - чтение всех портов, 
 - подключен звуковой генератор
 - Подключена SD память
 - подключены часы, память, 
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

#define  ledPin13  13                               // Назначение светодиодов на плате
#define  ledPin12  12                               // Назначение светодиодов на плате
#define  ledPin11  11                               // Назначение светодиодов на плате
#define  ledPin10  10                               // Назначение светодиодов на плате
#define  Front_led_Blue 14                          // Назначение светодиодов на передней панели
#define  Front_led_Red  15                          // Назначение светодиодов на передней панели

//+++++++++++++++++++++++++++++ Внешняя память +++++++++++++++++++++++++++++++++++++++
int deviceaddress        = 80;                      // Адрес микросхемы памяти
unsigned int eeaddress   =  0;                      // Адрес ячейки памяти
byte hi;                                            // Старший байт для преобразования числа
byte low;                                           // Младший байт для преобразования числа


//  Порты управления платой Камертон
#define DTR  8                                      // DTR out выходной сигнал  сформировать 0 для старта
#define RTS  9                                      // RTS out выходной сигнал   
#define CTS  5                                      // CTS in  входной сигнал  флаг нажатия тангенты контролировать!!!!
#define DSR  6                                      // DSR in  входной сигнал  флаг нажатия "Связь - передача"
#define DCD  7                                      // DCD in  входной сигнал  флаг снятия трубки с ложемента 


//  Порты управления платой Arduino Nano
#define  kn1Nano   34                               // Назначение кнопок управления Nano генератор качения
#define  kn2Nano   36                               // Назначение кнопок управления Nano генератор 1000 гц
#define  kn3Nano   38                               // Назначение кнопок управления Nano генератор 2000 гц
#define  InNano12  40                               // Назначение входов - индикация генератор 1000 или 2000 гц
#define  InNano13  39                               // Назначение входов - индикация генератор качения 
bool var_sound = false;                             // Признак качания частоты звука

// 0 - 500-1000гц
// 1 - 500-3500гц
// 2 - 100-5500гц
// 3 - 100-1000гц
// 4 - 500гц
// 5 - 1000гц
// 6 - 2000гц
// 7 - 3500гц

//+++++++++++++++++++++++ Настройка электронного резистора +++++++++++++++++++++++++++++++++++++
#define address_AD5252   0x2F                       // Адрес микросхемы AD5252  
#define control_word1    0x07                       // Байт инструкции резистор №1
#define control_word2    0x87                       // Байт инструкции резистор №2
byte resistance        = 0x00;                      // Сопротивление 0x00..0xFF - 0Ом..100кОм
//byte level_resist      = 0;                       // Байт считанных данных величины резистора
//-----------------------------------------------------------------------------------------------
unsigned int volume1     = 0;                       //
unsigned int volume_max  = 0;                       //
unsigned int volume_min  = 0;                       //
unsigned int volume_fact = 0;                       //
volatile unsigned int Array_volume[600];                     //
unsigned int Array_min[40];                         //
unsigned int Array_max[40];                         //
unsigned int volume_porog_D = 40;                   // Максимальная величина порога при проверке исправности FrontL,FrontR
unsigned int volume_porog_L = 200;                  // Минимальная величина порога при проверке исправности FrontL,FrontR
float voltage ;
//float voltage_test = 0.60;                        // порог величины синусоиды звука
unsigned int  voltage10 ;
unsigned long number_audio ;   

//==========================================================================================================================

// Запись на ассемблере в регистры (1 sbi) или (0 cbi)
// Необходим для работы с АЦП (изменение частоты тактирования)
// Определяет для установки и сброса бита регистра
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//---------------------------------------------------------------------------------------------------------

//***************************** Назначение аналоговых входов   ****************************************
int analog_tok            = 0;       // Измерение тока питания платы Аудио-1 (не реализовано)
int analog_12V            = 1;       // Измерение напряжения питания 12в. платы Аудио-1
int analog_tok_x10        = 2;       // Измерение тока питания платы Аудио-1 х 10  (не реализовано)
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
int analog_13             = 13;      // Измерение напряжения питания  12в.на разъемах  платы Камертон
int analog_14             = 14;      // Измерение напряжения питания  12в.на разъемах  платы Камертон
int analog_3_6            = 15;      // Измерение напряжения питания 3,6в. на разъемах платы Камертон
int analogIn_oscill       = A0;      // Аналоговый вход осциллографа  

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
MCP23017 mcp_Out1;                                                 // Назначение портов расширения MCP23017  4 A - Out, B - Out
MCP23017 mcp_Out2;                                                 // Назначение портов расширения MCP23017  6 A - Out, B - Out
MCP23017 mcp_Analog;                                               // Назначение портов расширения MCP23017  5 A - Out, B - In
//----------------------------------------------------------------------------------------------
const int adr_reg_ind_CTS      PROGMEM           = 10081;          // Адрес флагa индикации состояния сигнала CTS
const int adr_reg_ind_DSR      PROGMEM           = 10082;          // Адрес флагa индикации состояния сигнала DSR
const int adr_reg_ind_DCD      PROGMEM           = 10083;          // Адрес флагa индикации состояния сигнала DCD

// **************** Адреса внешней памяти для хранения даты. Применяется при формировании имени файла *************
//const int adr_temp_day         PROGMEM           = 240;          // Адрес хранения переменной день
//const int adr_temp_mon         PROGMEM           = 241;          // Адрес хранения переменной месяц
//const int adr_temp_year        PROGMEM           = 242;          // Адрес хранения переменной год  
//const int adr_file_name_count  PROGMEM           = 243;          // Адрес хранения переменной счетчика номера файла
//------------------------------------------------------------------------------------------------------------------
int regcount_err        = 0;                                       // Переменная для хранения всех ошибок

//++++++++++++++++++++++ Работа с файлами +++++++++++++++++++++++++++++++++++++++
//#define chipSelect SS
#define chipSelect 49                                              // Настройка выбора SD
SdFat sd;
File myFile;
SdFile file;
Sd2Card card;
uint32_t cardSizeBlocks;
uint16_t cardCapacityMB;

// cache for SD block
cache_t cache;

//------------------------------------------------------------------------------
// созданы переменные, использующие функции библиотеки SD utility library functions: +++++++++++++++
// Change spiSpeed to SPI_FULL_SPEED for better performance
// Use SPI_QUARTER_SPEED for even slower SPI bus speed
const uint8_t spiSpeed = SPI_HALF_SPEED;


//++++++++++++++++++++ Назначение имени файла ++++++++++++++++++++++++++++++++++++++++++++
//const uint32_t FILE_BLOCK_COUNT = 256000;
// log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "160101"
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[13]            = FILE_BASE_NAME "00.KAM";
char fileName_p[13];
char fileName_F[13];
//------------------------------------------------------------------------------

char c;  // Для ввода символа с ком порта

// Serial output stream
//ArduinoOutStream cout(Serial);
//char bufferSerial2[128];  

//*********************Работа с именем файла ******************************

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

//+++++++++++++++++++++ Установки прерывания +++++++++++++++++++++++++++++++

unsigned int sampleCount1 = 0;

//+++++++++++++++++++ MODBUS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

modbusDevice regBank;
//Create the modbus slave protocol handler
modbusSlave slave;

//byte regs_in[5];                                  // Регистры работы с платой Аудио-1 CPLL
byte regs_out[4];                                   // Регистры работы с платой Аудио-1
byte regs_crc[1];                                   // Регистры работы с платой Аудио-1 контрольная сумма
byte regs_temp = 0;
byte regs_temp1 = 0;
byte Stop_Kam = 0;                                  // Флаг индикации чтения инф. из платы Аудио-1
volatile bool prer_Kmerton_On = true;               // Флаг разрешение прерывания Аудио-1
bool test_repeat     = true;                        // Флаг повторения теста
volatile bool prer_Kmerton_Run = false;             // Флаг разрешение прерывания Аудио-1
#define BUFFER_SIZEK 64                             // Размер буфера Камертон не более 128 байт
#define BUFFER_SIZEKF 128                           // Размер буфера Serial2 не более 128 байт
unsigned char bufferK;                              // Счетчик количества принимаемых байт

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Текущее время 
const unsigned int adr_kontrol_day        PROGMEM      = 40046;  // адрес день
const unsigned int adr_kontrol_month      PROGMEM      = 40047;  // адрес месяц
const unsigned int adr_kontrol_year       PROGMEM      = 40048;  // адрес год
const unsigned int adr_kontrol_hour       PROGMEM      = 40049;  // адрес час
const unsigned int adr_kontrol_minute     PROGMEM      = 40050;  // адрес минута
const unsigned int adr_kontrol_second     PROGMEM      = 40051;  // адрес секунда

// Установка времени в контроллере
const unsigned int adr_set_kontrol_day    PROGMEM      = 40052;  // адрес день
const unsigned int adr_set_kontrol_month  PROGMEM      = 40053;  // адрес месяц
const unsigned int adr_set_kontrol_year   PROGMEM      = 40054;  // адрес год
const unsigned int adr_set_kontrol_hour   PROGMEM      = 40055;  // адрес час
const unsigned int adr_set_kontrol_minute PROGMEM      = 40056;  // адрес минута

// Время старта теста
const unsigned int adr_Mic_Start_day      PROGMEM      = 40096;  // адрес день
const unsigned int adr_Mic_Start_month    PROGMEM      = 40097;  // адрес месяц
const unsigned int adr_Mic_Start_year     PROGMEM      = 40098;  // адрес год
const unsigned int adr_Mic_Start_hour     PROGMEM      = 40099;  // адрес час
const unsigned int adr_Mic_Start_minute   PROGMEM      = 40100;  // адрес минута
const unsigned int adr_Mic_Start_second   PROGMEM      = 40101;  // адрес секунда
// Время окончания теста
const unsigned int adr_Mic_Stop_day       PROGMEM       = 40102; // адрес день
const unsigned int adr_Mic_Stop_month     PROGMEM       = 40103; // адрес месяц
const unsigned int adr_Mic_Stop_year      PROGMEM       = 40104; // адрес год
const unsigned int adr_Mic_Stop_hour      PROGMEM       = 40105; // адрес час
const unsigned int adr_Mic_Stop_minute    PROGMEM       = 40106; // адрес минута
const unsigned int adr_Mic_Stop_second    PROGMEM       = 40107; // адрес секунда

// Продолжительность выполнения теста
const unsigned int adr_Time_Test_day      PROGMEM       = 40108; // адрес день
const unsigned int adr_Time_Test_hour     PROGMEM       = 40109; // адрес час
const unsigned int adr_Time_Test_minute   PROGMEM       = 40110; // адрес минута
const unsigned int adr_Time_Test_second   PROGMEM       = 40111; // адрес секунда
// Адрес текущего файла
const unsigned int adr_reg_temp_year      PROGMEM       = 40112; // Регистр хранения переменной год  
const unsigned int adr_reg_temp_mon       PROGMEM       = 40113; // Регистр хранения переменной месяц
const unsigned int adr_reg_temp_day       PROGMEM       = 40114; // Регистр хранения переменной день 
const unsigned int adr_reg_file_name      PROGMEM       = 40115; // Регистр хранения счетчик файлов  
const unsigned int adr_reg_file_tek       PROGMEM       = 40116; // Регистр хранения счетчик файлов  

const unsigned int adr_control_command    PROGMEM       = 40120; // Адрес передачи комманд на выполнение 
const unsigned int adr_reg_count_err      PROGMEM       = 40121; // Адрес счетчика всех ошибок

const unsigned int adr_set_time           PROGMEM       = 36;    // адрес флаг установки



//+++++++++ Адреса внешней памяти для хранения счетчиков ошибок и уровней сигналов +++++


const unsigned int adr_reg40000      PROGMEM       =    1024;  // 
const unsigned int adr_reg40001      PROGMEM       =    1026;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40002      PROGMEM       =    1028;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40003      PROGMEM       =    1030;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40004      PROGMEM       =    1032;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40005      PROGMEM       =    1034;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40006      PROGMEM       =    1036;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40007      PROGMEM       =    1038;  // Регистры обмена с Аудио 1
const unsigned int adr_reg40008      PROGMEM       =    1040;  // 
const unsigned int adr_reg40009      PROGMEM       =    1042;  // 

const unsigned int adr_reg40010      PROGMEM       =    1044;  // №  Аудио 1
const unsigned int adr_reg40011      PROGMEM       =    1046;  // №  Аудио 1
const unsigned int adr_reg40012      PROGMEM       =    1048;  // №  Аудио 1
const unsigned int adr_reg40013      PROGMEM       =    1050;  // №  Аудио 1
const unsigned int adr_reg40014      PROGMEM       =    1052;  // 
const unsigned int adr_reg40015      PROGMEM       =    1054;  // 
const unsigned int adr_reg40016      PROGMEM       =    1056;  // 
const unsigned int adr_reg40017      PROGMEM       =    1058;  // 
const unsigned int adr_reg40018      PROGMEM       =    1060;  // 
const unsigned int adr_reg40019      PROGMEM       =    1062;  // 
const unsigned int adr_reg40040      PROGMEM       =    1064;  // analogIn_oscill   Записать номер аналогового канала Камертон 5.0
const unsigned int adr_reg40041      PROGMEM       =    1066;  //  Установить частоту для измерения осциллографом (0-7)

						// Текущее время 
const unsigned int adr_reg40046      PROGMEM       =    1068;  // адрес день модуля часов контроллера
const unsigned int adr_reg40047      PROGMEM       =    1070;  // адрес месяц модуля часов контроллера
const unsigned int adr_reg40048      PROGMEM       =    1072;  // адрес год модуля часов контроллера
const unsigned int adr_reg40049      PROGMEM       =    1074;  // адрес час модуля часов контроллера
const unsigned int adr_reg40050      PROGMEM       =    1076;  // адрес минута модуля часов контроллера
const unsigned int adr_reg40051      PROGMEM       =    1078;  // адрес секунда модуля часов контроллера
 
						// Установка времени в контроллере
const unsigned int adr_reg40052      PROGMEM       =    1080;  // адрес день
const unsigned int adr_reg40053      PROGMEM       =    1082;  // адрес месяц
const unsigned int adr_reg40054      PROGMEM       =    1084;  // адрес год
const unsigned int adr_reg40055      PROGMEM       =    1086;  // адрес час
const unsigned int adr_reg40056      PROGMEM       =    1088;  // адрес минута
const unsigned int adr_reg40057      PROGMEM       =    1090;  // 
const unsigned int adr_reg40058      PROGMEM       =    1092;  // 
const unsigned int adr_reg40059      PROGMEM       =    1094;  // 
	
const unsigned int adr_reg40060      PROGMEM       =    1096;  // Адрес хранения величины сигнала резистором № 1
const unsigned int adr_reg40061      PROGMEM       =    1098;  // Адрес хранения величины яркости для управления
const unsigned int adr_reg40062      PROGMEM       =    1100;  // Адрес хранения величины яркости для передачи в программу
const unsigned int adr_reg40063      PROGMEM       =    1102;  // Адрес хранения длительности импульса яркости для передачи в программу ПК
const unsigned int adr_reg40064      PROGMEM       =    1104;  // Адрес хранения величины сигнала резистором № 2

const unsigned int adr_reg40065      PROGMEM       =    1106;  // адрес ошибки
const unsigned int adr_reg40066      PROGMEM       =    1108;  // адрес ошибки
const unsigned int adr_reg40067      PROGMEM       =    1110;  // адрес ошибки
const unsigned int adr_reg40068      PROGMEM       =    1112;  // адрес ошибки
const unsigned int adr_reg40069      PROGMEM       =    1114;  // адрес ошибки
const unsigned int adr_reg40070      PROGMEM       =    1116;  // адрес ошибки
const unsigned int adr_reg40071      PROGMEM       =    1118;  // адрес ошибки

const unsigned int adr_reg40072      PROGMEM       =    1120;  // адрес ошибки в %
const unsigned int adr_reg40073      PROGMEM       =    1122;  // адрес ошибки в %
const unsigned int adr_reg40074      PROGMEM       =    1124;  // адрес ошибки в %
const unsigned int adr_reg40075      PROGMEM       =    1126;  // адрес ошибки %
const unsigned int adr_reg40076      PROGMEM       =    1128;  // адрес ошибки %
const unsigned int adr_reg40077      PROGMEM       =    1130;  // адрес ошибки %
const unsigned int adr_reg40078      PROGMEM       =    1132;  // адрес ошибки %
const unsigned int adr_reg40079      PROGMEM       =    1134;  // адрес ошибки %
const unsigned int adr_reg40080      PROGMEM       =    1136;  // адрес ошибки %
const unsigned int adr_reg40081      PROGMEM       =    1138;  // адрес ошибки %
const unsigned int adr_reg40082      PROGMEM       =    1140;  // адрес ошибки %
const unsigned int adr_reg40083      PROGMEM       =    1142;  // адрес ошибки %

// Время ошибки на включение
const unsigned int adr_reg40084      PROGMEM       =    1144;  // адрес день adr_Mic_On_day 
const unsigned int adr_reg40085      PROGMEM       =    1146;  // адрес месяц adr_Mic_On_month  
const unsigned int adr_reg40086      PROGMEM       =    1148;  // адрес год adr_Mic_On_year  
const unsigned int adr_reg40087      PROGMEM       =    1150;  // адрес час adr_Mic_On_hour 
const unsigned int adr_reg40088      PROGMEM       =    1152;  // адрес минута adr_Mic_On_minute 
const unsigned int adr_reg40089      PROGMEM       =    1154;  // адрес секунда  adr_Mic_On_second    

// Время ошибки на выключение
const unsigned int adr_reg40090      PROGMEM       =    1156;  // адрес день adr_Mic_Off_day    
const unsigned int adr_reg40091      PROGMEM       =    1158;  // адрес месяц  adr_Mic_Off_month 
const unsigned int adr_reg40092      PROGMEM       =    1160;  // адрес год adr_Mic_Off_year  
const unsigned int adr_reg40093      PROGMEM       =    1162;  // адрес час adr_Mic_Off_hour   
const unsigned int adr_reg40094      PROGMEM       =    1164;  // адрес минута adr_Mic_Off_minute   
const unsigned int adr_reg40095      PROGMEM       =    1166;  // адрес секунда adr_Mic_Off_second    
	
// Время старта теста
const unsigned int adr_reg40096      PROGMEM       =    1168;  // адрес день  adr_Mic_Start_day    
const unsigned int adr_reg40097      PROGMEM       =    1170;  // адрес месяц adr_Mic_Start_month  
const unsigned int adr_reg40098      PROGMEM       =    1172;  // адрес год adr_Mic_Start_year  
const unsigned int adr_reg40099      PROGMEM       =    1174;  // адрес час adr_Mic_Start_hour 
const unsigned int adr_reg40100      PROGMEM       =    1176;  // адрес минута adr_Mic_Start_minute 
const unsigned int adr_reg40101      PROGMEM       =    1178;  // адрес секунда adr_Mic_Start_second  

// Время окончания теста
const unsigned int adr_reg40102      PROGMEM       =    1180;  // адрес день adr_Mic_Stop_day 
const unsigned int adr_reg40103      PROGMEM       =    1182;  // адрес месяц adr_Mic_Stop_month 
const unsigned int adr_reg40104      PROGMEM       =    1184;  // адрес год adr_Mic_Stop_year
const unsigned int adr_reg40105      PROGMEM       =    1186;  // адрес час adr_Mic_Stop_hour 
const unsigned int adr_reg40106      PROGMEM       =    1188;  // адрес минута adr_Mic_Stop_minute  
const unsigned int adr_reg40107      PROGMEM       =    1190;  // адрес секунда adr_Mic_Stop_second 

// Продолжительность выполнения теста
const unsigned int adr_reg40108      PROGMEM       =    1192;  // адрес день adr_Time_Test_day 
const unsigned int adr_reg40109      PROGMEM       =    1194;  // адрес час adr_Time_Test_hour 
const unsigned int adr_reg40110      PROGMEM       =    1196;  // адрес минута adr_Time_Test_minute
const unsigned int adr_reg40111      PROGMEM       =    1198;  // адрес секунда adr_Time_Test_second

// Имя файла
const unsigned int adr_reg40112      PROGMEM       =    1200;  // Адрес хранения переменной год 
const unsigned int adr_reg40113      PROGMEM       =    1202;  // Адрес хранения переменной месяц 
const unsigned int adr_reg40114      PROGMEM       =    1204;  // Адрес хранения переменной день
const unsigned int adr_reg40115      PROGMEM       =    1206;  // Адрес хранения переменной счетчика последнего номера файла
const unsigned int adr_reg40116      PROGMEM       =    1208;  // Адрес хранения переменной счетчика текущего номера файла

const unsigned int adr_reg40120      PROGMEM       =    1210;  // adr_control_command Адрес передачи комманд на выполнение
const unsigned int adr_reg40121      PROGMEM       =    1212;  // Адрес счетчика всех ошибок
const unsigned int adr_reg40122      PROGMEM       =    1214;  //
const unsigned int adr_reg40123      PROGMEM       =    1216;  //
const unsigned int adr_reg40124      PROGMEM       =    1218;  //
const unsigned int adr_reg40125      PROGMEM       =    1220;  //  
const unsigned int adr_reg40126      PROGMEM       =    1222;  //  
const unsigned int adr_reg40127      PROGMEM       =    1224;  //  Адрес блока регистров для передачи в ПК уровней порогов.
const unsigned int adr_reg40128      PROGMEM       =    1226;  //  Адрес блока памяти для передачи в ПК уровней порогов.
const unsigned int adr_reg40129      PROGMEM       =    1228;  //  Адрес длины блока памяти для передачи в ПК уровней порогов.
//--------------------------------------------------------------------------------------
const unsigned int adr_reg40130      PROGMEM       =    1230;  //  Регистры временного хранения для передачи уровней порогов 
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
	
const unsigned int adr_reg40200      PROGMEM       =    1310;  // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
const unsigned int adr_reg40400      PROGMEM       =    1312;  // Aдрес напряжение ADC0  ток x1 
const unsigned int adr_reg40201      PROGMEM       =    1314;  // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
const unsigned int adr_reg40401      PROGMEM       =    1316;  // Aдрес напряжение ADC1 напряжение 12/3 вольт
const unsigned int adr_reg40202      PROGMEM       =    1318;  // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
const unsigned int adr_reg40402      PROGMEM       =    1320;  // Aдрес напряжение ADC2 ток x10
const unsigned int adr_reg40203      PROGMEM       =    1322;  // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
const unsigned int adr_reg40403      PROGMEM       =    1324;  // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio1
const unsigned int adr_reg40204      PROGMEM       =    1326;  // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
const unsigned int adr_reg40404      PROGMEM       =    1328;  // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio2
const unsigned int adr_reg40205      PROGMEM       =    1330;  // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
const unsigned int adr_reg40405      PROGMEM       =    1332;  // Aдрес напряжение ADC14 напряжение 12/3 вольт ГГС
const unsigned int adr_reg40206      PROGMEM       =    1334;  // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
const unsigned int adr_reg40406      PROGMEM       =    1336;  // Aдрес напряжение ADC15 напряжение светодиода 3,6 вольта
const unsigned int adr_reg40207      PROGMEM       =    1338;  // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
const unsigned int adr_reg40407      PROGMEM       =    1340;  // Aдрес 
const unsigned int adr_reg40208      PROGMEM       =    1342;  // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
const unsigned int adr_reg40408      PROGMEM       =    1344;  // Aдрес 
const unsigned int adr_reg40209      PROGMEM       =    1346;  // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
const unsigned int adr_reg40409      PROGMEM       =    1348;  // Aдрес  

const unsigned int adr_reg40210      PROGMEM       =    1350;  // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
const unsigned int adr_reg40410      PROGMEM       =    1352;  // Aдрес счетчика 
const unsigned int adr_reg40211      PROGMEM       =    1354;  // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
const unsigned int adr_reg40411      PROGMEM       =    1356;  // Aдрес счетчика  
const unsigned int adr_reg40212      PROGMEM       =    1358;  // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
const unsigned int adr_reg40412      PROGMEM       =    1360;  // Aдрес счетчика  
const unsigned int adr_reg40213      PROGMEM       =    1362;  // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
const unsigned int adr_reg40413      PROGMEM       =    1364;  // Aдрес счетчика  
const unsigned int adr_reg40214      PROGMEM       =    1366;  // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
const unsigned int adr_reg40414      PROGMEM       =    1368;  // Aдрес счетчика  
const unsigned int adr_reg40215      PROGMEM       =    1370;  // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
const unsigned int adr_reg40415      PROGMEM       =    1372;  // Aдрес счетчика  
const unsigned int adr_reg40216      PROGMEM       =    1374;  // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
const unsigned int adr_reg40416      PROGMEM       =    1376;  // Aдрес счетчика  
const unsigned int adr_reg40217      PROGMEM       =    1378;  // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
const unsigned int adr_reg40417      PROGMEM       =    1380;  // Aдрес счетчика  
const unsigned int adr_reg40218      PROGMEM       =    1382;  // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
const unsigned int adr_reg40418      PROGMEM       =    1384;  // Aдрес счетчика  
const unsigned int adr_reg40219      PROGMEM       =    1386;  // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
const unsigned int adr_reg40419      PROGMEM       =    1388;  // Aдрес счетчика  

const unsigned int adr_reg40220      PROGMEM       =    1390;  // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        OFF - ";
const unsigned int adr_reg40420      PROGMEM       =    1392;  // Aдрес  ;
const unsigned int adr_reg40221      PROGMEM       =    1394;  // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        ON  - ";
const unsigned int adr_reg40421      PROGMEM       =    1396;  // Aдрес  ;
const unsigned int adr_reg40222      PROGMEM       =    1398;  // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
const unsigned int adr_reg40422      PROGMEM       =    1400;  // Aдрес  ;
const unsigned int adr_reg40223      PROGMEM       =    1402;  // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
const unsigned int adr_reg40423      PROGMEM       =    1404;  // Aдрес  ;
const unsigned int adr_reg40224      PROGMEM       =    1406;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal LineL                    ON  - ";
const unsigned int adr_reg40424      PROGMEM       =    1408;  // Aдрес данных измерения "Test headset instructor ** Signal LineL                    ON  - ";
const unsigned int adr_reg40225      PROGMEM       =    1410;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal LineR                    ON  - ";   
const unsigned int adr_reg40425      PROGMEM       =    1412;  // Aдрес данных измерения "Test headset instructor ** Signal LineR                    ON  - ";   
const unsigned int adr_reg40226      PROGMEM       =    1414;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40426      PROGMEM       =    1416;  // Aдрес данных измерения "Test headset instructor ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40227      PROGMEM       =    1418;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal LineL                    ON  - ";
const unsigned int adr_reg40427      PROGMEM       =    1420;  // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                    ON  - ";
const unsigned int adr_reg40228      PROGMEM       =    1422;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal LineR                    ON  - ";  
const unsigned int adr_reg40428      PROGMEM       =    1424;  // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                    ON  - ";  
const unsigned int adr_reg40229      PROGMEM       =    1426;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal Mag phone                ON  - ";
const unsigned int adr_reg40429      PROGMEM       =    1428;  // Aдрес данных измерения "Test headset dispatcher ** Signal Mag phone                ON  - ";

const unsigned int adr_reg40230      PROGMEM       =    1430;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40430      PROGMEM       =    1432;  // Aдрес данных измерения "Test headset instructor ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40231      PROGMEM       =    1434;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40431      PROGMEM       =    1436;  // Aдрес данных измерения "Test headset instructor ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40232      PROGMEM       =    1438;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal LineL                    OFF - ";
const unsigned int adr_reg40432      PROGMEM       =    1440;  // Aдрес данных измерения "Test headset instructor ** Signal LineL                    OFF - ";
const unsigned int adr_reg40233      PROGMEM       =    1442;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal LineR                    OFF - ";
const unsigned int adr_reg40433      PROGMEM       =    1444;  // Aдрес данных измерения "Test headset instructor ** Signal LineR                    OFF - ";
const unsigned int adr_reg40234      PROGMEM       =    1446;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40434      PROGMEM       =    1448;  // Aдрес данных измерения "Test headset instructor ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40235      PROGMEM       =    1450;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal mag phone                OFF - ";
const unsigned int adr_reg40435      PROGMEM       =    1452;  // Aдрес данных измерения "Test headset instructor ** Signal mag phone                OFF - ";
const unsigned int adr_reg40236      PROGMEM       =    1454;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal GGS                      OFF - ";
const unsigned int adr_reg40436      PROGMEM       =    1456;  // Aдрес данных измерения "Test headset instructor ** Signal GGS                      OFF - ";
const unsigned int adr_reg40237      PROGMEM       =    1458;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40437      PROGMEM       =    1460;  // Aдрес данных измерения "Test headset instructor ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40238      PROGMEM       =    1462;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal GG Radio2                OFF - ";
const unsigned int adr_reg40438      PROGMEM       =    1464;  // Aдрес данных измерения "Test headset instructor ** Signal GG Radio2                OFF - ";
const unsigned int adr_reg40239      PROGMEM       =    1466;  // Aдрес счетчика ошибки ADC0  ток x1 
const unsigned int adr_reg40439      PROGMEM       =    1468;  // Aдрес данных измерения ADC0  ток x1 

const unsigned int adr_reg40240      PROGMEM       =    1470;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40440      PROGMEM       =    1472;  // Aдрес данных измерения "Test headset dispatcher ** Signal FrontL                   OFF - ";
const unsigned int adr_reg40241      PROGMEM       =    1474;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40441      PROGMEM       =    1476;  // Aдрес данных измерения "Test headset dispatcher ** Signal FrontR                   OFF - ";
const unsigned int adr_reg40242      PROGMEM       =    1478;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal LineL                    OFF - "; 
const unsigned int adr_reg40442      PROGMEM       =    1480;  // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                    OFF - "; 
const unsigned int adr_reg40243      PROGMEM       =    1482;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal LineR                    OFF - ";
const unsigned int adr_reg40443      PROGMEM       =    1484;  // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                    OFF - ";
const unsigned int adr_reg40244      PROGMEM       =    1486;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40444      PROGMEM       =    1488;  // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                OFF - "; 
const unsigned int adr_reg40245      PROGMEM       =    1490;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal mag phone                OFF - ";
const unsigned int adr_reg40445      PROGMEM       =    1492;  // Aдрес данных измерения "Test headset dispatcher ** Signal mag phone                OFF - ";
const unsigned int adr_reg40246      PROGMEM       =    1494;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal GGS                      OFF - "; 
const unsigned int adr_reg40446      PROGMEM       =    1496;  // Aдрес данных измерения "Test headset dispatcher ** Signal GGS                      OFF - "; 
const unsigned int adr_reg40247      PROGMEM       =    1498;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40447      PROGMEM       =    1500;  // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio1                OFF - ";
const unsigned int adr_reg40248      PROGMEM       =    1502;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal GG Radio2                OFF - "; 
const unsigned int adr_reg40448      PROGMEM       =    1504;  // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio2                OFF - "; 
const unsigned int adr_reg40249      PROGMEM       =    1506;  // Aдрес счетчика ошибки ADC2 ток x10
const unsigned int adr_reg40449      PROGMEM       =    1508;  // Aдрес данных измерения ADC2 ток x10

const unsigned int adr_reg40250      PROGMEM       =    1510;  // Aдрес счетчика ошибки  "Test MTT ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40450      PROGMEM       =    1512;  // Aдрес данных измерения "Test MTT ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40251      PROGMEM       =    1514;  // Aдрес счетчика ошибки  "Test MTT ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40451      PROGMEM       =    1516;  // Aдрес данных измерения "Test MTT ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40252      PROGMEM       =    1518;  // Aдрес счетчика ошибки  "Test MTT ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40452      PROGMEM       =    1520;  // Aдрес данных измерения "Test MTT ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40253      PROGMEM       =    1522;  // Aдрес счетчика ошибки  "Test MTT ** Signal LineR                                   OFF - "; 
const unsigned int adr_reg40453      PROGMEM       =    1524;  // Aдрес данных измерения "Test MTT ** Signal LineR                                   OFF - "; 
const unsigned int adr_reg40254      PROGMEM       =    1526;  // Aдрес счетчика ошибки  "Test MTT ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40454      PROGMEM       =    1528;  // Aдрес данных измерения "Test MTT ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40255      PROGMEM       =    1530;  // Aдрес счетчика ошибки  "Test MTT ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40455      PROGMEM       =    1532;  // Aдрес данных измерения "Test MTT ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40256      PROGMEM       =    1534;  // Aдрес счетчика ошибки  "Test MTT ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40456      PROGMEM       =    1536;  // Aдрес данных измерения "Test MTT ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40257      PROGMEM       =    1538;  // Aдрес счетчика ошибки  "Test MTT ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40457      PROGMEM       =    1540;  // Aдрес данных измерения "Test MTT ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40258      PROGMEM       =    1542;  // Aдрес счетчика ошибки  "Test MTT ** Signal GG Radio2                               OFF - "; 
const unsigned int adr_reg40458      PROGMEM       =    1544;  // Aдрес данных измерения "Test MTT ** Signal GG Radio2                               OFF - "; 
const unsigned int adr_reg40259      PROGMEM       =    1546;  // Aдрес счетчика ошибки  "Test MTT ** Signal GGS                                     ON  - ";
const unsigned int adr_reg40459      PROGMEM       =    1548;  // Aдрес данных измерения "Test MTT ** Signal GGS                                     ON  - ";

const unsigned int adr_reg40260      PROGMEM       =    1550;  // Aдрес счетчика ошибки  "Test MTT ** Signal LineL                                   ON  - ";
const unsigned int adr_reg40460      PROGMEM       =    1552;  // Aдрес данных измерения "Test MTT ** Signal LineL                                   ON  - ";
const unsigned int adr_reg40261      PROGMEM       =    1554;  // Aдрес счетчика ошибки  "Test MTT ** Signal LineR                                   ON  - ";  
const unsigned int adr_reg40461      PROGMEM       =    1556;  // Aдрес данных измерения "Test MTT ** Signal LineR                                   ON  - ";  
const unsigned int adr_reg40262      PROGMEM       =    1558;  // Aдрес счетчика ошибки  "Test MTT ** Signal Mag phone                               ON  - ";
const unsigned int adr_reg40462      PROGMEM       =    1560;  // Aдрес данных измерения "Test MTT ** Signal Mag phone                               ON  - ";
const unsigned int adr_reg40263      PROGMEM       =    1562;  // Aдрес счетчика ошибки  "Test MTT PTT    (CTS)                                      OFF - ";
const unsigned int adr_reg40463      PROGMEM       =    1564;  // Aдрес данных измерения "Test MTT PTT    (CTS)                                      OFF - ";
const unsigned int adr_reg40264      PROGMEM       =    1566;  // Aдрес счетчика ошибки  "Test microphone PTT  (CTS)                                 OFF - ";
const unsigned int adr_reg40464      PROGMEM       =    1568;  // 
const unsigned int adr_reg40265      PROGMEM       =    1570;  // Aдрес счетчика ошибки  "Test MTT PTT    (CTS)                                      ON  - ";
const unsigned int adr_reg40465      PROGMEM       =    1572;  // Aдрес данных измерения "Test MTT PTT    (CTS)                                      ON  - ";
const unsigned int adr_reg40266      PROGMEM       =    1574;  // Aдрес счетчика ошибки  "Test microphone PTT  (CTS)                                 ON  - ";
const unsigned int adr_reg40466      PROGMEM       =    1576;  // 
const unsigned int adr_reg40267      PROGMEM       =    1578;  // Aдрес счетчика ошибки  "Test MTT HangUp (DCD)                                      OFF - ";
const unsigned int adr_reg40467      PROGMEM       =    1580;  // Aдрес данных измерения "Test MTT HangUp (DCD)                                      OFF - ";
const unsigned int adr_reg40268      PROGMEM       =    1582;  // Aдрес счетчика ошибки  "Test MTT HangUp (DCD)                                      ON  - ";
const unsigned int adr_reg40468      PROGMEM       =    1584;  // Aдрес данных измерения "Test MTT HangUp (DCD)                                      ON  - ";
const unsigned int adr_reg40269      PROGMEM       =    1586;  // Aдрес счетчика ошибки Длительность регулировки яркости
const unsigned int adr_reg40469      PROGMEM       =    1588;  // Длительность импульса регулировки яркости дисплея

const unsigned int adr_reg40270      PROGMEM       =    1590;  // Aдрес счетчика ошибки  "Command PTT1 tangenta ruchnaja (CTS)                       OFF - ";
const unsigned int adr_reg40470      PROGMEM       =    1592;  // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                       OFF - ";
const unsigned int adr_reg40271      PROGMEM       =    1594;  // Aдрес счетчика ошибки  "Command PTT2 tangenta ruchnaja (DCR)                       OFF - ";
const unsigned int adr_reg40471      PROGMEM       =    1596;  // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                       OFF - ";
const unsigned int adr_reg40272      PROGMEM       =    1598;  // Aдрес счетчика ошибки  "Command PTT1 tangenta ruchnaja (CTS)                       ON  - ";
const unsigned int adr_reg40472      PROGMEM       =    1600;  // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                       ON  - ";
const unsigned int adr_reg40273      PROGMEM       =    1602;  // Aдрес счетчика ошибки  "Command PTT2 tangenta ruchnaja (DCR)                       ON  - ";
const unsigned int adr_reg40473      PROGMEM       =    1604;  // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                       ON  - ";
const unsigned int adr_reg40274      PROGMEM       =    1606;  // Aдрес счетчика ошибки  "Command sensor tangenta ruchnaja                           OFF - ";
const unsigned int adr_reg40474      PROGMEM       =    1608;  // Aдрес данных измерения "Command sensor tangenta ruchnaja                           OFF - ";
const unsigned int adr_reg40275      PROGMEM       =    1610;  // Aдрес счетчика ошибки  "Command sensor tangenta ruchnaja                           ON  - ";
const unsigned int adr_reg40475      PROGMEM       =    1612;  // Aдрес данных измерения "Command sensor tangenta ruchnaja                           ON  - ";
const unsigned int adr_reg40276      PROGMEM       =    1614;  // Aдрес счетчика ошибки  "Command sensor tangenta nognaja                            OFF - ";
const unsigned int adr_reg40476      PROGMEM       =    1616;  // Aдрес данных измерения "Command sensor tangenta nognaja                            OFF - ";
const unsigned int adr_reg40277      PROGMEM       =    1618;  // Aдрес счетчика ошибки  "Command sensor tangenta nognaja                            ON  - ";
const unsigned int adr_reg40477      PROGMEM       =    1620;  // Aдрес данных измерения "Command sensor tangenta nognaja                            ON  - ";
const unsigned int adr_reg40278      PROGMEM       =    1622;  // Aдрес счетчика ошибки  "Command PTT tangenta nognaja (CTS)                         OFF - ";
const unsigned int adr_reg40478      PROGMEM       =    1624;  // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                         OFF - ";
const unsigned int adr_reg40279      PROGMEM       =    1626;  // Aдрес счетчика ошибки  "Command PTT tangenta nognaja (CTS)                         ON  - ";
const unsigned int adr_reg40479      PROGMEM       =    1628;  // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                         ON  - ";

const unsigned int adr_reg40280      PROGMEM       =    1630;  // Aдрес счетчика ошибки  "Test GGS ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40480      PROGMEM       =    1632;  // Aдрес данных измерения "Test GGS ** Signal FrontL                                  OFF - ";
const unsigned int adr_reg40281      PROGMEM       =    1634;  // Aдрес счетчика ошибки  "Test GGS ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40481      PROGMEM       =    1636;  // Aдрес данных измерения "Test GGS ** Signal FrontR                                  OFF - ";
const unsigned int adr_reg40282      PROGMEM       =    1638;  // Aдрес счетчика ошибки  "Test GGS ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40482      PROGMEM       =    1640;  // Aдрес данных измерения "Test GGS ** Signal LineL                                   OFF - ";
const unsigned int adr_reg40283      PROGMEM       =    1642;  // Aдрес счетчика ошибки  "Test GGS ** Signal LineR                                   OFF - ";
const unsigned int adr_reg40483      PROGMEM       =    1644;  // Aдрес данных измерения "Test GGS ** Signal LineR                                   OFF - ";
const unsigned int adr_reg40284      PROGMEM       =    1646;  // Aдрес счетчика ошибки  "Test GGS ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40484      PROGMEM       =    1648;  // Aдрес данных измерения "Test GGS ** Signal mag radio                               OFF - ";
const unsigned int adr_reg40285      PROGMEM       =    1650;  // Aдрес счетчика ошибки  "Test GGS ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40485      PROGMEM       =    1652;  // Aдрес данных измерения "Test GGS ** Signal mag phone                               OFF - ";
const unsigned int adr_reg40286      PROGMEM       =    1654;  // Aдрес счетчика ошибки  "Test GGS ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40486      PROGMEM       =    1656;  // Aдрес данных измерения "Test GGS ** Signal GGS                                     OFF - ";
const unsigned int adr_reg40287      PROGMEM       =    1658;  // Aдрес счетчика ошибки  "Test GGS ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40487      PROGMEM       =    1660;  // Aдрес данных измерения "Test GGS ** Signal GG Radio1                               OFF - ";
const unsigned int adr_reg40288      PROGMEM       =    1662;  // Aдрес счетчика ошибки  "Test GGS ** Signal GG Radio2                               OFF - ";
const unsigned int adr_reg40488      PROGMEM       =    1664;  // Aдрес данных измерения "Test GGS ** Signal GG Radio2                               OFF - ";
const unsigned int adr_reg40289      PROGMEM       =    1666;  // Aдрес счетчика ошибки  "Test GGS ** Signal GGS                                     ON  - ";
const unsigned int adr_reg40489      PROGMEM       =    1668;  // Aдрес данных измерения "Test GGS ** Signal GGS                                     ON  - ";

const unsigned int adr_reg40290      PROGMEM       =    1670;  // Aдрес счетчика ошибки  "Test GGS ** Signal FrontL                                  ON  - ";
const unsigned int adr_reg40490      PROGMEM       =    1672;  // Aдрес данных измерения "Test GGS ** Signal FrontL                                  ON  - ";
const unsigned int adr_reg40291      PROGMEM       =    1674;  // Aдрес счетчика ошибки  "Test GGS ** Signal FrontR                                  ON  - ";
const unsigned int adr_reg40491      PROGMEM       =    1676;  // Aдрес данных измерения "Test GGS ** Signal FrontR                                  ON  - ";
const unsigned int adr_reg40292      PROGMEM       =    1678;  // Aдрес счетчика ошибки  "Test GGS ** Signal mag phone                               ON  - ";
const unsigned int adr_reg40492      PROGMEM       =    1680;  // Aдрес данных измерения "Test GGS ** Signal mag phone                               ON  - ";
const unsigned int adr_reg40293      PROGMEM       =    1682;  // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
const unsigned int adr_reg40493      PROGMEM       =    1684;  // Aдрес данных измерения ADC1 напряжение 12/3 вольт
const unsigned int adr_reg40294      PROGMEM       =    1686;  // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
const unsigned int adr_reg40494      PROGMEM       =    1688;  // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
const unsigned int adr_reg40295      PROGMEM       =    1690;  // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
const unsigned int adr_reg40495      PROGMEM       =    1692;  // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
const unsigned int adr_reg40296      PROGMEM       =    1694;  // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
const unsigned int adr_reg40496      PROGMEM       =    1696;  // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
const unsigned int adr_reg40297      PROGMEM       =    1698;  // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта
const unsigned int adr_reg40497      PROGMEM       =    1700;  // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта
const unsigned int adr_reg40298      PROGMEM       =    1702;  // Aдрес счетчика ошибки  "Test Microphone ** Signal mag phone                        ON  - ";    
const unsigned int adr_reg40498      PROGMEM       =    1704;  // Aдрес данных измерения "Test Microphone ** Signal mag phone                        ON  - "; 
const unsigned int adr_reg40299      PROGMEM       =    1706;  // Aдрес счетчика ошибки  "Test Microphone ** Signal LineL                            ON  - ";   
const unsigned int adr_reg40499      PROGMEM       =    1708;  // Aдрес данных измерения "Test Microphone ** Signal LineL                            ON  - ";   

const unsigned int adr_reg40300      PROGMEM       =    1710;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40500      PROGMEM       =    1712;  // Aдрес данных измерения "Test Radio1 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40301      PROGMEM       =    1714;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40501      PROGMEM       =    1716;  // Aдрес данных измерения "Test Radio1 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40302      PROGMEM       =    1718;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40502      PROGMEM       =    1720;  // Aдрес данных измерения "Test Radio1 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40303      PROGMEM       =    1722;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40503      PROGMEM       =    1724;  // Aдрес данных измерения "Test Radio1 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40304      PROGMEM       =    1726;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40504      PROGMEM       =    1728;  // Aдрес данных измерения "Test Radio1 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40305      PROGMEM       =    1730;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40505      PROGMEM       =    1732;  // Aдрес данных измерения "Test Radio1 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40306      PROGMEM       =    1734;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40506      PROGMEM       =    1736;  // Aдрес данных измерения "Test Radio1 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40307      PROGMEM       =    1738;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40507      PROGMEM       =    1740;  // Aдрес данных измерения "Test Radio1 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40308      PROGMEM       =    1742;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40508      PROGMEM       =    1744;  // Aдрес данных измерения "Test Radio1 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40309      PROGMEM       =    1746;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal Radio1                               ON  - ";
const unsigned int adr_reg40509      PROGMEM       =    1748;  // Aдрес данных измерения "Test Radio1 ** Signal Radio1                               ON  - ";

const unsigned int adr_reg40310      PROGMEM       =    1750;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40510      PROGMEM       =    1752;  // Aдрес данных измерения "Test Radio2 ** Signal FrontL                               OFF - ";
const unsigned int adr_reg40311      PROGMEM       =    1754;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40511      PROGMEM       =    1756;  // Aдрес данных измерения "Test Radio2 ** Signal FrontR                               OFF - ";
const unsigned int adr_reg40312      PROGMEM       =    1758;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40512      PROGMEM       =    1760;  // Aдрес данных измерения "Test Radio2 ** Signal LineL                                OFF - ";
const unsigned int adr_reg40313      PROGMEM       =    1762;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40513      PROGMEM       =    1764;  // Aдрес данных измерения "Test Radio2 ** Signal LineR                                OFF - ";
const unsigned int adr_reg40314      PROGMEM       =    1766;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40514      PROGMEM       =    1768;  // Aдрес данных измерения "Test Radio2 ** Signal mag radio                            OFF - ";
const unsigned int adr_reg40315      PROGMEM       =    1770;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40515      PROGMEM       =    1772;  // Aдрес данных измерения "Test Radio2 ** Signal mag phone                            OFF - ";
const unsigned int adr_reg40316      PROGMEM       =    1774;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40516      PROGMEM       =    1776;  // Aдрес данных измерения "Test Radio2 ** Signal GGS                                  OFF - ";
const unsigned int adr_reg40317      PROGMEM       =    1778;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40517      PROGMEM       =    1780;  // Aдрес данных измерения "Test Radio2 ** Signal GG Radio1                            OFF - ";
const unsigned int adr_reg40318      PROGMEM       =    1782;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40518      PROGMEM       =    1784;  // Aдрес данных измерения "Test Radio2 ** Signal GG Radio2                            OFF - ";
const unsigned int adr_reg40319      PROGMEM       =    1786;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal Radio2                               ON  - ";
const unsigned int adr_reg40519      PROGMEM       =    1788;  // Aдрес данных измерения "Test Radio2 ** Signal Radio2                               ON  - ";

const unsigned int adr_reg40320      PROGMEM       =    1790;  // Aдрес счетчика ошибки  "Test Microphone ** Signal FrontL                           OFF - ";
const unsigned int adr_reg40520      PROGMEM       =    1792;  // Aдрес данных измерения "Test Microphone ** Signal FrontL                           OFF - ";
const unsigned int adr_reg40321      PROGMEM       =    1794;  // Aдрес счетчика ошибки  "Test Microphone ** Signal FrontR                           OFF - ";
const unsigned int adr_reg40521      PROGMEM       =    1796;  // Aдрес данных измерения "Test Microphone ** Signal FrontR                           OFF - ";
const unsigned int adr_reg40322      PROGMEM       =    1798;  // Aдрес счетчика ошибки  "Test Microphone ** Signal LineL                            OFF - ";
const unsigned int adr_reg40522      PROGMEM       =    1800;  // Aдрес данных измерения "Test Microphone ** Signal LineL                            OFF - ";
const unsigned int adr_reg40323      PROGMEM       =    1802;  // Aдрес счетчика ошибки  "Test Microphone ** Signal LineR                            OFF - ";
const unsigned int adr_reg40523      PROGMEM       =    1804;  // Aдрес данных измерения "Test Microphone ** Signal LineR                            OFF - ";
const unsigned int adr_reg40324      PROGMEM       =    1806;  // Aдрес счетчика ошибки  "Test Microphone ** Signal mag radio                        OFF - ";
const unsigned int adr_reg40524      PROGMEM       =    1808;  // Aдрес данных измерения "Test Microphone ** Signal mag radio                        OFF - ";
const unsigned int adr_reg40325      PROGMEM       =    1810;  // Aдрес счетчика ошибки  "Test Microphone ** Signal mag phone                        OFF - ";
const unsigned int adr_reg40525      PROGMEM       =    1812;  // Aдрес данных измерения "Test Microphone ** Signal mag phone                        OFF - ";
const unsigned int adr_reg40326      PROGMEM       =    1814;  // Aдрес счетчика ошибки  "Test Microphone ** Signal GGS                              OFF - ";
const unsigned int adr_reg40526      PROGMEM       =    1816;  // Aдрес данных измерения "Test Microphone ** Signal GGS                              OFF - ";
const unsigned int adr_reg40327      PROGMEM       =    1818;  // Aдрес счетчика ошибки  "Test Microphone ** Signal GG Radio1                        OFF - ";
const unsigned int adr_reg40527      PROGMEM       =    1820;  // Aдрес данных измерения "Test Microphone ** Signal GG Radio1                        OFF - ";
const unsigned int adr_reg40328      PROGMEM       =    1822;  // Aдрес счетчика ошибки  "Test Microphone ** Signal GG Radio2                        OFF - ";
const unsigned int adr_reg40528      PROGMEM       =    1824;  // Aдрес данных измерения "Test Microphone ** Signal GG Radio2                        OFF - ";
const unsigned int adr_reg40329      PROGMEM       =    1826;  // Aдрес счетчика ошибки Код регулировки яркости                             // 
const unsigned int adr_reg40529      PROGMEM       =    1828;  // Код регулировки яркости дисплея

const unsigned int adr_reg40330      PROGMEM       =    1830;  // Aдрес счетчика ошибки  "Test Radio1 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40530      PROGMEM       =    1832;  // Aдрес данных измерения "Test Radio1 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40331      PROGMEM       =    1834;  // Aдрес счетчика ошибки  "Test Radio2 ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40531      PROGMEM       =    1836;  // Aдрес данных измерения "Test Radio2 ** Signal mag radio                            ON  - ";                    // 
const unsigned int adr_reg40332      PROGMEM       =    1838;  // Aдрес счетчика ошибки  "Test GGS    ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40532      PROGMEM       =    1840;  // Aдрес данных измерения "Test GGS    ** Signal mag radio                            ON  - ";
const unsigned int adr_reg40333      PROGMEM       =    1842;  // Aдрес счетчика ошибки  "Test headset instructor ** Signal mag radio                ON  - ";
const unsigned int adr_reg40533      PROGMEM       =    1844;  // Aдрес данных измерения "Test headset instructor ** Signal mag radio                ON  - ";
const unsigned int adr_reg40334      PROGMEM       =    1846;  // Aдрес счетчика ошибки  "Test headset dispatcher ** Signal mag radio                ON  - ";
const unsigned int adr_reg40534      PROGMEM       =    1848;  // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                ON  - ";
const unsigned int adr_reg40335      PROGMEM       =    1850;  // Aдрес счетчика ошибки  "Test MTT ** Signal mag radio                               ON  - ";
const unsigned int adr_reg40535      PROGMEM       =    1852;  // Aдрес данных измерения "Test MTT ** Signal mag radio                               ON  - ";
const unsigned int adr_reg40336      PROGMEM       =    1854;  // Aдрес счетчика ошибки  "Test Microphone ** Signal mag radio                        ON  - "; 
const unsigned int adr_reg40536      PROGMEM       =    1856;  // Aдрес данных измерения "Test Microphone ** Signal mag radio                        ON  - "; 
const unsigned int adr_reg40337      PROGMEM       =    1858;  // Aдрес счетчика ошибки   
const unsigned int adr_reg40537      PROGMEM       =    1860;  // Aдрес данных измерения   
const unsigned int adr_reg40338      PROGMEM       =    1862;  // Aдрес счетчика ошибки   
const unsigned int adr_reg40538      PROGMEM       =    1864;  // Aдрес данных измерения   
const unsigned int adr_reg40339      PROGMEM       =    1866;  // Aдрес счетчика ошибки        
const unsigned int adr_reg40539      PROGMEM       =    1868;  // 

//------------------------- Уровни пороговых значений сигналов при тестировании устройств--------------------------------------
//++++++++++++++++++++++++++++ Заводские установки уровней порогов +++++++++++++++++++++++++++++++++++++

// Адреса внешней памяти для хранения порогов уровней измерения сигналов

const  int adr_set_USB                     = 180;  

//Новые пороги две ячейки, 2 байта
const  int adr_int_porog_instruktor            = 201;          // 201,203 уровень звука, 206 - 276   пороги
const  int adr_int_porog_dispatcher            = 277;          // 277,279 уровень звука, 282 - 352   пороги
const  int adr_int_porog_MTT                   = 353;          // 353,355 уровень звука, 358 - 428   пороги
const  int adr_int_porog_Microphone            = 429;          // 429,431 уровень звука, 434 - 504   пороги
const  int adr_int_porog_GGS                   = 505;          // 505,507 уровень звука, 510 - 580   пороги
const  int adr_int_porog_Radio1                = 581;          // 581,584 уровень звука, 586 - 656   пороги
const  int adr_int_porog_Radio2                = 657;          // 657,659 уровень звука, 662 - 734   пороги

int por_int_buffer[40] ;                                       // Буфер хранения временной информации уровней порогов                      
const unsigned int porog_default[]    PROGMEM  = {             // 270 ячеек
//++++++++++++++++  Test headset instructor ++++++++++++++++++++++++++++
	44,  // [1] resistor(1, 25);     Установить уровень сигнала 30 мв
	44,  // [2] resistor(2, 25);     Установить уровень сигнала 30 мв
	8,   // [3] measure_vol_min(analog_FrontL,   40230,230,18);  уровень сигнала на выходе FrontL  
	8,   // [4] measure_vol_min(analog_FrontL,   40230,230,18);  уровень сигнала на выходе FrontL  
	8,   // [5] measure_vol_min(analog_FrontR,   40231,231,18);  уровень сигнала на выходе FrontR
	8,   // [6] measure_vol_min(analog_FrontR,   40231,231,18);  уровень сигнала на выходе FrontR
	8,   // [7] measure_vol_min(analog_LineL,    40232,232,18);  уровень сигнала на выходе LineL 
	8,   // [8] measure_vol_min(analog_LineL,    40232,232,18);  уровень сигнала на выходе LineL 
	8,   // [9] measure_vol_min(analog_LineR,    40233,233,18);  уровень сигнала на выходе LineR
	8,   //[10] measure_vol_min(analog_LineR,    40233,233,18);  уровень сигнала на выходе LineR
	8,   //[11] measure_vol_min(analog_mag_radio,40234,234,18);  уровень сигнала на выходе mag radio 
	8,   //[12] measure_vol_min(analog_mag_radio,40234,234,18);  уровень сигнала на выходе mag radio 
	8,   //[13] measure_vol_min(analog_mag_phone,40238,238,18);  уровень сигнала на выходе mag phone
	8,   //[14] measure_vol_min(analog_mag_phone,40235,235,15);  уровень сигнала на выходе mag phone
	20,  //[15] measure_vol_min(analog_ggs,      40236,236,15);  уровень сигнала на выходе GGS 
	20,  //[16] measure_vol_min(analog_ggs,      40236,236,15);  уровень сигнала на выходе GGS 
	20,  //[17] measure_vol_min(analog_gg_radio1,40237,237,15);  уровень сигнала на выходе GG Radio1
	20,  //[18] measure_vol_min(analog_gg_radio1,40237,237,15);  уровень сигнала на выходе GG Radio1
	20,  //[19] measure_vol_min(analog_gg_radio2,40238,238,15);  уровень сигнала на выходе GG Radio2 
	20,  //[20] measure_vol_min(analog_gg_radio2,40238,238,15);  уровень сигнала на выходе GG Radio2 
	// ------------- MAX ---------------------------------------------
	0,   //[21]
	0,   //[22]
	0,   //[23]
	0,   //[24]
	75,  //[25] min measure_vol_max(analog_LineL,    40224,224,100); уровень сигнала на выходе LineL
	95,  //[26] max measure_vol_max(analog_LineL,    40224,224,150); уровень сигнала на выходе LineL
	0,   //[27]
	0,   //[28]
	45,  //[29] min measure_vol_max(analog_mag_radio,40333,333,40) ; уровень сигнала на выходе mag phone 
	65,  //[30] max measure_vol_max(analog_mag_radio,40333,333,100); уровень сигнала на выходе mag phone 
	45,  //[31] min measure_vol_max(analog_mag_phone,40226,226,80) ; уровень сигнала на выходе mag phone 
	65,  //[32] max measure_vol_max(analog_mag_phone,40226,226,120); уровень сигнала на выходе mag phone 
	0,   //[33]
	0,   //[34]
	0,   //[35]
	0,   //[36]
	0,   //[37]
	0,   //[38]
//++++++++++++++++  Test headset dispatcher ++++++++++++++++++++++++++++
	44,  //[39] resistor(1, 30);   Установить уровень сигнала 30 мв
	44,  //[40] resistor(2, 30);   Установить уровень сигнала 30 мв

	8,   //[41] measure_vol_min(analog_FrontL,   40240,240,18);  уровень сигнала на выходе FrontL 
	8,   //[42] measure_vol_min(analog_FrontL,   40240,240,18);  уровень сигнала на выходе FrontL 
	8,   //[43] measure_vol_min(analog_FrontR,   40241,241,18);  уровень сигнала на выходе FrontR
	8,   //[44] measure_vol_min(analog_FrontR,   40241,241,18);  уровень сигнала на выходе FrontR
	8,   //[45] measure_vol_min(analog_LineL,    40242,242,38);  уровень сигнала на выходе LineL
	8,   //[46] measure_vol_min(analog_LineL,    40242,242,38);  уровень сигнала на выходе LineL
	8,   //[47] measure_vol_min(analog_LineR,    40243,243,38);  уровень сигнала на выходе LineR
	8,   //[48] measure_vol_min(analog_LineR,    40243,243,38);  уровень сигнала на выходе LineR
	8,   //[49] measure_vol_min(analog_mag_radio,40244,244,35);  уровень сигнала на выходе mag radio
	8,   //[50] measure_vol_min(analog_mag_radio,40244,244,35);  уровень сигнала на выходе mag radio
	8,   //[51] measure_vol_min(analog_mag_phone,40245,245,35);  уровень сигнала на выходе mag phone
	8,   //[52] measure_vol_min(analog_mag_phone,40245,245,35);  уровень сигнала на выходе mag phone
	20,  //[53] measure_vol_min(analog_ggs,      40246,246,35);  уровень сигнала на выходе GGS 
	20,  //[54] measure_vol_min(analog_ggs,      40246,246,35);  уровень сигнала на выходе GGS 
	20,  //[55] measure_vol_min(analog_gg_radio1,40247,247,35);  уровень сигнала на выходе GG Radio1
	20,  //[56] measure_vol_min(analog_gg_radio1,40247,247,35);  уровень сигнала на выходе GG Radio1
	20,  //[57] measure_vol_min(analog_gg_radio2,40248,248,35);  уровень сигнала на выходе GG Radio2 
	20,  //[58] measure_vol_min(analog_gg_radio2,40248,248,35);  уровень сигнала на выходе GG Radio2 
  // ------------- MAX ---------------------------------------------
	0,   //[59]
	0,   //[60]
	0,   //[61]
	0,   //[62]
	75,  //[63] min measure_vol_max(analog_LineL,    40227,227,200); уровень сигнала на выходе LineL
	95,  //[64] max measure_vol_max(analog_LineL,    40227,227,200); уровень сигнала на выходе LineL
	0,   //[65]
	0,   //[66]
	45,  //[67] min  measure_vol_max(analog_mag_radio
	65,  //[68] max  measure_vol_max(analog_mag_radio
	45,  //[69] min  measure_vol_max(analog_mag_phone,40229,229,200); уровень сигнала на выходе mag phone
	65,  //[70] max  measure_vol_max(analog_mag_phone,40229,229,200); уровень сигнала на выходе mag phone
	0,   //[71]
	0,   //[72]
	0,   //[73]
	0,   //[74]
	0,   //[75]
	0,   //[76]

//++++++++++++++++  Test MTT ++++++++++++++++++++++++++++
	173, //[77] resistor(1, 200);  Установить уровень сигнала 60 мв
	173, //[78] resistor(2, 200);  Установить уровень сигнала 60 мв

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

15,			// 2                          // measure_vol_min(analog_FrontL,    40250,250,35); уровень сигнала на выходе FrontL  
15,			// 3                          // measure_vol_min(analog_FrontR,    40251,251,35); уровень сигнала на выходе FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40252,252,35); уровень сигнала на выходе LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40253,253,35); уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40254,254,35); уровень сигнала на выходе mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40255,255,35); уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40256,256,35); уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40257,257,35); уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40258,258,35); уровень сигнала на выходе GG Radio2 
			//++++++++++++++++++++++++++++++++++ Сигнал подан на вход МТТ ++++++++++++++++++++++++++++++++++++++++++++
15,			// 11                         // measure_vol_min(analog_FrontL,    40250,250,35); уровень сигнала на выходе FrontL
15,			// 12                         // measure_vol_min(analog_FrontR,    40251,251,35); уровень сигнала на выходе FrontR
15,			// 13                         // measure_vol_min(analog_mag_radio, 40254,254,35); уровень сигнала на выходе mag radio
15,			// 14                         // measure_vol_min(analog_ggs,       40256,256,35); уровень сигнала на выходе GGS
15,			// 15                         // measure_vol_min(analog_gg_radio1, 40257,257,35); уровень сигнала на выходе GG Radio1
15,			// 16                         // measure_vol_min(analog_gg_radio2, 40258,258,35); уровень сигнала на выходе GG Radio2 
70,	    	// 17                         // measure_vol_max(analog_LineL,     40260,260,35);  "Test MTT ** Signal LineL 
55,	        // 18                         // measure_vol_max(analog_mag_phone, 40262,262, + 18)); 
15,         // 19                         // measure_vol_min(analog_ggs,       40256,256,por_buffer[19]);  // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
120			// 20                         // measure_vol_max(analog_ggs,       40259,259,  + 20));         //  Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS             On 
};




const byte  porog_Microphone[]    PROGMEM = {
		//+++++++++++++++++++++++++++++++++ Test Microphone +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
150,		// 0                          // resistor(1, 200);  Установить уровень сигнала 60 мв
150,		// 1                          // resistor(2, 200);  Установить уровень сигнала 60 мв
15,			// 2                          // measure_vol_min(analog_FrontL,    40320,320,35); уровень сигнала на выходе FrontL
15,			// 3                          // measure_vol_min(analog_FrontR,    40321,321,35); уровень сигнала на выходе FrontR 
15,			// 4                          // measure_vol_min(analog_LineL,     40322,322,35); уровень сигнала на выходе LineL
15,			// 5                          // measure_vol_min(analog_LineR,     40323,323,35); уровень сигнала на выходе LineR
15,			// 6                          // measure_vol_min(analog_mag_radio, 40324,324,35); уровень сигнала на выходе mag radio
15,			// 7                          // measure_vol_min(analog_mag_phone, 40325,325,35); уровень сигнала на выходе mag phone
15,			// 8                          // measure_vol_min(analog_ggs,       40326,326,35); уровень сигнала на выходе GGS 
15,			// 9                          // measure_vol_min(analog_gg_radio1, 40327,327,35); уровень сигнала на выходе GG Radio1
15,			// 10                         // measure_vol_min(analog_gg_radio2, 40328,328,35); уровень сигнала на выходе GG Radio2
			//++++++++++++++++++++++++++++++++++ Сигнал подан на вход микрофона ++++++++++++++++++++++++
60,	    	// 11                         // measure_vol_max(analog_mag_phone, 40298,298,180);уровень сигнала на выходе mag phone
75,		    // 12                         // measure_vol_max(analog_LineL,     40299,299,180);уровень сигнала на выходе "Test Microphone ** Signal LineL 
15,			// 13                         // measure_vol_min(analog_FrontL,    40320,320,35); уровень сигнала на выходе FrontL 
15,			// 14                         // measure_vol_min(analog_FrontR,    40321,321,35); уровень сигнала на выходе FrontR
15,			// 15                         // measure_vol_min(analog_LineR,     40323,323,35); уровень сигнала на выходе LineR
15,			// 16                         // measure_vol_min(analog_mag_radio, 40324,324,35); уровень сигнала на выходе mag radio 
15,			// 17                         // measure_vol_min(analog_ggs,       40326,326,35); уровень сигнала на выходе GGS
15,			// 18                         // measure_vol_min(analog_gg_radio1, 40327,327,35); уровень сигнала на выходе GG Radio1
15          // 19                         // measure_vol_min(analog_gg_radio2, 40328,328,35); уровень сигнала на выходе GG Radio2
};

const byte  porog_GGS[]    PROGMEM = {
	//++++++++++++++++++++++++++++++++ Test GGS ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
230,		//  0                         // resistor(1, 200); Установить уровень сигнала xx мв
230,		//  1                         // resistor(2, 200); Установить уровень сигнала xx мв
				//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15, 		//  2                         // measure_vol_min(analog_FrontL,    40280,280,35); уровень сигнала на выходе "Test GGS ** Signal FrontL 
15, 		//  3                         // measure_vol_min(analog_FrontR,    40281,281,35); уровень сигнала на выходе "Test GGS ** Signal FrontR        
15, 		//  4                         // measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL 
15, 		//  5                         // measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR 
15, 		//  6                         // measure_vol_min(analog_mag_radio, 40284,284,35); уровень сигнала на выходе "Test GGS ** Signal mag radio
15, 		//  7                         // measure_vol_min(analog_mag_phone, 40285,285,35); уровень сигнала на выходе "Test GGS ** Signal mag phone 
15, 		//  8                         // measure_vol_min(analog_ggs,       40286,286,35); уровень сигнала на выходе "Test GGS ** Signal GGS    
15, 		//  9                         // measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1   
15, 		//  10                        // measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2
			   //++++++++++++++++++++++++++++++++++ Сигнал подан на вход GGS ++++++++++++++++++++++++
110,    	// 11    					  //measure_vol_max(analog_FrontL,    40290,290,40); уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
130,	   	// 12  						  //measure_vol_max(analog_FrontR,    40291,291,40); уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";
15,			// 13						  //measure_vol_min(analog_LineL,     40282,282,35); уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
15,			// 14						  //measure_vol_min(analog_LineR,     40283,283,35); уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
16,			// 15	                 	  //measure_vol_max(analog_mag_radio, 40332,332,35); уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
16,	    	// 16						  //measure_vol_max(analog_mag_phone, 40292,292,50); уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
130,	    // 17			     		  //measure_vol_max(analog_ggs,       40286,289,160); уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
15,			// 18						  //measure_vol_min(analog_gg_radio1, 40287,287,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
15			// 19						  //measure_vol_min(analog_gg_radio2, 40288,288,35); уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

};

const byte  porog_Radio1[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio1 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
150,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
15,			// 6							//measure_vol_min(analog_mag_radio, 40304,304,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40307,307,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40308,308,35); Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";
				  //++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio1 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40300,300,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40301,301,35); уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40302,302,35); уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40303,303,35); уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40330,330,35); уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40305,305,35); уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40306,306,35); уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
220,		// 18							//measure_vol_max(analog_gg_radio1, 40309,309,250);уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
15			// 19							//measure_vol_min(analog_gg_radio2, 40308,308,35); уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2       
};

const byte  porog_Radio2[]    PROGMEM = {
// ++++++++++++++++++++++++++++++++++++++ Test Radio2 +++++++++++++++++++++++++++++++++++++++++++++++++++++

150,		// 0							//resistor(1, 250);  Установить уровень сигнала xx мв
150,		// 1							//resistor(2, 250);  Установить уровень сигнала xx мв
				 //+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
15,			// 2							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 3							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 4							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 5							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
15,			// 6 							//measure_vol_min(analog_mag_radio, 40314,314,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 7							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 8							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 9							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
15,			// 10							//measure_vol_min(analog_gg_radio2, 40318,318,35); уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";
					//++++++++++++++++++++++++++++++++++ Сигнал подан на вход  Radio2 ++++++++++++++++++++++++
15,			// 11							//measure_vol_min(analog_FrontL,    40310,310,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
15,			// 12							//measure_vol_min(analog_FrontR,    40311,311,35); уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
15,			// 13							//measure_vol_min(analog_LineL,     40312,312,35); уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
15,			// 14							//measure_vol_min(analog_LineR,     40313,313,35); уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
40,			// 15							//measure_vol_max(analog_mag_radio, 40331,331,35); уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
15,			// 16							//measure_vol_min(analog_mag_phone, 40315,315,35); уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
15,			// 17							//measure_vol_min(analog_ggs,       40316,316,35); уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
15,			// 18							//measure_vol_min(analog_gg_radio1, 40317,317,35); уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
200			// 19							//measure_vol_max(analog_gg_radio2, 40319,319,250);уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2        
};


*/


//---------------------------Тексты сообщений   ---------------------------------------------------
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

// Тексты ошибок
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
const char  txt_error93[]  PROGMEM               = "Error power 12 Volt"                               ; // Ошибка ADC1 напряжение 12/3 вольт"
const char  txt_error94[]  PROGMEM               = "Error power 12 Volt Radio1"                        ; // Ошибки ADC14 напряжение 12/3 вольт Radio1
const char  txt_error95[]  PROGMEM               = "Error power 12 Volt Radio2"                        ; // Oшибки ADC14 напряжение 12/3 вольт Radio2
const char  txt_error96[]  PROGMEM               = "Error power 12 Volt GGS"                           ; // Oшибки ADC14 напряжение 12/3 вольт ГГС
const char  txt_error97[]  PROGMEM               = "Error power 3,6 Volt Led Mic"                      ; // Oшибки ADC15 напряжение светодиода 3,6 вольта
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
txt_message54,                                // " ****** Test miсrophone start! ******"                       ;
txt_message55,                                // "Signal miсrophone 30  mV                      ON"            ;
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

// ========================= Блок программ ============================================

void dateTime(uint16_t* date, uint16_t* time)                  // Программа записи времени и даты файла
{
  DateTime now = RTC.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void serial_print_date()                           // Печать даты и времени    
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
//	DateTime set_time = DateTime(year, month, day, hour, minute, second); // Занести данные о времени в строку "set_time"
//	RTC.adjust(set_time);             
}

void flash_time()                                              // Программа обработчик прерывания 
{ 
	   // PORTB = B00000000; // пин 12 переводим в состояние LOW
		prer_Kmerton_Run = true;
		prer_Kamerton();
		//sendPacketK ();  
		slave.run();
		prer_Kmerton_Run = false;
	   // PORTB = B01000000; // пин 12 переводим в состояние HIGH
}

void serialEvent3()
{

	////wdt_reset();  // Сброс сторожевого таймера при наличии связи с ПК
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
//	if (Serial2.available())                             // есть что-то проверить? Есть данные в буфере?
//		  {
//			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
//			unsigned char buffer_count = 0;                      // Установить в начало чтения буфера
//
//			while (Serial2.available())
//				{
//				  if (overflowFlag)                        // Если буфер переполнен - очистить
//					 Serial2.read();
//				  else                                     // Размер буфера в норме, считать информацию
//					{
//					if (buffer_count == BUFFER_SIZEKF)           // Проверить размер буфера
//						{
//							overflowFlag = 1;              // Установить флаг превышения размера буфера
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
////	wdt_reset();  // Сброс сторожевого таймера при наличии связи с ПК
//}
//void serialEvent1()
//{
//	/*
//	if (Serial1.available())                               // есть что-то проверить? Есть данные в буфере?
//		  {
//			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
//			unsigned char buffer = 0;                      // Установить в начало чтения буфера
//
//			while (Serial1.available())
//				{
//				  if (overflowFlag)                        // Если буфер переполнен - очистить
//					 Serial1.read();
//				  else                                     // Размер буфера в норме, считать информацию
//					{
//					if (bufferK == BUFFER_SIZEK)           // Проверить размер буфера
//						{
//							overflowFlag = 1;              // Установить флаг превышения размера буфера
//						}
//						 regBank.set(40004+buffer,Serial1.read());
//						//regs_in[buffer] = Serial1.read(); 
//						buffer++;
//					}
//				}
////			calculateCRC_In();
//			regBank.set(124,1);                              // Связь с "Камертон" установлена
//		   }
//	 else 
//		{
//			Stop_Kam = 0;                                    // Флаг отсутств. инф. из Камертона
//			regBank.set(124,0);                              // Флаг ошибки  связи с "Камертон"
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
	if (Serial2.available())                             // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer_count = 0;                      // Установить в начало чтения буфера

			while (Serial2.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial2.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (buffer_count == BUFFER_SIZEKF)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
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
void prer_Kamerton()                                          // Произвести обмен информации с модулем Камертон
{
//	clear_serial1();
	sendPacketK ();  
	// Отправить информацию в модуль Камертон
	waiting_for_replyK();                                  // Получить подтверждение
}
void sendPacketK () 
{              // Программа передачи пакета в Камертон
	calculateCRC_Out();
	for (int i = 0; i <3; i++)
		{
			Serial1.write(regs_out[i]);
			regBank.set(40001+i,regs_out[i]);
		}
}
void waiting_for_replyK()                                  // Чтение данных из Камертона
{
	delayMicroseconds(5);
	   //blink_red = !blink_red;
	   //digitalWrite(ledPin13,!digitalRead(ledPin13));
	//  Уточнить задержку и применение Stop_Kam = 0; 
	if (Serial1.available())                               // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;               // Флаг превышения размера буфера
			unsigned char buffer = 0;                      // Установить в начало чтения буфера

			while (Serial1.available())
				{
				  if (overflowFlag)                        // Если буфер переполнен - очистить
					 Serial1.read();
				  else                                     // Размер буфера в норме, считать информацию
					{
					if (bufferK == BUFFER_SIZEK)           // Проверить размер буфера
						{
							overflowFlag = 1;              // Установить флаг превышения размера буфера
						}
						 regBank.set(40004+buffer,Serial1.read());
						//regs_in[buffer] = Serial1.read(); 
						buffer++;
					}
				}
//			calculateCRC_In();
			regBank.set(124,1);                              // Связь с "Камертон" установлена
		   }
	 else 
		{
			Stop_Kam = 0;                                    // Флаг отсутств. инф. из Камертона
			regBank.set(124,0);                              // Флаг ошибки  связи с "Камертон"
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
void Stop_Kamerton ()                  //Если не приходит информация с Камертона - регистры обнулить
  {
	 for (unsigned char i = 0; i <4; i++)
	 {
		 regBank.set(40004+i,0);
		// regs_in[i]=0;
	 }
  }

void calculateCRC_Out()                // Вычисление контрольной суммы ниблов байта
{ 
// Байт из regs_out[0] передается без изменений,в вычислении контрольной суммы не применяется; 

  byte temp1, temp2, temp3, temp4, crc;
  temp1 = regs_out[1];                 // записать  1 байт в temp1
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта XXXX0000
  temp2 = temp1>>4;                    // Переместить старший нибл в младший 0000XXXX(Первый блок готов в temp2)
 
  temp3 = regs_out[2];                 // записать 2 байт в temp3
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 2 байта XXXX0000
  temp3 = temp3>>4;                    // Переместить старший нибл в младший 0000XXXX(Второй блок готов в temp3)
  temp4 = regs_out[2];                 // записать 2 байт в temp4
  temp4 = temp4&0x0F;                  // Наложить маску 0F на младший нибл 2 байта 0000XXXX(Третий блок готов в temp4)
  crc =  temp2 ^  temp3 ^  temp4  ;    // Сложить ниблы трех блоков
  crc = crc&0x0F;                      // Наложить маску 0F на младший нибл crc
  regs_out[1]= temp1 | crc;            // Получить 2 байт с ниблом контрольной суммы
}
void calculateCRC_In()                 // Вычисление контрольной суммы ниблов байта
{ 
	/*
  byte temp1,temp1H,temp1L, temp2,temp2H,temp2L, temp3,temp3H,temp3L, temp4, temp4H, crc_in;

  temp1 = regs_in[0];                  // записать  
  temp1 = temp1&0xF0;                  // Наложить маску F0 на старший нибл 1 байта
  temp1H = temp1>>4;                   // Переместить старший нибл в младший
  temp1 = regs_in[0];                  // записать 
  temp1L = temp1&0x0F;                 // Наложить маску 0F на младший нибл 1 байта

  temp2 = regs_in[1];                  // записать  
  temp2 = temp2&0xF0;                  // Наложить маску F0 на старший нибл 2 байта
  temp2H = temp2>>4;                   // Переместить старший нибл в младший
  temp2 = regs_in[1];                  // записать 
  temp2L = temp2&0x0F;                 // Наложить маску 0F на младший нибл 2 байта

  temp3 = regs_in[2];                  // записать  
  temp3 = temp3&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp3H = temp3>>4;                   // Переместить старший нибл в младший
  temp3 = regs_in[2];                  // записать 
  temp3L = temp3&0x0F;                 // Наложить маску 0F на младший нибл 3 байта

  temp4 = regs_in[3];                  // записать  
  temp4 = temp4&0xF0;                  // Наложить маску F0 на старший нибл 3 байта
  temp4H = temp4>>4;                   // Переместить старший нибл в младший
  crc_in =   temp1H ^  temp1L  ^   temp2H ^  temp2L  ^  temp3H ^  temp3L  ^  temp4H ;
  crc_in =  crc_in&0x0F;               // Наложить маску F0 на младший нибл 4 байта
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
void UpdateRegs()                                        // Обновить регистры
{
	//-----Первый байт ------------
	//-----Установить бит 0
	 while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	 boolean set_rele ;
	 prer_Kmerton_On = false;                            // Запретить прерывание Камертон ??
	// reg_Kamerton();                                     // Записать данные из Камертон в    регистры 
		// Подпрограмма переноса данных из регистров на порты вывода
	  //-----Установить бит 0
	 set_rele = regBank.get(1);
	 mcp_Out1.digitalWrite(0, set_rele);                 // Реле RL0 Звук  Звук Mic1p Диспетчер

	 //-----Установить бит 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);               // Реле RL1 Звук Mic2p  Инструктор

	 //-----Установить бит 2
	  set_rele = regBank.get(3);
	  mcp_Out1.digitalWrite(2, set_rele);               // Реле RL2 Звук Mic3p MTT
  
	 //-----Установить бит 3
	  set_rele = regBank.get(4);
	  mcp_Out1.digitalWrite(3, set_rele);               // Реле RL3 Звук

	 //-----Установить бит 4                            // Реле RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    

	 //-----Установить бит 5
	  set_rele = regBank.get(6);                        // Реле RL5 Звук
	  mcp_Out1.digitalWrite(5, set_rele);              

	 //-----Установить бит 6	 
	  set_rele = regBank.get(7);
	  mcp_Out1.digitalWrite(6, set_rele);              // Реле RL6 Звук

	 //-----Установить бит 7
	  set_rele = regBank.get(8);
	  mcp_Out1.digitalWrite(7, set_rele);              // Реле RL7 Питание платы

	 //---- Второй байт----------
	 //-----Установить бит 8
	  set_rele = regBank.get(9);                        // Реле RL8 Звук на микрофон
	  mcp_Out1.digitalWrite(8, set_rele);    

	 //-----Установить бит 9
	  set_rele = regBank.get(10);
	  mcp_Out1.digitalWrite(9, set_rele);               // Реле RL9 XP1 10

	 //-----Установить бит 10                           // Реле RL10 Включение питания на высоковольтный модуль 
	  set_rele = regBank.get(11);
	  mcp_Out1.digitalWrite(10, set_rele);    


	//-----Установить бит 11                            // adr_set_USB
	





	 //-----Установить бит 12
	  set_rele = regBank.get(13);
	  mcp_Out1.digitalWrite(12, set_rele);              // XP8 - 2   sensor Тангента ножная

	 //-----Установить бит 13
	  set_rele = regBank.get(14);
	  mcp_Out1.digitalWrite(13, set_rele);              // XP8 - 1   PTT Тангента ножная

	 //-----Установить бит 14

	  set_rele = regBank.get(15);
	  mcp_Out1.digitalWrite(14, set_rele);              // XS1 - 5   PTT Мик

	  //-----Установить бит 15
	  set_rele = regBank.get(16);
	  mcp_Out1.digitalWrite(15, set_rele);              // XS1 - 6   sensor Мик

	  //  Test 3
	 //-----Первый байт ------------
	 //-----Установить бит 0

	  set_rele = regBank.get(17);
	  mcp_Out2.digitalWrite(0, set_rele);                // J8-12     XP7 4 PTT2   Танг. р.

	 //-----Установить бит 1
	  set_rele = regBank.get(18);
	  mcp_Out2.digitalWrite(1, set_rele);                // XP1 - 20  HangUp  DCD

	 //-----Установить бит 2
	  set_rele = regBank.get(19);
	  mcp_Out2.digitalWrite(2, set_rele);                // J8-11     XP7 2 sensor  Танг. р.
  
	//-----Установить бит 3

	  set_rele = regBank.get(20);
	  mcp_Out2.digitalWrite(3, set_rele);                 // J8-23     XP7 1 PTT1 Танг. р.

	 //-----Установить бит 4
	  set_rele = regBank.get(21);
	  mcp_Out2.digitalWrite(4, set_rele);                 // XP2-2     sensor "Маг." 

	 //-----Установить бит 5

	  set_rele = regBank.get(22);
	  mcp_Out2.digitalWrite(5, set_rele);                  // XP5-3     sensor "ГГC."

	 //-----Установить бит 6
	  set_rele = regBank.get(23);
	  mcp_Out2.digitalWrite(6, set_rele);                  // XP3-3     sensor "ГГ-Радио1."

	 //-----Установить бит 7
	  set_rele = regBank.get(24);
	  mcp_Out2.digitalWrite(7, set_rele);                  // XP4-3     sensor "ГГ-Радио2."

	  // Test 4
	//-----Первый байт ------------
	 //-----Установить бит 8
	  set_rele = regBank.get(25);
	  mcp_Out2.digitalWrite(8, set_rele);                  // XP1- 19 HaSs      флаг подключения трубки  

	  //-----Установить бит 9
	  set_rele = regBank.get(26);
	  mcp_Out2.digitalWrite(9, set_rele);                  // XP1- 17 HaSPTT    CTS DSR вкл.

	  //-----Установить бит 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками

	  //-----Установить бит 11
	  set_rele = regBank.get(28);
	  mcp_Out2.digitalWrite(11, set_rele);                 // XP1- 15 HeS2PTT   CTS вкл

	  //-----Установить бит 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 

	  //-----Установить бит 13
	  set_rele = regBank.get(30);
	  mcp_Out2.digitalWrite(13, set_rele);                 // XP1- 6  HeS1PTT   CTS вкл

	  //-----Установить бит 14
	  set_rele = regBank.get(31);
	  mcp_Out2.digitalWrite(14, set_rele);                 // XP1- 5  HeS1Rs    Флаг подкючения гарнитуры диспетчера с 2 наушниками

	  //-----Установить бит 15
	  set_rele = regBank.get(32);
	  mcp_Out2.digitalWrite(15, set_rele);                 // XP1- 1  HeS1Ls    Флаг подкючения гарнитуры диспетчера

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
		//-----Установить бит 1
	  set_rele = regBank.get(2);
	  mcp_Out1.digitalWrite(1, set_rele);                  // Реле RL1 Звук Mic2p  Инструктор
		 //-----Установить бит 4                           // Реле RL4 XP1 12
	  set_rele = regBank.get(5);
	  mcp_Out1.digitalWrite(4, set_rele);    
		//-----Установить бит 10
	  set_rele = regBank.get(27);
	  mcp_Out2.digitalWrite(10, set_rele);                 // XP1- 16 HeS2Rs    флаг подключения гарнитуры инструктора с 2 наушниками
		  //-----Установить бит 12
	  set_rele = regBank.get(29);
	  mcp_Out2.digitalWrite(12, set_rele);                 // XP1- 13 HeS2Ls    флаг подключения гарнитуры инструктора 
	
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
		DateTime set_time = DateTime(year, month, day, hour, minute, second); // Занести данные о времени в строку "set_time"
		RTC.adjust(set_time);                                                 // Записать время в контроллер часов  
		regBank.set(adr_set_time, 0);                                         // Записать в регистр признак окончания выполнения команды
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
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_day,day_temp);  
		  b = i2c_eeprom_read_byte(0x50, adr_temp_mon);                             //access an address from the memory
		  delay(10);

		if (b!= mon_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_mon,mon_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		  regBank.set(adr_reg_temp_mon,mon_temp); 
		  b = i2c_eeprom_read_byte(0x50, adr_temp_year);                            //access an address from the memory
		  delay(10);


		if (b!= year_temp)
			{
				i2c_eeprom_write_byte(0x50, adr_temp_year,year_temp);
				i2c_eeprom_write_byte(0x50, adr_file_name_count,0);                 // при смене даты счетчик номера файла сбросить в "0"
			}
		 regBank.set(adr_reg_temp_year,year_temp); 

		  b = i2c_eeprom_read_byte(0x50, adr_file_name_count);                             //access an address from the memory
		  regBank.set(adr_reg_file_name,b);                                                // Регистр  хранения переменной номер файла
		  */
}
void time_control() // Программа записи текущего времени в регистры для передачи в ПК
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

void file_print_date()  //программа  записи даты в файл
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
//void serial_print_date()                           // Печать даты и времени    
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
			//Wire.requestFrom(address_AD5252, 1, true);  // Считать состояние движка резистора 
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
		regBank.set(122,1);                              // Флаг ошибки  открытия файла
	}
  }

  temp_file_name = ((_fileName[BASE_NAME_SIZE]-48)*10) + (_fileName[BASE_NAME_SIZE + 1]-48); // преобразование символьного номера файла в числа
   
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
		regBank.set(122,1);                              // Флаг ошибки  открытия файла
	}
  }

 
  temp_file_name = ((fileName[BASE_NAME_SIZE]-48)*10) + (fileName[BASE_NAME_SIZE + 1]-48); // преобразование символьного номера файла в числа
  regBank.set(adr_reg_file_name,temp_file_name);      
//  i2c_eeprom_write_byte(0x50, adr_file_name_count,temp_file_name);                 // при смене даты счетчик номера файла сбросить в "0"

  if (!myFile.open(fileName, O_CREAT | O_WRITE | O_EXCL)) //sdError("file.open");
  {
	regBank.set(122,1);                              // Флаг ошибки  открытия файла
  }
  else
  {
	Serial.print(fileName);
	Serial.println(F("  Open Ok!"));

	DateTime now = RTC.now();

	regBank.set(adr_Mic_Start_day , now.day());           // Время старта теста
	regBank.set(adr_Mic_Start_month, now.month());
	regBank.set(adr_Mic_Start_year, now.year());
	regBank.set(adr_Mic_Start_hour, now.hour());
	regBank.set(adr_Mic_Start_minute, now.minute());
	regBank.set(adr_Mic_Start_second, now.second());
	// Уточнить 			
	regBank.set(adr_Time_Test_day, 0); 
	regBank.set(adr_Time_Test_hour, 0); 
	regBank.set(adr_Time_Test_minute, 0); 
	regBank.set(adr_Time_Test_second, 0); 
	myFile.println ("");
	myFile.print ("Report of test module Audio-1 N ");
	// Получить данные int из регистров
	int val_3 = regBank.get(40010);
	int val_2 = regBank.get(40011);
	int val_1 = regBank.get(40012);
	int val_0 = regBank.get(40013);
	 
	// подготовить для преобразования побитного чтения
	 byte *x3 = (byte *)&val_3;  
	 byte *x2 = (byte *)&val_2;
	 byte *x1 = (byte *)&val_1;
	 byte *x0 = (byte *)&val_0;

	 // Записать число в виде младшего byte  в массив
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
	regBank.set(122,0);                              // Флаг индикации открытия файла                                   
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
			regBank.set(123,0);                                  // Флаг закрытия файла
		}
	else 
		{
			Serial.println();
			Serial.print(fileName);
			Serial.println(" doesn't exist.");  
			regBank.set(123,1);                              // Флаг ошибки  закрытия файла
		}
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
void preob_num_str() // Программа формирования имени файла, состоящего из текущей даты и счетчика файлов
{
	DateTime now = RTC.now();
	day   = now.day();
	month = now.month();
	year  = now.year();
	int year_temp = year-2000;
	itoa (year_temp,str_year_file, 10);                                        // Преобразование даты год в строку ( 10 - десятичный формат) 

	if (month <10)
		{
		   itoa (0,str_mon_file0, 10);                                         //  Преобразование даты месяц  в строку ( 10 - десятичный формат) 
		   itoa (month,str_mon_file10, 10);                                    //  Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_mon_file, "%s%s", str_mon_file0, str_mon_file10);       // Сложение 2 строк
		}
	else
		{
		   itoa (month,str_mon_file, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		}
	if (day <10)
		{
		   itoa (0,str_day_file0, 10);                                         // Преобразование числа в строку ( 10 - десятичный формат) 
		   itoa (day,str_day_file10, 10);                                      // Преобразование числа в строку ( 10 - десятичный формат) 
		   sprintf(str_day_file, "%s%s", str_day_file0, str_day_file10);       // Сложение 2 строк
		}
	else
		{
		itoa (day,str_day_file, 10);                                           // Преобразование числа в строку ( 10 - десятичный формат) 
		}
		 
	sprintf(str1, "%s%s",str_year_file, str_mon_file);                         // Сложение 2 строк
	sprintf(str2, "%s%s",str1, str_day_file);                                  // Сложение 2 строк
	sprintf(fileName, "%s%s", str2, "00.KAM");                                 // Получение имени файла в file_name
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
	Для вызова подпрограммы проверки необходимо записать номер проверки по адресу adr_control_command (40120) 
	Код проверки
	0 -  Выполнение команды окончено
	1 -  Отключить все сенсоры
	2 -  Включить все сенсоры
	3 -  Тест Инструктора
	4 -  Тест диспетчера
	5 -  Тест МТТ
	6 -  Тест Танг.р
	7 -  Тест Микрофон
	8 -  Тест ГГС
	9 -  Тест Радио 1
	10 - Тест Радио 2
	11 - Тест Магнитофон
	12 - Открыть файл
	13 - Закрыть файл
	14 - Записать время
	15 - Установить уровень сигнала
	16 - Reg_count_clear();			                                        // Сброс счетчиков ошибок                    
	17 - test_power();                                                    	// Проверить напряжение  питания
	18 - set_video();				 //
	19 - test_video();				 //
	20 - Записать уровни порогов заводские
	21 - Записать уровни порогов пользовательские
	22 - Получить уровни порогов пользовательские
	23 - Контроль имени файла
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

	int test_n = regBank.get(adr_control_command);                                  //адрес  40120
	if (test_n != 0)
	{
		if(test_n != 0) Serial.println(test_n);	
		switch (test_n)
		{
			case 1:
				 sensor_all_off();                           // Отключить все сенсоры
				 break;
			case 2:	
				 sensor_all_on();                            // Включить все сенсоры
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
				 test_mikrophon();                           // Тестирование микрофона
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
				 read_reg_error();                           // Получить ошибки
				 break;
			case 16:
				 Reg_count_clear();			                 // Сброс счетчиков ошибок                    
				 break;
			case 17:
				 test_power();                               // Проверить напряжение  питания
				 break;
			case 18:
				 set_video();				                 //
				 break;
			case 19:
				 test_video();				                 //
				 break;
			case 20:                                         // Записать уровни порогов заводские
				 default_mem_porog();
				 break;
			case 21:                      		 		     // Записать уровни порогов пользовательские
				 set_mem_porog();
				 break;
			case 22:                                         // Получить уровни порогов пользовательские
				 read_mem_porog();
				 break;
			case 23: 
				 controlFileName();                          // Контроль имени файла
				 break;
			case 24: 
				 set_SD();                                   // Проверка SD памяти
				 break;
			case 25: 
				 send_file_PC();                             // 
				 break;
			case 26: 
				 load_list_files();                          // Отправить список файлов в Камертон   
				 break;
			case 27: 
				 file_del_SD();
				 break;
			case 28:
				 clear_serial2();                            // Очистить буфер serial2
				 break;
			case 29:
				 set_serial2();                              // Поиск serial 2
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
				 set_analog_pin();                           // Установить номер аналового входа осциллографа
				 break;
			case 35:
				 oscilloscope();                             // Произвести измерения осциллографом
				 break;
			case 36: 
															 // Serial.println("  Reset");	
				 wdt_disable();                              // отключить, так как он может быть уже включен
				 wdt_enable(WDTO_15MS);                      // Программный сброс. установим минимально возможное время 15мс
				 while (1) {}	                             // Выполнить 1 раз
				 break;
			case 37: 
				 set_sound_oscill();                         // Установить частоту для измерения осциллографом
				 break;
			case 38: 
				 set_rezistor1();                            // Установить резистором №1 уровень сигнала
				 break;
			case 39:
				 set_rezistor2();                            // Установить резистором №2 уровень сигнала
				 break;
			case 40:
				 set_rezistor_All();                           
				 break;
			case 41:
				 test_serial2();
				 break;
			case 42:
				 i2c_eeprom_write_byte(deviceaddress,adr_set_USB,0);    // Реле переключения USB - RS232 
				 set_USB0();
				 break;
			case 43:
				 i2c_eeprom_write_byte(deviceaddress,adr_set_USB,255);  // Реле переключения USB - RS232 
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
				 regBank.set(adr_control_command,0);        // Установить резистором №1,№2  уровень сигнала
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
	regBank.set(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(100);
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,0);                                                              // Микрофон диспетчера отключить
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(14,0);                                                              // XP8 - 1   PTT     Тангента ножная
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 тангента ручная DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS вкл
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	byte i50 = regBank.get(40004);    
	byte i52 = regBank.get(40006);     
	byte i53 = regBank.get(40007);     

		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки отключен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) != 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40201);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			save_reg_eeprom(adr_reg40201,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(201,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));         // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[1])));     // "Sensor tangenta ruchnaja            XP7 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja отключен  - Pass
				}
		  }

		if(bitRead(i50,4) != 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
		  {
			regcount = read_reg_eeprom(adr_reg40202);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			save_reg_eeprom(adr_reg40202,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(202,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));         // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[2])));     // "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 OFF - ";  отключен  - Pass
			  }
		  }

		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			save_reg_eeprom(adr_reg40203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = read_reg_eeprom(adr_reg40204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			save_reg_eeprom(adr_reg40204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor отключен  - Pass
			  }
		  }

		if(bitRead(i52,3) != 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40205);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			save_reg_eeprom(adr_reg40205,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(205,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 отключен  - Pass
			  }
		  }

		
		if(bitRead(i52,4) != 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = read_reg_eeprom(adr_reg40206);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			save_reg_eeprom(adr_reg40206,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(206,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }

		if(bitRead(i52,5) != 0)                                                     // XS1 - 6   sensor отключения микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			save_reg_eeprom(adr_reg40207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone отключен  - Pass
			  }
		  }

		if(bitRead(i53,4) != 0)                                                     // Реле RL4 XP1 12  HeS2e   Выключение микрофона инструктора
		  {
			regcount = read_reg_eeprom(adr_reg40208);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			save_reg_eeprom(adr_reg40208,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(208,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));         // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[8])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора отключен  - Pass
			  }
		  }

		if(bitRead(i53,6) != 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = read_reg_eeprom(adr_reg40209);                                          // адрес счетчика ошибки Выключение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Выключение микрофона диспетчера
			save_reg_eeprom(adr_reg40209,regcount);                                            // адрес счетчика ошибки Выключение микрофона диспетчера
			regBank.set(209,1);                                                     // установить флаг ошибки Выключение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));         // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[9])));     // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. отключен  - Pass
			   }
		  }
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
	regBank.set(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(100);
	regBank.set(5,1);                                                               // Микрофон инструктора включить
	regBank.set(10,1);                                                              // Микрофон диспетчера включить
	regBank.set(13,1);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	//regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	//regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	//regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	//regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
			   }
		  }
	
		if(bitRead(i50,3) == 0)                                                     // J8-11  тангента ручная                           "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			save_reg_eeprom(adr_reg40211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));    // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
				}
		  }

		if(bitRead(i50,4) == 0)                                                     // XP8 - 2   sensor Тангента ножная                  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
		  {
			regcount = read_reg_eeprom(adr_reg40212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			save_reg_eeprom(adr_reg40212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));    // "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  - Pass
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
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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

		if(bitRead(i52,1) == 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

//--------------------------------------------------------------------------------
		Serial.println("Instr");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);
	i52 = regBank.get(40006);     


		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
			  }
		  }
//-------------------------------------------------------------------------------
		Serial.println("Disp2");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = regBank.get(40006);    

		if(bitRead(i52,3) == 0)                                                     // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
			  }
		  }
		//--------------------------------------------------------------------------------------------------
		Serial.println("Disp");
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	delay(500);
	UpdateRegs(); 
	delay(500);

	i52 = regBank.get(40006);    
		
		if(bitRead(i52,4) == 0)                                                     // XP1- 1  HeS1Ls   sensor подкючения гарнитуры диспетчера 
		  {
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
			  }
		  }
//------------------------------------------------------------------------------------------------------------------------
*/
		if(bitRead(i52,5) == 0)                                                     // XS1 - 6   sensor включения микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			save_reg_eeprom(adr_reg40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass
			  }
		  }

		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = read_reg_eeprom(adr_reg40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			save_reg_eeprom(adr_reg40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
			  }
		  }

		if(bitRead(i53,6) == 0)                                                     // Реле RL9 XP1 10 Выключение микрофона диспетчера
		  {
			regcount = read_reg_eeprom(adr_reg40219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			save_reg_eeprom(adr_reg40219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			regBank.set(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));    // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
			   }
		  }



	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,0);                                                              // Микрофон диспетчера отключить




	/*
	unsigned int regcount = 0;
	myFile.println("");
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[3])));                    // " ****** Test sensor ON start! ******";    
	myFile.println(buffer);                                                         // " ****** Test sensor ON start! ******";    
	file_print_date();
	myFile.println();
	regBank.set(8,1);                                                               // Включить питание Камертон
	UpdateRegs(); 
	delay(1000);
	//++++++++++++++++++++++++++++++++++++++++++ Начало проверки ++++++++++++++++++++++++++++++++++++++
	bool test_sens = true;
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
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
						if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor 2 включен  - Pass
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
			regcount = regBank.get(40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset instructor включен  - Pass
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
			regcount = regBank.get(40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			regBank.set(40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher 2 включен  - Pass
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
			regcount = regBank.get(40215);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(40215,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера с 2 наушниками
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок                                            // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor headset dispatcher отключен  - Pass
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
			regcount = regBank.get(40216);                                          // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подкючения гарнитуры диспетчера 
			regBank.set(40216,regcount);                                            // адрес счетчика ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(216,1);                                                     // установить флаг ошибки sensor подкючения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибокчетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки        
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
							if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
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
			regcount = regBank.get(40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки        
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta ruchnaja включен  - Pass
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
			regcount = regBank.get(40211);                                          // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(40211,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(211,1);                                                     // установить флаг ошибки sensor тангента ручная    "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[11])));        // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
			myFile.print(buffer);                                                   // "Sensor tangenta ruchnaja            XP7 - 2                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(13,1);                                                              // XP8 - 2   sensor Тангента ножная
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor tangenta nognaja             XP8 - 2                 ON - ";  включен  -
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
			regcount = regBank.get(40212);                                          // адрес счетчика ошибки sensor Тангента ножная      "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок  sensor Тангента ножная  "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(40212,regcount);                                            // адрес счетчика ошибки  sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(212,1);                                                     // установить флаг ошибки sensor Тангента ножная     "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[12])));        // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			myFile.print(buffer);                                                   // "Sensor tangenta nognaja             XP8 - 2                 ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
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
							if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
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
			regcount = regBank.get(40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			regBank.set(40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }



	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона отключить
	regBank.set(5,1);                                                               // Микрофон инструктора включить "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
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
							if (test_repeat == false) myFile.println(buffer);                   // Микрофон инструктора включен  - Pass
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
			regcount = regBank.get(40218);                                          // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			regBank.set(40218,regcount);                                            // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,true);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	test_sens = true;                                                               // Флаг выполнения теста
	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,1);                                                              // Микрофон диспетчера включить
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
							if (test_repeat == false) myFile.println(buffer);                   // Microphone headset dispatcher Sw. включен  - Pass
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
			regcount = regBank.get(40219);                                          // адрес счетчика ошибки Включение микрофона диспетчера
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона диспетчера
			regBank.set(40219,regcount);                                            // адрес счетчика ошибки Включение микрофона диспетчера
			regBank.set(219,1);                                                     // установить флаг ошибки Включение микрофона диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
	   }

	regBank.set(5,0);                                                               // Микрофон инструктора отключить
	regBank.set(10,0);                                                              // Микрофон диспетчера отключить
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(14,0);                                                              // XP8 - 1   PTT     Тангента ножная
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
 
	regBank.set(17,0);                                                              // J8-12    XP7 4 PTT2 тангента ручная DSR
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS DSR вкл.
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   CTS вкл
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	*/
	UpdateRegs(); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	delay(100);
	UpdateRegs(); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    

}

void set_rezistor1()
{
	int mwt1 = regBank.get(40060);             // Адрес хранения величины сигнала резистором № 1
	resistor(1, mwt1);
	//Serial.println(mwt1);
	regBank.set(adr_control_command,0);
}
void set_rezistor2()
{
	int mwt2 = regBank.get(40064);             // Адрес хранения величины сигнала резистором № 2
	resistor(2, mwt2);
	regBank.set(adr_control_command,0);
}
void set_rezistor_All()
{
	int mwt1 = regBank.get(40060);             // Адрес хранения величины сигнала резистором № 1
	resistor(1, mwt1);
	resistor(2, mwt1);
	regBank.set(adr_control_command,0);
}

void test_headset_instructor()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(201, 39);                                       // Считать из EEPROM уровни порогов инструктора
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
	test_instr_off();                                                               // Отключить реле и сенсоры, прверить отключение
	test_instr_on();                                                                // Включить необходимые сенсоры, проверить состояние
	//myFile.println("");
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, por_int_buffer[1]);                                                                // Установить уровень сигнала 30 мв
	resistor(2, por_int_buffer[2]);                                                                // Установить уровень сигнала 30 мв
	regBank.set(2,1);                                                               // Подать сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();                                                                   // Выполнить команду
	delay(300);

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[4])));                              // "Signal headset instructor microphone 30mv     ON"            ;   
	if (test_repeat == false)  myFile.println(buffer);                                        // "Signal headset instructor microphone 30mv     ON"            ;   
	
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40230, adr_reg40430, 230,por_int_buffer[3]);      // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40231, adr_reg40431, 231,por_int_buffer[5]);      // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineL,    adr_reg40232, adr_reg40432, 232,por_int_buffer[7]);      // Измерить уровень сигнала на выходе LineL     "Test headset instructor ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40233, adr_reg40433, 233,por_int_buffer[9]);      // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,adr_reg40234, adr_reg40434, 234,por_int_buffer[11]);     // Измерить уровень сигнала на выходе mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	measure_vol_min(analog_mag_phone,adr_reg40235, adr_reg40435, 235,por_int_buffer[13]);     // Измерить уровень сигнала на выходе mag phone "Test headset instructor ** Signal mag phone                 OFF - ";
	measure_vol_min(analog_ggs,      adr_reg40236, adr_reg40436, 236,por_int_buffer[15]);     // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40237, adr_reg40437, 237,por_int_buffer[17]);     // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40238, adr_reg40438, 238,por_int_buffer[19]);     // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";

	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
												   //
	regBank.set(5,1);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Включить микрофон)
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT Включить PTT инструктора
	regBank.set(16,0);                                                              // Сенсор микрофона отключить
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(29,1);                                                              // ВКЛ XP1- 13 HeS2Ls Кнопка  ВКЛ флаг подключения гарнитуры инструктора 
	UpdateRegs();                                                                   // 
	delay(300); 

	byte i53 = regBank.get(40007);                                                  // Получить текущее состояние Камертона
		if(bitRead(i53,4) == 0)                                                     // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
		  {
			regcount = read_reg_eeprom(adr_reg40218);                               // адрес счетчика ошибки Включение микрофона инструктора
			regcount++;                                                             // увеличить счетчик ошибок Включение микрофона инструктора
			save_reg_eeprom(adr_reg40218,regcount);                                 // адрес счетчика ошибки Включение микрофона инструктора
			regBank.set(218,1);                                                     // установить флаг ошибки Включение микрофона инструктора
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));        // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[18])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				if (test_repeat == false) myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                                             // Микрофон инструктора включен  - Pass
			  }
		  }
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[5])));                    // "Microphone headset instructor signal          ON"            ;  
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone headset instructor signal          ON"            ;    Звуковой сигнал подан на вход микрофона инструктора
	delay(100);

	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    adr_reg40224, adr_reg40424, 224,por_int_buffer[25],por_int_buffer[26]);                                // Измерить уровень сигнала на выходе LineL      "Test headset instructor ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,adr_reg40226, adr_reg40426, 226,por_int_buffer[31],por_int_buffer[32]);                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40230, adr_reg40430, 230,por_int_buffer[4]);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset instructor ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40231, adr_reg40431, 231,por_int_buffer[6]);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset instructor ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40233, adr_reg40433, 233,por_int_buffer[10]);                                 // Измерить уровень сигнала на выходе LineR     "Test headset instructor ** Signal LineR                     OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      adr_reg40236, adr_reg40436, 236,por_int_buffer[16]);                                 // Измерить уровень сигнала на выходе GGS       "Test headset instructor ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40237, adr_reg40437, 237,por_int_buffer[18]);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset instructor ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40238, adr_reg40438, 238,por_int_buffer[20]);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset instructor ** Signal GG Radio2                 OFF - ";
	
// Проверка analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));  // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // Флаг управления радиопередачей
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40333, adr_reg40533, 333,por_int_buffer[29],por_int_buffer[30]);                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));  // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio,adr_reg40234, adr_reg40434, 234,por_int_buffer[11]);     // Измерить уровень сигнала на выходе mag radio "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.set(42,0);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // Флаг управления радиопередачей
	delay(50);
	set_radio_send1();
	delay(100);
	
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(5,0);                                                               // Подать управляющую команду на вывод 12 ХР1 HeS2e (Выключить микрофон инструктора)
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	regBank.set(2,0);                                                               // Выключить сигнал на вход микрофона инструктора  Mic2p
	UpdateRegs();     
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
}
void test_headset_dispatcher()
 {
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	read_porog_eeprom(277, 39);                                                    // Считать из EEPROM уровни порогов инструктора
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
	test_disp_off();                                                                // Отключить реле и сенсоры, прверить отключение
	test_disp_on();                                                                 // Включить необходимые сенсоры, проверить состояние
	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона ++++++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1, por_int_buffer[1]);                                                                // Установить уровень сигнала 30 мв
	resistor(2, por_int_buffer[2]);                                                                // Установить уровень сигнала 30 мв
	regBank.set(1,1);                                                               // Подать сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[13])));                   // "Signal headset dispatcher microphone 30mv     ON"            ;    
	if (test_repeat == false)  myFile.println(buffer);                              // "Signal headset dispatcher microphone 30mv     ON"            ;   
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях FrontL FrontR +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40240, adr_reg40440, 240,por_int_buffer[3]);                  // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40241, adr_reg40441, 241,por_int_buffer[5]);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на "Маг"  линиях Radio, Phane +++++++++++++++++++++++++++
	measure_vol_min(analog_LineL,    adr_reg40242, adr_reg40442, 242,por_int_buffer[7]);                  // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40243, adr_reg40443, 243,por_int_buffer[9]);                  // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_mag_radio,adr_reg40244, adr_reg40444, 244,por_int_buffer[11]);                 // Измерить уровень сигнала на выходе mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	measure_vol_min(analog_mag_phone,adr_reg40245, adr_reg40445, 245,por_int_buffer[13]);                                 // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal mag phone                 OFF - ";
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях ГГС +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_ggs,      adr_reg40246, adr_reg40446, 246,por_int_buffer[15]);                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40247, adr_reg40447, 247,por_int_buffer[17]);                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40248, adr_reg40448, 248,por_int_buffer[19]);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	//++++++++++++++++++++++++++++++++++++++++ Включить микрофон инструктора ++++++++++++++++++++++++++++++++++++++++++++++++++
//	myFile.println("");                                                             //
	regBank.set(10,1);                                                              // Подать управляющую команду на вывод XP1 10 Включение микрофона диспетчера
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	regBank.set(16,0);                                                              // Сенсор микрофона отключить
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера

	UpdateRegs();                                                                   // 
	delay(100);                                                                     //

	byte i53 = regBank.get(40007);     
		if(bitRead(i53,6) == 0)                                                      // Проверка  включения микрофона диспетчера
		  {
			regcount = read_reg_eeprom(adr_reg40219);                               // адрес счетчика ошибки включения микрофона диспетчера          "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок включения микрофона диспетчера       "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			save_reg_eeprom(adr_reg40219,regcount);                                 // адрес счетчика ошибки включения микрофона диспетчера
			regBank.set(219,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			resistor(1, 255);                                                       // Установить уровень сигнала в исходное состояниe
			resistor(2, 255);                                                       // Установить уровень сигнала в исходное состояниe
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[19])));        // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			myFile.print(buffer);                                                   // "Microphone headset dispatcher Sw.   XP1 10 HeS1e            ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	if (test_repeat == false) myFile.println(buffer);                               // "Microphone dispatcher signal ON"  Звуковой сигнал подан на вход микрофона диспетчера
	delay(100);

	//+++++++++++++++++++++++++++ Проверить наличие сигнала на линиях LineL  mag phone  ++++++++++++++++++++++++++++++++++
	measure_vol_max(analog_LineL,    adr_reg40227, adr_reg40427, 227,por_int_buffer[25],por_int_buffer[26]);                                // Измерить уровень сигнала на выходе LineL     "Test headset dispatcher ** Signal LineL                     ON  - ";
	measure_vol_max(analog_mag_phone,adr_reg40229, adr_reg40429, 229,por_int_buffer[31],por_int_buffer[32]);                                // Измерить уровень сигнала на выходе mag phone "Test headset dispatcher ** Signal Mag phone                 ON  - ";

   //++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях +++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,   adr_reg40240, adr_reg40440, 240,por_int_buffer[4]);                                 // Измерить уровень сигнала на выходе FrontL    "Test headset dispatcher ** Signal FrontL                    OFF - ";
	measure_vol_min(analog_FrontR,   adr_reg40241, adr_reg40441, 241,por_int_buffer[6]);                                 // Измерить уровень сигнала на выходе FrontR    "Test headset dispatcher ** Signal FrontR                    OFF - ";
	measure_vol_min(analog_LineR,    adr_reg40243, adr_reg40443, 243,por_int_buffer[10]);                                 // Измерить уровень сигнала на выходе LineR     "Test headset dispatcher ** Signal LineR                     OFF - ";
	measure_vol_min(analog_ggs,      adr_reg40246, adr_reg40446, 246,por_int_buffer[16]);                                 // Измерить уровень сигнала на выходе GGS       "Test headset dispatcher ** Signal GGS                       OFF - ";
	measure_vol_min(analog_gg_radio1,adr_reg40247, adr_reg40447, 247,por_int_buffer[18]);                                 // Измерить уровень сигнала на выходе GG Radio1 "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	measure_vol_min(analog_gg_radio2,adr_reg40248, adr_reg40448, 248,por_int_buffer[20]);                                 // Измерить уровень сигнала на выходе GG Radio2 "Test headset dispatcher ** Signal GG Radio2                 OFF - ";
	// Проверка analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));     // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // Флаг управления радиопередачей
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40334, adr_reg40534, 334,por_int_buffer[29],por_int_buffer[30]);              // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));     // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio,adr_reg40244, adr_reg40444, 244,por_int_buffer[11]);                                 // Измерить уровень сигнала на выходе mag radio "Test headset dispatcher ** Signal mag radio                 OFF - ";
	regBank.set(42,0);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // Флаг управления радиопередачей
	delay(50);
	set_radio_send1();
	delay(100);
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs   Отключить sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls   Отключить  sensor подкючения гарнитуры диспетчера
	regBank.set(15,0);                                                              // РТТ микрофона отключить
	regBank.set(10,0);                                                              // Подать управляющую команду на вывод XP1 10  (Выключить микрофон диспетчера)
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	regBank.set(28,0);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.set(1,0);                                                               // Отключить сигнал на вход микрофона диспетчера Mic1p
	UpdateRegs(); 
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
	test_MTT_off();                                                                 // Отключить реле и сенсоры, прверить отключение
	test_MTT_on();                                                                  // Включить необходимые сенсоры, проверить состояние
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT включить должно быть в "0"
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command sensor ON MTT  send!         
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD Трубку поднять DCD должно быть в "0"
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));
	if (test_repeat == false)  myFile.println(buffer);                              // "Command  HangUp MTT OFF send!"
	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала динамиков на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40250, adr_reg40450, 250,por_int_buffer[3]);                // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40251, adr_reg40451, 251,por_int_buffer[5]);                // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40252, adr_reg40452, 252,por_int_buffer[7]);                // Измерить уровень сигнала на выходе LineL     "Test MTT ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40253, adr_reg40453, 253,por_int_buffer[9]);                // Измерить уровень сигнала на выходе LineR     "Test MTT ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40254, adr_reg40454, 254,por_int_buffer[11]);               // Измерить уровень сигнала на выходе mag radio "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40255, adr_reg40455, 255,por_int_buffer[13]);               // Измерить уровень сигнала на выходе mag phone "Test MTT ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[15]);               // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40257, adr_reg40457, 257,por_int_buffer[17]);               // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40258, adr_reg40458, 258,por_int_buffer[19]);               // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";

	// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона MTT +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,por_int_buffer[1]);                                                                       // Установить уровень сигнала 60 мв
	resistor(2,por_int_buffer[2]);                                                                       // Установить уровень сигнала 60 мв
	regBank.set(3,1);                                                                                    // Включить сигнал на вход микрофона трубки Mic3p
	UpdateRegs();                                                                                        // Выполнить команду
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[33])));                                        // "Signal MTT microphone 30mv                    ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal MTT microphone 30mv                    ON"            ;
	//++++++++++++++++++++++++++++++++++ Проверить отсутствие сигнала на линиях  +++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40250, adr_reg40450, 250,por_int_buffer[4]);                 // Измерить уровень сигнала на выходе FrontL    "Test MTT ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40251, adr_reg40451, 251,por_int_buffer[6]);                 // Измерить уровень сигнала на выходе FrontR    "Test MTT ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40254, adr_reg40454, 254,por_int_buffer[12]);                // Измерить уровень сигнала на выходе mag radio   "Test MTT ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[16]);                // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40257, adr_reg40457, 257,por_int_buffer[18]);                // Измерить уровень сигнала на выходе GG Radio1 "Test MTT ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40258, adr_reg40458, 258,por_int_buffer[20]);                // Измерить уровень сигнала на выходе GG Radio2 "Test MTT ** Signal GG Radio2                                OFF - ";
	// ++++++++++++++++++++++++++++++++++ Проверить наличие сигнала  ++++++++++++++++++++++++++++++++++++
	//measure_vol_max(analog_LineL,     40260,260,por_int_buffer[25],por_int_buffer[26]);                               // "Test MTT ** Signal LineL                                    ON  - ";  
	measure_vol_max(analog_LineR,     adr_reg40261, adr_reg40461,  261,por_int_buffer[27],por_int_buffer[28]);                              // "Test MTT ** Signal LineR                                    ON  - ";  
	measure_vol_max(analog_mag_phone, adr_reg40262, adr_reg40462,  262,por_int_buffer[31],por_int_buffer[32]);                               // Измерить уровень сигнала на выходе mag phone  "Test MTT ** Signal Mag phone                                ON  - ";
	// +++++++++++++++++++++ Проверка реагирования вывода ГГС на сигнал HangUp  DCD ON +++++++++++++++++++++++++++++++++
	regBank.set(3,0);                                                                                      // Отключить сигнал на вход микрофона трубки Mic3p
	regBank.set(6,1);                                                                                      // Реле RL5. Подать звук Front L, Front R
	UpdateRegs();                                                                                          // Выполнить команду
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[35])));                                          //   
	if (test_repeat == false) myFile.println(buffer);                                                      // "Signal FrontL, FrontR  ON                             - "
	measure_vol_min(analog_ggs,       adr_reg40256, adr_reg40456, 256,por_int_buffer[16]);                 // Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS                                      OFF - ";
	regBank.set(18,1);                                                                                     // XP1 - 20  HangUp  DCD ON
	UpdateRegs();                                                                                          // Выполнить команду
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                                          // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                      // "Command HangUp ON  MTT                           send!"      ;
	measure_vol_max(analog_ggs,  adr_reg40289, adr_reg40489, 289,  por_int_buffer[33],por_int_buffer[34]);                       //  Измерить уровень сигнала на выходе GGS       "Test MTT ** Signal GGS             On      
	regBank.set(18,0);                                                                                     // XP1 - 20  HangUp  DCD ON  Положить трубку
	regBank.set(26,0);                                                                                     // XP1- 17 HaSPTT    CTS DSR вкл. Отключить PTT MTT
	regBank.set(25,1);                                                                                     //  XP1- 19 HaSs  sensor подключения трубки    MTT отключить должно быть в "1"
	regBank.set(6,0);                                                                                      // Реле RL5. Отключить звук Front L, Front R
	UpdateRegs();                                                                                          // Выполнить команду
	set_sound(0);
	regBank.set(adr_control_command,0);                                                                    // Завершить программу    
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
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[37])));                   // "Command sensor OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[38])));                   // "Command PTT1   OFF tangenta ruchnaja             send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[39])));                   // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT2   OFF tangenta ruchnaja             send!"      ; 

	byte i50 = regBank.get(40004);    

	if(bitRead(i50,3) != 0)                                                         // J8-11     XP7 2 sensor тангента ручная               "Command sensor tangenta ruchnaja                            OFF - ";
		{
			regcount = read_reg_eeprom(adr_reg40274);                               // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(adr_reg40274,regcount);                                     // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta ruchnaja                            OFF - ";
			regBank.set(274,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[74])));        // "Command sensor tangenta ruchnaja                            OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40270);                               // адрес счетчика ошибки                                  "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(adr_reg40270,regcount);                                     // адрес счетчика ошибки 
			regBank.set(270,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[70])));        // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) != 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR) "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40271);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			regBank.set(adr_reg40271,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                 "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			regBank.set(271,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[71])));        // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor тангента ручная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[40])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(17,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[41])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	regBank.set(20,1);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[42])));                   // "Command PTT2   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //

	UpdateRegs();                                                                   // Выполнить команду
	delay(400);
	i50 = regBank.get(40004);    

		if(bitRead(i50,3) == 0)                                          // J8-11     XP7 2 sensor тангента ручная             "Command sensor tangenta ruchnaja                            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40275);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta ruchnaja                            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta ruchnaja                            ON  - ";
			regBank.set(adr_reg40275,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			regBank.set(275,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[75])));        // "Command sensor tangenta ruchnaja                            ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta ruchnaja                            ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение J8-23     XP7 1 PTT1 тангента ручная CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP7 1 PTT1 тангента ручная CTS    "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40272);                                          // адрес счетчика ошибки PTT  MTT (CTS)                      "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40272,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS)
			regBank.set(272,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[72])));        // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // 3)  Проверка  на отключение PTT2 тангента ручная (DSR)

		if(regBank.get(adr_reg_ind_DSR) == 0)                                       // Проверка  на отключение  PTT2 тангента ручная (DSR)    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40273);                                          // адрес счетчика ошибки  PTT  MTT (DSR)                   "Command PTT2 tangenta ruchnaja (DCR)                        ON  - "; 
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(40273,regcount);                                            // адрес счетчика ошибки  PTT  MTT (DSR)                    "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			regBank.set(273,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[73])));        // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	regBank.set(17,0);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	regBank.set(19,0);                                                              // J8-11     XP7 2 sensor тангента ручная
	regBank.set(20,0);                                                              // J8-23     XP7 1 PTT1 тангента ручная CTS
	UpdateRegs();                                                                   // Выполнить команду
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
	regBank.set(13,0);                                                              // XP8 - 2   sensor Тангента ножная
	regBank.set(14,0);                                                              // XP8 - 1   PTT Тангента ножная
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[44])));                   // "Command sensor OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[45])));                   // "Command PTT    OFF tangenta nognaja              send!"      ;
	if (test_repeat == false)  myFile.println(buffer);                              //
  
	byte i50 = regBank.get(40004);    

	if(bitRead(i50,4) != 0)                                                         // J8-11     XP8 2 sensor тангента                  "Command sensor tangenta nognaja                             OFF - ";
		{
			regcount = read_reg_eeprom(adr_reg40276);                                          // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная  "Command sensor tangenta nognaja                             OFF - ";
			save_reg_eeprom(adr_reg40276,regcount);                                            // адрес счетчика ошибки sensor тангента ручная     "Command sensor tangenta nognaja                             OFF - ";
			regBank.set(276,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[76])));        // "Command sensor tangenta nognaja                             OFF - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента ножная CTS
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение XP8 1 PTT1 тангента   "Command PTT tangenta nognaja (CTS)                          OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40278);                                          // адрес счетчика ошибки 
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40278,regcount);                                            // адрес счетчика ошибки                          "Command PTT tangenta nognaja (CTS)                          OFF - ";
			regBank.set(278,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[78])));        // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	regBank.set(13,1);                                                              // XP8 2 sensor тангента ножная
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[46])));                   // "Command sensor ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  tangenta ruchnaja             send!"      ;
	regBank.set(14,1);                                                              // J8-12     XP7 4 PTT2 тангента ручная DSR
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[47])));                   // "Command PTT1   ON  tangenta ruchnaja             send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT1   ON  tangenta ruchnaja             send!"      ;

	UpdateRegs();                                                                   // Выполнить команду
	delay(100);

	i50 = regBank.get(40004);    

			if(bitRead(i50,4) == 0)                                                 // J8-11     XP7 2 sensor тангента                    "Command sensor tangenta nognaja                             ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40277);                                          // адрес счетчика ошибки sensor тангента ручная       "Command sensor tangenta nognaja                             ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor тангента ручная    "Command sensor tangenta nognaja                             ON  - ";
			save_reg_eeprom(adr_reg40277,regcount);                                            // адрес счетчика ошибки sensor тангента ручная
			regBank.set(277,1);                                                     // установить флаг ошибки sensor тангента ручная
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[77])));        // "Command sensor tangenta nognaja                             ON  - ";
			myFile.print(buffer);                                                   // "Command sensor tangenta nognaja                             ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	  // 2)  Проверка  на отключение  XP8 1 PTT1 тангента  CTS
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на отключение XP8 1         "Command PTT tangenta nognaja (CTS)                          ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40279);                                          // адрес счетчика ошибки                 "Command PTT tangenta nognaja (CTS)                          ON  - ";          
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40279,regcount);                                            // адрес счетчика ошибки                  "Command PTT tangenta nognaja (CTS)                          ON  - ";
			regBank.set(279,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[79])));        // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			myFile.print(buffer);                                                   // "Command PTT tangenta nognaja (CTS)                          ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	regBank.set(14,0);                                                              //   XP8 1 PTT тангента  
	regBank.set(13,0);                                                              //   XP8 2 sensor тангента  
	UpdateRegs();                                                                   // Выполнить команду
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
		strcpy_P(buffer, (char*)pgm_read_word(&(table_message[54])));                   // " ****** Test miсrophone start! ******"                       ;
		myFile.println(buffer);                                                         // " ****** Test miсrophone start! ******"                       ;
		file_print_date();
		myFile.println("");
	}
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона

	regBank.set(14,0);    // XP8 - 1   PTT Тангента ножная
	regBank.set(17,0);    // J8-12     XP7 4 PTT2   Танг. р.
	regBank.set(18,0);    // XP1 - 20  HangUp  DCD
	regBank.set(20,0);    // J8-23     XP7 1 PTT1 Танг. р.
	regBank.set(26,0);    // XP1- 17 HaSPTT    CTS DSR вкл.  
	regBank.set(28,0);    // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.set(30,0);    // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   // "Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на отключение сенсора и  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++

	byte i52 = regBank.get(40006);     

			if(bitRead(i52,5) != 0)                                                 // XS1 - 6   sensor отключения микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			save_reg_eeprom(adr_reg40207,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone отключен  - Pass
			  }
		  }

	 UpdateRegs(); 
	 delay(100);

	  // 2)  Проверка  на отключение PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40264);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40264,regcount);                                            // адрес счетчика ошибки 
			regBank.set(264,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	 // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение сенсора  microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);

	i52 = regBank.get(40006);     

	  if(bitRead(i52,5) == 0)                                                       // XS1 - 6   sensor отключения микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			save_reg_eeprom(adr_reg40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }
	  // +++++++++++++++++++++++++++++++++++++++ Проверка  на включение  PTT microphone ++++++++++++++++++++++++++++++++++++++++++++
	regBank.set(15,1);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[57])));                   // "Command PTT    ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[58])));                   // "Command sensor OFF microphone                    send!"      ;  
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду

	i52 = regBank.get(40006);     
 

	if(bitRead(i52,5) == 0)                                                         // XS1 - 6   sensor отключения микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40217);                                          // адрес счетчика ошибки sensor подключения микрофона
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения микрофона
			save_reg_eeprom(adr_reg40217,regcount);                                            // адрес счетчика ошибки sensor подключения микрофона
			regBank.set(217,1);                                                     // установить флаг ошибки sensor подключения микрофона
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));        // "Sensor microphone                   XS1 - 6                 ON  - "; 
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			if (test_repeat == false)
			{
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[17])));    // "Sensor microphone                   XS1 - 6                 ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone включен  - Pass;  включен  -
			}
		  }

	 UpdateRegs(); 

	  // 2)  Проверка  на включение  PTT microphone
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение      "Test microphone PTT  (CTS)                                  ON  
		  {
			regcount = read_reg_eeprom(adr_reg40266);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40266,regcount);                                            // адрес счетчика ошибки 
			regBank.set(266,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[66])));        // "Test microphone PTT  (CTS)                                  ON  - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	// ++++++++++++++++++++++++++++++++++ Проверить исправность канала  на отсутствие наводок ++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320, por_int_buffer[3]);                 // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[5]);                 // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40322, adr_reg40522, 322,por_int_buffer[7]);                 // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[9]);                 // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[11]);                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40325, adr_reg40525, 325,por_int_buffer[13]);                // Измерить уровень сигнала на выходе mag phone "Test Microphone ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[15]);                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[17]);                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[19]);                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     


		// ++++++++++++++++++++++++++++++++++ Подать сигнал на вход микрофона +++++++++++++++++++++++++++++++++++++++++++++++++
	resistor(1,por_int_buffer[1]);                                                  // Установить уровень сигнала 60 мв
	resistor(2,por_int_buffer[2]);                                                  // Установить уровень сигнала 60 мв
	regBank.set(9,1);                                                               // Включить сигнал на вход микрофона Реле RL8 Звук на микрофон
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs();                                                                   // Выполнить команду

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[55])));                   // "Signal miсrophone 30  mV                      ON"     
	if (test_repeat == false) myFile.println(buffer);                               //

	measure_vol_max(analog_mag_phone, adr_reg40298, adr_reg40498, 298,por_int_buffer[31],por_int_buffer[32]);                // Измерить уровень сигнала на выходе mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,     adr_reg40299, adr_reg40499, 299,por_int_buffer[25],por_int_buffer[26]);                // Измерить уровень сигнала на выходе "Test Microphone ** Signal LineL                      ON  - ";  

	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320,por_int_buffer[4]);                 // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[6]);                 // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[10]);                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[16]);                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[18]);                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[20]);                // Измерить уровень сигнала на выходе GG Radio2 "Test Microphone ** Signal GG Radio2     

	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	regBank.set(16,1);                                                              // XS1 - 6   sensor подключения микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[56])));                   // "Command PTT    OFF microphone                    send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               //
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[59])));                   // "Command sensor ON  microphone                    send!"      ; 
	if (test_repeat == false) myFile.println(buffer);                               //
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);

	  // 2)  Проверка  на отключение PTT microphone
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение "Test microphone PTT  (CTS)                                  OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40264);                                          // адрес счетчика ошибки       "Test microphone PTT  (CTS)                                  OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40264,regcount);                                            // адрес счетчика ошибки 
			regBank.set(264,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[64])));        // "Test microphone PTT  (CTS)                                  OFF - ";
			myFile.print(buffer);                                                   // "Test microphone PTT  (CTS)                                  OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	measure_vol_max(analog_mag_phone, adr_reg40298, adr_reg40498, 298,por_int_buffer[31],por_int_buffer[32]);                // Измерить уровень сигнала на выходе mag phone  "Test Microphone ** Signal Mag phone      
	measure_vol_max(analog_LineL,     adr_reg40299, adr_reg40499, 299,por_int_buffer[25],por_int_buffer[26]);                // Измерить уровень сигнала на выходе "Test Microphone ** Signal LineL                      ON  - ";  

	measure_vol_min(analog_FrontL,    adr_reg40320, adr_reg40520, 320,por_int_buffer[4]);                 // Измерить уровень сигнала на выходе FrontL    "Test Microphone ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40321, adr_reg40521, 321,por_int_buffer[6]);                 // Измерить уровень сигнала на выходе FrontR    "Test Microphone ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40323, adr_reg40523, 323,por_int_buffer[10]);                // Измерить уровень сигнала на выходе LineR     "Test Microphone ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40326, adr_reg40526, 326,por_int_buffer[16]);                // Измерить уровень сигнала на выходе GGS       "Test Microphone ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40327, adr_reg40527, 327,por_int_buffer[18]);                // Измерить уровень сигнала на выходе GG Radio1 "Test Microphone ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40328, adr_reg40528, 328,por_int_buffer[20]);    
	// Проверка analog_mag_radio
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[72])));  // "Command radioperedacha  ON "    ;
	myFile.println (buffer);                                          // Command radioperedacha  ON "
	regBank.set(41,1);                                                // Флаг управления радиопередачей
	set_radio_send1();
	delay(100);
	measure_vol_max(analog_mag_radio,adr_reg40336, adr_reg40536, 336,por_int_buffer[29],por_int_buffer[30]);                                // Измерить уровень сигнала на выходе mag phone  "Test headset instructor ** Signal Mag phone                 ON  - ";
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[73])));  // "Command GGS mute  ON "    ;
	myFile.println (buffer);                                          // Command GGS mute  ON "  
	regBank.set(42,1);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	delay(100);
	measure_vol_min(analog_mag_radio, adr_reg40324, adr_reg40524, 324,por_int_buffer[12]);                // Измерить уровень сигнала на выходе mag radio "Test Microphone ** Signal mag radio                                OFF - ";
	regBank.set(42,0);                                                // Флаг управления ГГС (mute)
	set_ggs_mute1();
	regBank.set(41,0);                                                // Флаг управления радиопередачей
	delay(50);
	set_radio_send1();
	delay(100);
	regBank.set(9,0);                                                               // Отключить сигнал на вход микрофона Реле RL8 Звук на микрофон
	regBank.set(16,0);                                                              // XS1 - 6   sensor подключения микрофона
	regBank.set(15,0);                                                              // XS1 - 5   PTT Мик CTS
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	set_sound(0);
	delay(200);
	regBank.set(adr_control_command,0);                                             // Завершить программу    

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
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки  
	regBank.set(18,0);     
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[27])));                  // "Command sensor OFF  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                                         // "Command sensor ON  MTT                           send!"      ;
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                  // "Command HangUp OFF MTT                              send! "      ;
	if (test_repeat == false) myFile.println(buffer);              
	resistor(1,por_int_buffer[1]);                                                            // Установить уровень сигнала 60 мв
	resistor(2,por_int_buffer[2]);                                                              // Установить уровень сигнала 60 мв
	regBank.set(6,0);                                                               // Реле RL5 Звук Front L, Front R
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[71])));                   // "Signal FrontL, FrontR                         OFF "      ;
	if (test_repeat == false) myFile.println(buffer);              
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	byte i50 = regBank.get(40004);                                                 


		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки отключен  - Pass
			   }
		  }
		if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // Проверить включение HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                          // адрес счетчика ошибки отключения HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40267,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // установить флаг ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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

	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40280, adr_reg40480, 280,por_int_buffer[3]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40281, adr_reg40481, 281,por_int_buffer[5]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[7]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[9]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40284, adr_reg40484, 284,por_int_buffer[11]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40285, adr_reg40485, 285,por_int_buffer[13]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40286, adr_reg40486, 286,por_int_buffer[15]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[17]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[19]);                                // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";
	//----------------------------------------------------------------------------------------------------------------------------------------

	regBank.set(6,1);                                                               // Реле RL5 Звук Front L, Front R
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[49])));                   // "Signal GGS  FrontL, FrontR   0,7V             ON"   
	if (test_repeat == false) myFile.println(buffer);              
	delay(100);
	UpdateRegs(); 
	delay(100);
	UpdateRegs(); 

	measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";

	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);                                    // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);                                   // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";

	measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);      
	measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       adr_reg40289, adr_reg40489, 289,por_int_buffer[33],por_int_buffer[34]);                // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      ON  - ";

	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);                                   // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                                   // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(25,0);                                                              // XP1- 19 HaSs      sensor подключения трубки          
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs(); 
	delay(100);
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;

	i50 = regBank.get(40004);    

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   //  sensor  трубки включен  - Pass
			   }
		  }

		if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // Проверить включение HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                           // адрес счетчика ошибки отключения HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                              // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40267,regcount);                                             // адрес счетчика ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                      // установить флаг ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                      // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                           // Получить данные счетчика всех ошибок
			regcount_err++;                                                          // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                             // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));         // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                    // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));             // "    Error! - "; 
			myFile.print(buffer);                                                    // "    Error! - "; 
			myFile.println(regcount);                                                // Показания счетчика ошибок
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
		//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
		measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
		measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   OFF - ";
	
		measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);              // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
		measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";
	
		measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal mag radio                                OFF - ";
		measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                OFF - ";
	
		measure_vol_min(analog_ggs,       adr_reg40286, adr_reg40486, 286,por_int_buffer[16]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      OFF - ";
		measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);             // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
		measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                                 // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";
		//----------------------------------------------------------------------------------------------------------------------------------------

	regBank.set(18,1);                                                               // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                    // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                                // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(100);
	UpdateRegs();

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                          // Проверить включение HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40268);                                           // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                              // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40268,regcount);                                             // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                      // установить флаг ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                      // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                           // Получить данные счетчика всех ошибок
			regcount_err++;                                                          // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                             // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));         // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                    // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));             // "    Error! - "; 
			myFile.print(buffer);                                                    // "    Error! - "; 
			myFile.println(regcount);                                                // Показания счетчика ошибок
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));     // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                                // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));         // "Pass";
				myFile.println(buffer);                                              //  "Test MTT HangUp (DCD)                                       ON  - ";трубки включен  - Pass
			   }
		 }

	measure_vol_max(analog_FrontL,    adr_reg40290, adr_reg40490, 290,por_int_buffer[21],por_int_buffer[22]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontL                                   ON  - ";
	measure_vol_max(analog_FrontR,    adr_reg40291, adr_reg40491, 291,por_int_buffer[23],por_int_buffer[24]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal FrontR                                   ON  - ";

	measure_vol_min(analog_LineL,     adr_reg40282, adr_reg40482, 282,por_int_buffer[8]);                  // Измерить уровень сигнала на выходе "Test GGS ** Signal LineL                                    OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40283, adr_reg40483, 283,por_int_buffer[10]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal LineR                                    OFF - ";

	measure_vol_max(analog_mag_radio, adr_reg40332, adr_reg40532, 332,por_int_buffer[29],por_int_buffer[30]);         
	measure_vol_max(analog_mag_phone, adr_reg40292, adr_reg40492, 292,por_int_buffer[31],por_int_buffer[32]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal mag phone                                ON  - ";
	measure_vol_max(analog_ggs,       adr_reg40289, adr_reg40489, 289,por_int_buffer[33],por_int_buffer[34]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal GGS                                      ON  - ";

	measure_vol_min(analog_gg_radio1, adr_reg40287, adr_reg40487, 287,por_int_buffer[18]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio1                                OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40288, adr_reg40488, 288,por_int_buffer[20]);                 // Измерить уровень сигнала на выходе "Test GGS ** Signal GG Radio2                                OFF - ";

	regBank.set(6,0);                                                                // Реле RL5 Звук Front L, Front R
	UpdateRegs();    
	set_sound(0);
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);                                              // Завершить программу    
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
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1,por_int_buffer[1]);                                                  // Установить уровень сигнала 300 мв
	resistor(2,por_int_buffer[2]);                                                  // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs(); 
	
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40300, adr_reg40500, 300,por_int_buffer[3]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40301, adr_reg40501, 301,por_int_buffer[5]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40302, adr_reg40502, 302,por_int_buffer[7]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40303, adr_reg40503, 303,por_int_buffer[9]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40304, adr_reg40504, 304,por_int_buffer[11]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40305, adr_reg40505, 305,por_int_buffer[13]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40306, adr_reg40506, 306,por_int_buffer[15]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40307, adr_reg40507, 307,por_int_buffer[17]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40308, adr_reg40508, 308,por_int_buffer[19]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[51])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(4,1);                                                               // Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs();  

//	Serial.println("test_GG_Radio1 - on ");
	measure_vol_min(analog_FrontL,    adr_reg40300, adr_reg40500, 300,por_int_buffer[4]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40301, adr_reg40501, 301,por_int_buffer[6]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40302, adr_reg40502, 302,por_int_buffer[8]);                 // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40303, adr_reg40503, 303,por_int_buffer[10]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal LineR                                 OFF - ";
	measure_vol_max(analog_mag_radio, adr_reg40330, adr_reg40530, 330,por_int_buffer[29],por_int_buffer[30]);  // !!         // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40305, adr_reg40505, 305,por_int_buffer[14]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40306, adr_reg40506, 306,por_int_buffer[16]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GGS                                   OFF - ";
	measure_vol_max(analog_gg_radio1, adr_reg40309, adr_reg40509, 309,por_int_buffer[35],por_int_buffer[36]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal Radio1                                ON  - ";
	measure_vol_min(analog_gg_radio2, adr_reg40308, adr_reg40508, 308,por_int_buffer[20]);                // Измерить уровень сигнала на выходе "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();     
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	set_sound(0);
	delay(100);
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
	regBank.set(7,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	resistor(1,por_int_buffer[1]);                                                              // Установить уровень сигнала 300 мв
	resistor(2,por_int_buffer[2]);                                                               // Установить уровень сигнала 300 мв
	UpdateRegs();                                                                   // Выполнить команду
	delay(500);
	UpdateRegs(); 
	delay(100);
	//+++++++++++++++++++++++++++++++++++   Проверка отсутствия сигнала на выходах +++++++++++++++++++++++++++++++++++++++++++++++++++++++
	measure_vol_min(analog_FrontL,    adr_reg40310, adr_reg40510, 310,por_int_buffer[3]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40311, adr_reg40511, 311,por_int_buffer[5]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40312, adr_reg40512, 312,por_int_buffer[7]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40313, adr_reg40513, 313,por_int_buffer[9]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_min(analog_mag_radio, adr_reg40314, adr_reg40514, 314,por_int_buffer[11]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40315, adr_reg40515, 315,por_int_buffer[13]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40316, adr_reg40516, 316,por_int_buffer[15]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40317, adr_reg40517, 317,por_int_buffer[17]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio1                             OFF - ";
	measure_vol_min(analog_gg_radio2, adr_reg40318, adr_reg40518, 318,por_int_buffer[19]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";

	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[53])));                   // "Signal Radio1 300 mV    LFE                   ON"            ;
	if (test_repeat == false) myFile.println(buffer);                               // "Signal Radio1 300 mV    LFE                   ON"            ;
	regBank.set(7,1);                                                               //  Реле RL3 Звук  LFE  "Маг."
	UpdateRegs();                                                                   // Выполнить команду
	delay(100);
	UpdateRegs();  

	//Serial.println("test_GG_Radio2 - on ");

	measure_vol_min(analog_FrontL,    adr_reg40310, adr_reg40510, 310,por_int_buffer[4]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontL                                OFF - ";
	measure_vol_min(analog_FrontR,    adr_reg40311, adr_reg40511, 311,por_int_buffer[6]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal FrontR                                OFF - ";
	measure_vol_min(analog_LineL,     adr_reg40312, adr_reg40512, 312,por_int_buffer[8]);                 // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineL                                 OFF - ";
	measure_vol_min(analog_LineR,     adr_reg40313, adr_reg40513, 313,por_int_buffer[10]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal LineR                                 OFF - ";
	measure_vol_max(analog_mag_radio, adr_reg40331, adr_reg40531, 331,por_int_buffer[29],por_int_buffer[30]);   // !!        // Измерить уровень сигнала на выходе "Test Radio1 ** Signal mag radio                             OFF - ";
	measure_vol_min(analog_mag_phone, adr_reg40315, adr_reg40515, 315,por_int_buffer[14]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal mag phone                             OFF - ";
	measure_vol_min(analog_ggs,       adr_reg40316, adr_reg40516, 316,por_int_buffer[16]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GGS                                   OFF - ";
	measure_vol_min(analog_gg_radio1, adr_reg40317, adr_reg40517, 317,por_int_buffer[18]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal Radio1                                ON  - ";
	measure_vol_max(analog_gg_radio2, adr_reg40319, adr_reg40519, 319,por_int_buffer[37],por_int_buffer[38]);                // Измерить уровень сигнала на выходе "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.set(7,0);                                                               // Реле RL6 Звук Center
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
	//const unsigned int adr_reg40000      PROGMEM       = 40293);                         // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
	//regBank.add(40294);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
	//regBank.add(40295);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
	//regBank.add(40296);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
	//regBank.add(40297);                         // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта

	//regBank.add(40493);                         // Aдрес данных измерения ADC1 напряжение 12/3 вольт
	//regBank.add(40494);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
	//regBank.add(40495);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
	//regBank.add(40496);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
	//regBank.add(40497);                         // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта

	measure_power();

	// Проверка напряжения 12 вольт платы Камертон
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[61])));                   // "Power Kamerton V  - "                                        ;
	if(read_reg_eeprom(adr_reg40493)*2.61/100 < 11 || read_reg_eeprom(adr_reg40493)*2.61/100 >13)
	{
		if (test_repeat)                                          // Параметры в норме. Тест одиночный
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

	// Проверка напряжения 12 вольт Радио 1
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[63])));                   // "Power Radio1 V    - "                                        ;
	if(read_reg_eeprom(adr_reg40494)*2.61/100 < 11 || read_reg_eeprom(adr_reg40494)*2.61/100 >13)
	{
		if (test_repeat)                                          // Параметры в норме. Тест одиночный
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

	// Проверка напряжения 12 вольт Радио 2
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[64])));                   // "Power Radio2 V    - "                                        ;
	if(read_reg_eeprom(adr_reg40495)*2.61/100 < 11 || read_reg_eeprom(adr_reg40495)*2.61/100 >13)
	{
		if (test_repeat)                                          // Параметры в норме. Тест одиночный
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

	// Проверка напряжения 12 вольт  ГГС
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[65])));                   // "Power GGS    V    - "                                        ;
	if(read_reg_eeprom(adr_reg40496)*2.61/100 < 11 || read_reg_eeprom(adr_reg40496)*2.61/100 >13)
	{
		if (test_repeat)                                          // Параметры в норме. Тест одиночный
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

	// Проверка напряжения 3,6 вольт на светодиоде микрофона
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[66])));                   // "Power Led mic.V   - "  
	if(read_reg_eeprom(adr_reg40497)/100 < 2 || read_reg_eeprom(adr_reg40497)/100 >4)
	{
		if (test_repeat)                                          // Параметры в норме. Тест одиночный
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
	regBank.set(40061,100);                                                         // Уровень яркости 100
	while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	//delay(300);
	regs_out[0]= 0x2B;                                                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0x84;                                                              // 
	regs_out[2]= regBank.get(40061);                                                // Уровень яркости
	delay(100);
	while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	regs_out[0]= 0x2B;                                                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                                                              // 
	regs_out[2]= 0x7F;                                                              // Уровень яркости
	measure_mks();                                                                  // Измерить длительность импульсов
	
	regBank.set(40062,regBank.get(40005));                                          // Передать уровень яркости в программу

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
	if (regBank.get(40063) < 18 || regBank.get(40063) > 26)                         // Диапазон измерения длительности импульса яркости
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
		save_reg_eeprom(adr_reg40469,regBank.get(40063));                   // Передать длительность импульса в ПК
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
	while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	regBank.set(adr_control_command,0);    
}
void set_video()
{
	while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
	//delay(300);
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0x84;                              // 
	regs_out[2]= regBank.get(40061);                // Уровень яркости
	delay(300);
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 
	regs_out[2]= 0x7F;                              // Уровень яркости
	measure_mks();                                  // Измерить длительность импульсов
	regBank.set(40062,regBank.get(40005));          // Передать уровень яркости в программу
	regs_out[0]= 0x2B;                              // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                              // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                              // 127 Изменять в реальной схеме
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
 regBank.set(40063,duration);                          // Передать длительность импульса яркости в программу
}
void set_radio_send()
{
	bool radio_on_off =	regBank.get(41);            // Флаг управления радиопередачей
	while(prer_Kmerton_Run == true){}               // Ждем окончания получения данных из Камертон

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
	bool ggs_mute = regBank.get(42);                // Флаг ГГС (mute)
	while(prer_Kmerton_Run == true){}               // Ждем окончания получения данных из Камертон
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
	bool radio_on_off =	regBank.get(41);            // Флаг управления радиопередачей
	while(prer_Kmerton_Run == true){}               // Ждем окончания получения данных из Камертон

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
	bool ggs_mute = regBank.get(42);                // Флаг ГГС (mute)
	while(prer_Kmerton_Run == true){}                  // Ждем окончания получения данных из Камертон
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
	regBank.set(29,0);                                                              // XP1- 13 HeS2Ls  Отключить сенсор инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[6])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(27,0);                                                              // XP1- 16 HeS2Rs  Отключить сенсор инструктора c 2  наушниками
	regBank.set(16,0);                                                              // XS1 - 6   sensor Мик
	regBank.set(1,0);                                                               // Реле RL0 Звук
	regBank.set(2,0);                                                               // Реле RL1 Звук
	regBank.set(3,0);                                                               // Реле RL2 Звук
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	regBank.set(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // Реле RL5 Звук
	regBank.set(7,0);                                                               // Реле RL6 Звук
	regBank.set(9,0);                                                               // Реле RL8 Звук на микрофон
	regBank.set(10,0);                                                              // Реле RL9 XP1 10
	regBank.set(28,0);                                                              // XP1- 15 HeS2Ls Отключить PTT инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[8])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!"
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(100);
	UpdateRegs(); 
 
	byte i52 = regBank.get(40006);     
	 
	  // 1)  Проверка сенсора на отключение гарнитуры инструктора 2 наушниками
		if(bitRead(i52,1) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40203);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			save_reg_eeprom(adr_reg40203,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(203,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));         // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[3])));     // "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 отключен  - Pass
			  }
		  }

		if(bitRead(i52,2) != 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = read_reg_eeprom(adr_reg40204);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			save_reg_eeprom(adr_reg40204,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(204,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = read_reg_eeprom(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));         // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[4])));     // "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";     
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				if (test_repeat == false) myFile.print(buffer);                     // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				if (test_repeat == false) myFile.println(buffer);                   // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40220);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40220,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(220,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[20])));        // "Command PTT headset instructor (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        OFF - "  отключен  - Pass
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
	regBank.set(29,1);                                                              // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[11])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON headset instructor 2  send!"
	regBank.set(27,1);                                                              // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	regBank.set(16,1);                                                              // XS1 - 6   sensor Мик
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	regBank.set(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	regBank.set(28,1);                                                              // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[12])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset instructor (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(100);
	UpdateRegs(); 
 
	byte i52 = regBank.get(40006);     

	  // 3)  Проверка сенсора на подключение гарнитуры инструктора 2 наушниками
			if(bitRead(i52,1) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40213);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора с 2 наушниками
			save_reg_eeprom(adr_reg40213,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора с 2 наушниками
			regBank.set(213,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));        // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
			myFile.print(buffer);                                                   // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[13])));    // "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor 2 включен  - Pass
			  }
		  }

		if(bitRead(i52,2) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
		  {
			regcount = read_reg_eeprom(adr_reg40214);                                          // адрес счетчика ошибки sensor подключения гарнитуры инструктора
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры инструктора
			save_reg_eeprom(adr_reg40214,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры инструктора 
			regBank.set(214,1);                                                     // установить флаг ошибки sensor подключения гарнитуры инструктора 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));        // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";   
			myFile.print(buffer);                                                   // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - ";    
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[14])));    // "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset instructor включен  - Pass
			  }
		  }

		UpdateRegs(); 
	
	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT инструктора   CTS "Command PTT headset instructor (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40221);                                          // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40221,regcount);                                            // адрес счетчика ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(221,1);                                                     // установить флаг ошибки отключения PTT гарнитуры инструктора "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - "; 
			myFile.print(buffer);                                                   // "Command PTT headset instructor (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[21])));        // "Command PTT headset instructor (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset instructor (CTS)                        ON  - "  включен  - Pass
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
	regBank.set(32,0);                                                              // XP1- 1  HeS1Ls    Отключить сенсор гарнитуры диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[15])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor OFF headset instructor 2 send!"
	regBank.set(31,0);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(16,0);                                                              // XS1 - 6   sensor Мик
	regBank.set(1,0);                                                               // Реле RL0 Звук
	regBank.set(2,0);                                                               // Реле RL1 Звук
	regBank.set(3,0);                                                               // Реле RL2 Звук
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	regBank.set(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // Реле RL5 Звук
	regBank.set(7,0);                                                               // Реле RL6 Звук
	regBank.set(9,0);                                                               // Реле RL8 Звук на микрофон
	regBank.set(10,0);                                                              // Реле RL9 XP1 10
	regBank.set(30,0);                                                              // XP1- 6  HeS1PTT   Отключить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[17])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT headset instructor OFF      send!""

	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(100);
	UpdateRegs(); 
  
	byte i52 = regBank.get(40006);     
	
	  // 1)  Проверка сенсора на отключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) != 0)                                                     // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40205);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			save_reg_eeprom(adr_reg40205,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(205,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));         // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[5])));     // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - " отключен  - Pass
			  }
		  }

		if(bitRead(i52,4) != 0)                                                     //"Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - "  подключения гарнитуры диспетчера
		  {
			regcount = read_reg_eeprom(adr_reg40206);                                          // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			save_reg_eeprom(adr_reg40206,regcount);                                            // адрес счетчика ошибки sensor подключения гарнитуры диспетчера
			regBank.set(206,1);                                                     // установить флаг ошибки sensor подключения гарнитуры диспетчера
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));         // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[6])));     // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - " отключен  - Pass
			  }
		  }

	 // 3)  Проверка сенсора на отключение микрофона

		if(bitRead(i52,5) != 0)                                                     // Проверка  флага на отключение микрофона
		  {
			regcount = read_reg_eeprom(adr_reg40207);                                          // адрес счетчика ошибки сенсора микрофона 
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40207,regcount);                                            // адрес счетчика ошибки сенсора микрофона
			regBank.set(207,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));         // "Sensor microphone                   XS1 - 6                 OFF - ";  
			myFile.print(buffer);                                                   // "Sensor microphone                   XS1 - 6                 OFF - ";   
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			 if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[7])));     // "Sensor microphone                   XS1 - 6                 OFF - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor microphone                   XS1 - 6                 OFF - ";   отключен  - Pass
			   }
		  }

		UpdateRegs(); 

	   if(regBank.get(adr_reg_ind_CTS) != 0)                                        // Проверить отключения PTT диспетчера   CTS "Command PTT headset instructor (CTS)                        OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40222);                                          // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40222,regcount);                                            // адрес счетчика   ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(222,1);                                                     // установить флаг  ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		  if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[22])));        // "Command PTT headset dispatcher (CTS)                        OFF - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        OFF - "  отключен  - Pass
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
	regBank.set(32,1);                                                              // XP1- 1  HeS1Ls    sensor подключения гарнитуры диспетчера 
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[20])));                   // "Command sensor ON  headset dispatcher            send!"      ;    
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  headset dispatcher            send!"      ;    
	regBank.set(31,1);                                                              // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.set(19,1);                                                              // J8-11     XP7 2 sensor  Танг. р.
	regBank.set(16,1);                                                              // XS1 - 6   sensor Мик
	regBank.set(25,1);                                                              // XP1- 19 HaSs      sensor подключения трубки      
	regBank.set(13,1);                                                              // XP8 - 2           sensor Тангента ножная
	regBank.set(30,1);                                                              // XP1- 6  HeS1PTT   Включить PTT диспетчера
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[21])));
	if (test_repeat == false) myFile.println(buffer);                               // "Command        ON  PTT headset dispatcher (CTS)  send!"      ;  
	UpdateRegs();                                                                   // Выполнить команду включения сенсоров
	delay(100);
	UpdateRegs(); 

	byte i52 = regBank.get(40006);     

	  // 3)  Проверка сенсора на подключение гарнитуры диспетчера 2 наушниками
		if(bitRead(i52,3) == 0)                                                 // XP1- 16 HeS2Rs    sensor подключения гарнитуры диспетчера с 2 наушниками
		  {
			regcount = read_reg_eeprom(adr_reg40215);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера с 2 наушниками
			save_reg_eeprom(adr_reg40215,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера с 2 наушниками
			regBank.set(215,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера с 2 наушниками 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));        // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
			myFile.print(buffer);                                                   // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[15])));    // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher 2         XP1- 5  HeS1Rs          ON  - ";  включен  - Pass
			  }
		  }

		if(bitRead(i52,4) == 0)                                                     // XP1- 13 HeS2Ls    sensor подключения гарнитуры диспетчера 
		  {
			regcount = read_reg_eeprom(adr_reg40216);                                          // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера
			regcount++;                                                             // увеличить счетчик ошибок sensor подключения гарнитуры диспетчера
			save_reg_eeprom(adr_reg40216,regcount);                                            // адрес счетчика ошибки    sensor подключения гарнитуры диспетчера 
			regBank.set(216,1);                                                     // установить флаг ошибки   sensor подключения гарнитуры диспетчера 
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));        // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			myFile.print(buffer);                                                   // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			  if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[16])));    // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             // "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - " включен  - Pass
			  }
		  }

		UpdateRegs(); 
		//wdt_reset();
	   if(regBank.get(adr_reg_ind_CTS)== 0)                                         // Проверить включение PTT диспетчера   "Command PTT headset dispatcher (CTS)                        ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40223);                                          // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40223,regcount);                                            // адрес счетчика ошибки  отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                        ON  - ";
			regBank.set(223,1);                                                     // установить флаг ошибки отключения PTT гарнитуры диспетчера "Command PTT headset instructor (CTS)                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // "Command PTT headset dispatcher (CTS)                        ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
		   if (test_repeat == false)
		   {
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[23])));        // "Command PTT headset dispatcher (CTS)                        ON  - ";
			myFile.print(buffer);                                                   // 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));            // "Pass";
			myFile.println(buffer);                                                 // "Command PTT headset dispatcher (CTS)                        ON  - "  включен  - Pass
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
	regBank.set(26,0);                                                              // XP1- 17 HaSPTT    CTS  вкл. Отключить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[28])));                   // "Command PTT    OFF MTT                           send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    OFF MTT                           send! "     ;
	regBank.set(18,0);                                                              // XP1 - 20  HangUp  DCD
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[29])));                   // "Command        OFF HangUp MTT                    send! "     ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command        OFF HangUp MTT                    send! "     ;
	regBank.set(16,0);                                                              // XS1 - 6   sensor Мик
	regBank.set(1,0);                                                               // Реле RL0 Звук
	regBank.set(2,0);                                                               // Реле RL1 Звук
	regBank.set(3,0);                                                               // Реле RL2 Звук
	regBank.set(4,0);                                                               // Реле RL3 Звук  LFE  "Маг."
	regBank.set(5,0);                                                               // Реле RL4 XP1 12  HeS2e 
	regBank.set(6,0);                                                               // Реле RL5 Звук
	regBank.set(7,0);                                                               // Реле RL6 Звук
	regBank.set(9,0);                                                               // Реле RL8 Звук на микрофон
	regBank.set(10,0);                                                              // Реле RL9 XP1 10
	UpdateRegs();                                                                   // Выполнить команду отключения сенсоров
	delay(100);
	UpdateRegs(); 
	delay(100);
	byte i50 = regBank.get(40004);    

	//wdt_reset();
		if(bitRead(i50,2) != 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40200);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			save_reg_eeprom(adr_reg40200,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            OFF - ";  
			regBank.set(200,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));         // "Sensor MTT                      XP1- 19 HaSs   OFF               - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[0])));     // "Sensor MTT                     XP1- 19 HaSs   OFF               - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  трубки отключен  - Pass
			   }
		  }
		   UpdateRegs(); 
		   delay(100);

	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) != 0)                                       // Проверка  на отключение CTS MTT
		  {
			regcount = read_reg_eeprom(adr_reg40263);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40263,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       OFF - ";
			regBank.set(263,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[63])));        // "Test MTT PTT    (CTS)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       OFF - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	   if(regBank.get(adr_reg_ind_DCD)!= 0)                                         // Проверить включение HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
		  {
			regcount = read_reg_eeprom(adr_reg40267);                                          // адрес счетчика ошибки отключения HangUp  DCD  "Test MTT HangUp (DCD)                                       OFF - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40267,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(267,1);                                                     // установить флаг ошибки отключения HangUp  DCD   "Test MTT HangUp (DCD)                                       OFF - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[67])));        // "Test MTT HangUp (DCD)                                       OFF - ";
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       OFF - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
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
	regBank.set(25,0);                                                              //  XP1- 19 HaSs  sensor подключения трубки    MTT ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[30])));                   // "Command sensor ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command sensor ON  MTT                           send!"      ;              
	regBank.set(26,1);                                                              // XP1- 17 HaSPTT    CTS DSR вкл. включить PTT MTT
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[31])));                   // "Command PTT    ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command PTT    ON  MTT                           send!"      ;
	regBank.set(18,1);                                                              // XP1 - 20  HangUp  DCD ON
	strcpy_P(buffer, (char*)pgm_read_word(&(table_message[32])));                   // "Command HangUp ON  MTT                           send!"      ;
	if (test_repeat == false) myFile.println(buffer);                               // "Command HangUp ON  MTT                           send!"      ;

	UpdateRegs(); 
	delay(100);
	UpdateRegs();

	  // 1)  Проверка сенсора MTT на включение 
	byte i50 = regBank.get(40004);    

		if(bitRead(i50,2) == 0)                                                     // XP1- 19 HaSs sensor контроля подключения трубки    "Sensor MTT                          XP1- 19 HaSs            ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40210);                                          // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок sensor отключения трубки  "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			save_reg_eeprom(adr_reg40210,regcount);                                            // адрес счетчика ошибки                              "Sensor MTT                          XP1- 19 HaSs            ON  - ";  
			regBank.set(210,1);                                                     // установить флаг ошибки                             "Sensor MTT                          XP1- 19 HaSs            ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));        // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			myFile.print(buffer);                                                   // "Sensor MTT                      XP1- 19 HaSs   ON                - ";  
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[10])));    // "Sensor MTT                     XP1- 19 HaSs   ON                 - ";  
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  sensor  трубки включен  - Pass
			   }
		  }

		delay(100);
		UpdateRegs(); 

	  // 2)  Проверка  на отключение PTT  MTT (CTS)
		if(regBank.get(adr_reg_ind_CTS) == 0)                                       // Проверка  на включение CTS MTT
		  {
			regcount = read_reg_eeprom(adr_reg40265);                                          // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40265,regcount);                                            // адрес счетчика ошибки PTT  MTT (CTS) "Test MTT PTT    (CTS)                                       ON  - ";
			regBank.set(265,1);                                                     // установить флаг ошибки
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));        // "Test MTT PTT    (CTS)                                       ON  - ";
			myFile.print(buffer);                                                   // "Test MTT PTT    (CTS)                                       ON  - ";
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		  }
		else
		  {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[65])));    // "Test MTT PTT    (CTS)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT PTT    (CTS)                                       ON  - " трубки включен  - Pass
			   }
		  }

	   if(regBank.get(adr_reg_ind_DCD)== 0)                                         // Проверить включение HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
		  {
			regcount = read_reg_eeprom(adr_reg40268);                                          // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regcount++;                                                             // увеличить счетчик ошибок
			save_reg_eeprom(adr_reg40268,regcount);                                            // адрес счетчика ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(268,1);                                                     // установить флаг ошибки отключения HangUp  DCD "Test MTT HangUp (DCD)                                       ON  - ";
			regBank.set(120,1);                                                     // установить общий флаг ошибки
			regcount_err = regBank.get(adr_reg_count_err);                          // Получить данные счетчика всех ошибок
			regcount_err++;                                                         // увеличить счетчик всех ошибок 
			regBank.set(adr_reg_count_err,regcount_err);                            // Сохранить данные счетчика всех ошибок
			strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));        // "Test MTT HangUp (DCD)                                       ON  - ";  
			myFile.print(buffer);                                                   // "Test MTT HangUp (DCD)                                       ON  - "; 
			strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));            // "    Error! - "; 
			myFile.print(buffer);                                                   // "    Error! - "; 
			myFile.println(regcount);                                               // Показания счетчика ошибок
		 }
	  else
		 {
			   if (test_repeat == false)
			   {
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[68])));    // "Test MTT HangUp (DCD)                                       ON  - "; 
				myFile.print(buffer);                                               // 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[1])));        // "Pass";
				myFile.println(buffer);                                             //  "Test MTT HangUp (DCD)                                       ON  - ";трубки включен  - Pass
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
		measure_volume(_istochnik);                                                 // Измерить уровень сигнала на выходе
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[80])));    //  ошибки "Test GGS ** Signal FrontL                                   OFF - ";
				break;
			case 281:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[81])));    // ошибки "Test GGS ** Signal FrontR                                   OFF - ";
				break;
			case 282:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[82])));    // ошибки "Test GGS ** Signal LineL                                    OFF - ";
				break;
			case 283:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[83])));    // ошибки "Test GGS ** Signal LineR                                    OFF - ";
				break;
			case 284:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[84])));    // ошибки "Test GGS ** Signal mag radio                                OFF - ";
				break;
			case 285:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[85])));    // ошибки "Test GGS ** Signal mag phone                                OFF - ";
				break;
			case 286:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[86])));    // ошибки "Test GGS ** Signal GGS                                      OFF - ";
				break;
			case 287:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[87])));    // ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
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
		save_reg_eeprom(_adr_count200,voltage10);                                     // Сохранить результат измерения V
		if(voltage10 > _porogV)                                                      // Проверить исправность канала
			{
				if (test_repeat)                                          // Параметры в норме. Тест одиночный
				{
					file_print_date();
					myFile.print ("  ");  
				}
				myFile.print(buffer); 
				regcount = read_reg_eeprom(_adr_count);                              // адрес счетчика ошибки 
				regcount++;                                                          // увеличить счетчик ошибок канала 
				save_reg_eeprom(_adr_count,regcount);                                // адрес счетчика ошибки канала 
				regBank.set(_adr_flagErr,1);                                         // установить флаг ошибки  канала 
				regcount_err = regBank.get(adr_reg_count_err);                       // Получить данные счетчика всех ошибок
				regcount_err++;                                                      // увеличить счетчик всех ошибок 
				regBank.set(adr_reg_count_err,regcount_err);                         // Сохранить данные счетчика всех ошибок
				regBank.set(120,1);                                                  // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));         // "    Error! - "; 
				myFile.print(buffer);                                                // "    Error! - "; 
				myFile.print(regcount);                                              // Показания счетчика ошибок
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
					myFile.print(buffer);                                           // Наименование проверки
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

	measure_volume(_istochnik);                                                     // Измерить уровень сигнала на выходе
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
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[89])));    // ошибки "Test GGS ** Signal GGS                                      ON  - ";
				break;
			case 290:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[90])));    // ошибки "Test GGS ** Signal FrontL                                   ON  - ";
				break;
			case 291:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[91])));    // ошибки "Test GGS ** Signal FrontR                                   ON  - ";
				break;
			case 292:
				strcpy_P(buffer, (char*)pgm_read_word(&(string_table_err[92])));    // ошибки "Test GGS ** Signal mag phone                                ON  - ";
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

		save_reg_eeprom(_adr_count200,voltage10);                                   // адрес данных ошибки канала 
		if(voltage10 < _porogV || voltage10 > _porogVmax )                          // Проверить исправность канала
			   {
				if (test_repeat)                                          // Параметры в норме. Тест одиночный
				{
				  file_print_date();
				  myFile.print ("  ");  
				}
				myFile.print(buffer); 
				regcount = read_reg_eeprom(_adr_count);                             // адрес счетчика ошибки 
				regcount++;                                                         // увеличить счетчик ошибок канала 
				save_reg_eeprom(_adr_count,regcount);                               // адрес счетчика ошибки канала 
				regBank.set(_adr_flagErr,1);                                        // установить флаг ошибки  канала 
				regcount_err = regBank.get(adr_reg_count_err);                      // Получить данные счетчика всех ошибок
				regcount_err++;                                                     // увеличить счетчик всех ошибок 
				regBank.set(adr_reg_count_err,regcount_err);                        // Сохранить данные счетчика всех ошибок
				regBank.set(120,1);                                                 // установить общий флаг ошибки 
				strcpy_P(buffer, (char*)pgm_read_word(&(table_message[0])));        // "    Error! - "; 
				myFile.print(buffer);                                               // "    Error! - "; 
				myFile.print(regcount);                                             // Показания счетчика ошибок
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
			if (test_repeat == false)                                          // Параметры в норме. Тест одиночный
				{
					myFile.print(buffer);                                           // Наименование проверки
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


	if (var_sound)// Признак качания частоты звука
	{
		do                           //Ожидание начала строба
			{
				strobe_sound = digitalRead(InNano12);
			} while(strobe_sound == LOW);    
		do                          //Ожидание конца строба
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
					Array_volume[i] = analogRead(analog);               // считываем значение
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
				Array_volume[i] = analogRead(analog);               // считываем значение
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

			//PORTB = B01000000; // пин 12 переводим в состояние HIGH
			//delay(5);
			//PORTB = B00000000; // пин 12 переводим в состояние LOW
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
		volume_fact = analogRead(analog);               // считываем значение
		voltage = volume_fact * (5.0 / 1023.0);
		voltage10 = voltage * 100;
}
void measure_power()
{
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	regBank.set(21,0);                           // XP2-2     sensor "Маг."  
	regBank.set(22,0);                           // XP5-3     sensor "ГГC."
	regBank.set(23,0);                           // XP3-3     sensor "ГГ-Радио1."
	regBank.set(24,0);                           // XP4-3     sensor "ГГ-Радио2."
	UpdateRegs();         
	delay(100);

	measure_volume_P(analog_tok);     
	save_reg_eeprom(adr_reg40400,voltage10);                     
	measure_volume_P(analog_12V);   
	save_reg_eeprom(adr_reg40493,voltage10);   
	measure_volume_P(analog_tok_x10);   
	save_reg_eeprom(adr_reg40402,voltage10);   

	regBank.set(23,1);                           // XP3-3     sensor "ГГ-Радио1."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40494,voltage10);   

	regBank.set(23,0);                           // XP3-3     sensor "ГГ-Радио1."
	regBank.set(24,1);                           // XP4-3     sensor "ГГ-Радио2."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40495,voltage10);   

	regBank.set(24,0);                           // XP4-3     sensor "ГГ-Радио2."
	regBank.set(22,1);                           // XP5-3     sensor "ГГC."
	UpdateRegs();         
	delay(200);
	measure_volume_P(analog_14); 
	save_reg_eeprom(adr_reg40496,voltage10);   
	regBank.set(22,0);                           // XP5-3     sensor "ГГC."
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
	 else var_sound = false;   // Признак качания частоты звука
	digitalWrite(kn1Nano, !bitRead(cannel_sound, 0));
	digitalWrite(kn2Nano, !bitRead(cannel_sound, 1));
	digitalWrite(kn3Nano, !bitRead(cannel_sound, 2));
}

void read_reg_error()
{
	 //regBank.add(40150);  // Блок для передачи информации
	unsigned int res_eeprom;
	unsigned int _adr_reg  = regBank.get(40124);                         // Адрес блока регистров для передачи в ПК результатов ошибки. 
	unsigned int _adr_mem  = regBank.get(40125);                         // Адрес блока памяти для передачи в ПК результатов ошибки.
	int _step_mem          = regBank.get(40126);                         // Адрес длины блока памяти для передачи в ПК результатов ошибки.
	int i_k                = 0;                                                   // Смещение адреса блока памяти
	//Serial.println("");
	//Serial.println(_adr_reg);
	//Serial.println(_adr_mem);
	//Serial.println(_step_mem);


	for (int i = 0; i < _step_mem;i++)                                   // Копирование блока памяти в регистры.        
		{
			hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem + i_k);    // 
			i_k++;
			low = i2c_eeprom_read_byte(deviceaddress,_adr_mem + i_k);
			i_k++;
			res_eeprom = (hi<<8) | low; 
			regBank.set(_adr_reg+40000+i,res_eeprom);                    //regBank.add(40150);  //  Получить результат ошибок
		}

	//for (int i = 0; i < _step_mem;i++)
	//{
	//		Serial.print(_adr_reg+40000+i);
	//		Serial.print(" - ");
	//		Serial.println(regBank.get(_adr_reg+40000+i));
	//}

	regBank.set(adr_control_command,0);                                  // Завершить программу    
}
void save_reg_error()
{
	//regBank.add(40150);                                     // Блок для передачи информации
	unsigned int _adr_reg  = regBank.get(40124);              // Начальный адрес блока регистров, 
	unsigned int _adr_mem  = regBank.get(40125);              // Начальный адрес блока памяти
	unsigned int _step_mem = regBank.get(40126);              // Длина блока с учетом хранения двухбайтных чисел
	unsigned int _u_count  = 0;                               // Временное хранения содержимого регистра.
	unsigned int i_k       = 0;                                        // Смещение адреса блока памяти

	for (unsigned int i = 0; i < _step_mem;i++)                        // Копирование блока памяти в регистры.        
		{
			_u_count = regBank.get(_adr_reg+40000+i);
			// разбираем _u_count на byte
			hi=highByte(_u_count);
			low=lowByte(_u_count);
			// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k, low); 
			i_k++;
		}

	regBank.set(adr_control_command,0);                        // Завершить программу    
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
			// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,adr, hi); 
			i2c_eeprom_write_byte(deviceaddress,adr+1, low); 
}

void clear_reg_eeprom()
{
	for (int i = 1023; i < 1870;i++)                            // Очистить блока регистров в памяти.        
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
		int _start1     = regBank.get(40124);  //  Адрес начала блока EEPROM для передачи в ПК результатов.
		int _end1       = regBank.get(40125);  //  Адрес конца блока памяти для передачи в ПК результатов.
		int _start2     = regBank.get(40124);  //  Адрес начала блока EEPROM для вывода в ПК результатов.
		int _end2       = regBank.get(40125);  //  Адрес конца блока памяти для вывода в ПК результатов.

		int _u_porog  = 0;  
		for (int i    = _start1; i < _end1;i++)                            // Копирование блока памяти в регистры.        
			{
				_u_porog = i;
				// разбираем _u_porog на byte
				hi=highByte(_u_porog);
				low=lowByte(_u_porog);
				// тут мы эти hi,low можем сохранить EEPROM
				i2c_eeprom_write_byte(deviceaddress,i, hi); 
				i++;
				i2c_eeprom_write_byte(deviceaddress,i, low); 
			}

		for (int i = _start2; i < _end2;i++)                            // Копирование блока памяти в регистры.        
			{

			  hi  = i2c_eeprom_read_byte(deviceaddress,i);   // 
			  i++;
			  low = i2c_eeprom_read_byte(deviceaddress,i);
			   _u_porog = (hi<<8) | low;                              // собираем как "настоящие программеры"
 
			   Serial.print(i-1);
			   Serial.print(" - ");   
			   Serial.println(_u_porog);
			 }
	regBank.set(adr_control_command,0);                                             // Завершить программу    
}
void i2c_test_int()
{
		int i_k       = 0;      
		int _u_porog  = 0;  
		for (int i = 0; i < 267;i++)                            // Копирование блока памяти в регистры.        
		{
			_u_porog = i+1;

		//	Serial.println(_u_porog);
		   // разбираем _u_porog на byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,i_k+201, low); 
			i_k++;
		}

		i_k       = 0;  
		for (int i = 0; i < 267;i++)                            // Копирование блока памяти в регистры.        
		{

		  hi  = i2c_eeprom_read_byte(deviceaddress,i_k+201);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,i_k+201);
		  i_k++;
		   _u_porog = (hi<<8) | low;                              // собираем как "настоящие программеры"
 /*
		   Serial.print(i+201);
		   Serial.print(" - ");   
		   Serial.println(_u_porog);*/
		 }

}

void test_RS232()
{
	regBank.set(adr_control_command,0);                                         // Завершить программу    
	delay(100);
}
void set_USB0()
{
	 // Внимание!
	 // true  - Включает реле в положение RS232 
	 // false - Включает реле в положение USB 
	
	  byte usb_rs232 = i2c_eeprom_read_byte(deviceaddress, adr_set_USB);    // Реле переключения USB - RS232 

	  if (usb_rs232 == 0)
		  {
			  regBank.set(12,false);
			  mcp_Out1.digitalWrite(11, false);                             //  Реле переключения USB - RS232 
			  Serial.println("Serial 2 - USB");
		  }
	  else
		  {
			 regBank.set(12,true);
			 mcp_Out1.digitalWrite(11, true);                               //  Реле переключения USB - RS232 
			 Serial.println("Serial 2 - RS232");
		  }
	regBank.set(adr_control_command,0);                                     // Завершить программу    
	delay(100);
}

void read_porog_eeprom(int adr_eeprom, int step_mem )                       // 
{
	int _adr_eeprom = adr_eeprom;                                           //
	int _step_mem = step_mem;                                               // 
	int _u_porog = 0;                                                       // Временное хранения содержимого регистра.
	int i_eeprom = 0;                                                       // Смещение адреса блока памяти

	for (int i = 1; i < _step_mem;i++)                                      // Копирование блока памяти в регистры.        
		{
		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_eeprom+i_eeprom);   // 
		  i_eeprom++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_eeprom+i_eeprom);
		  i_eeprom++;
		   _u_porog = (hi<<8) | low;                                        // собираем как "настоящие программеры"
		  por_int_buffer[i] = _u_porog;
		  //Serial.print(i);
		  //Serial.print(" - ");
		  //Serial.println(por_int_buffer[i]);
		 }
		 
}

void default_mem_porog()                                                    // Запись заводских установок уровней порога
{
	int _step_mem = 266;                                                    // Длина блока с учетом хранения двухбайтных чисел
	int _u_porog  = 0;                                                      // Временное хранения содержимого регистра.
	int i_k       = 0;                                                      // Смещение адреса блока памяти
	for (int i = 0; i < _step_mem;i++)                    
		{
			_u_porog = pgm_read_word_near(porog_default+i);
		   // разбираем _u_porog на byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,i_k+201, low); 
			i_k++;
			//Serial.print(i);
			//Serial.print(" - ");
			//Serial.println(_u_porog);
		}
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
// Чтение и запись информиции двух байтных слов
void set_mem_porog()
{
	/*
		Программа записи порогов в EEPROM
		Стартовый адрес памяти 200
		Стартовый адрес регистров 40130 
		Длина блока не более ??? байт
		regBank.get(40127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
		regBank.get(40128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
		regBank.get(40129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.
	*/
	int _adr_reg  = regBank.get(40127);              // Начальный адрес блока регистров, 
	int _adr_mem  = regBank.get(40128);              // Начальный адрес блока памяти
	int _step_mem = regBank.get(40129);              // Длина блока с учетом хранения двухбайтных чисел
	int _u_porog  = 0;                               // Временное хранения содержимого регистра.
	int i_k       = 0;                               // Смещение адреса блока памяти

	for (int i = 0; i < _step_mem;i++)                            // Копирование блока памяти в регистры.        
		{
			_u_porog = regBank.get(_adr_reg+40000+i);

		//	Serial.println(_u_porog);
		   // разбираем _u_porog на byte
			hi=highByte(_u_porog);
			low=lowByte(_u_porog);
		//	// тут мы эти hi,low можем сохранить EEPROM
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k+201, hi); 
			i_k++;
			i2c_eeprom_write_byte(deviceaddress,_adr_mem+i_k+201, low); 
			i_k++;
		}

	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(200);
}
void read_mem_porog() 
{
	/*
		Программа записи уровней порога из памяти в регистры
		regBank.get(40127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
		regBank.get(40128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
		regBank.get(40129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.
		Стартовый адрес памяти 200
		Стартовый адрес регистров 40130 
		Длина блока не более ??? байт
	*/
	unsigned int _adr_reg  = regBank.get(40127)+40000;              // Начальный адрес блока регистров, 
	int _adr_mem  = regBank.get(40128)+200;                         // Начальный адрес блока памяти
	int _step_mem = regBank.get(40129);                             // Длина блока с учетом хранения двухбайтных чисел
	int _u_porog = 0;                                               // Временное хранения содержимого регистра.
	int i_k = 0;                                                    // Смещение адреса блока памяти

	for (int i = 0; i < _step_mem;i++)                              // Копирование блока памяти в регистры.        
		{
		  hi  = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);   // 
		  i_k++;
		  low = i2c_eeprom_read_byte(deviceaddress,_adr_mem+i_k);
		  i_k++;
		   _u_porog = (hi<<8) | low;                                // собираем как "настоящие программеры"
		  regBank.set(_adr_reg+i,_u_porog);
		  //Serial.print(_adr_reg+i);
		  //Serial.print(" - ");
		  //Serial.print(_adr_mem+i_k-1);
		  //Serial.print(" - ");
		  //Serial.println(_u_porog);
		 }

	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(200);
}
// Чтение и запись информиции одно байтных слов
void mem_byte_trans_read()
{
	unsigned int _adr_reg = regBank.get(40127)+40000;           //  Адрес блока регистров для передачи в ПК уровней порогов.
	unsigned int _adr_mem = regBank.get(40128)+200;             //  Адрес блока памяти для передачи в ПК уровней порогов.
	unsigned int _size_block = regBank.get(40129);                       //  Адрес длины блока

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		regBank.set(_adr_reg+x_mem,i2c_eeprom_read_byte(deviceaddress,_adr_mem + x_mem));
	}

	regBank.set(adr_control_command,0);                         // Завершить программу    
	delay(200);
}
void mem_byte_trans_save()
{
	unsigned int _adr_reg    = regBank.get(40127);                     //  Адрес блока регистров для передачи в ПК уровней порогов.
	unsigned int _adr_mem    = regBank.get(40128);                     //  Адрес блока памяти для передачи в ПК уровней порогов.
	unsigned int _size_block = regBank.get(40129);                     //  Адрес длины блока

	for (unsigned int x_mem = 0;x_mem < _size_block;x_mem++)
	{
		i2c_eeprom_write_byte(deviceaddress, _adr_mem + x_mem, regBank.get(_adr_reg+x_mem));
	}
	regBank.set(adr_control_command,0);                                // Завершить программу    
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
	if (Serial2.available())                                     // есть что-то проверить? Есть данные в буфере?
		  {
			unsigned char overflowFlag = 0 ;                     // Флаг превышения размера буфера
			unsigned char buffer_count = 0;                      // Установить в начало чтения буфера

			while (Serial2.available())
				{
				  if (overflowFlag)                              // Если буфер переполнен - очистить
					 Serial2.read();
				  else                                           // Размер буфера в норме, считать информацию
					{
					if (buffer_count == BUFFER_SIZEKF)           // Проверить размер буфера
						{
							overflowFlag = 1;                    // Установить флаг превышения размера буфера
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
	 //Serial.println("error opening file");                            // Флаг ошибки  открытия файла
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

	regBank.set(adr_control_command,0);                                             // Завершить программу    
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
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(100);
}
void oscilloscope()
{

		do{

			set_sound_oscill();                      // Установить частоту для измерения осциллографом
			set_rezistor1();                         // Установить уровень сигнала №1 
			set_rezistor2();                         // Установить уровень сигнала №2
			set_analog_pin();                        // Установить вход осциллографа



		 for (int i = 0;i < 600;i++)
				{
					 Array_volume[i] = analogRead(analogIn_oscill);               // считываем значение
				}

			for (int i = 0;i < 600;i++)
				{
					PrintInt(Array_volume[i]);
					Serial2.write('\r'); // print new line
				}
	
		 }while(regBank.get(40));    
						   
	regBank.set(adr_control_command,0);                                             // Завершить программу    
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

//====================== Осциллограф завершающая версия =========================================


void setup_mcp()
{
	// Настройка расширителя портов
 
  mcp_Out1.begin(4);              //  Адрес (4) второго  расширителя портов
  mcp_Out1.pinMode(0, OUTPUT);    // Реле №0  Звук    
  mcp_Out1.pinMode(1, OUTPUT);    // Реле №1  Звук    
  mcp_Out1.pinMode(2, OUTPUT);    // Реле №2  Звук    
  mcp_Out1.pinMode(3, OUTPUT);    // Реле №3  Звук    
  mcp_Out1.pinMode(4, OUTPUT);    // Реле №4  Звук   XP1 10-12
  mcp_Out1.pinMode(5, OUTPUT);    // Реле №5  Звук    
  mcp_Out1.pinMode(6, OUTPUT);    // Реле №6  Звук   
  mcp_Out1.pinMode(7, OUTPUT);    // Реле №7  включить +12вольт  Питание платы Камертон
  
  mcp_Out1.pinMode(8, OUTPUT);    //  Реле №8 Звук на микрофон дифф.  
  mcp_Out1.pinMode(9, OUTPUT);    // Свободен J24 - 3    
  mcp_Out1.pinMode(10, OUTPUT);   // Свободен J24 - 2    
  mcp_Out1.pinMode(11, OUTPUT);   // Свободен J24 - 1   
  mcp_Out1.pinMode(12, OUTPUT);   // XP8 - 2  sensor     
  mcp_Out1.pinMode(13, OUTPUT);   // XP8 - 1  PTT       
  mcp_Out1.pinMode(14, OUTPUT);   // XS1 - 5   PTT      
  mcp_Out1.pinMode(15, OUTPUT);   // XS1 - 6 sensor      

	
  mcp_Out2.begin(6);              //  Адрес (6) второго  расширителя портов
  mcp_Out2.pinMode(0, OUTPUT);    // J8-12    XP7 4 PTT2    
  mcp_Out2.pinMode(1, OUTPUT);    // XP1 - 20  HandUp    
  mcp_Out2.pinMode(2, OUTPUT);    // J8-11    XP7 2 sensor
  mcp_Out2.pinMode(3, OUTPUT);    // J8-23    XP7 1 PTT1    
  mcp_Out2.pinMode(4, OUTPUT);    // XP2-2    sensor "Маг."    
  mcp_Out2.pinMode(5, OUTPUT);    // XP5-3    sensor "ГГC." 
  mcp_Out2.pinMode(6, OUTPUT);    // XP3-3    sensor "ГГ-Радио1."
  mcp_Out2.pinMode(7, OUTPUT);    // XP4-3    sensor "ГГ-Радио2."
  
  mcp_Out2.pinMode(8, OUTPUT);    // XP1- 19 HaSs
  mcp_Out2.pinMode(9, OUTPUT);    // XP1- 17 HaSPTT
  mcp_Out2.pinMode(10, OUTPUT);   // XP1- 16 HeS2Rs
  mcp_Out2.pinMode(11, OUTPUT);   // XP1- 15 HeS2PTT
  mcp_Out2.pinMode(12, OUTPUT);   // XP1- 13 HeS2Ls           
  mcp_Out2.pinMode(13, OUTPUT);   // XP1- 6  HeS1PTT            
  mcp_Out2.pinMode(14, OUTPUT);   // XP1- 5  HeS1Rs            
  mcp_Out2.pinMode(15, OUTPUT);   // XP1- 1  HeS1Ls          

 
  mcp_Analog.begin(5);            //  Адрес (5)  расширителя портов 
  mcp_Analog.pinMode(8, OUTPUT);  // DTR_D
  mcp_Analog.pinMode(9, OUTPUT);  // RTS_D
  mcp_Analog.pinMode(10, OUTPUT); // J15-2 Свободен
  mcp_Analog.pinMode(11, OUTPUT); // J15-3 Свободен
  mcp_Analog.pinMode(12, OUTPUT); // J15-4 Свободен
  mcp_Analog.pinMode(13, OUTPUT); // J15-5
  mcp_Analog.pinMode(14, OUTPUT); // J15-6
  mcp_Analog.pinMode(15, OUTPUT); // J15-7 
  
  mcp_Analog.pinMode(0, INPUT);   //  J22-1 Свободен
  mcp_Analog.pullUp(0, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(1, INPUT);   // J22-2 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(1, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(2, INPUT);   // J22-3 Свободен Свободен Расширитель портов на ввод 
  mcp_Analog.pullUp(2, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(3, INPUT);   // J22-4 Свободен Расширитель портов на ввод
  mcp_Analog.pullUp(3, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(4, INPUT);   // J22-5 Свободен  Расширитель портов на ввод
  mcp_Analog.pullUp(4, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(5, INPUT);   //CTS Расширитель портов на ввод
  mcp_Analog.pullUp(5, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(6, INPUT);   // DSR Расширитель портов на ввод
  mcp_Analog.pullUp(6, HIGH);     // Подключить внутренний резистор 100K к 5В.
  mcp_Analog.pinMode(7, INPUT);   //  DCD Расширитель портов на ввод
  mcp_Analog.pullUp(7, HIGH);     // Подключить внутренний резистор 100K к 5В.

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
Присвоить объект Modbus устройства обработчик протокола
Это то, где обработчик протокола будет смотреть, чтобы читать и писать
зарегистрированные данные. В настоящее время протокол Modbus Slave проводник может
имеется только одно устройство возложенные на него.
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
Лучше всего, чтобы настроить регистры как типа в смежных блоках. это
обеспечивает более эффективный поиск и регистра и уменьшает количество сообщений
требуются мастера для извлечения данных.
*/
	regBank.add(1);                           // Реле RL0 Звук  MIC1P
	regBank.add(2);                           // Реле RL1 Звук  MIC2P
	regBank.add(3);                           // Реле RL2 Звук  MIC3P
	regBank.add(4);                           // Реле RL3 Звук  LFE  "Маг."
	regBank.add(5);                           // Реле RL4 XP1 12  HeS2e   Включение микрофона инструктора
	regBank.add(6);                           // Реле RL5 Звук Front L, Front R
	regBank.add(7);                           // Реле RL6 Звук Center
	regBank.add(8);                           // Реле RL7 Питание платы
  
	regBank.add(9);                           // Реле RL8 Звук на микрофон
	regBank.add(10);                          // Реле RL9 XP1 10 Включение микрофона диспетчера
	regBank.add(11);                          // Реле RL10 Включение питания на высоковольтный модуль 
	regBank.add(12);                          // Свободен J24 - 1 
	regBank.add(13);                          // XP8 - 2   sensor Тангента ножная
	regBank.add(14);                          // XP8 - 1   PTT Тангента ножная
	regBank.add(15);                          // XS1 - 5   PTT Мик
	regBank.add(16);                          // XS1 - 6   sensor Мик
 
	regBank.add(17);                          // J8-12     XP7 4 PTT2   Танг. р.
	regBank.add(18);                          // XP1 - 20  HangUp  DCD
	regBank.add(19);                          // J8-11     XP7 2 sensor  Танг. р.
	regBank.add(20);                          // J8-23     XP7 1 PTT1 Танг. р.
	regBank.add(21);                          // XP2-2     sensor "Маг."  
	regBank.add(22);                          // XP5-3     sensor "ГГC."
	regBank.add(23);                          // XP3-3     sensor "ГГ-Радио1."
	regBank.add(24);                          // XP4-3     sensor "ГГ-Радио2."
 
	regBank.add(25);                          // XP1- 19 HaSs      sensor подключения трубки    MTT                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
	regBank.add(26);                          // XP1- 17 HaSPTT    CTS DSR вкл.  
	regBank.add(27);                          // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	regBank.add(28);                          // XP1- 15 HeS2PTT   CTS вкл PTT Инструктора
	regBank.add(29);                          // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 
	regBank.add(30);                          // XP1- 6  HeS1PTT   CTS вкл   РТТ Диспетчера
	regBank.add(31);                          // XP1- 5  HeS1Rs    sensor подкючения гарнитуры диспетчера с 2 наушниками
	regBank.add(32);                          // XP1- 1  HeS1Ls    sensor подкючения гарнитуры диспетчера
	regBank.add(39);                          // Флаг окончания загрузки системы
	regBank.add(40);                          // Флаг разрешения измерения осциллоскопом
	regBank.add(41);                          // Флаг управления радиопередачей
	regBank.add(42);                          // Флаг ГГС (mute)

	regBank.add(118);                         // Флаг индикации многоразовой проверки
	regBank.add(119);                         // 

	regBank.add(120);                         // Флаг индикации возникновения любой ошибки
	regBank.add(122);                         // Флаг индикации открытия файла
	regBank.add(123);                         // Флаг индикации закрытия файла
	regBank.add(124);                         // Флаг индикации связи с модулем "Камертон"
	regBank.add(125);                         // Флаг индикации инициализации SD памяти
	regBank.add(126);                         //  
	regBank.add(127);                         //  
	regBank.add(128);                         //  
	regBank.add(129);                         //  

	regBank.add(130);                         //  Флаг индикации порта 0 - RS232, 1 - USB0
	regBank.add(131);                         //  


	regBank.add(200);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(201);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(202);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(203);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(204);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(205);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(206);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(207);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(208);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(209);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(210);                         // Флаг ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(211);                         // Флаг ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(212);                         // Флаг ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(213);                         // Флаг ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(214);                         // Флаг ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(215);                         // Флаг ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(216);                         // Флаг ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(217);                         // Флаг ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(218);                         // Флаг ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(219);                         // Флаг ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 
	 
	regBank.add(220);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(221);                         // Флаг ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(222);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(223);                         // Флаг ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(224);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(225);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(226);                         // Флаг ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(227);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(228);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(229);                         // Флаг ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(230);                         // Флаг ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(231);                         // Флаг ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(232);                         // Флаг ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(233);                         // Флаг ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(234);                         // Флаг ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(235);                         // Флаг ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(236);                         // Флаг ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(237);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(238);                         // Флаг ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(239);                         // Флаг ошибки  ADC0  ток x1 

	regBank.add(240);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(241);                         // Флаг ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(242);                         // Флаг ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(243);                         // Флаг ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(244);                         // Флаг ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(245);                         // Флаг ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(246);                         // Флаг ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(247);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(248);                         // Флаг ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(249);                         // Флаг ошибки ADC2 ток x10  

	regBank.add(250);                         // Флаг ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(251);                         // Флаг ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(252);                         // Флаг ошибки "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(253);                         // Флаг ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(254);                         // Флаг ошибки "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(255);                         // Флаг ошибки "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(256);                         // Флаг ошибки "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(257);                         // Флаг ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(258);                         // Флаг ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(259);                         // Флаг ошибки "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(260);                         // Флаг ошибки "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(261);                         // Флаг ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(262);                         // Флаг ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(263);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(264);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(265);                         // Флаг ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(266);                         // Флаг ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(267);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(268);                         // Флаг ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(269);                         // Флаг ошибки Длительность регулировки яркости 

	regBank.add(270);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(271);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(272);                         // Флаг ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(273);                         // Флаг ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(274);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(275);                         // Флаг ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(276);                         // Флаг ошибки "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(277);                         // Флаг ошибки "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(278);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(279);                         // Флаг ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(280);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(281);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(282);                         // Флаг ошибки "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(283);                         // Флаг ошибки "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(284);                         // Флаг ошибки "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(285);                         // Флаг ошибки "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(286);                         // Флаг ошибки "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(287);                         // Флаг ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(288);                         // Флаг ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(289);                         // Флаг ошибки "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(290);                         // Флаг ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(291);                         // Флаг ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(292);                         // Флаг ошибки "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(293);                         // Флаг ошибки ADC1 напряжение 12/3 вольт
	regBank.add(294);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio1
	regBank.add(295);                         // Флаг ошибки ADC14 напряжение 12/3 вольт Radio2
	regBank.add(296);                         // Флаг ошибки ADC14 напряжение 12/3 вольт ГГС
	regBank.add(297);                         // Флаг ошибки ADC15 напряжение светодиода 3,6 вольта
	regBank.add(298);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         ON  - ";      
	regBank.add(299);                         // Флаг ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(300);                         // Флаг ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(301);                         // Флаг ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(302);                         // Флаг ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(303);                         // Флаг ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(304);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(305);                         // Флаг ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(306);                         // Флаг ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(307);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(308);                         // Флаг ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(309);                         // Флаг ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(310);                         // Флаг ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(311);                         // Флаг ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(312);                         // Флаг ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(313);                         // Флаг ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(314);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(315);                         // Флаг ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(316);                         // Флаг ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(317);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(318);                         // Флаг ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(319);                         // Флаг ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(320);                         // Флаг ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(321);                         // Флаг ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(322);                         // Флаг ошибки "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(323);                         // Флаг ошибки "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(324);                         // Флаг ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(325);                         // Флаг ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(326);                         // Флаг ошибки "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(327);                         // Флаг ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(328);                         // Флаг ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(329);                         // Флаг ошибки Код яркости дисплея

	regBank.add(330);                         // Флаг ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(331);                         // Флаг ошибки "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	regBank.add(332);                         // Флаг ошибки "Test GGS ** Signal mag radio                                ON  - ";
	regBank.add(333);                         // "Test headset instructor ** Signal mag radio                             ON  - ";
	regBank.add(334);                         // "Test headset dispatcher ** Signal mag radio                             ON  - ";
	regBank.add(335);                         // "Test MTT ** Signal mag radio                                            ON  - ";
	regBank.add(336);                         // "Test Microphone ** Signal mag radio                                     ON  - ";
	regBank.add(337);                         // Свободен 

	regBank.add(10081);                       // Адрес флагa индикации состояния сигнала CTS
	regBank.add(10082);                       // Адрес флагa индикации состояния сигнала DSR
	regBank.add(10083);                       // Адрес флагa индикации состояния сигнала DCD

						                      //Add Input registers 30001-30040 to the register bank

	//regBank.add(30000);  // байт 0 отпр бит 0 - Камертон   бит "D"
	//regBank.add(30001);  // байт 0 отпр бит 1 - Камертон   "1"
	//regBank.add(30002);  // байт 0 отпр бит 2 - Камертон   "0"  
	//regBank.add(30003);  // байт 0 отпр бит 3 - Камертон   "1"
	//regBank.add(30004);  // байт 0 отпр бит 4 - Камертон   "0" число байт пересылки (2)
	//regBank.add(30005);  // байт 0 отпр бит 5 - Камертон   "1" число байт пересылки (2)
	//regBank.add(30006);  // байт 0 отпр бит 6 - Камертон   "0" число байт пересылки (2)
	//regBank.add(30007);  // байт 0 отпр бит 7 - Камертон   "0"
 // 
	//regBank.add(30008);  // байт 1 отпр бит 0 - Камертон   CRC0
	//regBank.add(30009);  // байт 1 отпр бит 1 - Камертон   CRC1
	//regBank.add(30010);  // байт 1 отпр бит 2 - Камертон   CRC2
	//regBank.add(30011);  // байт 1 отпр бит 3 - Камертон   CRC3
	//regBank.add(30012);  // байт 1 отпр бит 4 - Камертон   выключение ГГС (Mute)
	//regBank.add(30013);  // байт 1 отпр бит 5 - Камертон   радиопередача
	//regBank.add(30014);  // байт 1 отпр бит 6 - Камертон   чтение/ запись кода яркости
	//regBank.add(30015);  // байт 1 отпр бит 7 - Камертон   "1"
 // 
	//regBank.add(30016);  // байт 2 отпр бит 0 - Камертон   код яркости экрана
	//regBank.add(30017);  // байт 2 отпр бит 1 - Камертон   код яркости экрана
	//regBank.add(30018);  // байт 2 отпр бит 2 - Камертон   код яркости экрана
	//regBank.add(30019);  // байт 2 отпр бит 3 - Камертон   код яркости экрана
	//regBank.add(30020);  // байт 2 отпр бит 4 - Камертон   код яркости экрана
	//regBank.add(30021);  // байт 2 отпр бит 5 - Камертон   код яркости экрана
	//regBank.add(30022);  // байт 2 отпр бит 6 - Камертон   код яркости экрана
	//regBank.add(30023);  // байт 2 отпр бит 7 - Камертон   "0" 

	//					 // Биты строки  полученные из  платы камертон. Получено 4 байта
 // 
	//regBank.add(30024);  // байт 1 прием бит 0 - Камертон  флаг подключения ГГ Радио2
	//regBank.add(30025);  // байт 1 прием бит 1 - Камертон  флаг подключения ГГ Радио1
	//regBank.add(30026);  // байт 1 прием бит 2 - Камертон  флаг подключения трубки
	//regBank.add(30027);  // байт 1 прием бит 3 - Камертон  флаг подключения ручной тангенты
	//regBank.add(30028);  // байт 1 прием бит 4 - Камертон  флаг подключения педали
	//regBank.add(30029);  // байт 1 прием бит 5 - Камертон   "1"
	//regBank.add(30030);  // байт 1 прием бит 6 - Камертон   "0" 
	//regBank.add(30031);  // байт 1 прием бит 7 - Камертон   "1"
 // 
	//regBank.add(30032);  // байт 2 прием бит 0 - Камертон   код яркости экрана
	//regBank.add(30033);  // байт 2 прием бит 1 - Камертон   код яркости экрана
	//regBank.add(30034);  // байт 2 прием бит 2 - Камертон   код яркости экрана
	//regBank.add(30035);  // байт 2 прием бит 3 - Камертон   код яркости экрана
	//regBank.add(30036);  // байт 2 прием бит 4 - Камертон   код яркости экрана
	//regBank.add(30037);  // байт 2 прием бит 5 - Камертон   код яркости экрана
	//regBank.add(30038);  // байт 2 прием бит 6 - Камертон   код яркости экрана
	//regBank.add(30039);  // байт 2 прием бит 7 - Камертон   "0" 
 // 
	//regBank.add(30040);  // байт 3 прием бит 0 - Камертон   флаг подключения магнитофона
	//regBank.add(30041);  // байт 3 прием бит 1 - Камертон   флаг подключения гарнитуры инструктора 2 наушниками
	//regBank.add(30042);  // байт 3 прием бит 2 - Камертон   флаг подключения гарнитуры инструктора
	//regBank.add(30043);  // байт 3 прием бит 3 - Камертон   флаг подключения гарнитуры диспетчера с 2 наушниками
	//regBank.add(30044);  // байт 3 прием бит 4 - Камертон   флаг подключения гарнитуры диспетчера
	//regBank.add(30045);  // байт 3 прием бит 5 - Камертон   флаг подключения микрофона XS1 - 6 sensor
	//regBank.add(30046);  // байт 3 прием бит 6 - Камертон   флаг подключения ГГС
	//regBank.add(30047);  // байт 3 прием бит 7 - Камертон   "0" 
 // 
	//regBank.add(30048);  // байт 4 прием бит 0 - Камертон   CRC0
	//regBank.add(30049);  // байт 4 прием бит 1 - Камертон   CRC1
	//regBank.add(30050);  // байт 4 прием бит 2 - Камертон   CRC2   
	//regBank.add(30051);  // байт 4 прием бит 3 - Камертон   CRC3   
	//regBank.add(30052);  // байт 4 прием бит 4 - Камертон   флаг выключения микрофона инструктора
	//regBank.add(30053);  // байт 4 прием бит 5 - Камертон    флаг радиопередачи
	//regBank.add(30054);  // байт 4 прием бит 6 - Камертон   флаг выключения микрофона диспетчера
	//regBank.add(30055);  // байт 4 прием бит 7 - Камертон   "0" 


	//regBank.set(40004+buffer,Serial1.read());
	regBank.add(40000);  // 
	regBank.add(40001);  // Регистры обмена с Аудио 1
	regBank.add(40002);  // Регистры обмена с Аудио 1
	regBank.add(40003);  // Регистры обмена с Аудио 1
	regBank.add(40004);  // Регистры обмена с Аудио 1
	regBank.add(40005);  // Регистры обмена с Аудио 1
	regBank.add(40006);  // Регистры обмена с Аудио 1
	regBank.add(40007);  // Регистры обмена с Аудио 1
	regBank.add(40008);  // 
	regBank.add(40009);  // 

	regBank.add(40010);  // №  Аудио 1
	regBank.add(40011);  // №  Аудио 1
	regBank.add(40012);  // №  Аудио 1
	regBank.add(40013);  // №  Аудио 1
	regBank.add(40014);  // 
	regBank.add(40015);  // 
	regBank.add(40016);  // 
	regBank.add(40017);  // 
	regBank.add(40018);  // 
	regBank.add(40019);  // 
	regBank.add(40040);  // analogIn_oscill   Записать номер аналогового канала Камертон 5.0
	regBank.add(40041);  //  Установить частоту для измерения осциллографом (0-7)

						 // Текущее время 
	regBank.add(40046);  // адрес день модуля часов контроллера
	regBank.add(40047);  // адрес месяц модуля часов контроллера
	regBank.add(40048);  // адрес год модуля часов контроллера
	regBank.add(40049);  // адрес час модуля часов контроллера
	regBank.add(40050);  // адрес минута модуля часов контроллера
	regBank.add(40051);  // адрес секунда модуля часов контроллера
 
						 // Установка времени в контроллере
	regBank.add(40052);  // адрес день
	regBank.add(40053);  // адрес месяц
	regBank.add(40054);  // адрес год
	regBank.add(40055);  // адрес час
	regBank.add(40056);  // адрес минута
	regBank.add(40057);  // 
	regBank.add(40058);  // 
	regBank.add(40059);  // 
	
	regBank.add(40060);  // Адрес хранения величины сигнала резистором № 1
	regBank.add(40061);  // Адрес хранения величины яркости для управления
	regBank.add(40062);  // Адрес хранения величины яркости для передачи в программу
	regBank.add(40063);  // Адрес хранения длительности импульса яркости для передачи в программу ПК
	regBank.add(40064);  // Адрес хранения величины сигнала резистором № 2

	/*

	regBank.add(40064); // адрес ошибки
	regBank.add(40065); // адрес ошибки
	regBank.add(40066); // адрес ошибки
	regBank.add(40067); // адрес ошибки
	regBank.add(40068); // адрес ошибки
	regBank.add(40069); // адрес ошибки
	regBank.add(40070); // адрес ошибки
	regBank.add(40071); // адрес ошибки

	regBank.add(40072); // адрес ошибки в %
	regBank.add(40073); // адрес ошибки в %
	regBank.add(40074); // адрес ошибки в %
	regBank.add(40075); // адрес ошибки %
	regBank.add(40076); // адрес ошибки %
	regBank.add(40077); // адрес ошибки %
	regBank.add(40078); // адрес ошибки %
	regBank.add(40079); // адрес ошибки %
	regBank.add(40080); // адрес ошибки %
	regBank.add(40081); // адрес ошибки %
	regBank.add(40082); // адрес ошибки %
	regBank.add(40083); // адрес ошибки %

	// Время ошибки на включение
	regBank.add(40084); // адрес день adr_Mic_On_day 
	regBank.add(40085); // адрес месяц adr_Mic_On_month  
	regBank.add(40086); // адрес год adr_Mic_On_year  
	regBank.add(40087); // адрес час adr_Mic_On_hour 
	regBank.add(40088); // адрес минута adr_Mic_On_minute 
	regBank.add(40089); // адрес секунда  adr_Mic_On_second    

	// Время ошибки на выключение
	regBank.add(40090); // адрес день adr_Mic_Off_day    
	regBank.add(40091); // адрес месяц  adr_Mic_Off_month 
	regBank.add(40092); // адрес год adr_Mic_Off_year  
	regBank.add(40093); // адрес час adr_Mic_Off_hour   
	regBank.add(40094); // адрес минута adr_Mic_Off_minute   
	regBank.add(40095); // адрес секунда adr_Mic_Off_second    
	*/
	// Время старта теста
	regBank.add(40096);  // адрес день  adr_Mic_Start_day    
	regBank.add(40097);  // адрес месяц adr_Mic_Start_month  
	regBank.add(40098);  // адрес год adr_Mic_Start_year  
	regBank.add(40099);  // адрес час adr_Mic_Start_hour 
	regBank.add(40100);  // адрес минута adr_Mic_Start_minute 
	regBank.add(40101);  // адрес секунда adr_Mic_Start_second  

	// Время окончания теста
	regBank.add(40102);  // адрес день adr_Mic_Stop_day 
	regBank.add(40103);  // адрес месяц adr_Mic_Stop_month 
	regBank.add(40104);  // адрес год adr_Mic_Stop_year
	regBank.add(40105);  // адрес час adr_Mic_Stop_hour 
	regBank.add(40106);  // адрес минута adr_Mic_Stop_minute  
	regBank.add(40107);  // адрес секунда adr_Mic_Stop_second 

	// Продолжительность выполнения теста
	regBank.add(40108);  // адрес день adr_Time_Test_day 
	regBank.add(40109);  // адрес час adr_Time_Test_hour 
	regBank.add(40110);  // адрес минута adr_Time_Test_minute
	regBank.add(40111);  // адрес секунда adr_Time_Test_second

	// Имя файла
	regBank.add(40112);  // Адрес хранения переменной год 
	regBank.add(40113);  // Адрес хранения переменной месяц 
	regBank.add(40114);  // Адрес хранения переменной день
	regBank.add(40115);  // Адрес хранения переменной счетчика последнего номера файла
	regBank.add(40116);  // Адрес хранения переменной счетчика текущего номера файла

	regBank.add(40120);  // adr_control_command Адрес передачи комманд на выполнение
	regBank.add(40121);  // Адрес счетчика всех ошибок
	regBank.add(40122);  //
	regBank.add(40123);  //
	regBank.add(40124);  //  Адрес блока регистров для передачи в ПК результатов ошибки.
	regBank.add(40125);  //  Адрес блока памяти для передачи в ПК результатов ошибки.
	regBank.add(40126);  //  Адрес длины блока памяти для передачи в ПК результатов ошибки.
	regBank.add(40127);  //  Адрес блока регистров для передачи в ПК уровней порогов.
	regBank.add(40128);  //  Адрес блока памяти для передачи в ПК уровней порогов.
	regBank.add(40129);  //  Адрес длины блока памяти для передачи в ПК уровней порогов.

	regBank.add(40130);  //  Регистры временного хранения для передачи уровней порогов 
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

	regBank.add(40150);  //  Получить результат ошибок
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
	regBank.add(40200);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            OFF - ";
	regBank.add(40201);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 OFF - ";
	regBank.add(40202);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 OFF - "; 
	regBank.add(40203);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          OFF - ";
	regBank.add(40204);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          OFF - "; 
	regBank.add(40205);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          OFF - ";
	regBank.add(40206);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          OFF - ";
	regBank.add(40207);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 OFF - "; 
	regBank.add(40208);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            OFF - "; 
	regBank.add(40209);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            OFF - ";  

	regBank.add(40210);                         // Aдрес счетчика ошибки "Sensor MTT                          XP1- 19 HaSs            ON  - ";
	regBank.add(40211);                         // Aдрес счетчика ошибки "Sensor tangenta ruchnaja            XP7 - 2                 ON  - ";
	regBank.add(40212);                         // Aдрес счетчика ошибки "Sensor tangenta nognaja             XP8 - 2                 ON  - "; 
	regBank.add(40213);                         // Aдрес счетчика ошибки "Sensor headset instructor 2         XP1- 16 HeS2Rs          ON  - ";
	regBank.add(40214);                         // Aдрес счетчика ошибки "Sensor headset instructor           XP1- 13 HeS2Ls          ON  - "; 
	regBank.add(40215);                         // Aдрес счетчика ошибки "Sensor headset dispatcher 2         XP1- 13 HeS2Ls          ON  - ";
	regBank.add(40216);                         // Aдрес счетчика ошибки "Sensor headset dispatcher           XP1- 1  HeS1Ls          ON  - ";
	regBank.add(40217);                         // Aдрес счетчика ошибки "Sensor microphone                   XS1 - 6                 ON  - "; 
	regBank.add(40218);                         // Aдрес счетчика ошибки "Microphone headset instructor Sw.   XP1 12 HeS2e            ON  - "; 
	regBank.add(40219);                         // Aдрес счетчика ошибки "Microphone headset dispatcher Sw.   XP1 12 HeS2e            ON  - "; 

	regBank.add(40220);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        OFF - ";
	regBank.add(40221);                         // Aдрес счетчика ошибки "Command PTT headset instructor (CTS)                        ON  - ";
	regBank.add(40222);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        OFF - ";
	regBank.add(40223);                         // Aдрес счетчика ошибки "Command PTT headset dispatcher (CTS)                        ON  - ";
	regBank.add(40224);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40225);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40226);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40227);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40228);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40229);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40230);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40231);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40232);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40233);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40234);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40235);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40236);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40237);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40238);                         // Aдрес счетчика ошибки "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40239);                         // Aдрес счетчика ошибки ADC0  ток x1 

	regBank.add(40240);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40241);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40242);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40243);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40244);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40245);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40246);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40247);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40248);                         // Aдрес счетчика ошибки "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40249);                         // Aдрес счетчика ошибки ADC2 ток x10

	regBank.add(40250);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40251);                         // Aдрес счетчика ошибки "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40252);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40253);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40254);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40255);                         // Aдрес счетчика ошибки "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40256);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40257);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40258);                         // Aдрес счетчика ошибки "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40259);                         // Aдрес счетчика ошибки "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40260);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40261);                         // Aдрес счетчика ошибки "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40262);                         // Aдрес счетчика ошибки "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40263);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40264);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  OFF - ";
	regBank.add(40265);                         // Aдрес счетчика ошибки "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40266);                         // Aдрес счетчика ошибки "Test microphone PTT  (CTS)                                  ON  - ";
	regBank.add(40267);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40268);                         // Aдрес счетчика ошибки "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40269);                         // Aдрес счетчика ошибки Длительность регулировки яркости

	regBank.add(40270);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40271);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40272);                         // Aдрес счетчика ошибки "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40273);                         // Aдрес счетчика ошибки "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40274);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40275);                         // Aдрес счетчика ошибки "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40276);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40277);                         // Aдрес счетчика ошибки "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40278);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40279);                         // Aдрес счетчика ошибки "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40280);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40281);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40282);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40283);                         // Aдрес счетчика ошибки "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40284);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40285);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40286);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40287);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40288);                         // Aдрес счетчика ошибки "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40289);                         // Aдрес счетчика ошибки "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40290);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40291);                         // Aдрес счетчика ошибки "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40292);                         // Aдрес счетчика ошибки "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40293);                         // Aдрес счетчика  ошибки ADC1 напряжение 12/3 вольт
	regBank.add(40294);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio1
	regBank.add(40295);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт Radio2
	regBank.add(40296);                         // Aдрес счетчика  ошибки ADC14 напряжение 12/3 вольт ГГС
	regBank.add(40297);                         // Aдрес счетчика  ошибки ADC15 напряжение светодиода 3,6 вольта
	regBank.add(40298);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         ON  - ";    
	regBank.add(40299);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40300);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40301);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40302);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40303);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40304);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40305);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40306);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40307);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40308);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40309);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40310);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40311);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40312);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40313);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40314);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40315);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40316);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40317);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40318);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40319);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40320);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40321);                         // Aдрес счетчика ошибки "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40322);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40323);                         // Aдрес счетчика ошибки "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40324);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40325);                         // Aдрес счетчика ошибки "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40326);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40327);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40328);                         // Aдрес счетчика ошибки "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40329);                         // Aдрес счетчика ошибки Код регулировки яркости                             // 

	regBank.add(40330);                         // Aдрес счетчика ошибки "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(40331);                         // Aдрес счетчика ошибки "Test Radio2 ** Signal mag radio                             ON  - ";
	regBank.add(40332);                         // Aдрес счетчика ошибки "Test GGS    ** Signal mag radio                             ON  - ";

	*/
	// ++++++++++++++++++++++ Регистры хранения данных при проверке модулей ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
	regBank.add(40400);                         // Aдрес напряжение ADC0  ток x1 
	regBank.add(40401);                         // Aдрес напряжение ADC1 напряжение 12/3 вольт
	regBank.add(40402);                         // Aдрес напряжение ADC2 ток x10
	regBank.add(40403);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio1
	regBank.add(40404);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт Radio2
	regBank.add(40405);                         // Aдрес напряжение ADC14 напряжение 12/3 вольт ГГС
	regBank.add(40406);                         // Aдрес напряжение ADC15 напряжение светодиода 3,6 вольта
	regBank.add(40407);                         // Aдрес 
	regBank.add(40408);                         // Aдрес 
	regBank.add(40409);                         // Aдрес  

	regBank.add(40410);                         // Aдрес счетчика 
	regBank.add(40411);                         // Aдрес счетчика  
	regBank.add(40412);                         // Aдрес счетчика  
	regBank.add(40413);                         // Aдрес счетчика  
	regBank.add(40414);                         // Aдрес счетчика  
	regBank.add(40415);                         // Aдрес счетчика  
	regBank.add(40416);                         // Aдрес счетчика  
	regBank.add(40417);                         // Aдрес счетчика  
	regBank.add(40418);                         // Aдрес счетчика  
	regBank.add(40419);                         // Aдрес счетчика  

	regBank.add(40420);                         // Aдрес  ;
	regBank.add(40421);                         // Aдрес  ;
	regBank.add(40422);                         // Aдрес  ;
	regBank.add(40423);                         // Aдрес  ;
	regBank.add(40424);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     ON  - ";
	regBank.add(40425);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     ON  - ";   
	regBank.add(40426);                         // Aдрес данных измерения "Test headset instructor ** Signal Mag phone                 ON  - ";
	regBank.add(40427);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     ON  - ";
	regBank.add(40428);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     ON  - ";  
	regBank.add(40429);                         // Aдрес данных измерения "Test headset dispatcher ** Signal Mag phone                 ON  - ";

	regBank.add(40430);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontL                    OFF - ";
	regBank.add(40431);                         // Aдрес данных измерения "Test headset instructor ** Signal FrontR                    OFF - ";
	regBank.add(40432);                         // Aдрес данных измерения "Test headset instructor ** Signal LineL                     OFF - ";
	regBank.add(40433);                         // Aдрес данных измерения "Test headset instructor ** Signal LineR                     OFF - ";
	regBank.add(40434);                         // Aдрес данных измерения "Test headset instructor ** Signal mag radio                 OFF - "; 
	regBank.add(40435);                         // Aдрес данных измерения "Test headset instructor ** Signal mag phone                 OFF - ";
	regBank.add(40436);                         // Aдрес данных измерения "Test headset instructor ** Signal GGS                       OFF - ";
	regBank.add(40437);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio1                 OFF - ";
	regBank.add(40438);                         // Aдрес данных измерения "Test headset instructor ** Signal GG Radio2                 OFF - ";
	regBank.add(40439);                         // Aдрес данных измерения ADC0  ток x1 

	regBank.add(40440);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontL                    OFF - ";
	regBank.add(40441);                         // Aдрес данных измерения "Test headset dispatcher ** Signal FrontR                    OFF - ";
	regBank.add(40442);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineL                     OFF - "; 
	regBank.add(40443);                         // Aдрес данных измерения "Test headset dispatcher ** Signal LineR                     OFF - ";
	regBank.add(40444);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag radio                 OFF - "; 
	regBank.add(40445);                         // Aдрес данных измерения "Test headset dispatcher ** Signal mag phone                 OFF - ";
	regBank.add(40446);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GGS                       OFF - "; 
	regBank.add(40447);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio1                 OFF - ";
	regBank.add(40448);                         // Aдрес данных измерения "Test headset dispatcher ** Signal GG Radio2                 OFF - "; 
	regBank.add(40449);                         // Aдрес данных измерения ADC2 ток x10

	regBank.add(40450);                         // Aдрес данных измерения "Test MTT ** Signal FrontL                                   OFF - ";
	regBank.add(40451);                         // Aдрес данных измерения "Test MTT ** Signal FrontR                                   OFF - ";
	regBank.add(40452);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    OFF - ";
	regBank.add(40453);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    OFF - "; 
	regBank.add(40454);                         // Aдрес данных измерения "Test MTT ** Signal mag radio                                OFF - ";
	regBank.add(40455);                         // Aдрес данных измерения "Test MTT ** Signal mag phone                                OFF - ";
	regBank.add(40456);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      OFF - ";
	regBank.add(40457);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio1                                OFF - ";
	regBank.add(40458);                         // Aдрес данных измерения "Test MTT ** Signal GG Radio2                                OFF - "; 
	regBank.add(40459);                         // Aдрес данных измерения "Test MTT ** Signal GGS                                      ON  - ";

	regBank.add(40460);                         // Aдрес данных измерения "Test MTT ** Signal LineL                                    ON  - ";
	regBank.add(40461);                         // Aдрес данных измерения "Test MTT ** Signal LineR                                    ON  - ";  
	regBank.add(40462);                         // Aдрес данных измерения "Test MTT ** Signal Mag phone                                ON  - ";
	regBank.add(40463);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       OFF - ";
	regBank.add(40464);                         // 
	regBank.add(40465);                         // Aдрес данных измерения "Test MTT PTT    (CTS)                                       ON  - ";
	regBank.add(40466);                         // 
	regBank.add(40467);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       OFF - ";
	regBank.add(40468);                         // Aдрес данных измерения "Test MTT HangUp (DCD)                                       ON  - ";
	regBank.add(40469);                         // Длительность импульса регулировки яркости дисплея

	regBank.add(40470);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        OFF - ";
	regBank.add(40471);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        OFF - ";
	regBank.add(40472);                         // Aдрес данных измерения "Command PTT1 tangenta ruchnaja (CTS)                        ON  - ";
	regBank.add(40473);                         // Aдрес данных измерения "Command PTT2 tangenta ruchnaja (DCR)                        ON  - ";
	regBank.add(40474);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            OFF - ";
	regBank.add(40475);                         // Aдрес данных измерения "Command sensor tangenta ruchnaja                            ON  - ";
	regBank.add(40476);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             OFF - ";
	regBank.add(40477);                         // Aдрес данных измерения "Command sensor tangenta nognaja                             ON  - ";
	regBank.add(40478);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          OFF - ";
	regBank.add(40479);                         // Aдрес данных измерения "Command PTT tangenta nognaja (CTS)                          ON  - ";

	regBank.add(40480);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   OFF - ";
	regBank.add(40481);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   OFF - ";
	regBank.add(40482);                         // Aдрес данных измерения "Test GGS ** Signal LineL                                    OFF - ";
	regBank.add(40483);                         // Aдрес данных измерения "Test GGS ** Signal LineR                                    OFF - ";
	regBank.add(40484);                         // Aдрес данных измерения "Test GGS ** Signal mag radio                                OFF - ";
	regBank.add(40485);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                OFF - ";
	regBank.add(40486);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      OFF - ";
	regBank.add(40487);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio1                                OFF - ";
	regBank.add(40488);                         // Aдрес данных измерения "Test GGS ** Signal GG Radio2                                OFF - ";
	regBank.add(40489);                         // Aдрес данных измерения "Test GGS ** Signal GGS                                      ON  - ";

	regBank.add(40490);                         // Aдрес данных измерения "Test GGS ** Signal FrontL                                   ON  - ";
	regBank.add(40491);                         // Aдрес данных измерения "Test GGS ** Signal FrontR                                   ON  - ";
	regBank.add(40492);                         // Aдрес данных измерения "Test GGS ** Signal mag phone                                ON  - ";
	regBank.add(40493);                         // Aдрес данных измерения ADC1 напряжение 12/3 вольт
	regBank.add(40494);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio1
	regBank.add(40495);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт Radio2
	regBank.add(40496);                         // Aдрес данных измерения ADC14 напряжение 12/3 вольт ГГС
	regBank.add(40497);                         // Aдрес данных измерения ADC15 напряжение светодиода 3,6 вольта
	regBank.add(40498);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         ON  - "; 
	regBank.add(40499);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             ON  - ";   

	regBank.add(40500);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontL                                OFF - ";
	regBank.add(40501);                         // Aдрес данных измерения "Test Radio1 ** Signal FrontR                                OFF - ";
	regBank.add(40502);                         // Aдрес данных измерения "Test Radio1 ** Signal LineL                                 OFF - ";
	regBank.add(40503);                         // Aдрес данных измерения "Test Radio1 ** Signal LineR                                 OFF - ";
	regBank.add(40504);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             OFF - ";
	regBank.add(40505);                         // Aдрес данных измерения "Test Radio1 ** Signal mag phone                             OFF - ";
	regBank.add(40506);                         // Aдрес данных измерения "Test Radio1 ** Signal GGS                                   OFF - ";
	regBank.add(40507);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio1                             OFF - ";
	regBank.add(40508);                         // Aдрес данных измерения "Test Radio1 ** Signal GG Radio2                             OFF - ";
	regBank.add(40509);                         // Aдрес данных измерения "Test Radio1 ** Signal Radio1                                ON  - ";

	regBank.add(40510);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontL                                OFF - ";
	regBank.add(40511);                         // Aдрес данных измерения "Test Radio2 ** Signal FrontR                                OFF - ";
	regBank.add(40512);                         // Aдрес данных измерения "Test Radio2 ** Signal LineL                                 OFF - ";
	regBank.add(40513);                         // Aдрес данных измерения "Test Radio2 ** Signal LineR                                 OFF - ";
	regBank.add(40514);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             OFF - ";
	regBank.add(40515);                         // Aдрес данных измерения "Test Radio2 ** Signal mag phone                             OFF - ";
	regBank.add(40516);                         // Aдрес данных измерения "Test Radio2 ** Signal GGS                                   OFF - ";
	regBank.add(40517);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio1                             OFF - ";
	regBank.add(40518);                         // Aдрес данных измерения "Test Radio2 ** Signal GG Radio2                             OFF - ";
	regBank.add(40519);                         // Aдрес данных измерения "Test Radio2 ** Signal Radio2                                ON  - ";

	regBank.add(40520);                         // Aдрес данных измерения "Test Microphone ** Signal FrontL                            OFF - ";
	regBank.add(40521);                         // Aдрес данных измерения "Test Microphone ** Signal FrontR                            OFF - ";
	regBank.add(40522);                         // Aдрес данных измерения "Test Microphone ** Signal LineL                             OFF - ";
	regBank.add(40523);                         // Aдрес данных измерения "Test Microphone ** Signal LineR                             OFF - ";
	regBank.add(40524);                         // Aдрес данных измерения "Test Microphone ** Signal mag radio                         OFF - ";
	regBank.add(40525);                         // Aдрес данных измерения "Test Microphone ** Signal mag phone                         OFF - ";
	regBank.add(40526);                         // Aдрес данных измерения "Test Microphone ** Signal GGS                               OFF - ";
	regBank.add(40527);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio1                         OFF - ";
	regBank.add(40528);                         // Aдрес данных измерения "Test Microphone ** Signal GG Radio2                         OFF - ";
	regBank.add(40529);                         // Код регулировки яркости дисплея
	regBank.add(40530);                         // Aдрес данных измерения "Test Radio1 ** Signal mag radio                             ON  - ";
	regBank.add(40531);                         // Aдрес данных измерения "Test Radio2 ** Signal mag radio                             ON  - ";                    // 
	regBank.add(40532);                         // Aдрес данных измерения "Test GGS ** Signal mag radio   
	*/
	slave._device = &regBank;  
}

void test_serial2()
{
  // Поиск ком порта
	Serial.println("COM port find...");
	wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
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
			wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
		}
	clear_serial2();
	
//	wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(200);
	Serial.println("COM port find End");
}
void set_serial2()
{
  // Поиск ком порта
	Serial.println("COM port find...");
	wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
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
			wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
		}
	clear_serial2();
	
//	wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
	regBank.set(adr_control_command,0);                                             // Завершить программу    
	delay(200);
	Serial.println("COM port find End");
}
void set_serial3()
{
   clear_serial3();
   delay(200);
// Поиск ком порта
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
//	wdt_enable (WDTO_8S); // Для тестов не рекомендуется устанавливать значение менее 8 сек.
	//digitalWrite(ledPin13,LOW);
	//mcp_Analog.digitalWrite(Front_led_Red, LOW); 
}
void clear_serial()
{
  if (Serial.available())                             // есть что-то проверить? Есть данные в буфере?
		  {

			while (Serial.available())
				{
					 Serial.read();
				}
		   }
}
void clear_serial2()
{
  if (Serial2.available())                             // есть что-то проверить? Есть данные в буфере?
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
  if (Serial1.available())                             // есть что-то проверить? Есть данные в буфере?
		  {

			while (Serial1.available())
				{
					 Serial1.read();
				}
		   }
}
void clear_serial3()
{
  if (Serial3.available())                             // есть что-то проверить? Есть данные в буфере?
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
	wdt_disable();                                                 // бесполезная строка до которой не доходит выполнение при bootloop
	Wire.begin();
	if (!RTC.begin())                                              // Настройка часов 
		{
			Serial.println("RTC failed");
			while(1);
		};
	setup_mcp();                                                   // Настроить порты расширения  MCP23017
	mcp_Analog.digitalWrite(DTR, HIGH);                            // Разрешение вывода (обмена)информации с модулем Аудио-1
	mcp_Analog.digitalWrite(Front_led_Blue, LOW); 
	mcp_Analog.digitalWrite(Front_led_Red, HIGH); 
	Serial.begin(9600);                                            // Подключение к USB ПК
	Serial1.begin(115200);                                         // Подключение к звуковому модулю Камертон
	slave.setSerial(3,57600);                                      // Подключение к протоколу MODBUS компьютера Serial3 
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
	pinMode(kn1Nano, OUTPUT);                                     // Назначение кнопок управления Nano генератор качения
	pinMode(kn2Nano, OUTPUT);                                     // Назначение кнопок управления Nano генератор 1000 гц
	pinMode(kn3Nano, OUTPUT);                                     // Назначение кнопок управления Nano генератор 2000 гц
	pinMode(InNano12, INPUT);                                     // Назначение входов - индикация генератор 1000 или 2000 гц
	pinMode(InNano13, INPUT);                                     // Назначение входов - индикация генератор качения 
	digitalWrite(InNano12, HIGH);                                 // включение подтягивающего резистора

	// DateTime set_time = DateTime(15, 6, 15, 10, 51, 0);        // Занести данные о времени в строку "set_time"
	// RTC.adjust(set_time);                                      // Записать дату и время
	serial_print_date();
	Serial.println(" ");

	setup_resistor();                                             // Начальные установки резистора

	setup_regModbus();                                            // Настройка регистров MODBUS

	regs_out[0]= 0x2B;                                            // Код первого байта подключения к Камертону 43
	regs_out[1]= 0xC4;                                            // 196 Изменять в реальной схеме
	regs_out[2]= 0x7F;                                            // 127 Изменять в реальной схеме

	regBank.set(21,0);                                            // XP2-2     sensor "Маг."  
	regBank.set(22,0);                                            // XP5-3     sensor "ГГC."
	regBank.set(23,0);                                            // XP3-3     sensor "ГГ-Радио1."
	regBank.set(24,0);                                            // XP4-3     sensor "ГГ-Радио2."
	//regBank.set(8,1);                                           // Включить питание Камертон
	regBank.set(40,0);                                            // Осциллограф отключить
	UpdateRegs();                                                 // Обновить информацию в регистрах

	Serial.println("Initializing SD card...");
	pinMode(49, OUTPUT);                                          // Настройка выбора SD
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

	SdFile::dateTimeCallback(dateTime);                          // Настройка времени записи файла
	regBank.set(40120,0);                                        // Сбросить все команды на выполнение
	regBank.set(adr_reg_count_err,0);                            // Обнулить данные счетчика всех ошибок
	MsTimer2::set(30, flash_time);                               // 30ms период таймера прерывани
	resistor(1, 200);                                            // Установить уровень сигнала
	resistor(2, 200);                                            // Установить уровень сигнала
	preob_num_str();                                             // Записать начальное имя файла 
	//list_file();                                               // Вывод списка файлов в СОМ порт  
	//controlFileName();
	//default_mem_porog();
	prer_Kmerton_On = true;                                      // Разрешить прерывания на камертон
	clear_serial2();                                             // 

	Serial.println("Clear memory and registry");                 //
	for (int i = 120; i <= 131; i++)                             // Очистить флаги ошибок
		{
		   regBank.set(i,0);   
		}
	Reg_count_clear();                                           // Очистить счетчики ошибок
	Serial.println("Clear Ok!");                                 //
	// ------------  Настройка быстродействия АЦП --------------------------
	sbi(ADCSRA,ADPS2) ;                                          // Регистр ADCSRA бит 2 = 1
	cbi(ADCSRA,ADPS1) ;                                          // Регистр ADCSRA бит 1 = 0
	cbi(ADCSRA,ADPS0) ;                                          // Регистр ADCSRA бит 0 = 0

	delay(50);
	set_USB0();
	mcp_Analog.digitalWrite(Front_led_Red, LOW); 
	mcp_Analog.digitalWrite(Front_led_Blue, HIGH); 
	MsTimer2::start();                                           // Включить таймер прерывания
	regBank.set(adr_control_command,0);  
	Serial.println(" ");                                         //
	Serial.println("System initialization OK!.");                // Информация о завершении настройки
	regBank.set(39,1);                                           // Загрузка системы завершена
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

	//Serial.print(	regBank.get(136),HEX);    // XP1- 16 HeS2Rs    sensor подключения гарнитуры инструктора с 2 наушниками
	//Serial.print("--");
	//Serial.println(	regBank.get(137),HEX);    // XP1- 13 HeS2Ls    sensor подключения гарнитуры инструктора 



}
