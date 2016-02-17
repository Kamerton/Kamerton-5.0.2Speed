/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino Mega w/ ATmega2560 (Mega 2560), Platform=avr, Package=arduino
*/

#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
#define __AVR_ATmega2560__
#define F_CPU 16000000L
#define ARDUINO 165
#define ARDUINO_AVR_MEGA2560
#define ARDUINO_ARCH_AVR
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
//#define __ATTR_PURE__
//#define __ATTR_CONST__
#define __inline__
//#define __asm__ 
#define __volatile__
#define GCC_VERSION 40801
#define volatile(va_arg) 
#define _CONST
typedef void *__builtin_va_list;
#define __builtin_va_start
#define __builtin_va_end
//#define __DOXYGEN__
#define __attribute__(x)
#define NOINLINE __attribute__((noinline))
#define prog_void
#define PGM_VOID_P int
#ifndef __builtin_constant_p
#define __builtin_constant_p __attribute__((__const__))
#endif
#ifndef __builtin_strlen
#define __builtin_strlen  __attribute__((__const__))
#endif
#define NEW_H
/*
#ifndef __ATTR_CONST__
#define __ATTR_CONST__ __attribute__((__const__))
#endif

#ifndef __ATTR_MALLOC__
#define __ATTR_MALLOC__ __attribute__((__malloc__))
#endif

#ifndef __ATTR_NORETURN__
#define __ATTR_NORETURN__ __attribute__((__noreturn__))
#endif

#ifndef __ATTR_PURE__
#define __ATTR_PURE__ __attribute__((__pure__))
#endif            
*/
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}



#include <arduino.h>
#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const PROGMEM char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const PROGMEM char *)(string_literal))
#undef cli
#define cli()
#define pgm_read_byte(address_short)
#define pgm_read_word(address_short)
#define pgm_read_word2(address_short)
#define digitalPinToPort(P)
#define digitalPinToBitMask(P) 
#define digitalPinToTimer(P)
#define analogInPinToBit(P)
#define portOutputRegister(P)
#define portInputRegister(P)
#define portModeRegister(P)

void dateTime(uint16_t* date, uint16_t* time);
void serial_print_date();
void set_time();
void flash_time();
void serialEvent3();
int readString(char *buffer, int max_len, int terminator);
void prer_Kamerton();
void sendPacketK ();
void waiting_for_replyK();
void Stop_Kamerton ();
void calculateCRC_Out();
void calculateCRC_In();
void i2c_eeprom_write_byte( int deviceaddress, unsigned int eeaddress, byte data );
byte i2c_eeprom_read_byte( int deviceaddress, unsigned int eeaddress );
void UpdateRegs();
void UpdateRegs_istr();
void Reg_count_clear();
void set_clock();
void data_clock_exchange();
void time_control();
void time_control_get();
void list_file();
void load_list_files();
void file_print_date();
void resistor(int resist, int valresist);
void controlFileName();
void FileOpen();
void FileClose();
void file_name();
void preob_num_str();
void control_command();
void sensor_all_off();
void sensor_all_on();
void set_rezistor1();
void set_rezistor2();
void set_rezistor_All();
void test_headset_instructor();
void test_headset_dispatcher();
void test_MTT();
void test_tangR();
void test_tangN();
void test_mikrophon();
void testGGS();
void test_GG_Radio1();
void test_GG_Radio2();
void test_power();
void test_video();
void set_video();
void measure_mks();
void set_radio_send();
void set_ggs_mute();
void set_radio_send1();
void set_ggs_mute1();
void test_instr_off();
void test_instr_on();
void test_disp_off();
void test_disp_on();
void test_MTT_off();
void test_MTT_on();
void measure_vol_min(int istochnik, unsigned int adr_count, unsigned int adr_count200 , int adr_flagErr, unsigned int porogV);
void measure_vol_max(int istochnik, unsigned int adr_count, unsigned int adr_count200 , int adr_flagErr, unsigned int porogV,unsigned int porogVmax );
void measure_volume(int analog);
void measure_volume_P(int analog);
void measure_power();
void set_sound_oscill();
void set_sound(byte cannel_sound);
void read_reg_error();
void save_reg_error();
int read_reg_eeprom(unsigned int adr);
void save_reg_eeprom(unsigned int adr,unsigned int res);
void clear_reg_eeprom();
void i2c_test();
void i2c_test1();
void i2c_test_int();
void test_RS232();
void set_USB0();
void read_porog_eeprom(int adr_eeprom, int step_mem );
void default_mem_porog();
void set_mem_porog();
void read_mem_porog();
void mem_byte_trans_read();
void mem_byte_trans_save();
void set_mem_regBank(int adr_mem , int step_mem);
void read_mem_regBank(int adr_mem , int step_mem);
void send_file_PC();
void set_analog_pin();
void oscilloscope();
void PrintInt(int i);
void setup_mcp();
void setup_resistor();
void setup_regModbus();
void test_serial2();
void set_serial2();
void set_serial3();
void clear_serial();
void clear_serial2();
void clear_serial1();
void clear_serial3();
void set_SD();
void file_del_SD();
//
//

#include <Kamerton5_0_1ArduinoSpeed2.ino>
#include <AnalogBinLogger.h>
#endif
