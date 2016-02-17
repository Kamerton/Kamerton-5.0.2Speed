
/*
Прграмма управления звуковым генератором 
Микроконтроллер Atmega328 (Arduino Nano)
Применяется в модуле проверки звуковой платы "Камертон"
Версия тестера платы "Камертон 5.0"
*/
#include <Arduino.h>
#include <AH_AD9850.h>
#include <Bounce2.h>


//Настройка звукового генератора
//#define CLK     8      // Назначение выводов генератора сигналов
//#define FQUP    9      // Назначение выводов генератора сигналов
//#define BitData 10     // Назначение выводов генератора сигналов
//#define RESET   11     // Назначение выводов генератора сигналов

#define CLK     5      // Назначение выводов генератора сигналов
#define FQUP    6      // Назначение выводов генератора сигналов
#define BitData 7     // Назначение выводов генератора сигналов
#define RESET   8     // Назначение выводов генератора сигналов


#define led13    13    // 

#define Kn1      9     // 
#define Kn2     10     // 
#define Kn3     11     // 
#define Out1    12     // 


// Instantiate a Bounce object
Bounce debouncer1 = Bounce(); 
Bounce debouncer2 = Bounce(); 
Bounce debouncer3 = Bounce(); 

byte var = 0;
byte _var = 0;
bool sound_run = true;
bool sound_change = false;

AH_AD9850 AD9850(CLK, FQUP, BitData, RESET);// настройка звукового генератора

int frequency_start0 = 500;
int frequency_max0 = 1000;
int frequency_step0 = 20;
int frequency_start1 = 500;
int frequency_max1 = 3500;
int frequency_step1 = 50;
int frequency_start2 = 100;
int frequency_max2 = 5500;
int frequency_step2 = 50;
int frequency_start3 = 100;
int frequency_max3 = 1000;
int frequency_step3 = 20;

void change_func()                    // Проверка изменения режима генерации
{
	debouncer1.update();
	debouncer2.update();
	debouncer3.update();
	delay(20);
		var=0;
		byte value1 = debouncer1.read();
		byte value2 = debouncer2.read();
		byte value3 = debouncer3.read();

		bitWrite(var, 0, !value1);
		bitWrite(var, 1, !value2);
		bitWrite(var, 2, !value3);

	if ( _var != var)
	{
		digitalWrite(Out1, LOW);
		_var = var;
		sound_change = true;
	}
}
void step_sound0()
{

		Serial.println("step_sound 0 500-1000");
	  do{
			digitalWrite(Out1, HIGH);
			digitalWrite(led13, HIGH); 
			delay(5);
			digitalWrite(Out1, LOW);
			change_func();              // Проверка изменения режима генерации
			if(var!= 0)
				{
				  sound_run    = false;
				}
			for (int i=frequency_start0;i<= frequency_max0; i=i+frequency_step0)
				{
						AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
						delay(5); 
				}
			delay(5); 
			digitalWrite(led13, LOW); 
			for (int i=frequency_max0;i >  frequency_start0; i=i-frequency_step0)
				{
			
						AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
						delay(5); 
				}
		}while (sound_run == true);
	  sound_run = true;

}
void step_sound1()
{
		Serial.println("step_sound1 500-3500");
	  do{
			digitalWrite(Out1, HIGH);
			digitalWrite(led13, HIGH); 
			delay(5);
			digitalWrite(Out1, LOW);
			change_func();                  // Проверка изменения режима генерации
			if(var!= 1)
				{
				  sound_run    = false;
				}
			for (int i=frequency_start1;i<= frequency_max1; i=i+frequency_step1)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
			delay(5); 
			digitalWrite(led13, LOW); 
			for (int i=frequency_max1;i >  frequency_start1; i=i-frequency_step1)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
		}while (sound_run == true);
	  sound_run = true;
}
void step_sound2()
{
	Serial.println("step_sound2 100-5500");
  do{
			digitalWrite(Out1, HIGH);
			digitalWrite(led13, HIGH); 
			delay(5);
			digitalWrite(Out1, LOW);
			change_func();                     // Проверка изменения режима генерации
				if(var!= 2)
				{
				  sound_run    = false;
				}
			for (int i=frequency_start2;i<= frequency_max2; i=i+frequency_step2)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
			delay(5); 
			digitalWrite(led13, LOW); 
			for (int i=frequency_max2;i >  frequency_start2; i=i-frequency_step2)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
   }while (sound_run == true);
   sound_run = true;
}
void step_sound3()
{
		Serial.println("step_sound3 100 - 1000");
	  do{
			digitalWrite(Out1, HIGH);
			digitalWrite(led13, HIGH); 
			delay(5);
			digitalWrite(Out1, LOW);
			change_func();                  // Проверка изменения режима генерации
				if(var!= 3)
				{
					sound_run    = false;
				}
			for (int i=frequency_start3;i<= frequency_max3; i=i+frequency_step3)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
			delay(5); 
			digitalWrite(led13, LOW); 
			for (int i=frequency_max3;i >  frequency_start3; i=i-frequency_step3)
				{
					AD9850.set_frequency(0,0,i);    //set power=UP, phase=0, i= frequency
					delay(5); 
				}
	   }while (sound_run == true);
   sound_run = true;
}

void fix_sound1()
{
	digitalWrite(led13, HIGH); 
	digitalWrite(Out1, HIGH);
	delay(5);
	digitalWrite(Out1, LOW);
	Serial.println("fix_sound1 500");
	AD9850.set_frequency(0,0,500);                   //set power=UP, phase=0, 1kHz frequency
	delay(100); 
}
void fix_sound2()
{
	digitalWrite(led13, HIGH); 
	digitalWrite(Out1, HIGH);
	delay(5);
	digitalWrite(Out1, LOW);
	Serial.println("fix_sound2 1000");
	AD9850.set_frequency(0,0,1000);                   //set power=UP, phase=0, 1kHz frequency
	delay(100); 
}
void fix_sound3()
{
	digitalWrite(led13, HIGH); 
	digitalWrite(Out1, HIGH);
	delay(5);
	digitalWrite(Out1, LOW);
	Serial.println("fix_sound3 2000");
	AD9850.set_frequency(0,0,2000);                   //set power=UP, phase=0, 1kHz frequency
	delay(100); 
}
void fix_sound4()
{
	digitalWrite(led13, HIGH); 
	digitalWrite(Out1, HIGH);
	delay(5);
	digitalWrite(Out1, LOW);
	Serial.println("fix_sound4 3500");
	AD9850.set_frequency(0,0,3500);                   //set power=UP, phase=0, 1kHz frequency
	delay(100); 
}

void menu()
{
  change_func();                       // Проверка изменения режима генерации
  if (sound_change)           
	{
		Serial.println(var);
		switch (var) 
		{

		case 0:
			sound_change = false;
			step_sound0();
			break;
		case 1:
			sound_change = false;
			step_sound1();
			break;
		case 2:
			sound_change = false;
			step_sound2();
			break;
		case 3:
			sound_change = false;
			step_sound3();
			break;
		case 4:
			sound_change = false;
			fix_sound1();
			break;
		case 5:
			sound_change = false;
			fix_sound2();
			break;
		case 6:
			sound_change = false;
			fix_sound3();
			break;
		case 7:
			sound_change = false;
			fix_sound4();
			break;
		default: 
			sound_change = false;
			step_sound0();
		}
		
	}
}



void setup()
{

  AD9850.reset();                   //reset module
  delay(1000);
  AD9850.powerDown();               //set signal output to LOW
  pinMode(led13, OUTPUT); 
  pinMode(Kn1, INPUT_PULLUP);
  pinMode(Kn2, INPUT_PULLUP);
  pinMode(Kn3, INPUT_PULLUP);
  pinMode(Out1,OUTPUT);

  debouncer1.attach(Kn1);
  debouncer1.interval(5);             // interval in ms
  debouncer2.attach(Kn2);
  debouncer2.interval(5);             // interval in ms
  debouncer3.attach(Kn3);
  debouncer3.interval(5);             // interval in ms

  pinMode(led13,OUTPUT);
  digitalWrite(Out1, HIGH);
  AD9850.set_frequency(0,0,1000);    //set power=UP, phase=0, 2kHz frequency 
  Serial.begin(9600);
  Serial.println("Start sound");
}

void loop()
{
	//step_sound();
	menu();
	delay(10);

}
