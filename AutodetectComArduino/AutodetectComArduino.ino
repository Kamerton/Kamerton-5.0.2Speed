
int steps=0;
int inbyte;
int dela_time;

int serialdata;

//Setup message bytes

byte inputByte_0;

byte inputByte_1;

byte inputByte_2;

byte inputByte_3;

byte inputByte_4;

//byte inputByte_5;

//Setup
int ledPin_3 =13;

void setup() {

  Serial.begin(9600);
  Serial3.begin(115200);

}


//Main Loop

void loop() {

    //Read Buffer
  while (Serial3.available()==5) 
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
					
                  digitalWrite(ledPin_3, HIGH); 
                  break;
                }
                else
                {
                  digitalWrite(ledPin_3, LOW); 
                  break;
                }
              break;
            } 
            break;
          case 128:
            //Say hello
            Serial3.println("HELLO");
            break;
        } 
        //Clear Message bytes
        inputByte_0 = 0;
        inputByte_1 = 0;
        inputByte_2 = 0;
        inputByte_3 = 0;
        inputByte_4 = 0;
        //Let the PC know we are ready for more data
        delay(1000);
        Serial.print("-READY TO RECEIVE");
 
  }
}   
