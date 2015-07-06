#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits.h>

const int D[8] =  {13, 12, 11, 10, 9, 8, 7, 6};
unsigned long scheduleIn = ULONG_MAX;
long timerstep = 1;

#define WAITINGFIRSTBYTE 0
#define WAITINGSECONDBYTE 1
#define WAITINGID 2
#define WAITINGLENGTH 3
#define WAITINGINSTRUCTION 4
#define WAITINGPARAMETERS 5
#define WAITINGCHECKSUM 6
char CommandReceptionState = WAITINGFIRSTBYTE;
char currentCommandId = 0;
char currentCommandLength = 0;
char currentCommandInstruction = 0;
char parametersStillToReceive = 0;
char currentParameters[255];
char currentParameterToFill = 0;

#define PING 1
#define READ_DATA 2
#define WRITE_DATA 3
#define REG_WRITE 4
#define ACTION 5
#define RESET 6
#define SYNC_WRITE 0x83



#define ModelNumber 0
#define FirmwareVersion 2
#define Id 3
#define BaudRate 4
#define ReturnDelay 5
#define CWAngleLimit 6
#define CCWAngleLimit 8
#define TemperatureLimit 11
#define LowVoltageLimit 12
#define HighVoltageLimit 13
#define MaxTorque 14
#define StatusReturnLevel 16
#define AlarmLED 17
#define AlarmShutdown 18
#define DownCalibration 20
#define UpCalibration 22
#define TorqueEnable 24
#define LED 25
#define CWComplianceMargin 26
#define CCWComplianceMargin 27
#define CWComplianceSlope 28
#define CCWComplianceSlope 29
#define GoalPosition30
#define MovingSpeed32
#define TorqueLimit 34
#define CurrentPosition 36
#define CurrentSpeed 38
#define CurrentLoad 40
#define CurrentVoltage 42
#define CurrentTemperature 43
#define RegisteredInstruction 44
#define Moving 46
#define Lock 47
#define Punch 48

char controlTable[50] = {12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0};
void parallelOutput(int number)
{
    for( int i= 0;i<8; ++i)
    {
        digitalWrite(D[i], number%2);
        number = number >> 1;
    }
}

void setup() {
  // set the digital pin as output:
    for (int i = 0; i< 8; ++i)
    {
        pinMode(D[i], OUTPUT);
    }
//    parallelOutput(0x47);
//    parallelOutput(0x00);

    // initialize timer1
    cli();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 0;            // compare match register max
    TCCR1B |= (1 << CS10);    // no prescaler
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    sei();
    Serial.begin(9600);
}

void processCommand()
{
  if (currentCommandId < 10)
  {
    switch(currentCommandInstruction)
    {
      case PING:
      {
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId);
        Serial.write(0x02);
        Serial.write(0x00);
        char checksum = ~(currentCommandId + 2);
        Serial.write(checksum);
        Serial.flush();
        break;
      }
      case READ_DATA:
      {
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId);
        char checksum = currentCommandId;
        Serial.write(currentParameters[1] + 2);
        checksum += currentParameters[1] + 2;
        Serial.write(0x00);
        for (int i = currentParameters[0]; i<(currentParameters[0] + currentParameters[1]); ++i)
        {
          Serial.write(controlTable[i]);
          checksum += controlTable[i];
        }
        checksum = ~checksum;
        Serial.write(checksum);
        break;
      }
    }
  }
  //TODO handle broadcast Id
 
}
void loop()
{
  int inByte;
    // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    
    switch (CommandReceptionState)
    {
      case WAITINGFIRSTBYTE:
      case WAITINGSECONDBYTE:
      {
        if (inByte == 0xFF)
        {
          CommandReceptionState += 1;
        }
        else
        {
          CommandReceptionState = 0;
        }
        break; 
      }
      case WAITINGID:
      {
        
        currentCommandId = inByte;
        CommandReceptionState += 1;
        break;
      }
      case WAITINGLENGTH:
      {
        currentCommandLength = inByte;
        parametersStillToReceive = currentCommandLength -2;
        CommandReceptionState += 1;
        break;
      }
      case WAITINGINSTRUCTION:
      {
        
        currentCommandInstruction = inByte;
        CommandReceptionState += 1;
        if (parametersStillToReceive == 0)
        {
          CommandReceptionState += 1;
        }
        break;
      }      
      case WAITINGPARAMETERS:
      {
        currentParameters[currentParameterToFill] = inByte;
        currentParameterToFill += 1;
        parametersStillToReceive -=1;
        if ( parametersStillToReceive == 0)
        {
          CommandReceptionState += 1;
        }
        break;
      }   
      case WAITINGCHECKSUM:
      {
        
        processCommand();
        CommandReceptionState = 0;
        currentParameterToFill = 0;
        break;
      }       
    }
  }
//    int steps = 10;
//    //Orders earch motor to rotate
//    for(int j=0; j<6; j++) {
//        //...into one direction
//        //1-0-sens moteur6-sens moteur5-sens moteur4-sens moteur3-sens moteur2-sens moteur1
//        parallelOutput(0x80);
//        //0-0-sens moteur6-sens moteur5-sens moteur4-sens moteur3-sens moteur2-sens moteur1
//        parallelOutput(0x00);
//        for(int i=0; i<steps; i++) {
//            //0-1-X-X-X-numero du moteur en binaire
//            parallelOutput(0x40+j);
//            //0-0-X-X-X-numero du moteur en binaire
//            parallelOutput(0x00+j);
//        }
//
//        //...into the opposite direction
//        parallelOutput(0xBF);
//        parallelOutput(0x3F);
//        for(int i=0; i<steps; i++) {
//            parallelOutput(0x40+j);
//            parallelOutput(0x00+j);
//        }
//    }
}

ISR(TIMER1_OVF_VECT)
{

}
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  static int sign = 1;
  cli();
  if (scheduleIn == ULONG_MAX)
    return;
  if(scheduleIn > 65536)
  {
    scheduleIn -= 65536;
    if (scheduleIn > 65536)
    {
      OCR1A = TCNT1-1;
    }
    else
    {
      OCR1A  = (TCNT1 + scheduleIn)%65536;
    }    
  }
  else
  {

    if (scheduleIn > 65536)
    {
      OCR1A = TCNT1-1;
    }
    else
    {
      OCR1A  = (TCNT1 + scheduleIn)%65536;
    }
    timerstep= timerstep+10000*sign;
    if (timerstep > 4000000)
    {
      sign = -1;
    }
    if (timerstep < 2000)
    {
      sign = 1;
    }
    if (digitalRead(13) == LOW)
    {
      digitalWrite(13,HIGH);
      scheduleIn = 3000000;
    }
    else 
    {
      digitalWrite(13,LOW);
 //     scheduleIn = ULONG_MAX;
    }

  }

  sei();
}
