#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits.h>



#define WAITINGFIRSTBYTE 0
#define WAITINGSECONDBYTE 1
#define WAITINGID 2
#define WAITINGLENGTH 3
#define WAITINGINSTRUCTION 4
#define WAITINGPARAMETERS 5
#define WAITINGCHECKSUM 6

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
#define GoalPosition 30
#define MovingSpeed 32
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

const int D[8] =  {13, 12, 11, 10, 9, 8, 7, 6};
volatile unsigned long long scheduleAt = 9223372036854775807LL;
//unsigned long scheduleAt = 70000;
unsigned long long time = 0;
long timerstep = 1;

//Command parameters and state machine
char CommandReceptionState = WAITINGFIRSTBYTE;
char currentCommandId = 0;
char currentCommandLength = 0;
char currentCommandInstruction = 0;
char parametersStillToReceive = 0;
char currentParameters[255];
char currentParameterToFill = 0;


char controlTable[6][50] = {{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0}};

long YoupiPosition[6] = {0,0,0,0,0,0};
char YoupiSens[6] = {1,1,1,1,1,1};
unsigned char command;
unsigned char commandValue;
unsigned char commandId;
bool commandStart = true;
unsigned long baudRate = 100;

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
   parallelOutput(0x47);
   parallelOutput(0x00);

    // initialize timer1
    cli();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 65535;;            // compare match register max
    TCCR1B |= (1 << CS10);    // no prescaler
    TCCR1B |= (1 << WGM12);   // clear on compare match
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
        Serial.write(currentCommandId + 1);
        Serial.write(0x02);
        Serial.write(0x00);
        char checksum = ~(currentCommandId + 1 + 2);
        Serial.write(checksum);
        break;
      }
      case READ_DATA:
      {
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId + 1);
        char checksum = currentCommandId + 1;
        Serial.write(currentParameters[1] + 2);
        checksum += currentParameters[1] + 2;
        Serial.write(0x00);
        for (int i = currentParameters[0]; i<(currentParameters[0] + currentParameters[1]); ++i)
        {
          Serial.write(controlTable[currentCommandId][i]);
          checksum += controlTable[currentCommandId][i];
        }
        checksum = ~checksum;
        Serial.write(checksum);
        break;
      }
      case WRITE_DATA:
      {
        bool willMove = false;
        for (int i = 0; i< currentCommandLength -2 ; ++i)
         {
           int controlId = currentParameters[0] + i;
           if (controlId == 30 || controlId == 31)
           {
             willMove = true;
           }
            controlTable[currentCommandId][controlId] = currentParameters[i + 1 ];
         }
        if (willMove)
        {
          //TODO handle the 2 bytes of the GoalPosition/CurrentPosition
          if ((controlTable[currentCommandId][GoalPosition] > controlTable[currentCommandId][CurrentPosition]) && (YoupiSens[currentCommandId] == -1))
          {
            scheduleAt = time + 70000;
            command = 0;
            commandId = currentCommandId;
            commandValue = 1;
          }
          else if ((controlTable[currentCommandId][GoalPosition] < controlTable[currentCommandId][CurrentPosition]) && (YoupiSens[currentCommandId] == 1))
          {
            scheduleAt = time + 70000;
            command = 0;
            commandId = currentCommandId;
            commandValue = -1;
          }
          else
          {

            command = 1;
            commandId = currentCommandId;
          //  commandValue = 10;
            scheduleAt = time + 700000;
          }
        }
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId + 1);
        Serial.write(0x02);
        Serial.write(0x00);
        char checksum = ~(currentCommandId + 1 + 2);
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

        currentCommandId = inByte - 1; // diminish the Id to have starting from 0
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


ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  static int sign = 1;
  cli();
  time += OCR1A;

  if (abs(scheduleAt - time) < 10)
  {

    if (command == 0)
    {
      if (commandStart == true)
      {
             YoupiSens[commandId] = commandValue;
           // digitalWrite(13,HIGH);
            int i;
            char command = 0x80;
            for (i = 0; i < 6; ++i)
            {
              if (YoupiSens[i] == 1)
              {
                command |= 1 << (i - 1);
              }
            }
            commandStart = false;
            scheduleAt  = time + baudRate;
      }
      else
      {
          //          digitalWrite(13,LOW);
            int i;
            char command = 0x00;
            for (i = 0; i < 6; ++i)
            {
              if (YoupiSens[i] == 1)
              {
                command |= 1 << (i-1);
              }
            }
            parallelOutput(0x3F);
            commandStart = true;

            command = 1;
            commandId = commandId;
            scheduleAt = time + baudRate;

      }
    }
    else if (command == 1)
    {
      if (commandStart == true)
      {
            //digitalWrite(13,HIGH);
            parallelOutput(0x40+commandId +1); //moteur id start at 1
            commandStart = false;
            scheduleAt  = time + baudRate;
      }
      else
      {
            //digitalWrite(13,LOW);
            parallelOutput(0x00+commandId +1); //moteur id start at 1
            commandStart = true;
            YoupiPosition[commandId] += YoupiSens[commandId];
            controlTable[commandId][CurrentPosition] = YoupiPosition[commandId]/10;
            if (controlTable[commandId][CurrentPosition] == controlTable[commandId][GoalPosition])
            {
              scheduleAt  = ULONG_MAX;
            }
            else
            {
              scheduleAt = time + 700000;
            }

      }
    }
  }
  if ((scheduleAt - time) < 65535)
  {
    OCR1A = (scheduleAt - time);
  }
  else
  {
    OCR1A = 65535;
  }
  sei();
}
