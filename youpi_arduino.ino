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

//unsigned long scheduleAt = 70000;
unsigned long long time = 0;
long timerstep = 1;

//Command parameters and state machine
unsigned char CommandReceptionState = WAITINGFIRSTBYTE;
unsigned char currentCommandId = 0;
unsigned char currentCommandLength = 0;
unsigned char currentCommandInstruction = 0;
unsigned char parametersStillToReceive = 0;
unsigned char currentParameters[255];
unsigned char currentParameterToFill = 0;

//to check -> more than 256 in value command
//less than 70000 to start the command

unsigned char controlTable[6][50] = {{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0},
{12,0,0,1,1,250,0,0,255,3,0,85,60,190,255,3,2,4,4,0,0,0,0,0,0,0,0,0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,32,0}};

unsigned short getShortInControlTableForMotor(unsigned char motor, unsigned char parameter)
{
  return (((short)controlTable[motor][parameter]) || ((short)(controlTable[motor][parameter + 1]) << 8));
}

void setShortInControlTableForMotor(unsigned char motor, unsigned char parameter, short Position)
{
  controlTable[motor][parameter] = Position & 0xFF;
  controlTable[motor][parameter + 1] = Position >> 8;
}

unsigned short getPositionOfMotor(unsigned char motor)
{
  return getShortInControlTableForMotor(motor, CurrentPosition);
}

void setPositionOfMotor(unsigned char motor, short Position)
{
  setShortInControlTableForMotor(motor, CurrentPosition, Position);
}

unsigned short getVitesseOfMotor(unsigned char motor)
{
  return getShortInControlTableForMotor(motor, CurrentSpeed);
}

unsigned short getGoalOfMotor(unsigned char motor)
{
  return getShortInControlTableForMotor(motor, GoalPosition);
}

void setGoalOfMotor(unsigned char motor, short Position)
{
  setShortInControlTableForMotor(motor, GoalPosition, Position);
}

// Replicate the state of Youpi arm for the position of the step motor
// and the direction of moving for each motor
long YoupiPosition[6] = {0,0,0,0,0,0};
unsigned char YoupiSens[6] = {1,1,1,1,1,1};

struct Command
{
unsigned long long scheduleAt;
unsigned char order; //which command to execute
unsigned char value;//parameter of the command if needed
unsigned char id;//id of the motor
bool toStart;
};

struct RegisteredCommand
{
    unsigned char id;
    unsigned char instruction;
    unsigned char numberOfParameters;
    unsigned char parameters[255];
};

//To be executed at the next action order
RegisteredCommand registeredCommand[6];
unsigned char numberOfRegisteredCommand = 0;

//Current running command
Command command[6];

unsigned long baudRate = 10000;
long vitesse = 70000;

void parallelOutput(int number)
{
    for( int i= 0;i<8; ++i)
    {
        digitalWrite(D[i], number%2);
        number = number >> 1;
    }
}

void setup() {
  for (int i = 0; i < 6; ++i)
  {
    command[i].scheduleAt = 9223372036854775807LL;
    command[i].toStart = true;
  }
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
    //for one motor or broadcast ID
  if (currentCommandId < 6 || currentCommandId = 0xFE)
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
        unsigned char checksum = ~(currentCommandId + 1 + 2);
        Serial.write(checksum);
        break;
      }
      case READ_DATA:
      {
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId + 1);
        unsigned char checksum = currentCommandId + 1;
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
          if ((getGoalOfMotor(currentCommandId) > getPositionOfMotor(currentCommandId)) && (YoupiSens[currentCommandId] == -1))
          {
            command[currentCommandId].order = 0;
            command[currentCommandId].value = 1;
          }
          else if ((getGoalOfMotor(currentCommandId) < getPositionOfMotor(currentCommandId)) && (YoupiSens[currentCommandId] == 1))
          {
            command[currentCommandId].order = 0;
            command[currentCommandId].value = -1;
          }
          else
          {
            command[currentCommandId].order = 1;
          }
          command[currentCommandId].id = currentCommandId;
          command[currentCommandId].scheduleAt = time + vitesse;
          command[currentCommandId].toStart = true;
        }
        Serial.write(0xFF);
        Serial.write(0xFF);
        Serial.write(currentCommandId + 1);
        Serial.write(0x02);
        Serial.write(0x00);
        unsigned char checksum = ~(currentCommandId + 1 + 2);
        Serial.write(checksum);
        break;
      }
    case REG_WRITE:
      {         //Register the command
          registeredCommand[numberOfRegisteredCommand].id = currentCommandId;
          registeredCommand[numberOfRegisteredCommand].instruction = WRITE_DATA;
          registeredCommand[numberOfRegisteredCommand].numberOfParameters = currentCommandLength - 2;
          for (int i = 0; i< currentCommandLength -2 ; ++i)
          {
              registeredCommand[numberOfRegisteredCommand].parameters[i] = currentParameters[i];
          }
          numberOfRegisteredCommand +=1;
          Serial.write(0xFF);
          Serial.write(0xFF);
          Serial.write(currentCommandId + 1);
          Serial.write(0x02);
          Serial.write(0x00);
          unsigned char checksum = ~(currentCommandId + 1 + 2);
          Serial.write(checksum);
          break;
      }
    case ACTION:
      {
          for (int commandIter = 0; commandIter< numberOfRegisteredCommand; ++commandIter)
          {
              RegisteredCommand* aCommand = &registeredCommand[commandIter];
              bool willMove = false;
              for (int i = 0; i< registeredCommand[commandIter].numberOfParameters ; ++i)
              {
                  int controlId = registeredCommand[commandIter].parameters[0] + i;
                  if (controlId == 30 || controlId == 31)
                  {
                      willMove = true;
                  }
                  controlTable[currentCommandId][controlId] = registeredCommand[commandIter].parameters[i + 1 ];
              }
              if (willMove)
              {
                  int commandId = registeredCommand[commandIter].id;
                  if ((getGoalOfMotor(commandId) > getPositionOfMotor(commandId)) && (YoupiSens[commandId] == -1))
                  {
                      command[commandId].order = 0;
                      command[commandId].value = 1;
                  }
                  else if ((getGoalOfMotor(commandId) < getPositionOfMotor(commandId)) && (YoupiSens[commandId] == 1))
                  {
                      command[commandId].order = 0;
                      command[commandId].value = -1;
                  }
                  else
                  {
                      command[commandId].order = 1;
                  }
                  command[commandId].id = commandId;
                  command[commandId].scheduleAt = time + vitesse;
                  command[commandId].toStart = true;
              }
          }
          numberOfRegisteredCommand = 0;
      }
    }
  }
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
}

void executeCommand(unsigned char commandId)
{

    if (command[commandId].order == 0)
    {
        if (command[commandId].toStart == true)
        {
            YoupiSens[command[commandId].id] = command[commandId].value;
            int i;
            unsigned char output = 0x80;
            for (i = 0; i < 6; ++i)
            {
                if (YoupiSens[i] == 1)
                {
                    output |= 1 << (i);
                }
            }
            parallelOutput(output);
            command[commandId].toStart = false;
            command[commandId].scheduleAt  = time + baudRate;
        }
        else
        {
            int i;
            unsigned char output = 0x00;
            for (i = 0; i < 6; ++i)
            {
                if (YoupiSens[i] == 1)
                {
                    output |= 1 << (i);
                }
            }
            parallelOutput(output);
            command[commandId].toStart = true;

            command[commandId].order = 1;
            command[commandId].scheduleAt = time + getVitesseOfMotor(command[commandId].id);

        }
    }
    else if (command[commandId].order == 1)
    {
        if (command[commandId].toStart == true)
        {
            parallelOutput(0x40+command[commandId].id); //moteur id start at 1
            command[commandId].toStart = false;
            command[commandId].scheduleAt  = time + baudRate;
        }
        else
        {
            parallelOutput(0x00+command[commandId].id); //moteur id start at 1
            command[commandId].toStart = true;
            YoupiPosition[command[commandId].id] += YoupiSens[command[commandId].id];
            setPositionOfMotor(command[commandId].id, YoupiPosition[command[commandId].id]/10);

            if (getPositionOfMotor(command[commandId].id) == getGoalOfMotor(command[commandId].id))
            {
                command[commandId].scheduleAt  = ULONG_MAX;
            }
            else
            {
                command[commandId].scheduleAt = time + getVitesseOfMotor(command[commandId].id);
            }

        }
    }
}


ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  cli();
  time += OCR1A;

  for (int i=0;i < 6; ++i)
  {
      if (abs(command[i].scheduleAt - time) < 10)
      {
          executeCommand(i);
      }
  }

  unsigned int nextInterruptTime = 65535;
  for (int i=0;i < 6; ++i)
  {
      if ((command[i].scheduleAt - time) < nextInterruptTime)
      {
          nextInterruptTime = command[i].scheduleAt - time;
      }
  }

  OCR1A = nextInterruptTime;

  sei();
}
