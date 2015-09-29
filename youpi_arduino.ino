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
const int I[6] =  {14, 19, 18, 17, 16, 15};

//unsigned long scheduleAt = 70000;
unsigned long long time = 0;
long timerstep = 1;

//Command parameters and state machine
unsigned char CommandReceptionState = WAITINGFIRSTBYTE;
unsigned char currentCommandId = 0;
unsigned char currentCommandLength = 0;
unsigned char currentCommandInstruction = 0;
unsigned char parametersStillToReceive = 0;
unsigned char currentParameters[10];
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
  return (((short)controlTable[motor][parameter]) | ((short)(controlTable[motor][parameter + 1]) << 8));
}

void setShortInControlTableForMotor(unsigned char motor, unsigned char parameter, short Position)
{
  controlTable[motor][parameter] = (char)(Position & 0xFF);
  controlTable[motor][parameter + 1] = (char)(Position >> 8);
}

unsigned short getPositionOfMotor(unsigned char motor)
{
  return getShortInControlTableForMotor(motor, CurrentPosition);
}

void setPositionOfMotor(unsigned char motor, long Position)
{
  switch(motor)
  {
    case 1:
    {
      setShortInControlTableForMotor(motor, CurrentPosition, ((Position*1023)/6200));
      break;
    }
    case 2:
    {
      setShortInControlTableForMotor(motor, CurrentPosition, ((Position*1023)/7800));
      break;
    }
    default:
      setShortInControlTableForMotor(motor, CurrentPosition, Position/10);
  }
}

long getVitesseOfMotor(unsigned char motor)
{
  static long vitesse = 7000;
//  return vitesse;
  int control = getShortInControlTableForMotor(motor, MovingSpeed);
  if (control == 0)
    return vitesse;
  else
    return (vitesse*1023)/control;
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
signed char YoupiSens[6] = {0,0,0,0,0,0};

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
    unsigned char parameters[10];
};

//To be executed at the next action order
RegisteredCommand registeredCommand[6];
unsigned char numberOfRegisteredCommand = 0;

//Current running command
Command command[6];

unsigned long baudRate = 10000;


void parallelOutput(int number)
{
    for( int i= 0;i<8; ++i)
    {
        digitalWrite(D[i], number%2);
        number = number >> 1;
    }
}

void calibrate()
{
  {
  int motor = 1;
   if (digitalRead(I[motor])) //we are in the black of the first motor
   {
       parallelOutput(0xBF);
       delay(1);
       parallelOutput(0x3F);
       delay(1);
       while (digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   else  //we are in the white of the first motor
   {
       parallelOutput(0x80);
       delay(1);
       parallelOutput(0x00);
       delay(1);
       while (!digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   setPositionOfMotor(motor,431);
   setGoalOfMotor(motor,431);
   YoupiPosition[motor]=2610;
}
  {
  int motor = 2;
   if (digitalRead(I[motor])) //we are in the black of the first motor
   {
       parallelOutput(0x80);
       delay(1);
       parallelOutput(0x00);
       delay(1);
       while (digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   else  //we are in the white of the first motor
   {
      parallelOutput(0xBF);
       delay(1);
       parallelOutput(0x3F);
       delay(1);
       while (!digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   setPositionOfMotor(motor,578);
   setGoalOfMotor(motor,578);
   YoupiPosition[motor]=4410;
}
  {
  int motor = 3;
   if (digitalRead(I[motor])) //we are in the black of the first motor
   {
      parallelOutput(0xBF);
       delay(1);
       parallelOutput(0x3F);
       delay(1);
       while (digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   else  //we are in the white of the first motor
   {
       parallelOutput(0x80);
       delay(1);
       parallelOutput(0x00);
       delay(1);
       while (!digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   setPositionOfMotor(motor,0x1FF);
   setGoalOfMotor(motor,0x1FF);
   YoupiPosition[motor]=5110;
}
  {
  int motor = 4;
   if (digitalRead(I[motor])) //we are in the black of the first motor
   {
      parallelOutput(0xBF);
       delay(1);
       parallelOutput(0x3F);
       delay(1);
       while (digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   else  //we are in the white of the first motor
   {
       parallelOutput(0x80);
       delay(1);
       parallelOutput(0x00);
       delay(1);
       while (!digitalRead(I[motor]))
       {
           parallelOutput(0x40 + motor);
           delay(1);
           parallelOutput(0x00 + motor);
           delay(1);
       }
   }
   setPositionOfMotor(motor,0x1FF);
   setGoalOfMotor(motor,0x1FF);
   YoupiPosition[motor]=5110;
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
      // set the digital pin as output:
    for (int i = 0; i< 6; ++i)
    {
        pinMode(I[i], INPUT);
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

    calibrate();
}

void processCommand()
{
    //ugly hack -> we diminish the currentCommandId to have the id started at 0 but then the broadcast is not FE but FD…
  if (currentCommandId < 6 || currentCommandId == 0xFD)
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
        for (int i = 0; i< currentCommandLength -3 ; ++i)
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
          if ((getGoalOfMotor(currentCommandId) > getPositionOfMotor(currentCommandId)) && (YoupiSens[currentCommandId] != 1))
          {
            command[currentCommandId].order = 0;
            command[currentCommandId].value = 1;
          }
          else if ((getGoalOfMotor(currentCommandId) < getPositionOfMotor(currentCommandId)) && (YoupiSens[currentCommandId] != -1))
          {
            command[currentCommandId].order = 0;
            command[currentCommandId].value = -1;
          }
          else
          {
            command[currentCommandId].order = 1;
          }
          command[currentCommandId].id = currentCommandId;
          command[currentCommandId].scheduleAt = time + getVitesseOfMotor(command[currentCommandId].id);
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
          for (int i = 0; i< currentCommandLength -1 ; ++i)
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
              for (int i = 0; i< aCommand->numberOfParameters ; ++i)
              {
                  int controlId = aCommand->parameters[0] + i;
                  if (controlId == 30 || controlId == 31)
                  {
                      willMove = true;
                  }
                  controlTable[aCommand->id][controlId] = aCommand->parameters[i + 1 ];
              }
              if (willMove)
              {
                  int commandId = aCommand->id;
                  if ((getGoalOfMotor(commandId) > getPositionOfMotor(commandId)) && (YoupiSens[commandId] != 1))
                  {
                      command[commandId].order = 0;
                      command[commandId].value = 1;
                  }
                  else if ((getGoalOfMotor(commandId) < getPositionOfMotor(commandId)) && (YoupiSens[commandId] != -1))
                  {
                      command[commandId].order = 0;
                      command[commandId].value = -1;
                  }
                  else
                  {
                      command[commandId].order = 1;
                  }
                  command[commandId].id = commandId;
                  command[commandId].scheduleAt = time + getVitesseOfMotor(command[commandId].id);
                  command[commandId].toStart = true;
              }
          }
          numberOfRegisteredCommand = 0;
          break;
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
            setPositionOfMotor(command[commandId].id, YoupiPosition[command[commandId].id]);

            if (getPositionOfMotor(command[commandId].id) == getGoalOfMotor(command[commandId].id))
            {
                command[commandId].scheduleAt  = 9223372036854775807LL;
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
  bool commanddone = false;
  for (int i=0;i < 6; ++i)
  {
      if (command[i].toStart == false)
      {
          executeCommand(i);
          commanddone = true;
          break;
      }
  }
  if (commanddone == false)
  {
     for (int i=0;i < 6; ++i)
     {
         if (command[i].scheduleAt < time)
         {
             executeCommand(i);
             break;
         }

     }
  }

  unsigned long long nexttime = command[0].scheduleAt;
  for (int i=1;i < 6; ++i)
  {
     if ( command[i].scheduleAt< nexttime)
     {
          nexttime = command[i].scheduleAt;
     }
  }
  unsigned int nextInterruptTime = 65535;
  if (nexttime < (time + baudRate))
     nextInterruptTime = baudRate;
  else if (nexttime < (time + nextInterruptTime))
     nextInterruptTime = (nexttime - time);

  OCR1A = nextInterruptTime;

  sei();
}
