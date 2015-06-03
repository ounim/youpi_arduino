// debut des mockups
/*
#include "stdio.h"

const int OUTPUT = 1;
void pinMode(int pin, int mode)
{
    printf("pinMode D%d: mode %d\n", pin, mode);
}

void digitalWrite(int pin, int number)
{
    printf("pin D%d : output %d\n", pin ,number);
}

void delay(int milliseconds)
{
    printf("attend %d millisecondes\n", milliseconds);
}

int main()
    {
        setup();
        loop();
    }
}
*/
// fin des mockups
const int D[8] =  {13, 12, 11, 10, 9, 8, 7, 6};
void parallelOutput(int number)
{
    for( int i= 0;i<8; ++i)
    {
        digitalWrite(D[i], number%2);
        number = number >> 1;
    }
    delay(2);
}

void setup() {
  // set the digital pin as output:
    for (int i = 0; i< 8; ++i)
    {
        pinMode(D[i], OUTPUT);
    }
    parallelOutput(0x47);
    parallelOutput(0x00);
}

void loop()
{
    int steps = 10;
    //Orders earch motor to rotate
    for(int j=0; j<6; j++) {
        //...into one direction
        //1-0-sens moteur6-sens moteur5-sens moteur4-sens moteur3-sens moteur2-sens moteur1
        parallelOutput(0x80);
        //0-0-sens moteur6-sens moteur5-sens moteur4-sens moteur3-sens moteur2-sens moteur1
        parallelOutput(0x00);
        for(int i=0; i<steps; i++) {
            //0-1-X-X-X-numero du moteur en binaire
            parallelOutput(0x40+j);
            //0-0-X-X-X-numero du moteur en binaire
            parallelOutput(0x00+j);
        }

        //...into the opposite direction
        parallelOutput(0xBF);
        parallelOutput(0x3F);
        for(int i=0; i<steps; i++) {
            parallelOutput(0x40+j);
            parallelOutput(0x00+j);
        }
    }
}
