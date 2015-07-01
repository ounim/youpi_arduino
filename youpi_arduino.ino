#include <avr/io.h>
#include <avr/interrupt.h>
// fin des mockups
const int D[8] =  {13, 12, 11, 10, 9, 8, 7, 6};
long scheduleIn = 0;
long timerstep = 1;
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
 //   TCNT1  = 0;

    OCR1A = 1024;            // compare match register max
   // TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS10);    // no prescaler
 //   TCCR1B |= (1 << CS11);    // no prescaler
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    sei();
}

void loop()
{
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
    scheduleIn = timerstep;
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
    
    digitalWrite(13, digitalRead(13) ?LOW:HIGH);   // toggle LED pin
  }
  sei();
}
