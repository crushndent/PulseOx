#include <MsTimer2.h>

/*
 *  File.......  
 *  Purpose.... Records up to 128 signal changes
 *  Author..... T. Cruttenden
 *  E-mail..... tcrutt@gmail.com
 *  Started.... April 14, 2010
 *
 */ 
#include <avr/interrupt.h>
#include <avr/io.h>

#define TIMER_RESET  TCNT1 = 0

//PD5 is the desired pin??
int InPin = 5;
unsigned int TimerValue[2];
long time;

void setup() {
  Serial.begin(19200);
  Serial.println("PulseOxi");
  TCCR1A = 0x00;          // COM1A1=0, COM1A0=0 => Disconnect Pin OC1 from Timer/Counter 1 -- PWM11=0,PWM10=0 => PWM Operation disabled
  // ICNC1=0 => Capture Noise Canceler disabled -- ICES1=0 => Input Capture Edge Select (not used) -- CTC1=0 => Clear Timer/Counter 1 on Compare/Match
  // CS12=0 CS11=1 CS10=1 => Set prescaler to clock/8
  TCCR1B = 0x02;          // 16MHz clock with prescaler means TCNT1 increments every .5uS
  // ICIE1=0 => Timer/Counter 1, Input Capture Interrupt Enable -- OCIE1A=0 => Output Compare A Match Interrupt Enable -- OCIE1B=0 => Output Compare B Match Interrupt Enable
  // TOIE1=0 => Timer 1 Overflow Interrupt Enable
  TIMSK1 = 0x00;          
  pinMode(InPin, INPUT);
}

void loop()
{
  int index = 0;
  TIMER_RESET;
  while(1)
  {
    while(digitalRead(InPin) == LOW) {}                                 

    //Beginning of HIGH pulse
    TimerValue[index] = TCNT1;

    if (index==1)
    {
      time = (long) (TimerValue[1] - TimerValue[0]);
      index = 0;      
      //This line only to graph output.
      Serial.print("0,");
      Serial.print(time);
      Serial.print("\n");
    }
    else
    {
      TIMER_RESET;
      time = (long) (TimerValue[0] - TimerValue[1]);
      index++;
    }

    //remnant of HIGH pulse
    while(digitalRead(InPin) == HIGH) {}
  }
}

