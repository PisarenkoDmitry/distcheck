#include <avr/io.h>
#include <util/delay.h>
unsigned long pulse(uint8_t pin, uint8_t state, unsigned long timeout)//ф-я измерения длины сигнала
{

  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t stateMask = (state ? bit : 0);
  unsigned long width = 0; 
  unsigned long numloops = 0;
  unsigned long maxloops = microsecondsToClockCycles(timeout) / 16;
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;


  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;


  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    width++;
  }


  return clockCyclesToMicroseconds(width * 21 + 16); 
}
int main() {

  DDRD = 0b11111111;// устанавливаем весь порт д в режим output
  DDRB = 0b11110111;// устанавливаем весь порт б в режим output, кроме 11 пина. для echo нам нужен режим input. 
  unsigned int time_us=0;

  unsigned int distance_sm=0;
  while(1) {

    PORTB |= 1 << 4;
    _delay_ms(1); 
    PORTB &= ~(1 << 4); 
    time_us=pulse(11, 1, 100000); 
    distance_sm=time_us/58;
    if (distance_sm<5)
    { 
      PORTB |= 1 << 0;
      PORTB |= 1 << 1;
      PORTB |= 1 << 2;
      PORTD=B11111111;

    } 
    else if (distance_sm<10)
    { 
      PORTB &= ~(1 << 2);
      PORTB |= 1 << 0;
      PORTB |= 1 << 1;
      PORTD=B11111111;

    } 
    else if (distance_sm<15)
    { 
      PORTB &= ~(1 << 2);
      PORTB |= 1 << 0;
      PORTB &= ~(1 << 1);
      PORTD=B11111111;
    }
    else if (distance_sm<20)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B11111111;
    }
    else if (distance_sm<25)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B01111111;
    }
    else if (distance_sm<30)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B00111111;
    }
    else if (distance_sm<35)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B00011111;
    }
    else if (distance_sm<40)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B00001111;
    }
    else if (distance_sm<45)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B00000111;
    }
    else if (distance_sm>45)
    { 
      PORTB &= ~(1 << 2);
      PORTB &= ~(1 << 1);
      PORTB &= ~(1 << 0);
      PORTD=B00000011;
    }
    
    _delay_ms(1);
  }
  return 0;  
}
