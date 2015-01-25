#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU   16000000
#define BUAD    9600
#define BRC     ((F_CPU/16/BUAD) - 1)
#define TX_BUFFER_SIZE  128

char serialBuffer[TX_BUFFER_SIZE];
uint8_t serialReadPos = 0;
uint8_t serialWritePos = 0;

void appendSerial(char c);
void serialWrite(char  c[]);

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
int main(void)
{
	 DDRD = 0b11111111;// устанавливаем весь порт д в режим output
	 DDRB = 0b11110111;// устанавливаем весь порт б в режим output, кроме 11 пина. для echo нам нужен режим input.
	 unsigned int time_us=0;
	 unsigned int distance_sm=0;
	UBRR0H = (BRC >> 8);
	UBRR0L =  BRC;
	UCSR0B = (1 << TXEN0)  | (1 << TXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	
	sei();
	
	
	
	while(1)
	{
		
		
		PORTB |= 1 << 4;
		_delay_ms(1);
		PORTB &= ~(1 << 4);
		time_us=pulse(11, 1, 100000);
		distance_sm=time_us/58;
		char len[5];
		sprintf(len,"%d",distance_sm);
		serialWrite(len);
		serialWrite("\n\r");
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
		
		_delay_ms(100);
		
		
	}
	return 0;

	
}

void appendSerial(char c)
{
	serialBuffer[serialWritePos] = c;
	serialWritePos++;
	
	if(serialWritePos >= TX_BUFFER_SIZE)
	{
		serialWritePos = 0;
	}
}

void serialWrite(char c[])
{
	for(uint8_t i = 0; i < strlen(c); i++)
	{
		appendSerial(c[i]);
	}
	
	if(UCSR0A & (1 << UDRE0))
	{
		UDR0 = 0;
	}
}

ISR(USART_TX_vect)
{
	if(serialReadPos != serialWritePos)
	{
		UDR0 = serialBuffer[serialReadPos];
		serialReadPos++;
		
		if(serialReadPos >= TX_BUFFER_SIZE)
		{
			serialReadPos++;
		}
	}
}
