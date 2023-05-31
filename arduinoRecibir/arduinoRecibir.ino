#define F_CPU 8000000UL // Definir la frecuencia del MCU
#include <avr/io.h>
#include <util/delay.h>
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)

void setup(){
  USART_init();
  DDRB =0b00010000;
}

void loop() {
  char x  = USART_receive();
  //char x = Serial.readstring();
  //String x = String(recibido);
  //Serial.println(x);
  if (x == 0)
    PORTB |= (1<<PB5);
  else
    PORTB &= ~(1<<PB5);
}

void USART_init(void) {
  UBRR0H = (uint8_t)(BAUD_PRESCALLER >> 8);
  UBRR0L = (uint8_t)(BAUD_PRESCALLER);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
unsigned char USART_receive(void) {
  // Esperar a recibir datos
  while (!(UCSR0A & (1 << RXC0)));
  return UDR0; // Devolver el dato recibido
}
