
#include <avr/io.h>

int main(void)
{
  DDRD |= (1 << PD1); // TXD (pin 1) como salida
  DDRD &= ~(1 << PD0); // RXD (pin 0) como entrada
  // Configurar velocidad de transmisión a 9600 baudios
  UBRR0H = 0;
  UBRR0L = 103;
  // Habilitar USART0 en modo de transmisión
  UCSR0B = (1 << TXEN0);
  // Configurar 8 bits, sin bits de paridad, y 1 bit de parada
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  // Enviar un carácter por la USART0
  while (1)
  {
    UDR0 = 0;
    // Esperar a que se complete la transmisión
    while (!(UCSR0A & (1 << TXC0)));
    _delay_ms(1000);
    UDR0 = 1;
    // Esperar a que se complete la transmisión
    while (!(UCSR0A & (1 << TXC0)));
    _delay_ms(1000);
  }
  return 0;
  

}
