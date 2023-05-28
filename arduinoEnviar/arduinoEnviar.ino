
#include <avr/io.h>
#include <Servo.h>
Servo servo1;
void setup() {

  ADMUX = 0;              //canal 0 = PC0 = A0
  // Configurar el pin como entrada
  DDRC &= ~(1 << PC0);
 // DDRC &= ~(1 << PC1);
  //Vref interno = 01
  ADMUX &= ~(1<<REFS1);
  ADMUX |= (1<<REFS0);

  ADMUX &= ~(1<<ADLAR);     //ajuste a la derecha de la salida de 10 bits



  delay(200);  //esperar a que se estabilice el voltaje

  // Configurar el prescaler a 128, debe ser siempre menor a 200KHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 

  ADCSRA |= (1 << ADEN);    // Habilitar el ADC

  DDRC = 0;
  DDRD |= (1 << PD1); // TXD (pin 1) como salida
  //DDRD &= ~(1 << PD0); // RXD (pin 0) como entrada
  // Configurar velocidad de transmisión a 9600 baudios
  UBRR0H = 0;
  UBRR0L = 103;
  // Habilitar USART0 en modo de transmisión
  UCSR0B = (1 << TXEN0);
  // Configurar 8 bits, sin bits de paridad, y 1 bit de parada
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
  // Enviar un carácter por la USART0

  Serial.begin(9600);
    
}
void loop()
{
  ADCSRA |= (1 << ADSC);        // Iniciar la conversión
  delay(5);
  while (ADCSRA & (1 << ADSC)); // Esperar a que la conversión termine
  uint16_t adcValue = ADC;      // obtener los 10 bits de la conversión ADCH:ADCL
  //Serial.println(adcValue);
  //Serial.println("\n");
  
  //String espacio = " \n";
  //valorADC.concat(espacio);
  
  
    //UDR0 = adcValue;
    char cadena[16];
    sprintf(cadena, "Hola %d", adcValue);
    Serial.println(adcValue);
    // Esperar a que se complete la transmisión
    //while (!(UCSR0A & (1 << TXC0)));
    _delay_ms(1000);
    //UDR0 = 1;
    // Esperar a que se complete la transmisión
    //while (!(UCSR0A & (1 << TXC0)));
    //_delay_ms(1000);
}
