/**
#define F_CPU 8000000UL // Definir la frecuencia del MCU
#include <avr/io.h>
#include <util/delay.h>
#define BAUDRATE 9600
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)
**/

#include <LiquidCrystal.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
int motorPin = 11;
int motorPin2 =10;

LiquidCrystal lcd(7,8,A0,A1,A2,A3); //Conexion de LCD a arduino

void setup() {

  servo1.attach(6); //Servo en el pin 6
  servo2.attach(5);
  servo1.write(50);
  
  //***********************CONFIGURACIONES ADC*********************************
  ADMUX = 4;              //canal 0 = PC0 = A0
  // Configurar el pin como entrada
  DDRC &= ~(1 << PC4);
  // Configurar el pin como entrada
  DDRC &= ~(1 << PC5);
 // DDRC &= ~(1 << PC1);
  //Vref interno = 01
  ADMUX &= ~(1<<REFS1);
  ADMUX |= (1<<REFS0);

  ADMUX &= ~(1<<ADLAR);     //ajuste a la derecha de la salida de 10 bits

  delay(200);  //esperar a que se estabilice el voltaje

  // Configurar el prescaler a 128, debe ser siempre menor a 200KHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 

  ADCSRA |= (1 << ADEN);    // Habilitar el ADC
  //DDRC = 0;
  //************************************************************************************

  //**********************CONFIGURACIONES BOTON***********************
   // Configurar PD2 como entrada con pull-down
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);
// Configurar PD3 como entrada con pull-down
  DDRD &= (1 << DDD3);
  PORTD |= (1 << PORTD3);
// Configurar PD4 como entrada con pull-down
  DDRD &= (1 << DDD4);
  PORTD |= (1 << PORTD4);  

  //configuración de la interrupción
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, ponemos en 1 para limpiar
  PCICR |= (1 << PCIE2); //habilitar el grupo de interrupciones del puerto D
  PCMSK2 |= (1 << PCINT18);  // Seleccionar el pin a monitorear en el registro PCMSK2, Bit 18 corresponde a PD2
  PCMSK2 |= (1 << PCINT19);  // Seleccionar el pin a monitorear en el registro PCMSK2, Bit 19 corresponde a PD3
  PCMSK2 |= (1 << PCINT20);  // Seleccionar el pin a monitorear en el registro PCMSK2, Bit 20 corresponde a PD4

  //tiamo
//******************************************************************************

//********************CONFIGURACIONES INTERRUPCIONES servo****************************
//******************TEMP CTC***************
   // Configurar el temporizador en modo CTC
  TCCR0A = 0;//(1 << WGM01) | (0 << WGM00);
  // Configurar el divisor de reloj en 64
  TCCR0B = (0 << WGM02); // modo CTC
  TCCR0B |= (1 << CS02)| (0 << CS01)| (1 << CS00); //prescaler 1024
  TCNT0  = 0; // initialize counter value to 0
  // Configurar el valor del límite superior en 255
  OCR0A = 155;
  // Habilitar la interrupción de comparación
  TIMSK0 = (1 << OCIE0A);
  //*****************************************
  
  // TIMER 0 for interrupt frequency 1000 Hz:
  cli(); // stop interrupts
  TCCR0A = 0; // set entire TCCR0A register to 0
  TCCR0B = 0; // same for TCCR0B
  TCNT0  = 0; // initialize counter value to 0
  // set compare match register for 1000 Hz increments
  OCR0A = 249; // = 16000000 / (64 * 1000) - 1 (must be <256)
  // turn on CTC mode
  TCCR0B |= (1 << WGM01);
  // Set CS02, CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei(); // allow interrupts
//*********************************************************************************
  Serial.begin(9600);
  Serial.println("setup realizado");
  pinMode(motorPin, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  lcd.begin(16,2); //LCD de 16x2
  lcd.println("Bienvenido");

//*********************************************************************************
//USART
  //USART_init();
  //DDRB =0b00010000;
  
}
int pos1 = 0;
int pos2 = 0;
int bandera = 0;

void loop() {

/**
char x= USART_receive();
if (x == 0)
  PORTB |= (1<<PB5);
else  
  PORTB &= ~(1<<PB5);
  **/
  }
  
/**
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
**/

int count0 = 0;
ISR(TIMER0_COMPA_vect){ //1000HZ
   //interrupt commands for TIMER 0 here
   count0++;
   if (count0 >= 200){
      //Serial.println("0.2 seg en TMR0");
      ADCSRA |= (1 << ADSC);        // Iniciar la conversión
      while (ADCSRA & (1 << ADSC)); // Esperar a que la conversión termine
      uint16_t adcValue = ADC;      // obtener los 10 bits de la conversión ADCH:ADCL
      Serial.println(adcValue);
      count0 = 0;
      if (bandera == 1){
        Serial.println("Entra en admux 0");
          if(adcValue > 700){
          pos1 = pos1 +10;
          servo1.write(pos1);
          Serial.println("***********************");
        }
        else if (adcValue < 400){
          pos1 = pos1 -10;
          servo1.write(pos1);
          Serial.println("-----------------------");
        }
        lcd.setCursor(1,1);
        lcd.println("Girando...");
        }
      if (bandera == 0){
        Serial.println("Entra en admux 3");
        if(adcValue > 700){
          pos2 = pos2 +10;
          servo2.write(pos2);
        }
        else if (adcValue < 400){
          pos2 = pos2 -10;
          servo2.write(pos2);
        }
        lcd.setCursor(1,1);
        lcd.println("Apuntando...");
      }

  delay(200);
   }
}

ISR(PCINT2_vect) {
  cli(); 
  // Verificar si el pin 2 del puerto D cambió a HIGH
   	if ( PIND & (1 << PIND2) ) {       
      //Código cuando PORTD2  pasa a HIGH
      digitalWrite(motorPin, LOW);
      digitalWrite(motorPin2, LOW);
      Serial.println ("PD2 HIGH"); 
      lcd.setCursor(0,0);
      lcd.println("             ");
      lcd.setCursor(1,1);
      lcd.println("             ");     
    } else{
      //Código cuando PORTD2  pasa a LOW
      digitalWrite(motorPin, HIGH);
      digitalWrite(motorPin2, HIGH);
      Serial.println ("PD2 LOW");
      lcd.setCursor(0,0);
      lcd.println("DISPARANDO");
    }

    if ( (PIND & (1 << PIND3)) == 0) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD3 PRESIONADO");
      lcd.setCursor(0,0);
      lcd.println("Apunta");
      lcd.setCursor(1,1);
      lcd.println("         ");
      ADMUX =5;
      //Vref interno = 01
      ADMUX &= ~(1<<REFS1);
      ADMUX |= (1<<REFS0);

      ADMUX &= ~(1<<ADLAR);     //ajuste a la derecha de la salida de 10 bits
      //digitalWrite(motorPin, HIGH);
      bandera = 0;
    }
    if ( (PIND & (1 << PIND4)) == 0 ) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD4 PRESIONADO");
      lcd.setCursor(0,0);
      lcd.println("Gira:");
      lcd.setCursor(1,1);
      lcd.println("              ");
      ADMUX = 4;
      //Vref interno = 01
      ADMUX &= ~(1<<REFS1);
      ADMUX |= (1<<REFS0);

      ADMUX &= ~(1<<ADLAR);     //ajuste a la derecha de la salida de 10 bits
      //digitalWrite(motorPin, HIGH);
      bandera = 1;
    }
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, se apaga automatico en la interrupcion
  sei(); // Habilitar interrupciones globales
}

/**
//Escribir en EEPROM
void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
  //Wait for comletion of previous write
  while(EECR &(1<<EEPE));
  //Set up address and data registers
  EEAR = uiAdress;
  EEDR = ucData;
  //Write logical one to EEMPE
  EECR |= (1<<EEMPE);
  //Start eeprom write by setting EEPE
  EECR |= (1<<EEPE);
}

//Leer EEPROM
unsigned char EEPROM_read(unsigned int uiAddress){
  //wait for completion of previous write
  while(EECR & (1<<EEPE));
  //Set the address register
  EEAR = uiAddress;
  //Start eeprom read by writing EERE
  EECR |= (1<<EERE);
  //Return data from Data Register
  return EEDR;  
 }
 **/
