#include <Servo.h>
Servo servo1;
//int motorPin = 9;
void setup() {

  servo1.attach(6); //Servo en el pin 9
  servo1.write(100);
  //SERVO BASE DEBE INICIAR EN 100
  //SERVO APUNTAR DEBE INICIAR EN 0
  
  //***********************CONFIGURACIONES ADC*********************************
  ADMUX = 3;              //canal 0 = PC0 = A0
  // Configurar el pin como entrada
  DDRC &= ~(1 << PC3);
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
  DDRD &= (1 << DDD2);
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

  //?no se si debo hacer esto
  /**
  PCIFR |= (1 << PCIF3); // Limpiar bandera de interrupción en el puerto D, ponemos en 1 para limpiar
  PCICR |= (1 << PCIE3); //habilitar el grupo de interrupciones del puerto D

  PCIFR |= (1 << PCIF4); // Limpiar bandera de interrupción en el puerto D, ponemos en 1 para limpiar
  PCICR |= (1 << PCIE4); //habilitar el grupo de interrupciones del puerto D**/ 
  //al parecer no
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
  //pinMode(motorPin, OUTPUT);


}
int pos1 = 0;
void loop() {

}

int count0 = 0;
ISR(TIMER0_COMPA_vect){ //1000HZ
   //interrupt commands for TIMER 0 here
   count0++;
   if (count0 >= 200){
      //Serial.println("0.2 seg en TMR0");
      ADCSRA |= (1 << ADSC);        // Iniciar la conversión
      while (ADCSRA & (1 << ADSC)); // Esperar a que la conversión termine
      uint16_t adcValue = ADC;      // obtener los 10 bits de la conversión ADCH:ADCL
      //Serial.println(adcValue);
      count0 = 0;
      if(adcValue > 700){
        pos1 = pos1 +10;
        servo1.write(pos1);
        //Serial.println("***************************");
      }
      else if (adcValue < 400){
        //Serial.println("------------------------------");
        pos1 = pos1 -10;
        servo1.write(pos1);
      }

  delay(200);
   }
}

ISR(PCINT2_vect) {
  cli(); 
  // Verificar si el pin 2 del puerto D cambió a HIGH
   	if ( PIND & (1 << PIND2) ) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD2 HIGH");
      //digitalWrite(motorPin, HIGH);
    } else {
      //Código cuando PORTD2  pasa a LOW
      Serial.println ("PD2 LOW");
      //digitalWrite(motorPin, LOW);
    }
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, se apaga automatico en la interrupcion
  sei(); // Habilitar interrupciones globales
}

ISR(PCINT3_vect) {
  cli(); 
  // Verificar si el pin 3 del puerto D cambió a HIGH
   	if ( PIND & (1 << PIND3) ) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD3 PRESIONADO");
      //digitalWrite(motorPin, HIGH);
    } else {
      //Código cuando PORTD2  pasa a LOW
      //Serial.println ("PD2 LOW");
      //digitalWrite(motorPin, LOW);
    }
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, se apaga automatico en la interrupcion
  sei(); // Habilitar interrupciones globales
}

ISR(PCINT4_vect) {
  cli(); 
  // Verificar si el pin 4 del puerto D cambió a HIGH
   	if ( PIND & (1 << PIND4) ) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD4 PRESIONADO");
      //digitalWrite(motorPin, HIGH);
    } else {
      //Código cuando PORTD2  pasa a LOW
      //Serial.println ("PD2 LOW");
      //digitalWrite(motorPin, LOW);
    }
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, se apaga automatico en la interrupcion
  sei(); // Habilitar interrupciones globales
}
