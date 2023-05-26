void setup() {
  // put your setup code here, to run once:
  // Configurar el pin 9 como salida
  DDRB |= (1 << PB1);

  //Configurar el pin 10 como salida
  DDRB |= (1 << PB2);

  // Configurar el modo de operación del Timer1 en modo PWM
  //WGM13 12 11 10 = 1110 = fast PWM
  //CS12 11 10 = 011 = prescaler 64
  //COM1A1 0 = 10 = Clear OC1A on compare, Set OC1A at bottom
  TCCR1A |= (1 << COM1A1)  | (1 << COM1B1) | (1 << WGM11) | (1 << WGM10); //OC1A es el pin PB1
  TCCR1B |= (1 << WGM13)  | (1 << WGM12) | (1 << CS10) | (1 << CS11);
 
  // Configurar el periodo del PWM
  ICR1 = 5000; 
  // Frecuencia de reloj / (frecuencia del PWM * prescaler) - 1

  // Configurar el ancho de pulso del PWM
  OCR1A = 300; // ICR1 * ciclo de trabajo / 100 
  OCR1B = 400;
//( 125 - 600, 0.5ms - 2.4ms)
  //Compara el OCR1A (ancho de pulso) con el ICR1 (periodo)

  ADMUX = 0;              //canal 0 = PC0 = A0
  // Configurar el pin como entrada
  DDRC &= ~(1 << PC0);
 // DDRC &= ~(1 << PC1);
  //Vref interno = 01
  ADMUX &= ~(1<<REFS1);
  //ADMUX |= (1<<REFS0);

  ADMUX &= ~(1<<ADLAR);     //ajuste a la derecha de la salida de 10 bits

  delay(20);  //esperar a que se estabilice el voltaje

  // Configurar el prescaler a 128, debe ser siempre menor a 200KHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 

  ADCSRA |= (1 << ADEN);    // Habilitar el ADC
  //DDRC = 0;
  Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  //int suma = 10;
  //OCR1A = OCR1A + suma;
  ADCSRA |= (1 << ADSC);        // Iniciar la conversión
  while (ADCSRA & (1 << ADSC)); // Esperar a que la conversión termine
  uint16_t adcValue = ADC;      // obtener los 10 bits de la conversión ADCH:ADCL
  
  if (adcValue > 512){
    OCR1A = OCR1A +10;
    //OCR1B = OCR1B +10;
  }
  else if (adcValue < 512){
    OCR1A = OCR1A -10;
  }
  Serial.println(adcValue);
  delay(200);

}