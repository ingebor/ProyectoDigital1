int motorPin = 9;
void setup() {
  Serial.begin(9600);
  
  // Configurar PD2 como entrada con pull-up
  DDRD &= ~(1 << DDD2);
  PORTD |= (1 << PORTD2);

  //configuración de la interrupción
  PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, ponemos en 1 para limpiar
  PCICR |= (1 << PCIE2); //habilitar el grupo de interrupciones del puerto D
  PCMSK2 |= (1 << PCINT18);  // Seleccionar el pin a monitorear en el registro PCMSK2, Bit 18 corresponde a PD2
  
  // Habilitar interrupciones globales
  sei(); //por defecto vienen habilitadas
  Serial.println("setup realizado");
  pinMode(motorPin, OUTPUT);
}

void loop() {
  // Código de la función principal
  //digitalWrite(motorPin, HIGH);
}

ISR(PCINT2_vect) {
  cli(); 
  // Verificar si el pin 2 del puerto D cambió a HIGH
   	if ( PIND & (1 << PIND2) ) {       
      //Código cuando PORTD2  pasa a HIGH
      Serial.println ("PD2 HIGH");
      digitalWrite(motorPin, HIGH);
    } else {
      //Código cuando PORTD2  pasa a LOW
      Serial.println ("PD2 LOW");
      digitalWrite(motorPin, LOW);      
    }
  //PCIFR |= (1 << PCIF2); // Limpiar bandera de interrupción en el puerto D, se apaga automatico en la interrupcion
  sei(); // Habilitar interrupciones globales
}