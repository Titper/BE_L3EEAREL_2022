#define PWM 5 // pin de sortie pour un moteur 
#define Scope 6 // mesure


void setup() {
  
  pinMode(PWM, OUTPUT); //configuration de la pin pour le pwm
  pinMode(Scope, OUTPUT);

  //TCCR0B = TCCR0B & 0b11111000 | 0x05;

  int valeur = 20;
  
  analogWrite(PWM, valeur);
  analogWrite(Scope, valeur);
}

void loop() {
  // put your main code here, to run repeatedly:

}
