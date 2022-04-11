



void setup() {
  
  Serial.begin(9600);
  // PIN_ANEMOMETRE = A2
   //PIN_GIROUETTE = A0
  // PIN_PLUVIOMETRE = A1
  pinMode(A2,INPUT);
  pinMode(A1,INPUT);
  pinMode(A0,INPUT);

}

void loop() {
  int anemometre = analogRead(A2);
  int pluviometre = analogRead(A1);
  int girouette = analogRead(A0);
 // Serial.print("Valeur de l'anémomètre ="); 
 // Serial.println(anemometre); 

 // Serial.print("Valeur de la pluviomètre ="); 
 // Serial.println(pluviometre); 

  
  Serial.print("Valeur de la girouette ="); 
  Serial.println(girouette); 


  
  delay(100);
}
