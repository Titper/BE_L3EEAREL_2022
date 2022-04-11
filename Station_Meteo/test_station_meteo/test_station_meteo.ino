



void setup() {
  
  Serial.begin(115200);
  // PIN_ANEMOMETRE = A2
   //PIN_GIROUETTE = A1
  // PIN_PLUVIOMETRE = A0
  pinMode(4, INPUT);

  
  ////////////////////////////////////////////////
  ///Prise anémo à brancher sur port digital/////
  ////////////////////////////////////////////////
  

}

void loop() {
  
  float tempo1 =0;
  float tempo2=0;
  float tempo3 =0;
  float vitesse ;
  float vitesseRot;
  float dureeHaute, dureeBasse =0;
  int variableTempo;
  int frontMontant=20;
  int d4, d4avant =0;
  int variableFront = 0;
  int i,w;
  int girouette;



////////////////////////////////////////////////////
//////////////Code anemometre////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////
/*

  dureeBasse = pulseIn(4, LOW) / 1000;
  dureeHaute = pulseIn(4, HIGH)/ 1000;

  Serial.print(" dureeBasse  =  ");
  Serial.print(dureeBasse);

  tempo3 =(dureeBasse + dureeHaute)/100;     //vitesse = VitRotation * rayon
                                            // ici rayon = 7.5cm => 0.075m
                                            // VitRotation = 2pi / tempo3
  vitesseRot = 60 / tempo3;
  
  w = (vitesseRot*2*PI) / 60;
  
  vitesse = w*0.075;

  Serial.print("   Valeur temps = "); 
  Serial.print(tempo3); 
  
  Serial.print("  vitesse de rotation = "); 
  Serial.print(vitesseRot); 
  Serial.print(" tr/min  = "); 
  Serial.print(w); 
  Serial.print(" rad/s "); 

  Serial.print("  vitesse du vent = "); 
  Serial.print(vitesse); 
  Serial.print(" m/s"); 
  Serial.print("  =  "); 
  Serial.print(vitesse*3.6); 
  Serial.println(" Km/h"); 


*/





////////////////////////////////////////////////////
//////////////Code girouette////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////


girouette = analogRead(A1);
//Serial.println(girouette);

  if (girouette > 785)
  {
     if (girouette <= 790)
    {
      Serial.println("Nord");
    }
  }


  if (girouette > 460)
  {
     if (girouette <= 470)
    {
      Serial.println("Nord-Est");
    }
  }

  if (girouette > 90)
  {
     if (girouette <= 100)
    {
      Serial.println("Est");
    }
  }

  if (girouette > 180)
  {
     if (girouette <= 190)
    {
      Serial.println("Sud-Est");
    }
  }

  if (girouette > 285)
  {
     if (girouette <= 295)
    {
      Serial.println("Sud");
    }
  }

  if (girouette > 630)
  {
     if (girouette <= 640)
    {
      Serial.println("Sud-Ouest");
    }
  }
  
  if (girouette > 940)
  {
     if (girouette <= 950)
    {
      Serial.println("Ouest");
    }
  }


  if (girouette > 885)
  {
     if (girouette <=895)
    {
      Serial.println("Nord-Ouest");
    }
  }
  delay(100);
}
