//Wahl der Ausgabe: 1=Serieller Monitor, 2=Serieller Plotter, sontige=keine Ausgabe
#define Ausgabeformat 2

#define TACHO 5
#define WegProTrigger 0.717    //3 Trigger je Umdrehung, Gesamtumfang = 215cm @ 90kg


//######VARIABLEN####
//Tacho RPM Bestimmung
int TachoMerker = 1;
int Programmlaufzeit = 1;
float Difference = 1.0; //sonst Division durch 0
float UmdrehFreq = 3;
float GeschwindigkeitNeu = 1;
float GeschwindigkeitAlt = 1;
float Beschleunigung = 1;
bool TachoISRFlag = false;

//###### Funktionsprototypen #########
//void calculteAcceleration(void);
//void Ausgabe(void);

void setup() {                                        // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);
  
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TACHO, INPUT);

  //Interrupts initialisieren
  attachInterrupt(digitalPinToInterrupt(TACHO),ISR_Tacho, RISING);
}

//################## Main ################################
void loop() {
  
  digitalWrite(LED_BUILTIN,!digitalRead(TACHO));
  if(TachoISRFlag){
    calculteAcceleration();
    TachoISRFlag = false; 
    Ausgabe();
  }

}

//################### ISR Tacho ###########################

ICACHE_RAM_ATTR void ISR_Tacho() {
  Difference = (micros() - TachoMerker) / 1000000.0;    //Zeit zwischen 2 Magneten in sec
  TachoMerker = micros();
  TachoISRFlag = true;
}    

//################### Funktionen #########################

void calculteAcceleration(void){
  //Berechnen der Beschleungiung
  Programmlaufzeit = micros();  //nur zum checken wie lange er rechnet
  UmdrehFreq = 1 / Difference;    //in Hz
  GeschwindigkeitNeu =  WegProTrigger / Difference;    //in m/s
  Beschleunigung = (GeschwindigkeitNeu - GeschwindigkeitAlt) / Difference;    //in m/s^2
  GeschwindigkeitAlt = GeschwindigkeitNeu;
}



 void Ausgabe(void){
   //Ausgabe für Monitor
    if (Ausgabeformat == 1) {  
      Serial.print("Rechenzeit: ");
      Serial.print(micros()-Programmlaufzeit);
      Serial.println("us");
      
      Serial.print("Zeitdifferenz: ");
      Serial.print(Difference, 24); //max float Nachkommastellen
      Serial.println("s");
      
      Serial.print("Umdrehungsfrequenz: ");
      Serial.print(UmdrehFreq, 5);
      Serial.println("Hz");
      
      Serial.print("Geschwindigkeit: ");
      Serial.print(GeschwindigkeitNeu);
      Serial.println("m/s");
    
      Serial.print("Beschleunigung: ");
      Serial.print(Beschleunigung);
      Serial.println("m/s^2");
    
      Serial.print("Gesamtzeit ISR + Ausgabe: ");
      Serial.print(micros()-Programmlaufzeit);
      Serial.println("us\n\n\n");
    };

  //Ausgabe für Plotter
    if (Ausgabeformat == 2){
      Serial.println("Geschwindigkeit_in_m/s Beschleunigung_in_m/s^2"); //legend for Serial Plotter
      Serial.print(GeschwindigkeitNeu);
      Serial.print("\t");
      Serial.println(Beschleunigung);
    }
}
