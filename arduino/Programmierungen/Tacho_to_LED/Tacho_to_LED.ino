#define TACHO 5
#define WegProTrigger 71.7    //3 Trigger je Umdrehung, Gesamtumfang = 215cm @ 90kg

//######VARIABLEN####
//Tacho RPM Bestimmung
int TachoMerker = 0;
int BlinkMerker = 0;
int Difference = 50; //sonst Division durch 0
float UmdrehFreq = 3;
float GeschwindigkeitNeu = 0;
float GeschwindigkeitAlt = 0;
float Beschleunigung = 0;

//weitere

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(TACHO, INPUT);

  //Interrupts initialisieren
  attachInterrupt(digitalPinToInterrupt(TACHO),ISR_Tacho, RISING);
}

void loop() {
//  digitalWrite(LED_BUILTIN,!digitalRead(TACHO));
 
    //Umdrehungsfrequenz wiedergeben:
    if((millis()- BlinkMerker)>(Difference - 50)){
        digitalWrite(LED_BUILTIN, !HIGH); //LED ist lowAktiv
        delay(50);
        digitalWrite(LED_BUILTIN, !LOW);
        BlinkMerker = millis();
        }
}

ICACHE_RAM_ATTR void ISR_Tacho() {
  Difference = millis() - TachoMerker;
  TachoMerker = millis();
  UmdrehFreq = (float)1000 / (Difference);    //in Hz
  GeschwindigkeitNeu = ((float)1000 * WegProTrigger) / Difference;    //in m/s
  Beschleunigung = 1000*(GeschwindigkeitNeu - GeschwindigkeitAlt)/Difference;    //in m/s^2
  GeschwindigkeitAlt = GeschwindigkeitNeu;

  //Ausgabe
  Serial.print("Zeitdifferenz: ");
  Serial.print(Difference);
  Serial.println("ms");
  
  Serial.print("Umdrehungsfrequenz: ");
  Serial.print(UmdrehFreq, 5);
  Serial.println("Hz");
  
  Serial.print("Geschwindigkeit: ");
  Serial.print(GeschwindigkeitNeu);
  Serial.println("m/s");

  Serial.print("Beschleunigung: ");
  Serial.print(Beschleunigung);
  Serial.println("m/s^2\n\n\n");
}
