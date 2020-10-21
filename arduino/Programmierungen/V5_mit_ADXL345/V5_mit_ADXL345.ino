//Wahl der Ausgabe: 1=Serieller Monitor, 2=Serieller Plotter, sontige=keine Ausgabe
#define Ausgabeformat 2


#include <Wire.h>
#include <Ticker.h>

#define TACHO 13            //13=D7
#define BRAKES  14           //15=D5
#define BrakeScale  20        //Bremsenfaktor fuer Sichtbarkeit im Plotter
#define WegProTrigger 0.717    //3 Trigger je Umdrehung, Gesamtumfang = 215cm @ 90kg
#define LED_ONBOARD LED_BUILTIN

// Timer
Ticker timer; //Timer for ISR to get Sensor Data
//volatile int interrupts;
byte Timer1Flag = 0;

// ADXL345
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//int ADXLreadTimeMerker = 0;
//int ADXLreadTimeDifference = 0;
sensors_event_t eventADXL; 

//######VARIABLEN####
//Tacho RPM Bestimmung
int TachoMerker = 1;
int Programmlaufzeit = 1;
float Difference = 1.0; //sonst Division durch 0
float UmdrehFreq = 3;
float GeschwindigkeitNeu = 1;
float GeschwindigkeitAlt = 1;
float BeschleunigungTacho = 1;
float KannWegTacho = 1;
bool TachoISRFlag = false;
byte BrakeState = 0; //wiederspiegeln vom Bremsenzustand
byte BrakeStateMerker = 0;

//Filter anwenden
#define SizeFilter 10
byte BufferIndex = 0;
float RingspeicherTacho [SizeFilter] = {0};
//const float FilterKoef [SizeFilter] = {-0.0233, -0.0895, -0.0691, 0.2207, 0.5906, 0.5906, 0.2207, -0.0691, -0.0895, -0.0233};
//const float FilterKoef [SizeFilter] = {0.025129476627356,0.003381100423567,-0.093758409423979,0.076939531819290,0.496977176353796,0.496977176353796,0.076939531819290,-0.093758409423979,0.003381100423567,0.025129476627356};
const float FilterKoef [SizeFilter] = {0.0247012131298898,0.00332347877413301,-0.0921605526547271,0.0756283070182741,0.488507553732430,0.488507553732430,0.0756283070182741,-0.0921605526547271,0.00332347877413301,0.0247012131298898};
  
//###### Funktionsprototypen #########
//void calculteAcceleration(void);
//void Ausgabe(void);

void setup() {                                        // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial);

    /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
   accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);

  //I2C Clock Speed
  Wire.setClock(400000);
  
  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(TACHO, INPUT);
  pinMode(BRAKES, INPUT_PULLUP);
  
  //Interrupts initialisieren
  interrupts();
  timer1_attachInterrupt(ISR_Timer1);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
                  /* Dividers:
                    TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
                    TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
                    TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
                  Reloads:
                    TIM_SINGLE  0 //on interrupt routine you need to write a new value to start the timer again
                    TIM_LOOP  1 //on interrupt the counter will start with the same value again
                  */
  // Arm the Timer for our 200Hz / 5ms Interval
  timer1_write(25000);  // 25000 / (5ticks per us from TIM_DIV16) = 5000 us interval
  attachInterrupt(digitalPinToInterrupt(TACHO),ISR_Tacho, RISING); //Tachometer ISR
  attachInterrupt(digitalPinToInterrupt(BRAKES),ISR_Brakes, CHANGE); //Brakes ISR

}
//########################################################
//################## Main ################################
//########################################################

void loop() {
  
  digitalWrite(LED_ONBOARD,!digitalRead(TACHO));
  
  if(TachoISRFlag){ //wird ausgefuehrt wenn der Tacho Interrupt das TachoISRFlag gesetzt hat
    calculateAccelerationTacho();
    KannWegTacho = storeBufferAndFilter(&BeschleunigungTacho, 0);    //liefert den Filterwert zurück
    TachoISRFlag = false; 
    Ausgabe();
  }

  if(BrakeState != BrakeStateMerker){
    BrakeStateMerker = BrakeState;
    Ausgabe();    
  }

    /* Get a new sensor event */ 
  //sensors_event_t eventADXL; 
  //accel.getEvent(&eventADXL); //jetzt in ISR_Timer1

 if(Timer1Flag){
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(eventADXL.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(eventADXL.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(eventADXL.acceleration.z); Serial.print("  ");Serial.print("m/s^2 \t");
  //Serial.print(millis());
  Serial.println("");
  //delay(500);
  Timer1Flag = 0;
 }
}

//################### ISR Tacho ###########################
ICACHE_RAM_ATTR void ISR_Tacho() {
  Difference = (micros() - TachoMerker) / 1000000.0;    //Zeit zwischen 2 Magneten in sec
  TachoMerker = micros();
  TachoISRFlag = true;
}    

//################### ISR Brakes ###########################
ICACHE_RAM_ATTR void ISR_Brakes() {
  BrakeState = BrakeScale*digitalRead(BRAKES);
}

//################### ISR Timer 1###########################
//Beschleunigungssensor pollen
ICACHE_RAM_ATTR void ISR_Timer1() {
  accel.getEvent(&eventADXL);
  Timer1Flag = 1;
}

//################### Funktionen #########################

void calculateAccelerationTacho(void){
  //Berechnen der Beschleungiung
  Programmlaufzeit = micros();  //nur zum checken wie lange er rechnet
  UmdrehFreq = 1 / Difference;    //in Hz
  GeschwindigkeitNeu =  WegProTrigger / Difference;    //in m/s
  BeschleunigungTacho = (GeschwindigkeitNeu - GeschwindigkeitAlt) / Difference;    //in m/s^2
  GeschwindigkeitAlt = GeschwindigkeitNeu;
}


float storeBufferAndFilter(float* BeschlWert, byte WelcherBuffer){
  float Rueckgabewert = 1;
  //0=Tacho, 1=x, 2=y, 3=z
  switch (WelcherBuffer){
    case 0: //Buffer Tacho
      RingspeicherTacho[BufferIndex] = *BeschlWert;   
      //Serial.print(BufferIndex); Serial.print(" ");/* Serial.print(BeschlWert); Serial.print("  "); Serial.println((int)*BeschlWert);*/
      //Serial.println(RingspeicherTacho[BufferIndex]);
      Rueckgabewert = FilterBuffer(RingspeicherTacho);
      
      break;

    case 1: //Buffer x_Axis
      //RingspeicherTacho[BufferIndex] = *BeschlWert;
      break;

    default: ;
  }   
    BufferIndex = (++BufferIndex) % SizeFilter;
    return Rueckgabewert;
  
  
}

float FilterBuffer(float* Buffer){
  float Filterwert = 1;
  for( byte FilterIndex = 0; FilterIndex < SizeFilter; FilterIndex ++){
    Filterwert = Filterwert + Buffer[(BufferIndex - FilterIndex + SizeFilter) % SizeFilter] * FilterKoef[FilterIndex];
    //Serial.println (Filterwert);
    /* Der Filter geht ein mal ueber den Buffer, dabei muessen die Filterkoeffizienten 
    immer auf den vom aktuellen Bufferwert in der Zeit zurueck angewandt werden. (BufferIndex(fest) - Filterindex(laeuft)).
    damit der Index nicht negativ wird, wird zuerst um die SizeFilter, welche gleich der BufferSize ist addiert,
    anschliessend noch modulo geteilt um wieder den Index zu erhalten.*/
  }
  //Filterwert = Filterwert / 10;
  return Filterwert;
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
      Serial.print(BeschleunigungTacho);
      Serial.println("m/s^2");
    
      Serial.print("Gesamtzeit ISR + Ausgabe: ");
      Serial.print(micros()-Programmlaufzeit);
      Serial.println("us\n");
  
      Serial.print("Die Bremsen sind jetzt: ");
      Serial.println(BrakeState);

      Serial.println("\n####################");
    };

  //Ausgabe für Plotter
    if (Ausgabeformat == 2){
      Serial.println("Geschwindigkeit_in_m/s Beschleunigung_in_m/s^2 Gefilterte_Beschleunigungswerte_in_m/s^2 Bremsenstatus"); //legend for Serial Plotter
      Serial.print(GeschwindigkeitNeu, 20);
      Serial.print("\t");
      Serial.print(BeschleunigungTacho, 20);
      Serial.print("\t");
      Serial.print(KannWegTacho, 20);
      Serial.print("\t");
      Serial.println(BrakeState);
      
    }
}
