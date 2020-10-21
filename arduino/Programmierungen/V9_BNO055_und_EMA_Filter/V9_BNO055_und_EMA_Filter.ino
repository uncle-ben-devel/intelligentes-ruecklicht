//Wahl der Ausgabe: 1=Serieller Monitor, 2=Serieller Plotter, 3=Ausgabe fuer MatLab, sontige=keine Ausgabe
#define Ausgabeformat 20


#include <Wire.h>
#include <Ticker.h> //Timer
//Sensor Specific
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define TACHO 13            //13=D7
#define BRAKES  14           //15=D5
#define DataDumpPin 12      //12=D6
#define LED_BRAKE 15        //15=D8

#define BrakeScale  12        //Bremsenfaktor fuer Sichtbarkeit im Plotter
#define WegProTrigger 0.717    //3 Trigger je Umdrehung, Gesamtumfang = 215cm @ 90kg
#define LED_ONBOARD LED_BUILTIN

#define f_sample 200    // Samplefrequenz des Beschl.Sensors







////SpeicherArray
//uint16_t ii = 0;
//bool ArrayFullFlag = false;
//bool DataDumpPinMerker = false;
////typedef struct MessElement {int16_t x_AccelMess; int16_t y_AccelMess; int16_t z_AccelMess; byte Brake_Mess; int16_t TachoAccel_Mess;};
//typedef struct MessElement {int32_t MessZeitPunkt; int16_t x_AccelMess; byte Brake_Mess; int16_t TachoAccel_Mess;};
//#define SpeicherArraySize 4000  //max. 5000, dann Speicher voll
//#define DataScaleFactor 4000  //factor, by which the float gets multiplicated, before beeing saved to the Meassurement Array int16_t
//MessElement SpeicherArray[SpeicherArraySize] = {0};

// Timer
Ticker timer; //Timer for ISR to get Sensor Data
//volatile int interrupts;
volatile bool Timer1Flag = false;


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
bool Sensor_connected = false;

////FIR Filter anwenden
//#define SizeFilter 10
//byte BufferIndex = 0;
//float RingspeicherTacho [SizeFilter] = {0};
////const float FilterKoef [SizeFilter] = {-0.0233, -0.0895, -0.0691, 0.2207, 0.5906, 0.5906, 0.2207, -0.0691, -0.0895, -0.0233};
////const float FilterKoef [SizeFilter] = {0.025129476627356,0.003381100423567,-0.093758409423979,0.076939531819290,0.496977176353796,0.496977176353796,0.076939531819290,-0.093758409423979,0.003381100423567,0.025129476627356};
//const float FilterKoef [SizeFilter] = {0.0247012131298898,0.00332347877413301,-0.0921605526547271,0.0756283070182741,0.488507553732430,0.488507553732430,0.0756283070182741,-0.0921605526547271,0.00332347877413301,0.0247012131298898};

//CMA Commulative Moving Average
float cma_ynew = 0.0;
float cma_yold = 0.0;

//fuer CMA, EMA, Hysterese
uint32_t SamplesSinceBegin = 1;
float x_new = 1.2345;
float x_raw = 2.3456;

//EMA Filter anwenden
float alpha = 0.04;
float beta = 1 - alpha;
float ema_ynew = 0.0;
float ema_yold = 0.0;

//Schwellwert
float Schwellwert = -0.5;
bool Bremsvermutung = false;

//Hysterese
float t_run = 0.0;    //in Sekunden
float t_stop = 0.0;
float t_on = 0.05;  //Einschaltverzoegerung
float t_off = 0.15; //Ausschaltverzoegerung
bool licht_an = false;
float t_sample = (1 / (float)f_sample);  //Periodendauer einer Messung



sensors_event_t AccelData; //struct siehe ganz am Ende, enthaelt alle Werte die beim SenorLesen geholt werden.
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

//########################################################
//################## STARTUP #############################
//########################################################
void setup() {  
  pinMode(LED_ONBOARD, OUTPUT);
  pinMode(TACHO, INPUT);
  pinMode(BRAKES, INPUT_PULLUP);
  pinMode(DataDumpPin, INPUT_PULLUP);
  pinMode(LED_BRAKE, OUTPUT); 
                                    
  Serial.begin(115200);
  while(!Serial){};

  while(Sensor_connected == false){
        /* Initialise the sensor BNO055 */
        //if (!bno.begin(Adafruit_BNO055::OPERATION_MODE_ACCONLY))
        if (!bno.begin())
        {
          /* There was a problem detecting the BNO055 ... check your connections */
          Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
          Sensor_connected = false;
          digitalWrite(LED_ONBOARD, LOW);
          delay(500);
          digitalWrite(LED_ONBOARD, HIGH);
          delay(500);
        } else {
          Sensor_connected = true;
          digitalWrite(LED_ONBOARD, HIGH);
        }
  }
  delay(1000);
    
  //Bremslicht ansteuern zum Funktionstest
  licht_an = true;
  digitalWrite(LED_BRAKE, licht_an); 

    
  //I2C Clock Speed
  Wire.setClock(400000);

  
  //Interrupts initialisieren
  noInterrupts(); //enable Interrupts
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
  // Arm the Timer for our Sample Rete Interval
  timer1_write(25000);  // 25000 / (5ticks per us from TIM_DIV16) = 5.000 us interval =200Hz
  //timer1_write(200000);  // 200000 / (5ticks per us from TIM_DIV16) = 40.000 us interval =25Hz
//  attachInterrupt(digitalPinToInterrupt(TACHO),ISR_Tacho, RISING); //Tachometer ISR
//  attachInterrupt(digitalPinToInterrupt(BRAKES),ISR_Brakes, CHANGE); //Brakes ISR

//  //symbolize Start of Meassurement after Reset through OnBoard LED
//  ii = 0;
//  digitalWrite(LED_ONBOARD,HIGH); //lowaktiv
//  Serial.println("\n Druecke den Knopf, um den Messvorgang zu starten...");

  // Einmalig ersten Wert fuer CMA hohlen
   //bno.getEvent(&AccelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
   bno.getEvent(&AccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
   cma_yold = -1 * (AccelData.acceleration.x);
   ema_yold = 0;

  //Bremslicht zum Funktionstest wieder Ausschalten
    licht_an = false;
    digitalWrite(LED_BRAKE, licht_an);
}
//########################################################
//################## MAIN ################################
//########################################################

void loop() {
//  //after Tacho ISR
//  if(TachoISRFlag){ //wird ausgefuehrt wenn der Tacho Interrupt das TachoISRFlag gesetzt hat
//    calculateAccelerationTacho();
//    KannWegTacho = storeBufferAndFilter(&BeschleunigungTacho, 0);    //liefert den Filterwert zurück
//    TachoISRFlag = false; 
//  }

  
  //after Timer ISR
   if(Timer1Flag){ //nach Timerueberlauf alle 5ms
    //bno.getEvent(&AccelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&AccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    Serial.println("x_new(ohne_Offset),ema_ynew,Bremsvermutung,t_run,t_stop,licht_an"); //legende fuer Plotter
    x_raw = -1.0 * (AccelData.acceleration.x);
    cumulative_avg();
    //x_new = x_raw - cma_ynew;
    x_new = x_raw;
    Serial.print(x_new);
    Serial.print("  ");
    exp_moving_avg();
    Serial.print(ema_ynew);
    Serial.print("  ");
    wird_gebremst();   
    Serial.print(((int)Bremsvermutung)*3);
    Serial.print("  ");
    hysterese();
    Serial.print(t_run);
    Serial.print("  ");    
    Serial.print(t_stop);
    Serial.print("  ");
    Serial.print(((int)licht_an)*3);
    Serial.println("  ");
    //Bremslicht ansteuern
    digitalWrite(LED_BRAKE, licht_an);
 
    
    SamplesSinceBegin++;

    
//    //Daten ins Messarray fuer PC Ueberpruefung
//    if(ii < SpeicherArraySize) {
//      //Meassurement incomplete, store data
//      SaveMessurementToArray();
//      ii++;
//    } 
//    else {
//      //Meassurement complete, storage full
//      if(!ArrayFullFlag){
//        noInterrupts(); //disable all Interrupts
//        digitalWrite(LED_ONBOARD,HIGH); //lowaktiv
//        Serial.println("MessArray voll, Interrupts deaktiviert.");
//        ArrayFullFlag = true;
//      }
//    };
//    Timer1Flag = false;
//    Ausgabe();
   }

//   //after DataDumpPin
//   if(digitalRead(DataDumpPin) == LOW){
//    if(DataDumpPinMerker){
//          DumpWholeArray();
//          Serial.println("Daten erfolgreich ausgegeben.");
//       }
//       else{
//          Serial.println("Starte Messvorgang in 1 Sekunde.");
//          interrupts();
//          DataDumpPinMerker = true;
//          delay(1000);
//          digitalWrite(LED_ONBOARD,LOW); //lowaktiv
//       }
//    }
}

//########################################################
//################## MAIN is above #######################
//########################################################



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
  //accel.getEvent(&eventADXL);
  Timer1Flag = true;
}

//################### Funktionen #########################

void cumulative_avg(){
  //Berechnen des Commulativen Averrage
  cma_ynew = cma_yold + (x_raw - cma_yold) / SamplesSinceBegin;
  cma_yold = cma_ynew;
}

void exp_moving_avg(){
  //Berechnen des Exponental Moving Averrage Filters
  ema_ynew = (alpha * x_new) + (beta * ema_yold);
  ema_yold = ema_ynew;
}

void wird_gebremst(){
  //Vermutung, dass gebremst wird
  if (ema_ynew < Schwellwert)  Bremsvermutung = true;
  else            Bremsvermutung = false;
}

void hysterese(){
  //Zaehler fuer die Ein- und Ausschaltverzoegerung fuer Bremslicht
  if (Bremsvermutung){ t_run = t_run + t_sample;    t_stop = 0.0;
  }else {       t_stop = t_stop + t_sample;    t_run = 0.0;}

  //Echte Hysterese fuer Bremslicht
  if (licht_an && (t_stop >= t_off)) {   licht_an = false;
  }else{ if (!licht_an && (t_run >= t_on))  licht_an = true;}
}


void calculateAccelerationTacho(void){
  //Berechnen der Beschleungiung
  Programmlaufzeit = micros();  //nur zum checken wie lange er rechnet
  UmdrehFreq = 1 / Difference;    //in Hz
  GeschwindigkeitNeu =  WegProTrigger / Difference;    //in m/s
  BeschleunigungTacho = (GeschwindigkeitNeu - GeschwindigkeitAlt) / Difference;    //in m/s^2
  GeschwindigkeitAlt = GeschwindigkeitNeu;
}


//float storeBufferAndFilter(float* BeschlWert, byte WelcherBuffer){
//  float Rueckgabewert = 1;
//  //0=Tacho, 1=x, 2=y, 3=z
//  switch (WelcherBuffer){
//    case 0: //Buffer Tacho
//      RingspeicherTacho[BufferIndex] = *BeschlWert;   
//      //Serial.print(BufferIndex); Serial.print(" ");/* Serial.print(BeschlWert); Serial.print("  "); Serial.println((int)*BeschlWert);*/
//      //Serial.println(RingspeicherTacho[BufferIndex]);
//      Rueckgabewert = FilterBuffer(RingspeicherTacho);
//      
//      break;
//
//    case 1: //Buffer x_Axis
//      //RingspeicherTacho[BufferIndex] = *BeschlWert;
//      break;
//
//    default: ;
//  }   
//    BufferIndex = (++BufferIndex) % SizeFilter;
//    return Rueckgabewert;
//  
//  
//}

//float FilterBuffer(float* Buffer){
//  float Filterwert = 1;
//  for( byte FilterIndex = 0; FilterIndex < SizeFilter; FilterIndex ++){
//    Filterwert = Filterwert + Buffer[(BufferIndex - FilterIndex + SizeFilter) % SizeFilter] * FilterKoef[FilterIndex];
//    //Serial.println (Filterwert);
//    /* Der Filter geht ein mal ueber den Buffer, dabei muessen die Filterkoeffizienten 
//    immer auf den vom aktuellen Bufferwert in der Zeit zurueck angewandt werden. (BufferIndex(fest) - Filterindex(laeuft)).
//    damit der Index nicht negativ wird, wird zuerst um die SizeFilter, welche gleich der BufferSize ist addiert,
//    anschliessend noch modulo geteilt um wieder den Index zu erhalten.*/
//  }
//  //Filterwert = Filterwert / 10;
//  return Filterwert;
//}


//void SaveMessurementToArray(){
//  SpeicherArray[ii].x_AccelMess = AccelData.acceleration.x * DataScaleFactor;
//  //SpeicherArray[ii].y_AccelMess = AccelData.acceleration.y * DataScaleFactor;
//  //SpeicherArray[ii].z_AccelMess = AccelData.acceleration.z * DataScaleFactor;
//  SpeicherArray[ii].Brake_Mess = BrakeState;
//  SpeicherArray[ii].TachoAccel_Mess = BeschleunigungTacho * DataScaleFactor;
//  SpeicherArray[ii].MessZeitPunkt = AccelData.timestamp;
//}

//void DumpWholeArray(void){
//  for (int aa=0; aa<SpeicherArraySize; aa++){
//    //Beschleunigungen in m/s^2, später ggf. die Rohwerte verarbeiten, sodass man sich die Umrechnung schenken kann
//    
//      Serial.print(aa);
//      Serial.print("\t");
//      Serial.print(SpeicherArray[aa].MessZeitPunkt);
//      Serial.print("\t");
//      Serial.print(SpeicherArray[aa].x_AccelMess);
//      Serial.print("\t");
////      Serial.print(SpeicherArray[aa].y_AccelMess);
////      Serial.print("\t");
////      Serial.print(SpeicherArray[aa].z_AccelMess);
////      Serial.print("\t");
//      Serial.print(SpeicherArray[aa].Brake_Mess);
//      Serial.print("\t");
//      Serial.print(SpeicherArray[aa].TachoAccel_Mess);
//      Serial.println("\t");
//      delay(1); //otherwise output will fail due to output buffer overload
//  }
//}

//############################ Ausgabe ##################################
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

  //Ausgabe fuer MatLab
    if(Ausgabeformat == 3){
      //x, y, z, Brakes, TachoAcceleration, gefilterte Tacho Acceleration

      //Beschleunigungen in m/s^2, später ggf. die Rohwerte verarbeiten, sodass man sich die Umrechnung schenken kann
      Serial.print(AccelData.acceleration.x, 20); //20 Nachkommastellen
      Serial.print("\t");
      Serial.print(AccelData.acceleration.y, 20);
      Serial.print("\t");
      Serial.print(AccelData.acceleration.z, 20);
      Serial.print("\t");

      Serial.print(BrakeState);
      Serial.print("\t");
      Serial.print(BeschleunigungTacho, 20);
      Serial.print("\t");
      Serial.print(KannWegTacho, 20);
      Serial.println("\t");
      
    }
}



//Auszug aus der Adafruit BNO055.h/.cpp
///*!
// *  @brief  Reads the sensor and returns the data as a sensors_event_t
// *  @param  event
// *          Event description
// *  @param  vec_type
// *          specify the type of reading
// *  @return always returns true
// */
//bool Adafruit_BNO055::getEvent(sensors_event_t *event, adafruit_vector_type_t vec_type)
//{
//  /* Clear the event */
//  memset(event, 0, sizeof(sensors_event_t));
//
//  event->version = sizeof(sensors_event_t);
//  event->sensor_id = _sensorID;
//  event->timestamp = millis();
//
//  //read the data according to vec_type
//  imu::Vector<3> vec;
//  if (vec_type == Adafruit_BNO055::VECTOR_LINEARACCEL)
//  {
//    event->type = SENSOR_TYPE_LINEAR_ACCELERATION;
//    vec = getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//
//    event->acceleration.x = vec.x();
//    event->acceleration.y = vec.y();
//    event->acceleration.z = vec.z();
//  }
//  else if (vec_type == Adafruit_BNO055::VECTOR_ACCELEROMETER)
//  {
//    event->type = SENSOR_TYPE_ACCELEROMETER;
//    vec = getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//
//    event->acceleration.x = vec.x();
//    event->acceleration.y = vec.y();
//    event->acceleration.z = vec.z();
//  }
//  else if (vec_type == Adafruit_BNO055::VECTOR_GRAVITY)
//  {
//    event->type = SENSOR_TYPE_ACCELEROMETER;
//    vec = getVector(Adafruit_BNO055::VECTOR_GRAVITY);
//
//    event->acceleration.x = vec.x();
//    event->acceleration.y = vec.y();
//    event->acceleration.z = vec.z();
//  }
//  else if (vec_type == Adafruit_BNO055::VECTOR_EULER)
//  {
//    event->type = SENSOR_TYPE_ORIENTATION;
//    vec = getVector(Adafruit_BNO055::VECTOR_EULER);
//
//    event->orientation.x = vec.x();
//    event->orientation.y = vec.y();
//    event->orientation.z = vec.z();
//  }
//  else if (vec_type == Adafruit_BNO055::VECTOR_GYROSCOPE)
//  {
//    event->type = SENSOR_TYPE_ROTATION_VECTOR;
//    vec = getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//
//    event->gyro.x = vec.x();
//    event->gyro.y = vec.y();
//    event->gyro.z = vec.z();
//  }
//  else if (vec_type == Adafruit_BNO055::VECTOR_MAGNETOMETER)
//  {
//    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
//    vec = getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
//
//    event->magnetic.x = vec.x();
//    event->magnetic.y = vec.y();
//    event->magnetic.z = vec.z();
//  }
//  
//
//  return true;
//}

///* Sensor event (36 bytes) */
///** struct sensor_event_s is used to provide a single sensor event in a common
// * format. */
//typedef struct {
//  int32_t version;   /**< must be sizeof(struct sensors_event_t) */
//  int32_t sensor_id; /**< unique sensor identifier */
//  int32_t type;      /**< sensor type */
//  int32_t reserved0; /**< reserved */
//  int32_t timestamp; /**< time is in milliseconds */
//  union {
//    float data[4];              ///< Raw data
//    sensors_vec_t acceleration; /**< acceleration values are in meter per second
//                                   per second (m/s^2) */
//    sensors_vec_t
//        magnetic; /**< magnetic vector values are in micro-Tesla (uT) */
//    sensors_vec_t orientation; /**< orientation values are in degrees */
//    sensors_vec_t gyro;        /**< gyroscope values are in rad/s */
//    float temperature; /**< temperature is in degrees centigrade (Celsius) */
//    float distance;    /**< distance in centimeters */
//    float light;       /**< light in SI lux units */
//    float pressure;    /**< pressure in hectopascal (hPa) */
//    float relative_humidity; /**< relative humidity in percent */
//    float current;           /**< current in milliamps (mA) */
//    float voltage;           /**< voltage in volts (V) */
//    sensors_color_t color;   /**< color in RGB component values */
//  };                         ///< Union for the wide ranges of data we can carry
//} sensors_event_t;
