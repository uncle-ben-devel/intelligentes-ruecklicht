//Wahl der Ausgabe: 1=Serieller Monitor, 2=Serieller Plotter, 3=Ausgabe fuer MatLab, sonstige=keine Ausgabe
#define Ausgabeformat 20

#include <Wire.h>
#include <Ticker.h> //Timer
//Sensor Specific
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// unsere Klassen
#include "EMA.h"
#include "EMAx2.h"
#include "hysterese.h"
#include "streuung.h"
#include "lichtsteuerung.h"

// Definition der Pins
#define TACHO		13	//13=D7
#define BRAKES		14	//15=D5
#define DataDumpPin	12	//12=D6
#define LED_BRAKE	15	//15=D8

#define BrakeScale  12        //Bremsenfaktor fuer Sichtbarkeit im Plotter
#define WegProTrigger 0.717    //3 Trigger je Umdrehung, Gesamtumfang = 215cm @ 90kg
#define LED_ONBOARD LED_BUILTIN

#define f_sample 100.0    // Samplefrequenz, mit der wir die Sensorwerte abfragen

// Timer
Ticker timer; //Timer for ISR to get Sensor Data
volatile bool Timer1Flag = false;

//######VARIABLEN####
// Variablen fuer die Verarbeitungskette
	float x_raw = 0.0; // neuester Sensorwert
	float x_gleichanteil = 0.0;	// Gleichanteil in x_raw
	float y_tp = 0.0;	// nach einer Tiefpassfilterung
	float streuung_aktuell = 0.0;	// Streuung, um Schwellwert zu berechnen
	#define offset_normal	-0.15	// Gleichanteil der Schwelle
	#define offset_gefahr	-1.0	
	#define gewicht_normal	-0.07	// Gewicht, mit dem die Streuung multipliziert wird, um den Schwellwert zusammen mit dem Gleichanteil zu berechnen
	#define gewicht_gefahr	-0.07
	float schwelle_normal = 0.0;	// Schwellwert, der unterschritten werden muss, damit ein Bremsvorgang erkannt wird.
	float schwelle_gefahr = 0.0;	// Schwellwert, der unterschritten werden muss, damit eine Gefahrbremsung erkannt wird.
	bool bremsvermutung_normal = false;	// Normaler Bremsvorgang erkannt, vor der Hysterese
	bool bremsvermutung_gefahr = false;	// Gefahrbremsung erkannt, vor der Hysterese
// Variablen fuer Lichtsteuerung
	bool licht_heller = false;	// Bremsvorgang erkannt und Hysterese fuer normales Bremsen passiert
	bool licht_blink = false;	// Bremsvorgang erkannt und Hysterese fuer Gefahrbremsen passiert
  
// Variablen fuer Debug, R&D
	byte BrakeState = 0; //wiederspiegeln vom Bremsenzustand
	bool Sensor_connected = false;

//######OBJEKTE####
	streuung streuung_obj;
	EMA ema_dc(0.01);	// alpha-wert; EMA fuer die Berechnung des Gleichanteils; alpha = 2 / (filterbreite + 1) fuer aehnliches Ergebnis zu SMA. alpha = 0.005 entspricht N=400, mit group delay von 200 samples, also 2s.
	EMA ema_signal(0.04);	// alpha-wert; EMA fuer die Filterung des Signals
	//EMAx2 emax2_01_004(0.1, 0.04); // alpha0, alpha1
	hysterese hyst_normal(0.05, 0.15, f_sample);	// Einschaltverzoegerung, Ausschaltverzoegerung, Samplingfrequenz
	hysterese hyst_gefahr(0.01, 0.5, f_sample);	// Einschaltverzoegerung, Ausschaltverzoegerung, Samplingfrequenz
	lichtsteuerung lichtsteuerung_obj(25, f_sample)	// Blinkfrequenz, Samplefrequenz
	// Assign a unique ID to this sensor at the same time
	Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
	sensors_event_t eventADXL;	//struct siehe ganz am Ende, enthaelt alle Werte die beim Sensor lesen geholt werden.

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

  // Auskommentieren fuer Debug ohne Sensor
	while(!Serial){};	// wartet auf die serielle Verbindung
	while(Sensor_connected == false){
        // Initialise the sensor
        if (!accel.begin())
        {
			// There was a problem detecting the BNO055 ... check your connections
			Serial.println("Ooops, no ADXL345 detected ... Check your wiring or I2C ADDR!");
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
 
	digitalWrite(LED_BRAKE, true); //Bremslicht ansteuern zum Funktionstest
	Wire.setClock(400000);	//I2C Clock Speed
	/* Set the range to whatever is appropriate for your project */
   	accel.setRange(ADXL345_RANGE_4_G);   //2_G, 4_G 8_G 16_G  

	//Interrupts initialisieren
	noInterrupts(); //enable Interrupts
	timer1_attachInterrupt(ISR_Timer1);
	timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);
	/*	Dividers:
			TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
			TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
			TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
		Reloads:
			TIM_SINGLE  0 //on interrupt routine you need to write a new value to start the timer again
			TIM_LOOP  1 //on interrupt the counter will start with the same value again
	*/
	// Arm the Timer for our Sample Interval
	//timer1_write(25000);  // 25000 / (5ticks per us from TIM_DIV16) = 5.000 us interval =200Hz
	timer1_write(50000);	// 50k / 5M = 10k us = 10ms -> 100Hz

    digitalWrite(LED_BRAKE, false);	//Bremslicht zum Funktionstest wieder Ausschalten
}
//########################################################
//################## MAIN ################################
//########################################################

void loop() {
  //after Timer ISR
	if(Timer1Flag){ //nach Timerueberlauf alle t_sample (1 / f_sample)
		accel.getEvent(&eventADXL);
		//bno.getEvent(&eventADXL, Adafruit_BNO055::VECTOR_LINEARACCEL);
		// Rohdaten einlesen
			x_raw = +1.0 * (eventADXL.acceleration.x);
		// Offset fuer Korrektur berechnen
			x_gleichanteil = ema_dc.EMA_filter(&x_raw);
		// Filterung mit Tiefpass
			y_tp = ema_signal.EMA_filter(&x_raw);
		// Schwellwertbildung
      streuung_aktuell = streuung_obj.streuung_berechnen(&x_raw);
			schwelle_normal = offset_normal + x_gleichanteil + gewicht_normal * streuung_aktuell;
			schwelle_gefahr = offset_gefahr + x_gleichanteil + gewicht_gefahr * streuung_aktuell;
		// Bremsvermutungen aufstellen
			bremsvermutung_normal = (y_tp < schwelle_normal);
			bremsvermutung_gefahr = (y_tp < schwelle_gefahr);
		// Hysteresen anwenden
			licht_heller = hyst_normal.hysterese_anwenden(&bremsvermutung_normal);
			licht_blink = hyst_gefahr.hysterese_anwenden(&bremsvermutung_gefahr);
		// Plotten fuer Debug
			print_data();
		//Bremslicht ansteuern
			digitalWrite(LED_BRAKE, lichtsteuerung_obj.lichtsteuerung_ausfuehren(licht_heller, licht_blink));
	}
}
//########################################################
//################## MAIN is above #######################
//########################################################

//################### ISR Brake ###########################
ICACHE_RAM_ATTR void ISR_Brakes() {
	BrakeState = BrakeScale*digitalRead(BRAKES);
}

//################### ISR Timer 1###########################
//Beschleunigungssensor pollen
ICACHE_RAM_ATTR void ISR_Timer1() {
	Timer1Flag = true;
}

void print_data(){
	Serial.println("Rohdaten, Gleichanteil, tiefpassgefiltert, Streuung_aktuell, Schwellwert_normal, Schwellwert_gefahr, Bremsvermutung_normal, Bremsvermutung_gefahr, Licht_heller, Licht_blink, lBlinkzaehler"); //Legende fuer Plotter
						Serial.print(x_raw); 
	Serial.print("\t");	Serial.print(x_gleichanteil);
	Serial.print("\t");	Serial.print(10*y_tp);
  Serial.print("\t"); Serial.print(streuung_aktuell);
	Serial.print("\t");	Serial.print(10*schwelle_normal);
	Serial.print("\t");	Serial.print(10*schwelle_gefahr);
	Serial.print("\t");	Serial.print(bremsvermutung_normal);
	Serial.print("\t");	Serial.print(bremsvermutung_gefahr);
	Serial.print("\t");	Serial.print(licht_heller);
	Serial.print("\t");	Serial.print(licht_blink);
  Serial.print("\t");  Serial.print(blink_zaehler);
	Serial.println("\t");
}


//	Auszug aus der Adafruit BNO055.h/.cpp
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
