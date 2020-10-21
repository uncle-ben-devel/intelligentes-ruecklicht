// cpp-File fuer die lichtsteuerung-Klasse
#include "lichtsteuerung.h"
/*
Ueberladung des Konstruktors fuer Festlegen des teilerverhaeltnisses aus Blinkfrequenz und Samplefrequenz
*/
lichtsteuerung::lichtsteuerung(int blinkfrequenz, float samplingfrequenz){
	teilerverhaeltnis = samplingfrequenz / (int)blinkfrequenz;
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
}
/*
Funktion fuer die Filterung mit dem EMA. Nutzt die vorher festgelegten alpha und beta.
*/
void lichtsteuerung::lichtsteuerung_ausfuehren(bool licht_heller, bool licht_blink){
	if (licht_blink){
        	if (blink_zaehler >= teilerverhaeltnis){	// Modulozaehler
          		blink_zaehler = 0;
          		licht_toggle = !licht_toggle;	}	// bei Ueberlauf LED blinken lassen
        	else{	blink_zaehler = (blink_zaehler + 1);	}
      	}
	else{
        	blink_zaehler = 0;
        	licht_toggle = 0;
	}

  /* 
  Idle: Stufe 1 -> 50% der roten LEDs an
  Bei Bremsen: Stufe 2 -> alle roten LEDs an
  Bei Gefahrbremsung: Wechsel zwischen Stufe 1 und Stufe 2 mit der Blinkfrequenz (Bereich 5~25Hz)
  */
  setNeoPixel(1 + (int)(licht_heller && !licht_blink) + (int)(licht_blink && licht_toggle));  
  
}

void lichtsteuerung::setNeoPixel(int workingMode){
  strip.setBrightness(255);
  switch (workingMode) {
    case 0:
    //off
          strip.clear();
          strip.show();
          break;
          
    case 1:
    //normal rearlight
          strip.clear();
          for(int i=0; i<strip.numPixels(); i += 2) { // For every second pixel in strip...
            strip.setPixelColor(i, lightred);
          }
          strip.show();
          break;
          
    case 2:
    //normal brake light
          strip.clear();
          strip.fill(fullred);
          strip.show();
          break;
          
    case 3:
    //emergency brake light
          strip.clear();
          strip.fill(fullred);
          for(int i=0; i<strip.numPixels(); i += 2) { // For every second pixel in strip...
            strip.setPixelColor(i, redwhite);
          }
          strip.show();
          break;
          
    default:
    //you should not be here!
          strip.clear();
          strip.fill(fullblue);
          strip.show();
          break;
  }
}
