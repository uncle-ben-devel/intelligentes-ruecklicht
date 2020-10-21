// cpp-File fuer die lichtsteuerung-Klasse
#include "lichtsteuerung.h"

/*
Ueberladung des Konstruktors fuer Festlegen des teilerverhaeltnisses aus Blinkfrequenz und Samplefrequenz
Ausgang: alpha im Speicher des EMAx2-Objekts
Eingang: alpha
*/
lichtsteuerung::lichtsteuerung(int blinkfrequenz, float samplingfrequenz){
	teilerverhaeltnis = samplingfrequenz / (int)blinkfrequenz;
}

/*
Funktion fuer die Filterung mit dem EMA. Nutzt die vorher festgelegten alpha und beta.
Ausgang: Gefiltertes Sample
Eingang: Rohdaten
*/
bool lichtsteuerung::lichtsteuerung_ausfuehren(bool licht_heller, bool licht_blink){
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
	return((licht_heller && !licht_blink) || (licht_blink && licht_toggle));
}
