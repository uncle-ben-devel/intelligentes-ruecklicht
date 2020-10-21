// cpp-File fuer die Hysterese-Klasse
#include "hysterese.h"

/*
Ueberladung des Konstruktors fuer Festlegen von Start- und Stoppzeit sowie Samplingintervall
Ausgang: Ein- und 
		Ausschaltschwellen 
		Samplingintervall im Speicher des hysterese-Objekts
Eingang: Zeit, die das Signal high sein muss, damit von LOW nach HIGH geschaltet wird; 
	Zeit, die das Signal low sein muss, damit von HIGH auf LOW geschaltet wird; 
	Samplingfrequenz;
*/
hysterese::hysterese(float t_high, float t_low, float samplingfrequenz){
	t_on = t_high;
	t_off = t_low;
	samplingintervall = 1 / samplingfrequenz;	
}

/*
Funktion fuer die Zeithysterese. Nutzt die vorher festgelegten Start- und Stopzeiten, sowie die Samplingfreqenz.
Ausgang: Signal nach Zeithysterese
Eingang: Signal ohne Zeithysterese
*/
bool hysterese::hysterese_anwenden(bool *eingang){
	//Zaehler fuer die Ein- und Ausschaltverzoegerung
		if (*eingang){ 
			t_run = t_run + samplingintervall;    
			t_stop = 0.0;
		}
		else { 
			t_stop = t_stop + samplingintervall;  
			t_run = 0.0;
		}

	//Ausgang der Hysterese
		if (zustand && (t_stop >= t_off)) {   
			zustand = false;  
		}
		else if(!(zustand) && (t_run >= t_on)){
			zustand = true;
		}
	return zustand;
}
