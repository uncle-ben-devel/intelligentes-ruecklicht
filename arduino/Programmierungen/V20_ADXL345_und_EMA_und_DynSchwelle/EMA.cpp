// cpp-File fuer die EMA-Klasse
#include "EMA.h"

/*
Ueberladung des Konstruktors fuer Festlegen von alpha
Ausgang: alpha im Speicher des EMAx2-Objekts
Eingang: alpha
*/
EMA::EMA(float a){
		alpha = a; 	// Faktor fuer den x-Wert
		beta = 1 - alpha;	// Faktor fuer den vorherigen y-Wert der Stufe 0 (y0)
}

/*
Funktion fuer die Filterung mit dem EMA. Nutzt die vorher festgelegten alpha und beta.
Ausgang: Gefiltertes Sample
Eingang: Rohdaten
*/
float EMA::EMA_filter(float *x){
	y = alpha * *x + beta * y;
	return y;
}
