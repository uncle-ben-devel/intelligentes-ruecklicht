// cpp-File fuer die EMAx2-Klasse
#include "EMAx2.h"

/*
Ueberladung des Konstruktors fuer Festlegen von alpha0 und alpha1.
Ausgang: alpha0, alpha1 im Speicher des EMAx2-Objekts
Eingang: alpha0, alpha1
*/
EMAx2::EMAx2(float a0, float a1){
		alpha0 = a0; 	// Faktor fuer den x-Wert
		alpha1 = a1;	// Faktor fuer den Ausgang der Stufe 0
		beta0 = 1 - alpha0;	// Faktor fuer den vorherigen y-Wert der Stufe 0 (y0)
		beta1 = 1 - alpha1;	// Faktor fuer den vorherigen y-Wert der Stufe 1 (y1)
}

/*
Funktion fuer die Filterung mit dem EMAx2. Nutzt die vorher festgelegten alpha und beta.
Ausgang: Gefiltertes Sample
Eingang: Rohdatene
*/
float EMAx2::EMAx2_filter(float *x){
	y0 = alpha0 * *x + beta0 * y0;
	y1 = alpha1 * y0 + beta1 * y1;
	
	return y1;
}
