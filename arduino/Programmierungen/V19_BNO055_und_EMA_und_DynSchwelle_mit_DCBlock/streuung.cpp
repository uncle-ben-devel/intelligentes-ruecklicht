// cpp-File fuer die dynamischer_schwellwert-Klasse
#include "streuung.h"
#include <utility/imumaths.h>
/*
Berechnet die Streuung
Ausgang: Streuung
Eingang: Aktueller Messwert
*/
float streuung::streuung_berechnen(float* x){
	ringpuffer[pufferindex] = *x;	// neuester Wert in Ringpuffer
	x_plus = ringpuffer[pufferindex] - ringpuffer[(pufferindex + ringpuffer_size - 1) % (ringpuffer_size)];
	x_plus = abs(x_plus);	// x_plus ist der Abstand zwischen dem neuesten und dem zweit-neuesten Wert.
	
	x_minus = ringpuffer[(pufferindex + 2) % (ringpuffer_size)] - ringpuffer[(pufferindex + 1) % (ringpuffer_size)];
	x_minus = abs(x_minus);	// x_minus ist Abstand zwischen dem aeltesten und dem zweit-aeltesten Wert.
	
	x_sum = x_sum + x_plus - x_minus;	// Rekursiv implementiert: der neueste Wert kommt dazu, der aelteste Wert rotiert raus.
	streuung = x_sum / (ringpuffer_size - 1);	// Normierung durch die Filterbreite
	
	pufferindex = (pufferindex + 1) % (ringpuffer_size);	// Pufferindex hochzaehlen
	
	return streuung;
}
