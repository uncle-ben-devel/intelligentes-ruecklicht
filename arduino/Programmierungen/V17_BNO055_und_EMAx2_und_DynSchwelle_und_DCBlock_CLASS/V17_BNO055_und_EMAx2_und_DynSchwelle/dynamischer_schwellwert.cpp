// cpp-File fuer die dynamischer_schwellwert-Klasse
#include "dynamischer_schwellwert.h"
#include <utility/imumaths.h>
/*
Ueberladung des Konstruktors fuer Festlegen des Gleichanteils der Schwelle und der Gewichtung der Streuung.
Ausgang: offset, gewicht im Speicher des dynamischer_schwellwert-Objekts
Eingang: offset, gewicht
*/
dynamischer_schwellwert::dynamischer_schwellwert(float A, float B){
	offset = A;
	gewicht = B;
}

/*
Berechnet erst die Streuung und dann daraus die Schwelle.
Ausgang: Schwelle
Eingang: Aktueller Messwert
*/
float dynamischer_schwellwert::schwellwert_berechnen(float* x){
	ringpuffer[pufferindex] = *x;	// neuester Wert in Ringpuffer
	x_plus = ringpuffer[pufferindex] - ringpuffer[(pufferindex + ringpuffer_size) % (ringpuffer_size)];
	x_plus = abs(x_plus);	// x_plus ist der Abstand zwischen dem neuesten und dem zweit-neuesten Wert.
	
	x_minus = ringpuffer[(pufferindex + 2) % (ringpuffer_size)] - ringpuffer[(pufferindex + 1) % (ringpuffer_size)];
	x_minus = abs(x_minus);	// x_minus ist Abstand zwischen dem aeltesten und dem zweit-aeltesten Wert.
	
	x_sum = x_sum + x_plus - x_minus;	// Rekursiv implementiert: der neueste Wert kommt dazu, der aelteste Wert rotiert raus.
	streuung = x_sum / (ringpuffer_size - 1);	// Normierung durch die Filterbreite
	
	schwelle = offset + gewicht * streuung;	// Berechnung der Schwelle aus der Streuung 
	pufferindex = (pufferindex + 1) % (ringpuffer_size);	// Pufferindex hochzaehlen
	
	return schwelle;
}
