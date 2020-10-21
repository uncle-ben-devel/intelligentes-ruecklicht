// Klasse fuer Anlegung eines dynamischen Schwellwerts bestehend aus der gewichteteten Streuung und einem Offset.
#define ringpuffer_size 26

class dynamischer_schwellwert {
	public:
		dynamischer_schwellwert(float A, float B);
		float schwellwert_berechnen(float* x);
	private:
		float ringpuffer[ringpuffer_size] = {0.0};		// Puffer fuer Messwerte
		int pufferindex = 0;							// Laufender Index fuer den Ringpuffer
		
		float x_plus, x_minus, x_sum = 0.0;				// fuer die Berechnung der Streuung
		float streuung = 0.0;							// fuer die Berechnung des Schwellwerts
		float offset, gewicht = 0.0;					// fuer die Berechnung des Schwellwerts
		float schwelle = 0.0;							// dynamischer Schwellwert
};