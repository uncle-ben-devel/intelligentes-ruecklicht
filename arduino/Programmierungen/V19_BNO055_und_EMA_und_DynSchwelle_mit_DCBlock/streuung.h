// Klasse fuer Bestimmung der Streuung des Signals
#define ringpuffer_size 26

class streuung {
	public:
		float streuung_berechnen(float* x);
	private:
		float ringpuffer[ringpuffer_size] = {0.0};		// Puffer fuer Messwerte
		int pufferindex = 0;							// Laufender Index fuer den Ringpuffer
		
		float x_plus, x_minus, x_sum = 0.0;				// fuer die Berechnung der Streuung
		float streuung = 0.0;							// fuer die Berechnung des Schwellwerts
};
