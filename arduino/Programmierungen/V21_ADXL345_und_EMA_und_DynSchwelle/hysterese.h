// Klasse fuer Anwendung einer beliebig langen Zeithysterse auf ein digitales Signal
class hysterese {
	public:
		hysterese(float t_high, float t_low, float samplingfrequenz);
		bool hysterese_anwenden(bool *eingang);
	private:
		float samplingintervall = 0.0;	// Kehrwert der Samplingfrequenz, genutzt als Inkrement fuer den Zaehler von t_run und t_stop
		float t_run, t_stop = 0.0;	// Zeit, die das Eingangssignal am Stueck TRUE (t_run) oder FALSE (t_stop) ist.
		float t_on, t_off = 0.0;	// Zeit, die das Eingangssignal am Stueck TRUE (t_on) oder FALSE (t_off) sein muss, dass der Zustand sich aendert
		bool zustand = false;		// Hysteresenzustand.
};
