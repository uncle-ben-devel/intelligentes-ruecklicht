// Klasse fuer Filterung mit einfachem exponentiellem laufendem Mittelwertsfilter (EMA)
class EMA {
	public:
		EMA(float a);
		float EMA_filter(float *x);
	private:
		float alpha = 0.0;	// Faktor fuer den x-Wert
		float beta = 0.0;	// Faktor fuer den vorherigen y-Wert
		float y = 0.0;		// (Vorheriger) y-Wert, Ausgang des Filters
};
