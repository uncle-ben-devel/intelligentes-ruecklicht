// Klasse fuer Filterung mit zweifachem exponentiellem laufendem Mittelwertsfilter (EMAx2)
class EMAx2 {
	public:
		EMAx2(float a0, float a1);
		float EMAx2_filter(float *x);
	private:
		float alpha0,alpha1 = 0.0;	// Faktor fuer den x-Wert
		float beta0, beta1 = 0.0;	// Faktor fuer den vorherigen y-Wert
		float y0, y1 = 0.0;		// Vorherige y-Werte der Stufen 0 und 1
};
