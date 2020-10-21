// Klasse fuer Ansteuerung der LED fuer Blinken etc.
class lichtsteuerung {
	public:
		lichtsteuerung(int blinkfrequenz, float samplingfrequenz);
		bool lichtsteuerung_ausfuehren(bool licht_heller, bool licht_blink);
	private:
		int blink_zaehler = 0;	// Zaehler, um die Blinkfrequenz zu erzeugen
		int teilerverhaeltnis = 0;	// fuer Clkdivider
		bool licht_toggle = false;	// Wird mit licht_blink verarbeitet, um das Licht blinken zu lassen
};
