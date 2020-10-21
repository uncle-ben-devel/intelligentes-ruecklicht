// Klasse fuer Ansteuerung der LED fuer Blinken etc.
#include <Adafruit_NeoPixel.h>
#define LED_PIN 15
#define LED_COUNT 12
class lichtsteuerung {
	public:
		lichtsteuerung(int blinkfrequenz, float samplingfrequenz);
		void lichtsteuerung_ausfuehren(bool licht_heller, bool licht_blink);
    void rainbowFade2White(int wait, int rainbowLoops, int wait2, int whiteLoops);
    
	private:
    void setNeoPixel(int workingMode);
    
		int blink_zaehler = 0;	// Zaehler, um die Blinkfrequenz zu erzeugen
		int teilerverhaeltnis = 0;	// fuer Clkdivider
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
    // Custom Colors
      uint32_t lightred = strip.Color(50, 0, 0, 0);
      uint32_t fullred = strip.Color(255, 0, 0, 0);
      uint32_t yellow = strip.Color(255, 40, 0, 0);
      uint32_t redwhite = strip.Color(255, 30, 0, 20);
      uint32_t fullblue = strip.Color( 0, 0, 255, 0);
    
		bool licht_toggle = false;	// Wird mit licht_blink verarbeitet, um das Licht blinken zu lassen
};
