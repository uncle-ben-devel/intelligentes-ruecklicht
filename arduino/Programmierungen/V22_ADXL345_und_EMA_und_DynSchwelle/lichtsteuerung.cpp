// cpp-File fuer die lichtsteuerung-Klasse
#include "lichtsteuerung.h"
/*
Ueberladung des Konstruktors fuer Festlegen des teilerverhaeltnisses aus Blinkfrequenz und Samplefrequenz
*/
lichtsteuerung::lichtsteuerung(int blinkfrequenz, float samplingfrequenz){
	teilerverhaeltnis = samplingfrequenz / (int)blinkfrequenz;
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
}
/*
Funktion fuer die Filterung mit dem EMA. Nutzt die vorher festgelegten alpha und beta.
*/
void lichtsteuerung::lichtsteuerung_ausfuehren(bool licht_heller, bool licht_blink){
	if (licht_blink){
        	if (blink_zaehler >= teilerverhaeltnis){	// Modulozaehler
          		blink_zaehler = 0;
          		licht_toggle = !licht_toggle;	}	// bei Ueberlauf LED blinken lassen
        	else{	blink_zaehler = (blink_zaehler + 1);	}
      	}
	else{
        	blink_zaehler = 0;
        	licht_toggle = 0;
	}

  /* 
  Idle: Stufe 1 -> 50% der roten LEDs an
  Bei Bremsen: Stufe 2 -> alle roten LEDs an
  Bei Gefahrbremsung: Wechsel zwischen Stufe 1 und Stufe 2 mit der Blinkfrequenz (Bereich 5~25Hz)
  */
  setNeoPixel(1 + (int)(licht_heller && !licht_blink) + (int)(licht_blink && licht_toggle));  
  
}

void lichtsteuerung::setNeoPixel(int workingMode){
  strip.setBrightness(255);
  switch (workingMode) {
    case 0:
    //off
          strip.clear();
          strip.show();
          break;
          
    case 1:
    //normal rearlight
          strip.clear();
          for(int i=0; i<strip.numPixels(); i += 2) { // For every second pixel in strip...
            strip.setPixelColor(i, lightred);
          }
          strip.show();
          break;
          
    case 2:
    //normal brake light
          strip.clear();
          strip.fill(fullred);
          strip.show();
          break;
          
    case 3:
    //emergency brake light
          strip.clear();
          strip.fill(fullred);
          for(int i=0; i<strip.numPixels(); i += 2) { // For every second pixel in strip...
            strip.setPixelColor(i, redwhite);
          }
          strip.show();
          break;
          
    default:
    //you should not be here!
          strip.clear();
          strip.fill(fullblue);
          strip.show();
          break;
  }
}

void lichtsteuerung::rainbowFade2White(int wait, int rainbowLoops, int wait2, int whiteLoops) {
  int fadeVal=0, fadeMax=100;

  // Hue of first pixel runs 'rainbowLoops' complete loops through the color
  // wheel. Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to rainbowLoops*65536, using steps of 256 so we
  // advance around the wheel at a decent clip.
  for(uint32_t firstPixelHue = 0; firstPixelHue < rainbowLoops*65536;
    firstPixelHue += 256) {

    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...

      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      uint32_t pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());

      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the three-argument variant, though the
      // second value (saturation) is a constant 255.
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue, 255,
        255 * fadeVal / fadeMax)));
    }

    strip.show();
    delay(wait);

    if(firstPixelHue < 65536) {                              // First loop,
      if(fadeVal < fadeMax) fadeVal++;                       // fade in
    } else if(firstPixelHue >= ((rainbowLoops-1) * 65536)) { // Last loop,
      if(fadeVal > 0) fadeVal--;                             // fade out
    } else {
      fadeVal = fadeMax; // Interim loop, make sure fade is at max
    }
  }

  
  for(int k=0; k<whiteLoops; k++) {
    for(int j=0; j<256; j++) { // Ramp up 0 to 255
      // Fill entire strip with white at gamma-corrected brightness level 'j':
      strip.fill(strip.Color(0, 0, 0, strip.gamma8(j)));
      strip.show();
    }
    delay(300); // Pause 1 second
    for(int j=255; j>=0; j--) { // Ramp down 255 to 0
      strip.clear();
      for(int i=0; i<strip.numPixels(); i += 2) { // For every second pixel in strip...
        strip.setPixelColor(i,(255-j)/5, 0, 0, strip.gamma8(j));
      }
      //strip.fill(strip.Color(strip.gamma8(255-j), 0, 0, strip.gamma8(j)));
      strip.show();
      delay(wait2);
    }
  }
}
