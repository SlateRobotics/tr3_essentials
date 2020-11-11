#ifndef STATUSLED_H
#define STATUSLED_H

#include <FastLED.h>

#define NUM_LEDS 2
#define DATA_PIN 25

#define LED_RED 0
#define LED_GREEN 1
#define LED_BLUE 2
#define LED_CYAN 3
#define LED_MAGENTA 4
#define LED_YELLOW 5
#define LED_WHITE 6

class LED {
  private:
    CRGB leds[NUM_LEDS];

    int pulseDuration = 5000; // ms
    unsigned long pulseBegin = 0; // millis

    int i = 0;

    int blinkDuration = 750; // ms
    unsigned long blinkBegin = 0; // millis
    int blinkStage = 0;
    bool blinkStarted = false;

    float ledBrightness = 0.25;
  
  public:
    void setUp() {
      FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    }

    void show (int i, int r, int g, int b) {
      for (int j = 0; j < 2; j++) {
        leds[i].red = r;
        leds[i].green = g;
        leds[i].blue = b;
        leds[i] /= int(1.0 / ledBrightness);
        FastLED.show();
      }
    }

    void red () {
      show(0, 255, 0, 0);
      show(1, 255, 0, 0);
    }

    void green () {
      show(0, 0, 255, 0);
      show(1, 0, 255, 0);
    }

    void blue () {
      show(0, 0, 0, 255);
      show(1, 0, 0, 255);
    }

    void cyan () {
      show(0, 0, 255, 255);
      show(1, 0, 255, 255);
    }

    void magenta () {
      show(0, 255, 0, 255);
      show(1, 255, 0, 255);
    }

    void yellow () {
      show(0, 255, 255, 0);
      show(1, 255, 255, 0);
    }

    void white () {
      show(0, 255, 255, 255);
      show(1, 255, 255, 255);
    }

    void off () {
      show(0, 0, 0, 0);
      show(1, 0, 0, 0);
    }

    void increment() {
      float pulseDurationHalf = pulseDuration / 2.0;
      
      if (i <= 0) {
        // this restarts the pulse and kicks it 1 step forward
        pulseBegin = millis() - (pulseDurationHalf / 255.0);
      }

      float pulseDelta = millis() - pulseBegin;
    
      if (pulseDelta < pulseDurationHalf) {
        i = pulseDelta / pulseDurationHalf * 255.0;
      } else {
        i = 255 - ((pulseDelta - pulseDurationHalf) / pulseDurationHalf * 255.0);
      }

      if (i > 255) {
        i = 255;
      } else if (i < 0) {
        i = 0;
      }
    }

    void pulse (int c1 = LED_WHITE, int c2 = -1) {
      increment();
      
      if (c1 != LED_WHITE && c2 == -1) {
        c2 = c1;
      } else if (c2 == -1) {
        c2 = LED_WHITE;
      }
      
      if (c1 == LED_WHITE) {
        show(0, (0 + i), (0 + i), (0 + i));
      } else if (c1 == LED_CYAN) {
        show(0, 0, (0 + i), (0 + i));
      } else if (c1 == LED_MAGENTA) {
        show(0, (0 + i), 0, (0 + i));
      } else if (c1 == LED_YELLOW) {
        show(0, (0 + i), (0 + i), 0);
      } else if (c1 == LED_RED) {
        show(0, (0 + i), 0, 0);
      } else if (c1 == LED_GREEN) {
        show(0, 0, (0 + i), 0);
      } else if (c1 == LED_YELLOW) {
        show(0, 0, 0, (0 + i));
      }
      
      if (c2 == LED_WHITE) {
        show(1, (0 + i), (0 + i), (0 + i));
      } else if (c2 == LED_CYAN) {
        show(1, 0, (0 + i), (0 + i));
      } else if (c2 == LED_MAGENTA) {
        show(1, (0 + i), 0, (0 + i));
      } else if (c2 == LED_YELLOW) {
        show(1, (0 + i), (0 + i), 0);
      } else if (c1 == LED_RED) {
        show(1, (0 + i), 0, 0);
      } else if (c1 == LED_GREEN) {
        show(1, 0, (0 + i), 0);
      } else if (c1 == LED_YELLOW) {
        show(1, 0, 0, (0 + i));
      }
    }

    void blink (int r1 = -1, int g1 = -1, int b1 = -1, int r2 = -1, int g2 = -1, int b2 = -1) {
      bool customColorLed1 = false;
      bool customColorLed2 = false;
      
      if (r1 != -1 || g1 != -1 || b1 != -1) {
        customColorLed1 = true;
      }
      
      if (r2 != -1 || g2 != -1 || b2 != -1) {
        customColorLed2 = true;
      }
      
      if (blinkStarted == false) {
        blinkBegin = millis();
        blinkStarted = true;
      }

      if (millis() - blinkBegin < blinkDuration / 2.0) {
        blinkStage = 0;
      } else if (millis() - blinkBegin < blinkDuration) {
        blinkStage = 1;
      } else {
        blinkStarted = false;
      }

      if (blinkStage == 0) {
        if (customColorLed1 == true && customColorLed2 == false) {
          show(0, r1, g1, b1);
          show(1, r1, g1, b1);
        } else if (customColorLed1 == true && customColorLed2 == true) {
          show(0, r1, g1, b1);
          show(1, r2, g2, b2);
        } else {
          cyan();
        }
      } else {
        off();
      }
      
      FastLED.show();
    }
};

#endif
