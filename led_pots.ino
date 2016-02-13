// leds_pots
//
// use a potentiometer (or really any A2D input) to drive the PWM to control LED brightness
//
// Can handle any number of LEDs and A2D inputs (up to physical limits of course)
//
// Originally designed to support 3 LED outputs and 3 control inputs to allow the
// control of an RGB string of LEDs for color mixing
//
// Two operating modes:
//
//     If the LED_CONTROL_INPUT pin is low - just scale the A2D reading directly into
//     a PWM value.  This wotrks great but your eye responds in a non-linear fashion at
//     the high end fo the brightness so about 1/4+ of the A2D input causes almost no
//     perceptible brightness change
//
//     If the LED_CONTROL_INPUT pin is high - lookup a logarithmically-scaled value
//     to give more of the A2D range to the bottom end and less to the top end where
//     change are hardly noticable - actually is perceived to be more linear by
//     the human eye
//
// Changes to the LED_CONTROL_INPUT pin are picked up dynamically
//
// Built using the 1.6.7 Arduino IDE
//
//
// Copyright (c) 2016, Christian Herzog
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
//    conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior written
//    permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.



#define TRUE              1
#define FALSE             0


#define LED_CONTROL_INPUT 9     // if low, use a linear pot input to LED brightness

#define LED_MAX_BRIGHTNESS 255


#define LED_PORT_RED      3
#define LED_PORT_GREEN    5
#define LED_PORT_BLUE     6

#define POT_PORT_RED      A0
#define POT_PORT_GREEN    A1
#define POT_PORT_BLUE     A2


#define ADC_BITS          10    //a2d resolution - 10 works on all architectures


#define LED_HEARTBEAT_PIN 13    // Most Ardino boards have a built-in LED on pin 13
                                // we're using it as a heartbeat indicator so you can tell
                                // board is running if you have very long periods times and
                                // the LED outputs are off because of where they are in their period
                                
#define HEARTBEAT_PERIOD   500  // in milliseconds - on or off time - on 1/2 second - off 1/2 second


typedef struct {
  int   ledPort;
  int   potPort;
} LED_POT;

LED_POT leds[] = {

  LED_PORT_RED,   POT_PORT_RED,
  LED_PORT_GREEN, POT_PORT_GREEN,
  LED_PORT_BLUE,  POT_PORT_BLUE,
  0,              0
  };
 

unsigned char curve[] = {

// organized from dim to bright values - traversed appropriately in the bright and dim cycles
//
// generated from a spreadsheet using the log() function but could be any set of arbitrary values including
// non-linear or repeating values

// minimally populated log curve - little choppy in the bottom end
//
//    0,   1,   2,   2,   3,   4,   5,   6,   7,   8, 
//    8,   9,  10,  11,  12,  13,  14,  15,  16,  17, 
//   18,  19,  20,  21,  22,  23,  24,  25,  26,  28,  29, 
//   30,  31,  33,  34,  36,  37,  39,  40,  42,  43, 
//   45,  47,  48,  50,  52,  54,  56,  58,  60,  63, 
//   65,  67,  70,  72,  75,  78,  81,  84,  88,  91, 
//   95,  99, 104, 108, 113, 119, 125, 132, 140, 149, 
//  160, 173, 190, 214, 238, 255

// fully populated log curve (255 entries)

   0,   0,   0,   1,   1,   1,   1,   1, 
   1,   2,   2,   2,   2,   2,   3,   3, 
   3,   3,   3,   4,   4,   4,   4,   4, 
   5,   5,   5,   5,   5,   6,   6,   6, 
   6,   6,   7,   7,   7,   7,   7,   8, 
   8,   8,   8,   8,   9,   9,   9,   9, 
  10,  10,  10,  10,  10,  11,  11,  11, 
  11,  12,  12,  12,  12,  13,  13,  13, 
  13,  14,  14,  14,  14,  15,  15,  15, 
  15,  16,  16,  16,  16,  17,  17,  17, 
  17,  18,  18,  18,  18,  19,  19,  19, 
  19,  20,  20,  20,  21,  21,  21,  21, 
  22,  22,  22,  23,  23,  23,  24,  24, 
  24,  24,  25,  25,  25,  26,  26,  26, 
  27,  27,  27,  28,  28,  28,  29,  29, 
  29,  30,  30,  30,  31,  31,  31,  32, 
  32,  32,  33,  33,  34,  34,  34,  35, 
  35,  35,  36,  36,  37,  37,  37,  38, 
  38,  39,  39,  40,  40,  40,  41,  41, 
  42,  42,  43,  43,  44,  44,  44,  45, 
  45,  46,  46,  47,  47,  48,  48,  49, 
  49,  50,  51,  51,  52,  52,  53,  53, 
  54,  55,  55,  56,  56,  57,  58,  58, 
  59,  59,  60,  61,  62,  62,  63,  64, 
  64,  65,  66,  67,  67,  68,  69,  70, 
  71,  71,  72,  73,  74,  75,  76,  77, 
  78,  79,  80,  81,  82,  83,  84,  85, 
  86,  88,  89,  90,  91,  93,  94,  96, 
  97,  98, 100, 102, 103, 105, 107, 109, 
 111, 113, 115, 117, 120, 122, 125, 127, 
 130, 134, 137, 141, 145, 149, 154, 159, 
 165, 173, 181, 191, 204, 223, 255 

};


#define CURVE_STEPS     (sizeof(curve)/sizeof(curve[0]))  // dynamically adjusts to the number of entries in the curve


int MaxPotInput;        // most Arduinos are 10-bit A2D but if we will be running on a platform with more, we can adjust
                        // For Due and Zero platforms



void setup() {

int i;

  MaxPotInput = pow(2, ADC_BITS) - 1;        // maximum a2d input value
  
  for (i = 0 ; leds[i].ledPort != 0 ; i++) {
    pinMode(leds[i].ledPort, OUTPUT);
    analogWrite(leds[i].ledPort, 0);           // start with LEDs at minimum

    pinMode(leds[i].potPort, INPUT);
    digitalWrite(leds[i].potPort, LOW);        // no pull up resistors enabled
  } 

  analogReference(DEFAULT);                    // VCC (5V or 3.3V as appropriate)

  // analogReadResolution(ADC_BITS);           // set resolution - uncomment for Due or Zero platforms

  
  pinMode(LED_HEARTBEAT_PIN, OUTPUT);
  digitalWrite(LED_HEARTBEAT_PIN, HIGH);

  pinMode(LED_CONTROL_INPUT, INPUT_PULLUP);    // if this port is pulled low, linear pot -> LED mapping
}



void loop() {

int now, oldNow = millis();
int i, inputVal, curveIndex;
int heartbeat, heartbeatCount;

  heartbeat      = TRUE;
  heartbeatCount = 0;
  now            = 0;

  while (TRUE) {

   now = millis();
   
   // run through the LED table and read the corresponding analog potentiometer port,
   // do a mapping or lookup in the curve[] table, and set the corresponding LED PWM
   // rate appropriately

   for (i = 0 ; leds[i].ledPort != 0 ; i++) {

    inputVal = analogRead(leds[i].potPort);

    if ( ! digitalRead(LED_CONTROL_INPUT)) {         // map pot input directly to a LED PWM value if low

      analogWrite(leds[i].ledPort, (int)((float)inputVal / (float)MaxPotInput * LED_MAX_BRIGHTNESS));
      
    } else {                               // use brightness curve table
      
      curveIndex = (float)inputVal / (float)MaxPotInput * (CURVE_STEPS-1);

      analogWrite(leds[i].ledPort, curve[curveIndex]);
    }    
  }

  // handle the heartbeat LED - on/off as per defined cycle at a 50% duty cycle so you know it's still running
  
  heartbeatCount += now - oldNow;               // get the time elapsed since the last go around
  
  if (heartbeatCount > HEARTBEAT_PERIOD) {      // time to update the heartbeat LED?
    if (heartbeat) {                            // toggle it
      digitalWrite(LED_HEARTBEAT_PIN, LOW);
      heartbeat = FALSE;
    } else {
      digitalWrite(LED_HEARTBEAT_PIN, HIGH);
      heartbeat = TRUE;
    }
    heartbeatCount = 0;
  }
  
  oldNow = now;                                 // save the current "now" tick count

  }
}
