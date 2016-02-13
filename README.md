# LED_pots
LED PWM driving from an analog input (including linear and logarithmic curves)

Use a potentiometer (or really any A2D input) to drive the PWM to control LED brightness

Can handle any number of LEDs and A2D inputs (up to physical limits of course)

Originally designed to support 3 LED outputs and 3 control inputs to allow the
control of an RGB string of LEDs for color mixing

Two operating modes:

     If the LED_CONTROL_INPUT pin is low - just scale the A2D reading directly into
     a PWM value.  This wotrks great but your eye responds in a non-linear fashion at
     the high end fo the brightness so about 1/4+ of the A2D input causes almost no
     perceptible brightness change

     If the LED_CONTROL_INPUT pin is high - lookup a logarithmically-scaled value
     to give more of the A2D range to the bottom end and less to the top end where
     change are hardly noticable - actually is perceived to be more linear by
     the human eye

Changes to the LED_CONTROL_INPUT pin are picked up dynamically

Built using the 1.6.7 Arduino IDE
