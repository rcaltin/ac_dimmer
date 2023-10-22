# ACDimmer Lib

Simple, triac based, AC dimmer arduino library for ESP8266

- 50/60Hz AC leading edge dimming
- Zero cross detection
- User defined lower and upper dimming levels (to support dimmable led bulbs)
- RMS mapped dim levelling (bound to the lower and the upper limit)
- Ships with required interrupts implemented internally
- Hz, Dim Level, Min/Max Range, ZCD Threshold parameters can be modified on the fly

## Usage

```
#include "ac_dimmer.hpp"

#define GATE_PIN 12
#define ZERO_CROSS_PULSE_PIN 13

AcDimmer dimmer;

void setup()
{
  dimmer.setup(GATE_PIN, ZERO_CROSS_PULSE_PIN);

  // ...
}

void loop()
{
  // set a new dim level (note that: 0 fully off, 1 fully on, dim level always between 0 and 1)
  // dimmer.setDimLevel(0.5F);

  // set a new dim range as %23 to %77 on the fly, dimming level will be re-mapped into this range
  // dimmer.setDimRange(0.23F, 0.77F);

  // ...
}

```
