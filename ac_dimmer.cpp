#include "ac_dimmer.hpp"

#include <cmath>

//--------------------------------------------------
// AcDimmerInterruptHandler
//--------------------------------------------------

#define TICKS_PER_US 5U            // ticks per us (5 for TIM_DIV16 @ 80MHz)
#define GATE_PULSE_DURATION_US 50U // duration (us) to trigger the triac's gate
#define ZERO_CROSS_DURATION_US 16U // zero cross duration (us) between falling and rising edges

AcDimmer *AcDimmerInterruptHandler::smDimmer{nullptr};

AcDimmerInterruptHandler::AcDimmerInterruptHandler(AcDimmer *acDimmer)
{
  smDimmer = acDimmer;
}

AcDimmerInterruptHandler::~AcDimmerInterruptHandler()
{
  smDimmer = nullptr;
}

void IRAM_ATTR AcDimmerInterruptHandler::isr_timer()
{
  if (smDimmer->mGatePulseFlag)
  {
    GPOS = (1 << smDimmer->mGatePin);                    // turn the triac on
    smDimmer->mGatePulseFlag = false;                    // reset the flag to turn the triac off after gate pulse duration elapsed
    timer1_write(GATE_PULSE_DURATION_US * TICKS_PER_US); // set gate pulse duration time
  }
  else
  {
    GPOC = (1 << smDimmer->mGatePin); // turn the triac off
  }
}

void IRAM_ATTR AcDimmerInterruptHandler::isr_zeroCross()
{
  const uint32_t t = micros();
  const uint32_t tDelta = t - smDimmer->mLastZeroCrossTime;
  if (tDelta > smDimmer->mZcdThresholdTime)
  {
    if (smDimmer->mPhaseCutTime >= smDimmer->mMaxPhaseCutTime)
    {
      // too low dim level, turn the triac completely off
      GPOC = (1 << smDimmer->mGatePin);
    }
    else if (smDimmer->mPhaseCutTime <= smDimmer->mMinPhaseCutTime)
    {
      // too high dim level, turn the triac completely on
      GPOS = (1 << smDimmer->mGatePin);
    }
    else
    {
      smDimmer->mGatePulseFlag = true;                                                                         // set the flag to turn the triac on after phase cut off time elapsed
      const uint32_t deltaCorrFactor = smDimmer->mLastZeroCrossTime ? (smDimmer->mHalfCycleTime - tDelta) : 0; // calculate delta correction factor
      timer1_write((ZERO_CROSS_DURATION_US + smDimmer->mPhaseCutTime) * TICKS_PER_US - deltaCorrFactor);       // set phase cut off time with delta correction factor
    }

    smDimmer->mLastZeroCrossTime = t;
  }
}

//--------------------------------------------------
// AcDimmer
//--------------------------------------------------

AcDimmer::AcDimmer()
{
}

AcDimmer::~AcDimmer()
{
  destroy();
}

void AcDimmer::setup(const uint8_t gatePin, const uint8_t zeroCrossPulsePin, const uint8_t acHz, const float dimRangeMin, const float dimRangeMax, const float zcdThreshold)
{
  mGatePin = gatePin;
  mZeroCrossPulsePin = zeroCrossPulsePin;
  mAcHz = acHz;
  mDimRangeMin = dimRangeMin;
  mDimRangeMax = dimRangeMax;
  mZcdThreshold = zcdThreshold;
  updateConstants();

  mInterruptHandler = std::make_unique<AcDimmerInterruptHandler>(this);

  pinMode(mGatePin, OUTPUT);
  pinMode(mZeroCrossPulsePin, INPUT);
  digitalWrite(mGatePin, LOW);

  attachInterrupt(digitalPinToInterrupt(mZeroCrossPulsePin), &AcDimmerInterruptHandler::isr_zeroCross, FALLING);

  timer1_attachInterrupt(&AcDimmerInterruptHandler::isr_timer);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
}

void AcDimmer::destroy()
{
  timer1_disable();
  detachInterrupt(digitalPinToInterrupt(mZeroCrossPulsePin));
  mInterruptHandler.reset();

  digitalWrite(mGatePin, LOW);
}

void AcDimmer::setDimLevel(float dimLevel)
{
  mDimLevel = dimLevel;
  updateConstants();
}

float AcDimmer::getDimLevel()
{
  return mDimLevel;
}

void AcDimmer::setAcHertz(float acHz)
{
  mAcHz = acHz;
  updateConstants();
}

float AcDimmer::getAcHertz()
{
  return mAcHz;
}

void AcDimmer::setDimRange(const float dimRangeMin, const float dimRangeMax)
{
  mDimRangeMin = dimRangeMin;
  mDimRangeMax = dimRangeMax;
  updateConstants();
}

void AcDimmer::setDimRangeMin(const float dimRangeMin)
{
  mDimRangeMin = dimRangeMin;
  updateConstants();
}

float AcDimmer::getDimRangeMin()
{
  return mDimRangeMin;
}

void AcDimmer::setDimRangeMax(const float dimRangeMax)
{
  mDimRangeMax = dimRangeMax;
  updateConstants();
}

float AcDimmer::getDimRangeMax()
{
  return mDimRangeMax;
}

void AcDimmer::setZeroCrossDetectThreshold(const float zcdThreshold)
{
  mZcdThreshold = zcdThreshold;
}

float AcDimmer::getZeroCrossDetectThreshold()
{
  return mZcdThreshold;
}

void AcDimmer::updateConstants()
{
  // half cycle time = 0.5 seconds / AC Hz (e.g 10000us for 50Hz)
  const float halfCycleTime = 500000.0F / mAcHz;
  mHalfCycleTime = static_cast<uint32_t>(halfCycleTime);

  // minimum phase cut off time to apply maximum dim level (e.g 2500us at %25 for 50Hz)
  mMinPhaseCutTime = static_cast<uint32_t>(halfCycleTime * (1.0F - mDimRangeMax));

  // maximum phase cut off time to apply minimum dim level (e.g 7500us at %75 for 50Hz)
  mMaxPhaseCutTime = static_cast<uint32_t>(halfCycleTime * (1.0F - mDimRangeMin));

  // zero cross detection threshold time, (e.g when thresold is %98, interrupts which come during in %98 of the half cycle time will be discarded as false detection)
  // note: interrupts are not accurate enough on ESP8266, varying latencies always there up to 800us, so %99.2 is recommended as zcd detect threshold for 50Hz
  mZcdThresholdTime = static_cast<uint32_t>(halfCycleTime * mZcdThreshold) - 1U;

  // final RMS curved phase cut time, varies between max and min phase cut time regarding to the dim level
  mPhaseCutTime = static_cast<uint32_t>(mMaxPhaseCutTime - ((mMaxPhaseCutTime - mMinPhaseCutTime) * std::acos(1.0F - (2.0F * mDimLevel)) / 3.14159265F));
}