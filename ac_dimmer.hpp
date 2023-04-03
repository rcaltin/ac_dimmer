#pragma once

#include <Arduino.h>
#include <memory>

//--------------------------------------------------
// Forward Declerations
//--------------------------------------------------
class AcDimmer;

//--------------------------------------------------
// AcDimmerInterruptHandler
//--------------------------------------------------
class AcDimmerInterruptHandler
{
public:
  AcDimmerInterruptHandler(AcDimmer *acDimmer);

  ~AcDimmerInterruptHandler();

  static void IRAM_ATTR isr_timer();

  static void IRAM_ATTR isr_zeroCross();

private:
  static AcDimmer *smDimmer;
};

//--------------------------------------------------
// AcDimmer
//--------------------------------------------------
class AcDimmer
{
public:
  friend class AcDimmerInterruptHandler;

  AcDimmer();

  ~AcDimmer();

  void setup(
      const uint8_t gatePin,
      const uint8_t zeroCrossPulsePin,
      const uint8_t acHz = 50U,
      const float dimRangeMin = 0.25F,
      const float dimRangeMax = 0.75F,
      const float zcdThreshold = 0.992F);

  void destroy();

  void setDimLevel(const float dimLevel);

  float getDimLevel();

  void setAcHertz(const float acHz);

  float getAcHertz();

  void setDimRange(const float dimRangeMin, const float dimRangeMax);

  void setDimRangeMin(const float dimRangeMin);

  float getDimRangeMin();

  void setDimRangeMax(const float dimRangeMax);

  float getDimRangeMax();

  void setZeroCrossDetectThreshold(const float zcdThreshold);

  float getZeroCrossDetectThreshold();

private:
  void updateConstants();

  std::unique_ptr<AcDimmerInterruptHandler> mInterruptHandler;

  float mDimLevel{0};
  float mAcHz{0};
  float mDimRangeMin{0};
  float mDimRangeMax{0};
  float mZcdThreshold{0};
  volatile uint8_t mGatePin{0};
  volatile uint8_t mZeroCrossPulsePin{0};
  volatile uint32_t mHalfCycleTime{0};
  volatile uint32_t mMinPhaseCutTime{0};
  volatile uint32_t mMaxPhaseCutTime{0};
  volatile uint32_t mZcdThresholdTime{0};
  volatile uint32_t mPhaseCutTime{0};
  volatile uint32_t mLastZeroCrossTime{0};
  volatile bool mGatePulseFlag{false};
};