#include <iostream>
// #include <chrono>

#include "Timer.h"
#include "constants.h"
// #include "chrono_io"

Timer::Timer () {
  startTime_ms = 0;
  currentTickBegin = NULL;
  realTimeElapsedSinceSimStart = 0;
  simulatedTimeElapsedSinceSimStart = 0;
  realDeltaTime = 0;
  simulatedDeltaTime = 0;
  startTime_zulu = 0;
  warpFactor = 1;
  ticks = 0;
}

bool Timer::zuluTimeHasCome(int zuluTime) {
  long msOffsetFromStart = getMillisecondsOffsetFromStart(zuluTime);

  // std::cout << "simulatedTimeElapsedSinceSimStart: " << simulatedTimeElapsedSinceSimStart << std::endl;
  // std::cout << "msOffsetFromStart: " << msOffsetFromStart << std::endl;
  // std::cout << "Time has come? " << ((simulatedTimeElapsedSinceSimStart >= msOffsetFromStart) ? "Yes" : "No") << std::endl;

  if (simulatedTimeElapsedSinceSimStart >= msOffsetFromStart) {
    return true;
  } else {
    return false;
  }
}

long Timer::zulu_to_millisecondOffset() {

}

long Timer::getMillisecondsOffsetFromStart(int zuluTime) {
  long minutesOffsetFromStart = zuluTime - startTime_zulu;
  long millisecondsOffsetFromStart = minutesOffsetFromStart * UNIT_CONVERSION_CONSTANTS.MILLISEC_MIN;
  return millisecondsOffsetFromStart;
}

long Timer::getRealDeltaTime() {
  return realDeltaTime;
}

long Timer::getSimulatedDeltaTime() {
  return simulatedDeltaTime;
}

long Timer::getSimulatedTimeElapsedSinceSimStart() {
  return simulatedTimeElapsedSinceSimStart;
}

long Timer::getElapsedTime_NoWarp(long glutElapsedTime) {
  return glutElapsedTime - currentTickBegin;
}

long Timer::getRealTimeElapsedSinceSimStart () {
  return realTimeElapsedSinceSimStart;
}

void Timer::setWarpFactor (int newWarpFactor) {
  // -1 is for calibrating warpFactor to reflect the real-world time it is meant to simualte
  // if (newWarpFactor > 1) {
  //   warpFactor = newWarpFactor - 1;
  // }
  warpFactor = newWarpFactor;
}

void Timer::recordTick(long glutElapsedTime) {
  realTimeElapsedSinceSimStart += glutElapsedTime;
  std::cout << "recordTick: realTimeElapsedSinceSimStart = " << realTimeElapsedSinceSimStart << std::endl;
  std::cout << "recordTick: currentTickBegin = " << currentTickBegin << std::endl;
  realDeltaTime = realTimeElapsedSinceSimStart - currentTickBegin;
  simulatedDeltaTime = realDeltaTime * warpFactor;
  simulatedTimeElapsedSinceSimStart += simulatedDeltaTime;
  currentTickBegin = realTimeElapsedSinceSimStart;
  ticks++;
}
