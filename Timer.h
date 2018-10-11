#include <iostream>
// #include <chrono>

#ifndef TIMER_H
#define TIMER_H

class Timer {
  public:
    long realTimeElapsedSinceSimStart;
    long simulatedTimeElapsedSinceSimStart;
    long realDeltaTime;
    long simulatedDeltaTime;
    long startTime_ms;
    int startTime_zulu;
    int warpFactor;
    long currentTickBegin;
    long ticks;

    Timer();
    bool zuluTimeHasCome(int zuluTime);
    long getMillisecondsOffsetFromStart(int zuluTime);
    long getRealDeltaTime();
    long getSimulatedDeltaTime();
    long getSimulatedTimeElapsedSinceSimStart();
    long getRealTimeElapsedSinceSimStart ();
    long getElapsedTime_NoWarp(long glutElapsedTime);
    void recordTick(long glutElapsedTime);
    long zulu_to_millisecondOffset();
    void setWarpFactor(int newWarpFactor);
    // std::chrono::high_resolution_clock::time_point start;
    //
    // Timer();
  private:

};

#endif
