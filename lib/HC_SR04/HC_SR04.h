#ifndef HC_SR04_H
#define HC_SR04_H

#include <Arduino.h>

#define CM true
#define INCH false

class HC_SR04 {
  public:
    HC_SR04(int trigger, int echo, int interrupt, int max_dist=200);
    
    void begin();
    void start();
    bool isFinished(){ return _finished; }
    unsigned int getRange(bool units=CM);
    static HC_SR04* instance(){ return _instance; }

    int _int, _echo;
    volatile unsigned long _start, _end;
    volatile bool _finished;
    
  private:
    int _trigger, _max;
    static HC_SR04* _instance;
};

#endif
